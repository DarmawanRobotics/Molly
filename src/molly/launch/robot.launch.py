#!/usr/bin/env python3
"""
The whole Molly robot in one launch (production: molly.service).

    ros2 launch molly robot.launch.py                          # guide, the last site
    ros2 launch molly robot.launch.py mode:=mapping site:=lobby   # map a new site
    ros2 launch molly robot.launch.py site:=museum conversation:=true
    ros2 launch molly robot.launch.py robot_bringup:=lite3_bringup

Works with any quadruped: its driver, URDF and sensors come from a robot package started with
robot_bringup (contract in src/xtras/README.md; the Lite3 one is lite3_ros2). Every Molly node
respawns 3 s after it dies. A site configuration file (by default /etc/molly/site.yaml) is applied
on top of the package defaults of every node. With MediaMTX installed (script/setup.sh install
webrtc) the handheld gets its video over WebRTC; without it, as JPEG over the teleop link. With
web:=true the dashboard reaches the robot through rosbridge (whitelisted names only) and
web_video_server.
"""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from molly import video
from molly.bringup import (
    BringupError,
    expected_nodes,
    load_site_config,
    node_parameter,
    non_empty,
    package_version,
    resolve_mode,
    resolve_namespace,
    resolve_odometry,
    resolve_robot_bringup,
    resolve_site,
    site_manager_overrides,
    truthy,
    web_globs,
)

CAMERA = '/camera/camera/color'
MEDIAMTX_CONFIG = '~/.molly/mediamtx.yml'


def share(package: str, *path: str) -> str:
    """Return a file in the installed share directory of a package."""
    return os.path.join(get_package_share_directory(package), *path)


def robot_bringup_action(value: str, ns: str):
    """Include the robot package's launch file with namespace:=<robot_ns>, or None if unset."""
    robot = resolve_robot_bringup(value)
    if robot is None:
        return None
    package, launch_file = robot
    try:
        path = share(package, 'launch', launch_file)
    except PackageNotFoundError as error:
        raise BringupError(
            f'robot_bringup: package {package!r} is not built in this workspace '
            '(add the robot package, see src/xtras/README.md)'
        ) from error
    if not os.path.isfile(path):
        raise BringupError(f'robot_bringup: {path} does not exist')
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(path), launch_arguments={'namespace': ns}.items()
    )


def video_server(webrtc: str, binary: str, params_files: list):
    """
    Return (MediaMTX process or None, log line) for the handheld's WebRTC video.

    MediaMTX reads the teleop token from the same parameter files as teleop_udp_node, and starts
    video_stream_node with those files while the handheld watches.
    """
    if not truthy(webrtc):
        return None, 'WebRTC video off (WEBRTC=false): the handheld gets JPEG frames'
    found = video.find_mediamtx(binary)
    if found is None:
        return None, (
            'WebRTC video off: MediaMTX not found (script/setup.sh install webrtc, or MEDIAMTX); '
            'the handheld gets JPEG frames'
        )
    version = video.mediamtx_version(found)
    if version is None or version < video.MIN_VERSION:
        return None, (
            f'WebRTC video off: {found} is {video.version_text(version)}, it needs v1.11 or '
            'newer (script/setup.sh install webrtc)'
        )
    try:
        executable = os.path.join(
            get_package_prefix('molly_bridge'), 'lib', 'molly_bridge', 'video_stream_node'
        )
    except PackageNotFoundError:
        executable = ''
    if not os.path.isfile(executable):
        return None, 'WebRTC video off: video_stream_node is not built (setup.sh install build)'
    missing = video.gstreamer_missing()
    if missing:
        return None, f'WebRTC video off: {missing} (script/setup.sh install webrtc)'
    token = str(node_parameter(params_files, 'teleop_udp_node', 'token', '') or '')
    command = video.node_command(executable, params_files, video.rtsp_url())
    try:
        path = video.write_config(video.mediamtx_config(command, token, version), MEDIAMTX_CONFIG)
    except OSError as error:
        return None, f'WebRTC video off: cannot write {MEDIAMTX_CONFIG}: {error}'
    process = ExecuteProcess(
        cmd=[found, path], name='mediamtx', output='screen', respawn=True, respawn_delay=3.0
    )
    return process, (
        f'WebRTC video: MediaMTX {video.version_text(version)} ({found}), '
        f'WHEP on port {video.WEBRTC_PORT}, media on UDP {video.ICE_PORT}'
    )


def package_found(package: str) -> bool:
    """Return True when a package is installed (optional parts start only then)."""
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


def web_actions(ns: str, port: str, video_port: str):
    """Return (rosbridge, rosapi and web_video_server nodes, their names, log lines)."""
    globs = web_globs(ns)
    actions, names, notes = [], [], []
    if package_found('rosbridge_server') and package_found('rosapi'):
        actions += [
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                parameters=[
                    {
                        'port': int(port),
                        'address': '',
                        'call_services_in_new_thread': True,
                        'send_action_goals_in_new_thread': True,
                        **globs,
                    }
                ],
                respawn=True,
                respawn_delay=3.0,
                output='screen',
            ),
            Node(
                package='rosapi',
                executable='rosapi_node',
                name='rosapi_node',
                parameters=[globs],
                respawn=True,
                respawn_delay=3.0,
                output='screen',
            ),
        ]
        names += ['rosbridge_websocket', 'rosapi_node']
        notes.append(f'web: rosbridge on ws port {port} (whitelisted names, docs/web.md)')
    else:
        notes.append('web: rosbridge_server not installed (script/setup.sh install web)')
    if package_found('web_video_server'):
        actions.append(
            Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video_server',
                parameters=[{'port': int(video_port), 'default_stream_type': 'mjpeg'}],
                respawn=True,
                respawn_delay=3.0,
                output='screen',
            )
        )
        names.append('web_video_server')
        notes.append(f'web: camera MJPEG on http port {video_port}')
    else:
        notes.append('web: web_video_server not installed (script/setup.sh install web)')
    return actions, names, notes


def launch_setup(context):
    """Validate the arguments and build every action of the robot."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    mode = resolve_mode(arg('mode'))
    odometry = resolve_odometry(arg('odometry'))
    ns = resolve_namespace(arg('robot_ns'))
    site_name = resolve_site(arg('site'))
    if mode == 'mapping' and not site_name:
        raise BringupError('mode:=mapping needs site:=<name> (SITE in the env file)')
    site, warnings = load_site_config(arg('site_config'))
    robot = robot_bringup_action(arg('robot_bringup'), ns)
    sites_dir = arg('sites_dir').strip()

    def topic(name: str) -> str:
        return f'/{ns}/{name}'

    started = []  # node names, for the health monitor's liveness check

    def node(package, executable, defaults, overrides=None, remappings=None, **kwargs):
        """Molly node: package defaults, then the site file, then non-empty overrides."""
        params = [*defaults, *site]
        if overrides and non_empty(overrides):
            params.append(non_empty(overrides))
        name = kwargs.pop('name', executable)
        started.append(name)
        return Node(
            package=package,
            executable=executable,
            name=name,
            parameters=params,
            remappings=remappings or [],
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            **kwargs,
        )

    nav = share('molly', 'config', 'navigation_params.yaml')
    perception = share('molly', 'config', 'perception_params.yaml')
    bridge = share('molly_bridge', 'config', 'bridge_params.yaml')
    mapping = share('molly', 'config', 'mapping_params.yaml')
    interaction = share('molly_interaction', 'config', 'interaction_params.yaml')
    version = package_version(share('molly', 'package.xml'))
    git = os.environ.get('MOLLY_GIT', 'unknown')

    mediamtx, video_note = video_server(arg('webrtc'), arg('mediamtx'), [bridge, *site])
    teleop = {'robot_command_prefix': f'/{ns}/', 'routes_dir': sites_dir}
    if mediamtx is not None:
        teleop['video_port'] = video.WEBRTC_PORT  # the handheld asks for WebRTC video here

    actions = [
        LogInfo(
            msg=f'Molly {version} ({git}) mode={mode} site={site_name or "(last)"} '
            f'robot_ns={ns} robot_bringup={arg("robot_bringup") or "(started elsewhere)"} '
            f'odometry={odometry} site_config={site[0] if site else "(none)"}'
        ),
        *[LogInfo(msg=f'WARNING {w}') for w in warnings],
        LogInfo(msg=video_note),
    ]
    if robot is not None:
        actions.append(robot)
    if mediamtx is not None:
        actions.append(mediamtx)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                share('dharma_odometry', 'launch', 'odometry.launch.py')
            ),
            launch_arguments={
                'method': odometry,
                'save_map': str(mode == 'mapping'),
                'respawn': 'true',
                'record': 'true',
                'leg_odom_topic': topic('odom'),
                'site_config': site[0] if site else '',
                'sites_dir': sites_dir,
                'params_file': share('molly', 'config', 'odometry_params.yaml'),
            }.items(),
        )
    )
    actions += [
        # the site manager hands each site's tags.json to localization (localization/load_tags)
        node(
            'dharma_localization',
            'localization_node',
            [share('molly', 'config', 'localization_params.yaml')],
        ),
        node('dharma_mapping', 'grid_mapper_node', [mapping]),
        node(
            'dharma_mapping',
            'site_manager_node',
            [mapping],
            site_manager_overrides(mode, site_name, sites_dir),
        ),
        node('dharma_perception', 'lidar_sector_node', [perception]),
        # the tag detector sees the camera at rate_hz only: localization needs a few poses/s
        node(
            'dharma_perception',
            'image_throttle_node',
            [perception],
            name='apriltag_throttle',
            remappings=[
                ('in/image', f'{CAMERA}/image_raw'),
                ('in/camera_info', f'{CAMERA}/camera_info'),
                ('out/image', '/perception/apriltag/image_rect'),
                ('out/camera_info', '/perception/apriltag/camera_info'),
            ],
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[perception, *site],
            remappings=[
                ('image_rect', '/perception/apriltag/image_rect'),
                ('camera_info', '/perception/apriltag/camera_info'),
                ('detections', '/perception/apriltag/detections'),
            ],
            respawn=True,
            respawn_delay=3.0,
            output='screen',
        ),
        node('dharma_navigation', 'path_loader_node', [nav], {'waypoints_file': arg('route')}),
        node('dharma_navigation', 'obstacle_avoidance_node', [nav]),
        node(
            'dharma_navigation',
            'path_follower_node',
            [nav],
            remappings=[('cmd_vel', '/bridge/nav_cmd_vel'), ('battery', topic('battery'))],
        ),
        node('dharma_navigation', 'planner_node', [nav]),
        node('molly_guide', 'guide_node', [share('molly_guide', 'config', 'guide_params.yaml')]),
        # gestures (the guide, greeter, handheld and dashboard use them) and the greeter
        node(
            'molly_interaction',
            'interaction_node',
            [interaction],
            {'robot_ns': ns, 'greeter_available': truthy(arg('greeter'))},
        ),
        node(
            'dharma_control',
            'control_mux_node',
            [bridge],
            remappings=[('cmd_vel', topic('cmd_vel'))],
        ),
        node(
            'molly_bridge',
            'teleop_udp_node',
            [bridge],
            teleop,
            remappings=[
                ('cmd_vel', topic('cmd_vel')),
                ('battery', topic('battery')),
                ('diagnostics', topic('diagnostics')),
            ],
        ),
    ]
    if truthy(arg('visitors')):
        actions.append(node('dharma_perception', 'visitor_tracker_node', [perception]))
    if truthy(arg('speech')):
        actions.append(node('molly_interaction', 'speech_node', [interaction]))
    if truthy(arg('conversation')):
        actions.append(node('molly_interaction', 'conversation_node', [interaction]))
    if truthy(arg('web')):
        web, names, notes = web_actions(ns, arg('web_port'), arg('web_video_port'))
        actions += [*web, *[LogInfo(msg=n) for n in notes]]
        started += names
    if truthy(arg('health')):
        actions.append(
            node(
                'dharma_health',
                'health_monitor_node',
                [share('molly', 'config', 'health_params.yaml')],
                {'expected_nodes': expected_nodes(started, odometry), 'robot_ns': ns},
            )
        )
    return actions


def generate_launch_description():
    """Declare the arguments (most default to the environment of molly.service)."""
    env = lambda name, default='': EnvironmentVariable(name, default_value=default)  # noqa: E731
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_ns',
                default_value=env('ROBOT_NS', 'robot'),
                description='namespace of the robot driver (/<robot_ns>/cmd_vel, /odom, ...)',
            ),
            DeclareLaunchArgument(
                'robot_bringup',
                default_value=env('ROBOT_BRINGUP'),
                description='robot package <pkg>[/<file>.launch.py]; empty: started elsewhere',
            ),
            DeclareLaunchArgument(
                'mode',
                default_value=env('MODE', 'guide'),
                description='guide | mapping (maps site:= at start when it is new)',
            ),
            DeclareLaunchArgument(
                'site',
                default_value=env('SITE'),
                description='site to load (guide) or to map (mapping); empty: the last one',
            ),
            DeclareLaunchArgument(
                'sites_dir',
                default_value=env('SITES_DIR'),
                description='directory of the sites (map, tags, POIs, tours, routes)',
            ),
            DeclareLaunchArgument(
                'odometry',
                default_value=env('ODOMETRY', 'fast_lio'),
                description='fast_lio | leg_kilo | external (simulator)',
            ),
            DeclareLaunchArgument(
                'route', default_value=env('ROUTE'), description='route CSV; empty: last loaded'
            ),
            DeclareLaunchArgument(
                'site_config',
                default_value=env('SITE_CONFIG', '/etc/molly/site.yaml'),
                description='per-robot parameter overrides; missing file: none',
            ),
            DeclareLaunchArgument(
                'visitors', default_value=env('VISITORS', 'true'), description='LiDAR visitors'
            ),
            DeclareLaunchArgument(
                'speech', default_value=env('SPEECH', 'true'), description='text to speech'
            ),
            DeclareLaunchArgument(
                'greeter', default_value=env('GREETER', 'true'), description='greet visitors'
            ),
            DeclareLaunchArgument(
                'conversation',
                default_value=env('CONVERSATION', 'false'),
                description='answer questions with a local LLM (Ollama)',
            ),
            DeclareLaunchArgument(
                'web',
                default_value=env('WEB', 'true'),
                description='rosbridge + web_video_server for the dashboard',
            ),
            DeclareLaunchArgument('web_port', default_value=env('WEB_PORT', '9090')),
            DeclareLaunchArgument('web_video_port', default_value=env('WEB_VIDEO_PORT', '8080')),
            DeclareLaunchArgument('health', default_value='true'),
            DeclareLaunchArgument(
                'webrtc',
                default_value=env('WEBRTC', 'true'),
                description='handheld video over WebRTC (MediaMTX); false: JPEG frames only',
            ),
            DeclareLaunchArgument(
                'mediamtx',
                default_value=env('MEDIAMTX'),
                description=f'MediaMTX binary; empty: {video.INSTALL_DIR}, then PATH',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
