#!/usr/bin/env python3
"""
The whole Molly robot in one launch (production: molly.service).

    ros2 launch molly robot.launch.py                    # patrol
    ros2 launch molly robot.launch.py mode:=teach        # keep the map, record a route
    ros2 launch molly robot.launch.py odometry:=leg_kilo
    ros2 launch molly robot.launch.py robot_bringup:=my_dog_bringup/bringup.launch.py

Works with any quadruped: its driver, URDF and sensors come from a robot package started with
robot_bringup (contract in src/xtras/README.md). Every Molly node respawns 3 s after it dies.
A site configuration file (by default /etc/molly/site.yaml) is applied on top of the package
defaults of every node. With MediaMTX installed (script/setup.sh install webrtc) the handheld gets
its video over WebRTC; without it, as JPEG over the teleop link.
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
    truthy,
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


def launch_setup(context):
    """Validate the arguments and build every action of the robot."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    mode = resolve_mode(arg('mode'))
    odometry = resolve_odometry(arg('odometry'))
    ns = resolve_namespace(arg('robot_ns'))
    site, warnings = load_site_config(arg('site_config'))
    robot = robot_bringup_action(arg('robot_bringup'), ns)

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

    nav = share('molly_navigation', 'config', 'navigation_params.yaml')
    perception = share('molly_perception', 'config', 'perception_params.yaml')
    bridge = share('molly_bridge', 'config', 'bridge_params.yaml')
    version = package_version(share('molly', 'package.xml'))
    git = os.environ.get('MOLLY_GIT', 'unknown')

    mediamtx, video_note = video_server(arg('webrtc'), arg('mediamtx'), [bridge, *site])
    teleop = {'robot_command_prefix': f'/{ns}/'}
    if mediamtx is not None:
        teleop['video_port'] = video.WEBRTC_PORT  # the handheld asks for WebRTC video here

    actions = [
        LogInfo(
            msg=f'Molly {version} ({git}) robot_ns={ns} '
            f'robot_bringup={arg("robot_bringup") or "(started elsewhere)"} '
            f'mode={mode} odometry={odometry} site_config={site[0] if site else "(none)"}'
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
            PythonLaunchDescriptionSource(share('molly_odometry', 'launch', 'odometry.launch.py')),
            launch_arguments={
                'method': odometry,
                'save_map': str(mode == 'teach'),
                'respawn': 'true',
                'record': 'true',
                'leg_odom_topic': topic('odom'),
                'site_config': site[0] if site else '',
            }.items(),
        )
    )
    actions += [
        node(
            'molly_localization',
            'localization_node',
            [share('molly_localization', 'config', 'localization_params.yaml')],
            {'tags_config_file': arg('tags')},
        ),
        node('molly_perception', 'lidar_sector_node', [perception]),
        # the tag detector sees the camera at rate_hz only: localization needs a few poses/s
        node(
            'molly_perception',
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
        node('molly_navigation', 'path_loader_node', [nav], {'waypoints_file': arg('route')}),
        node('molly_navigation', 'obstacle_avoidance_node', [nav]),
        node(
            'molly_navigation',
            'path_follower_node',
            [nav],
            remappings=[('cmd_vel', '/bridge/nav_cmd_vel'), ('battery', topic('battery'))],
        ),
        node(
            'molly_bridge',
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
    if truthy(arg('people')):
        actions.append(node('molly_perception', 'person_detector_node', [perception]))
    if truthy(arg('health')):
        actions.append(
            node(
                'molly_health',
                'health_monitor_node',
                [share('molly_health', 'config', 'health_params.yaml')],
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
                'mode', default_value=env('MODE', 'patrol'), description='patrol | teach'
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
                'tags', default_value=env('TAGS'), description='AprilTag map poses (JSON)'
            ),
            DeclareLaunchArgument(
                'site_config',
                default_value=env('SITE_CONFIG', '/etc/molly/site.yaml'),
                description='per-robot parameter overrides; missing file: none',
            ),
            DeclareLaunchArgument(
                'people', default_value=env('PEOPLE', 'true'), description='person detection'
            ),
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
