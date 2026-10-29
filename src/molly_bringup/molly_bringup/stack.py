"""
The composition of Molly: the Dharma stack and Molly's packages, with their arguments.

This is the one place to adjust when a package's launch contract changes. Every package launch
accepts params_file, robot_params, use_sim_time and log_level (Dharma DESIGN section 7); only
non-empty arguments are passed, so an unset one never hides a value from a parameter file.
"""

from __future__ import annotations

from dataclasses import dataclass, field

HEALTH_NODE = 'health_monitor_node'
ROUTE_NODE = 'path_loader_node'  # loads taught routes from sites_dir
SITES_NODES = ('path_loader_node', 'site_content_node', 'teleop_udp_node')


@dataclass(frozen=True)
class Include:
    """One package launch file with its arguments and the remappings of its nodes."""

    package: str
    launch_file: str
    arguments: dict = field(default_factory=dict)
    remappings: tuple = ()


@dataclass(frozen=True)
class Options:
    """The resolved bringup arguments."""

    robot_params: str
    params_file: str
    robot_ns: str = 'robot'
    odometry: str = 'fast_lio'
    use_sim_time: str = ''
    log_level: str = 'info'
    site: str = ''
    sites_dir: str = ''
    route: str = ''
    start_mapping: bool = False
    fiducials: bool = True
    fiducials_detector: str = 'apriltag'  # apriltag | aruco
    camera_image: str = ''
    camera_info: str = ''
    health: bool = True
    speech: bool = True
    greeter: bool = True
    visitors: bool = True
    conversation: bool = False
    web: bool = True
    web_port: str = '9090'
    web_video_port: str = '8080'
    webrtc: bool = True
    mediamtx: str = ''
    web_nodes: tuple = ()  # the web gateways that are installed (they start only then)


# component -> (package, launch file, nodes it always starts, by node name)
COMPONENTS = {
    'odometry': ('dharma_odometry', 'odometry.launch.py', ('odometry_monitor_node',)),
    'localization': ('dharma_localization', 'localization.launch.py', ('localization_node',)),
    'perception': ('dharma_perception', 'perception.launch.py', ('lidar_sector_node',)),
    'mapping': (
        'dharma_mapping',
        'mapping.launch.py',
        ('grid_mapper_node', 'site_manager_node', 'route_recorder_node'),
    ),
    'navigation': (
        'dharma_navigation',
        'navigation.launch.py',
        (
            'planner_node',
            'path_loader_node',
            'path_follower_node',
            'obstacle_avoidance_node',
            'docking_node',
        ),
    ),
    'control': ('dharma_control', 'control.launch.py', ('control_mux_node',)),
    'health': ('dharma_health', 'health.launch.py', (HEALTH_NODE,)),
    'guide': ('molly_guide', 'guide.launch.py', ('guide_node', 'site_content_node')),
    'interaction': ('molly_interaction', 'interaction.launch.py', ('interaction_node',)),
    'bridge': ('molly_bridge', 'bridge.launch.py', ('teleop_udp_node',)),
    'web': ('molly_bridge', 'web.launch.py', ()),
}
# interaction nodes started by an option
INTERACTION_NODES = {
    'speech': 'speech_node',
    'visitors': 'visitor_tracker_node',
    'conversation': 'conversation_node',
}
WEB_NODES = ('rosbridge_websocket', 'rosapi_node', 'web_video_server')
ESTIMATOR_NODES = {'fast_lio': ('fastlio_mapping_node',)}  # leg_kilo names its own node
FIDUCIAL_NODES = ('apriltag_throttle', 'apriltag')
DETECTOR_NODES = {'apriltag': 'apriltag', 'aruco': 'aruco_detector_node'}
SIM = ('dharma_sim', 'sim.launch.py', ('sim_robot_node',))
PARAMETER_ONLY = ('odometry_gate', 'video_stream_node')  # wait_still; started by MediaMTX


def non_empty(arguments: dict) -> dict:
    """Return the arguments that are set (an empty one must never override a file)."""
    return {k: str(v) for k, v in arguments.items() if v is not None and str(v).strip()}


def flag(value: bool) -> str:
    """Return a launch bool."""
    return 'true' if value else 'false'


def common(o: Options) -> dict:
    """Return the arguments every package launch accepts."""
    return {
        'params_file': o.params_file,
        'robot_params': o.robot_params,
        'use_sim_time': o.use_sim_time,
        'log_level': o.log_level,
    }


def component_arguments(o: Options) -> dict:
    """Return the launch arguments of every component (before dropping empty ones)."""
    driver = f'/{o.robot_ns}'
    return {
        'odometry': {
            'method': o.odometry,
            'save_map': flag(o.start_mapping),
            'leg_odom_topic': f'{driver}/odom',
            'site_config': o.params_file,  # the estimator reads it too (FAST-LIO overrides)
        },
        'localization': {'sites_dir': o.sites_dir},
        'perception': {
            'fiducials': flag(o.fiducials),
            'fiducials_detector': o.fiducials_detector,
            'camera_image': o.camera_image,
            'camera_info': o.camera_info,
        },
        'mapping': {
            'sites_dir': o.sites_dir,
            'site': o.site,
            'start_mapping': 'true' if o.start_mapping else '',
        },
        'navigation': {'battery_topic': f'{driver}/battery', 'odom_topic': f'{driver}/odom'},
        'control': {'cmd_vel_topic': f'{driver}/cmd_vel'},  # the mux drives the robot
        'health': {},
        'guide': {'sites_dir': o.sites_dir},
        'interaction': {
            'robot_ns': o.robot_ns,
            'speech': flag(o.speech),
            'greeter': flag(o.greeter),
            'visitors': flag(o.visitors),
            'conversation': flag(o.conversation),
        },
        'bridge': {
            'robot_ns': o.robot_ns,
            'sites_dir': o.sites_dir,
            'webrtc': flag(o.webrtc),
            'mediamtx': o.mediamtx,
        },
        'web': {
            'robot_ns': o.robot_ns,
            'web_port': o.web_port,
            'web_video_port': o.web_video_port,
        },
    }


def components(o: Options) -> list[str]:
    """Return the components to start, in launch order."""
    off = {'health': o.health, 'web': o.web}
    return [name for name in COMPONENTS if off.get(name, True)]


def compose(o: Options) -> list[Include]:
    """Return the package launch files of the stack with their arguments."""
    arguments = component_arguments(o)
    out = []
    for name in components(o):
        package, launch_file, _ = COMPONENTS[name]
        contract = {'use_sim_time': o.use_sim_time} if name == 'web' else common(o)
        args = non_empty({**contract, **arguments[name]})
        out.append(Include(package, launch_file, args))
    return out


def sim_include(o: Options, world: str = '', scenario: str = '', rviz: bool = False) -> Include:
    """Return the simulated robot (driver, LiDAR, IMU and odometry in one node)."""
    package, launch_file, _ = SIM
    args = {
        **common(o),
        'world': world,
        'scenario': scenario,
        'robot_ns': o.robot_ns,
        'site': o.site,
        'sites_dir': o.sites_dir,
        'rviz': flag(rviz),
    }
    return Include(package, launch_file, non_empty(args))


def node_names(o: Options, sim: bool = False) -> list[str]:
    """Return the fully qualified names of every node the bringup starts."""
    started = components(o)
    names = [n for c in started for n in COMPONENTS[c][2]]
    names += [n for option, n in INTERACTION_NODES.items() if getattr(o, option)]
    if 'web' in started:
        names += o.web_nodes
    names += ESTIMATOR_NODES.get(o.odometry, ())
    if o.fiducials:
        names += (FIDUCIAL_NODES[0], DETECTOR_NODES[o.fiducials_detector])
    if sim:
        names += SIM[2]
    return sorted(f'/{n}' for n in names)


def known_sections() -> list[str]:
    """Return every node name a parameter file of the bringup may configure."""
    names = [n for c in COMPONENTS.values() for n in c[2]]
    names += [n for nodes in ESTIMATOR_NODES.values() for n in nodes]
    names += [*INTERACTION_NODES.values(), *WEB_NODES]
    return sorted({*names, *FIDUCIAL_NODES, *DETECTOR_NODES.values(), *SIM[2], *PARAMETER_ONLY})
