"""The stack composition: package launches, arguments, node names (no ROS)."""

from __future__ import annotations

from pathlib import Path

from molly_bringup import bringup, stack

CONFIG = Path(__file__).resolve().parents[2] / 'config'
CONTRACT = {'params_file', 'robot_params', 'use_sim_time', 'log_level'}


def options(**kwargs) -> stack.Options:
    return stack.Options(robot_params='/r.yaml', params_file='/p.yaml', **kwargs)


def by_package(includes) -> dict:
    return {i.package: i for i in includes}


def test_every_component_in_order():
    packages = [i.package for i in stack.compose(options())]
    assert packages == [
        'dharma_odometry',
        'dharma_localization',
        'dharma_perception',
        'dharma_mapping',
        'dharma_navigation',
        'dharma_control',
        'dharma_health',
        'molly_guide',
        'molly_interaction',
        'molly_bridge',
        'molly_bridge',
    ]
    assert 'dharma_health' not in by_package(stack.compose(options(health=False)))
    files = [i.launch_file for i in stack.compose(options(web=False))]
    assert 'web.launch.py' not in files and 'bridge.launch.py' in files


def test_contract_arguments_reach_every_launch():
    o = options(use_sim_time='true', log_level='debug', web=False)
    for include in stack.compose(o):
        assert include.launch_file.endswith('.launch.py')
        assert include.arguments['params_file'] == '/p.yaml'
        assert include.arguments['robot_params'] == '/r.yaml'
        assert include.arguments['use_sim_time'] == 'true'
        assert include.arguments['log_level'] == 'debug'


def test_empty_arguments_are_never_passed():
    for include in stack.compose(options()):
        assert 'use_sim_time' not in include.arguments
        assert all(str(v).strip() for v in include.arguments.values())
    mapping = by_package(stack.compose(options()))['dharma_mapping'].arguments
    assert 'site' not in mapping
    assert 'sites_dir' not in mapping
    assert 'start_mapping' not in mapping  # the parameter files decide


def test_robot_namespace_topics():
    includes = by_package(stack.compose(options(robot_ns='lite3')))
    assert includes['dharma_odometry'].arguments['leg_odom_topic'] == '/lite3/odom'
    nav = includes['dharma_navigation'].arguments
    assert (nav['battery_topic'], nav['odom_topic']) == ('/lite3/battery', '/lite3/odom')
    # the control mux is the only publisher of the driver's cmd_vel
    assert includes['dharma_control'].arguments['cmd_vel_topic'] == '/lite3/cmd_vel'
    assert all(not i.remappings for i in includes.values())


def test_mapping_site_and_odometry():
    o = options(odometry='leg_kilo', site='lobby', sites_dir='/s', start_mapping=True)
    includes = by_package(stack.compose(o))
    odometry = includes['dharma_odometry'].arguments
    assert (odometry['method'], odometry['save_map']) == ('leg_kilo', 'true')
    assert odometry['site_config'] == '/p.yaml'  # FAST-LIO overrides from the stack file
    mapping = includes['dharma_mapping'].arguments
    assert (mapping['sites_dir'], mapping['site'], mapping['start_mapping']) == (
        '/s',
        'lobby',
        'true',
    )
    assert includes['dharma_localization'].arguments['sites_dir'] == '/s'


def test_perception_fiducials():
    perception = by_package(stack.compose(options(fiducials=False, camera_image='/c/img')))
    args = perception['dharma_perception'].arguments
    assert args['fiducials'] == 'false'
    assert args['camera_image'] == '/c/img'
    assert 'camera_info' not in args


def test_perception_aruco_detector():
    o = options(fiducials_detector='aruco')
    perception = by_package(stack.compose(o))['dharma_perception'].arguments
    assert perception['fiducials_detector'] == 'aruco'
    names = stack.node_names(o)
    assert '/aruco_detector_node' in names and '/apriltag' not in names
    assert '/docking_node' in names
    assert 'aruco_detector_node' in stack.known_sections()


def test_sim_include():
    o = options(robot_ns='robot', site='mall_loop', sites_dir='/tmp/s', odometry='external')
    sim = stack.sim_include(o, world='', scenario='drift', rviz=True)
    assert (sim.package, sim.launch_file) == ('dharma_sim', 'sim.launch.py')
    assert CONTRACT - {'use_sim_time'} <= set(sim.arguments)
    assert sim.arguments['scenario'] == 'drift'
    assert sim.arguments['rviz'] == 'true'
    assert 'world' not in sim.arguments


def test_node_names():
    names = stack.node_names(options())
    assert '/health_monitor_node' in names
    assert '/fastlio_mapping_node' in names
    assert {'/apriltag', '/apriltag_throttle'} <= set(names)
    assert names == sorted(names)
    external = stack.node_names(options(odometry='external', fiducials=False), sim=True)
    assert '/fastlio_mapping_node' not in external
    assert '/apriltag' not in external
    assert '/sim_robot_node' in external
    assert '/odometry_gate' not in external  # it exits once the IMU is still


def test_shipped_files_only_configure_known_nodes():
    known = stack.known_sections()
    for name in ('stack.yaml', 'sim.yaml', 'site.example.yaml'):
        data = bringup.load_params(str(CONFIG / name))
        assert bringup.unknown_sections(data, known) == [], name


def test_molly_components():
    o = options(robot_ns='lite3', sites_dir='/s', speech=False, conversation=True, webrtc=False)
    includes = {i.launch_file: i.arguments for i in stack.compose(o)}
    assert includes['guide.launch.py']['sites_dir'] == '/s'
    interaction = includes['interaction.launch.py']
    assert (interaction['robot_ns'], interaction['speech'], interaction['conversation']) == (
        'lite3',
        'false',
        'true',
    )
    bridge = includes['bridge.launch.py']
    assert (bridge['robot_ns'], bridge['sites_dir'], bridge['webrtc']) == ('lite3', '/s', 'false')
    assert includes['web.launch.py'] == {'robot_ns': 'lite3', 'web_port': '9090',
                                         'web_video_port': '8080'}  # fmt: skip


def test_molly_node_names():
    names = stack.node_names(options(speech=False, web_nodes=('web_video_server',)))
    assert {'/guide_node', '/site_content_node', '/teleop_udp_node', '/interaction_node'} <= set(
        names
    )
    assert '/visitor_tracker_node' in names and '/speech_node' not in names
    assert '/conversation_node' not in names and '/web_video_server' in names
    assert '/rosbridge_websocket' not in names
    assert '/web_video_server' not in stack.node_names(options(web=False, web_nodes=('x',)))
