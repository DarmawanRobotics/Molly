"""Tests of the bringup helpers and of the launch description of robot.launch.py."""

import os
import shlex

from molly.bringup import (
    BringupError,
    expected_nodes,
    load_site_config,
    non_empty,
    package_version,
    resolve_mode,
    resolve_namespace,
    resolve_odometry,
    resolve_robot_bringup,
    resolve_site,
    site_manager_overrides,
    web_globs,
)
from molly.video import password_hash
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)


def test_modes_and_odometry():
    """Guide and mapping are the modes; Pashupati's patrol and teach map onto them."""
    assert resolve_mode('guide') == 'guide'
    assert resolve_mode(' Mapping ') == 'mapping'
    assert resolve_mode('patrol') == 'guide'
    assert resolve_mode('teach') == 'mapping'
    assert resolve_odometry('leg_kilo') == 'leg_kilo'
    for bad in ('', 'auto', 'map'):
        with pytest.raises(BringupError):
            resolve_mode(bad)
    with pytest.raises(BringupError):
        resolve_odometry('slam')


def test_site_names():
    """Site names are the directory names of map/: lower case, digits, _ and -."""
    assert resolve_site('') == ''
    assert resolve_site(' lobby_1 ') == 'lobby_1'
    assert resolve_site('museum-2f') == 'museum-2f'
    for bad in ('Lobby', '-x', 'a/b', '../x', 'a b', 'x' * 49):
        with pytest.raises(BringupError):
            resolve_site(bad)


def test_web_whitelist():
    """The rosbridge whitelist reaches the dashboard names and robot state, not cmd_vel."""
    globs = web_globs('dog')
    topics = globs['topics_glob']
    assert topics.startswith('[') and topics.endswith(']')
    assert "'/map'" in topics and "'/tour/state'" in topics and "'/dog/battery'" in topics
    assert "'/bridge/remote_cmd_vel'" in topics
    assert 'cmd_vel' not in topics.replace('/bridge/remote_cmd_vel', '')
    assert "'/dog/stand_up'" in globs['services_glob']
    assert "'/site/*'" in globs['services_glob']
    assert globs['params_glob'] == "['/path_follower_node/*']"


def test_namespace():
    """Driver namespaces are plain ROS names, slashes around them are dropped."""
    assert resolve_namespace('/robot/') == 'robot'
    assert resolve_namespace('fleet/dog_2') == 'fleet/dog_2'
    for bad in ('', '1robot', 'dog-2', 'a//b'):
        with pytest.raises(BringupError):
            resolve_namespace(bad)


def test_robot_bringup():
    """Any robot package: '<pkg>' (bringup.launch.py) or '<pkg>/<file>'; empty means elsewhere."""
    assert resolve_robot_bringup('') is None
    assert resolve_robot_bringup('  ') is None
    assert resolve_robot_bringup('go2_bringup') == ('go2_bringup', 'bringup.launch.py')
    assert resolve_robot_bringup('lite3_bringup/robot.launch.py') == (
        'lite3_bringup',
        'robot.launch.py',
    )
    assert resolve_robot_bringup('dog/sensors.launch.xml') == ('dog', 'sensors.launch.xml')
    for bad in ('2dog', 'dog/', 'dog/../x.launch.py', 'dog/bringup.py', 'a b'):
        with pytest.raises(BringupError):
            resolve_robot_bringup(bad)


def test_site_config(tmp_path):
    """Missing file: nothing; valid file: used, unknown nodes warned; broken file: error."""
    assert load_site_config('') == ([], [])
    assert load_site_config(str(tmp_path / 'missing.yaml')) == ([], [])
    good = tmp_path / 'site.yaml'
    good.write_text(
        'path_follower_node:\n  ros__parameters:\n    target_linear_velocity: 0.3\n'
        'path_folower_node:\n  ros__parameters:\n    x: 1\n'
    )
    files, warnings = load_site_config(str(good))
    assert files == [str(good)]
    assert len(warnings) == 1 and 'path_folower_node' in warnings[0]
    for text in ('- a\n- b\n', 'node:\n  x: 1\n', 'node: [\n'):
        bad = tmp_path / 'bad.yaml'
        bad.write_text(text)
        with pytest.raises(BringupError):
            load_site_config(str(bad))


def test_example_site_config_is_valid():
    """The shipped example passes the same checks without warnings."""
    files, warnings = load_site_config(os.path.join(PACKAGE, 'config', 'site.example.yaml'))
    assert files and warnings == []


def test_non_empty_and_version():
    """Unset launch arguments never override the site file; version comes from package.xml."""
    assert non_empty({'a': '', 'b': 'x', 'c': None}) == {'b': 'x'}
    assert package_version(os.path.join(PACKAGE, 'package.xml')) != 'unknown'
    assert package_version('/nonexistent/package.xml') == 'unknown'


def load_launch_module():
    """Import launch/robot.launch.py as a module."""
    import importlib.util

    path = os.path.join(PACKAGE, 'launch', 'robot.launch.py')
    spec = importlib.util.spec_from_file_location('robot_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_context(module, **overrides):
    """Return a LaunchContext with the declared defaults plus overrides."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value
            )
    context.launch_configurations['webrtc'] = 'false'  # a MediaMTX here keeps its configuration
    context.launch_configurations.update(overrides)
    return context


def test_launch_starts_every_node_once():
    """robot.launch.py builds every Molly node (no robot package: no robot needed)."""
    pytest.importorskip('launch_ros')
    from launch_ros.actions import Node
    from launch.utilities import perform_substitutions

    module = load_launch_module()
    context = launch_context(
        module, robot_bringup='', mode='guide', site_config='', people='true', web='false'
    )
    actions = module.launch_setup(context)

    def text(value):  # a str (Jazzy) or a list of substitutions (Humble)
        return value if isinstance(value, str) else perform_substitutions(context, value)

    def executables(actions):
        return sorted(text(a.node_executable) for a in actions if isinstance(a, Node))

    local = sorted([
        'localization_node', 'grid_mapper_node', 'site_manager_node', 'lidar_sector_node',
        'image_throttle_node', 'apriltag_node', 'visitor_tracker_node', 'person_detector_node',
        'path_loader_node', 'obstacle_avoidance_node', 'path_follower_node', 'planner_node',
        'guide_node', 'interaction_node', 'speech_node', 'control_mux_node', 'teleop_udp_node',
        'health_monitor_node',
    ])
    assert executables(actions) == local
    context = launch_context(
        module, robot_bringup='', site_config='', visitors='false', speech='false',
        conversation='true', web='false',
    )
    names = executables(module.launch_setup(context))
    assert 'conversation_node' in names
    assert not {'visitor_tracker_node', 'speech_node', 'person_detector_node'} & set(names)


def test_mapping_mode_maps_the_named_site():
    """mode:=mapping needs a site name and tells the site manager to map it at start."""
    assert site_manager_overrides('mapping', 'lobby', '/data/map') == {
        'sites_dir': '/data/map', 'site': 'lobby', 'start_mapping': True,
    }
    assert site_manager_overrides('guide', '', '')['start_mapping'] is False
    pytest.importorskip('launch_ros')
    module = load_launch_module()
    with pytest.raises(BringupError):
        module.launch_setup(launch_context(module, robot_bringup='', mode='mapping', site=''))
    context = launch_context(
        module, robot_bringup='', mode='mapping', site='lobby', site_config='', web='false'
    )
    assert module.launch_setup(context)


def test_launch_rejects_bad_arguments():
    """A wrong mode, namespace or robot package stops the launch with a clear error."""
    pytest.importorskip('launch_ros')
    module = load_launch_module()
    for overrides in (
        {'mode': 'auto'},
        {'site': 'Bad Site'},
        {'odometry': 'gmapping'},
        {'robot_ns': 'dog-2'},
        {'robot_bringup': 'no_such_robot_package'},
        {'robot_bringup': 'molly/missing.launch.py'},
    ):
        overrides = {'robot_bringup': '', **overrides}
        with pytest.raises(BringupError):
            module.launch_setup(launch_context(module, **overrides))


def test_launch_includes_the_robot_package():
    """robot_bringup includes that package's launch file with namespace:=robot_ns."""
    pytest.importorskip('launch_ros')
    from launch.actions import IncludeLaunchDescription

    module = load_launch_module()
    # any installed launch file stands in for a robot package here
    context = launch_context(
        module, robot_bringup='molly/robot.launch.py', robot_ns='dog', site_config=''
    )
    robot = [
        a for a in module.launch_setup(context)
        if isinstance(a, IncludeLaunchDescription)
        and dict(a.launch_arguments).get('namespace') == 'dog'
    ]
    assert len(robot) == 1
    source = robot[0].launch_description_source
    source.get_launch_description(context)  # resolves the path
    assert source.location.endswith(os.path.join('molly', 'launch', 'robot.launch.py'))


def test_health_watches_every_started_node():
    """The health monitor expects every launched node plus the odometry ones."""
    names = expected_nodes(['path_follower_node', 'control_mux_node'], 'fast_lio')
    assert names == sorted(names) and len(names) == len(set(names))
    assert {'path_follower_node', 'fastlio_mapping_node', 'health_monitor_node'} <= set(names)
    assert 'legkilo' in expected_nodes([], 'leg_kilo')
    external = expected_nodes([], 'external')  # the simulator brings its own odometry
    assert 'odometry_monitor_node' in external
    assert not {'fastlio_mapping_node', 'legkilo'} & set(external)


def test_launch_runs_mediamtx_for_the_webrtc_video(tmp_path, monkeypatch):
    """With MediaMTX the launch starts it, reading the token from the site file; else JPEG."""
    pytest.importorskip('launch_ros')
    from launch.actions import ExecuteProcess

    monkeypatch.setenv('HOME', str(tmp_path))  # the configuration goes to ~/.molly
    monkeypatch.setattr('molly.video.gstreamer_missing', lambda: None)
    binary = tmp_path / 'mediamtx'
    binary.write_text('#!/bin/sh\necho v1.20.1\n')
    binary.chmod(0o755)
    site = tmp_path / 'site.yaml'
    site.write_text('teleop_udp_node:\n  ros__parameters:\n    token: "s3cret"\n')
    module = load_launch_module()

    def servers(webrtc):
        context = launch_context(
            module, robot_bringup='', site_config=str(site), mediamtx=str(binary), webrtc=webrtc
        )
        return [a for a in module.launch_setup(context) if type(a) is ExecuteProcess]

    assert len(servers('true')) == 1
    with open(tmp_path / '.molly' / 'mediamtx.yml') as f:
        config = yaml.safe_load(f)
    assert config['authInternalUsers'][1]['pass'] == password_hash('s3cret')
    command = shlex.split(config['paths']['camera']['runOnDemand'])
    assert command[0].endswith(os.path.join('molly_bridge', 'video_stream_node'))
    assert str(site) in command  # the encoder gets the same site parameters
    assert servers('false') == []
    binary.write_text('#!/bin/sh\necho v1.9.0\n')
    assert servers('true') == []  # too old for the configuration
