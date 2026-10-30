"""The launch files: arguments, composition into scoped includes, merged parameters."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess

from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription, LogInfo
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest
import yaml

from molly_bringup import bringup, launch_utils, stack

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = {'params_file', 'robot_params', 'use_sim_time', 'log_level'}
STUB = """from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
"""


def load_launch(name: str):
    """Import launch/<name> as a module."""
    module_name = name.split('.', maxsplit=1)[0]
    spec = importlib.util.spec_from_file_location(module_name, ROOT / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def packages(tmp_path, monkeypatch):
    """Serve molly_bringup from the source tree and the other packages from stub shares."""

    def share(package, *path):
        if package == launch_utils.PACKAGE:
            return str(ROOT.joinpath(*path))
        root = tmp_path / 'share' / package
        (root / 'launch').mkdir(parents=True, exist_ok=True)
        target = root.joinpath(*path)
        if package.startswith(('dharma_', 'molly_')) and target.suffix == '.py':
            target.write_text(STUB)
        return str(target)

    monkeypatch.setattr(launch_utils, 'share', share)
    return tmp_path


def context(**arguments) -> LaunchContext:
    """Return a launch context with the declared defaults of robot.launch.py and arguments."""
    ctx = LaunchContext()
    defaults = {n: default for n, (_, default, _) in launch_utils.ARGUMENTS.items()}
    ctx.launch_configurations.update({**defaults, **arguments})
    return ctx


def includes(actions) -> list:
    """Return the package launch files the actions include, in order."""
    out = []
    for group in (a for a in actions if isinstance(a, GroupAction)):
        for sub in group.get_sub_entities():
            if isinstance(sub, IncludeLaunchDescription):
                source = sub.launch_description_source
                source.get_launch_description(LaunchContext())
                out.append(Path(source.location).parent.parent.name)
    return out


def visible_arguments(group: GroupAction, caller: dict) -> set:
    """Return the caller's launch configurations the include of a group still sees."""
    ctx = LaunchContext()
    ctx.launch_configurations.update(caller)
    for sub in group.get_sub_entities():
        if isinstance(sub, IncludeLaunchDescription):
            return set(caller) & set(ctx.launch_configurations)
        if 'LaunchConfiguration' in type(sub).__name__:  # push, reset, unset (not environment)
            sub.visit(ctx)
    raise AssertionError('no include')


def text(value) -> str:
    """Return the text of a launch argument name or value."""
    return perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(value))


def merged_parameters(actions) -> dict:
    """Return the merged parameter file passed to the package launches."""
    for group in (a for a in actions if isinstance(a, GroupAction)):
        for sub in group.get_sub_entities():
            if isinstance(sub, IncludeLaunchDescription):
                args = {text(k): text(v) for k, v in sub.launch_arguments}
                return bringup.load_params(args['params_file'])
    raise AssertionError('no include')


def test_arguments_of_both_launch_files():
    robot = {a.name for a in load_launch('robot.launch.py').generate_launch_description()
             .get_launch_arguments()}  # fmt: skip
    sim = {a.name for a in load_launch('sim.launch.py').generate_launch_description()
           .get_launch_arguments()}  # fmt: skip
    assert CONTRACT | {'robot_ns', 'robot_driver', 'odometry', 'site', 'sites_dir'} <= robot
    assert CONTRACT | {'world', 'scenario', 'rviz', 'site'} <= sim
    assert not {'robot_driver', 'odometry'} & sim


def test_robot_launch_composes_the_stack(packages):
    actions = load_launch('robot.launch.py').launch_setup(context())
    assert includes(actions) == [stack.COMPONENTS[c][0] for c in stack.COMPONENTS]
    assert isinstance(actions[0], LogInfo)
    caller = {**context().launch_configurations, 'world': 'hall'}
    for group in (a for a in actions if isinstance(a, GroupAction)):
        # Only the explicit arguments reach a package launch (Foxy too: no forwarding=False).
        assert visible_arguments(group, caller) == set()


def test_merged_parameters_name_the_started_nodes(packages):
    actions = load_launch('robot.launch.py').launch_setup(context())
    data = merged_parameters(actions)
    params = data[stack.HEALTH_NODE]['ros__parameters']
    assert '/control_mux_node' in params['nodes']['expected']
    assert '' not in params['nodes']['expected']
    assert data['control_mux_node']['ros__parameters']['use_health_limit'] is True


def test_params_file_overrides_the_stack_file(packages, tmp_path):
    extra = tmp_path / 'site.yaml'
    extra.write_text(yaml.safe_dump({'control_mux_node': {'ros__parameters': {'mode': 'remote'}}}))
    actions = load_launch('robot.launch.py').launch_setup(context(params_file=str(extra)))
    control = merged_parameters(actions)['control_mux_node']['ros__parameters']
    assert control['mode'] == 'remote'
    assert control['use_health_limit'] is True


def test_shared_overrides_are_flagged(packages, tmp_path):
    extra = tmp_path / 'site.yaml'
    extra.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'sites_dir': '/data/sites'}}}))
    actions = load_launch('robot.launch.py').launch_setup(context(params_file=str(extra)))
    notes = [text(a.msg) for a in actions if isinstance(a, LogInfo)]
    assert any('sites_dir' in n and 'Foxy' in n for n in notes), notes


def test_invalid_arguments_stop_the_launch(packages):
    launch = load_launch('robot.launch.py')
    with pytest.raises(bringup.BringupError, match='start_mapping'):
        launch.launch_setup(context(start_mapping='true'))
    with pytest.raises(bringup.BringupError, match='odometry'):
        launch.launch_setup(context(odometry='gmapping'))
    with pytest.raises(bringup.BringupError, match='params_file'):
        launch.launch_setup(context(params_file='/nonexistent.yaml'))


def test_driver_include(packages):
    launch = load_launch('robot.launch.py')
    with pytest.raises(bringup.BringupError, match='does not exist'):
        launch.launch_setup(context(robot_driver='my_driver'))
    driver = packages / 'share' / 'my_driver' / 'launch' / 'bringup.launch.py'
    driver.parent.mkdir(parents=True, exist_ok=True)
    driver.write_text(STUB)
    actions = launch.launch_setup(context(robot_driver='my_driver'))
    assert includes(actions)[0] == 'my_driver'


def test_sim_launch_adds_the_simulator(packages):
    launch = load_launch('sim.launch.py')
    defaults = {**launch.SIM_DEFAULTS, 'world': '', 'scenario': '', 'rviz': 'false'}
    actions = launch.launch_setup(context(**defaults))
    assert includes(actions)[0] == 'dharma_sim'
    data = merged_parameters(actions)
    assert data['lidar_sector_node']['ros__parameters']['input']['type'] == 'pointcloud2'
    expected = data[stack.HEALTH_NODE]['ros__parameters']['nodes']['expected']
    assert '/sim_robot_node' in expected
    assert '/fastlio_mapping_node' not in expected


@pytest.mark.skipif(shutil.which('ros2') is None, reason='ros2 CLI not available')
def test_show_args_smoke():
    """ros2 launch --show-args, when the package is installed (colcon test)."""
    try:
        launch_utils.get_package_share_directory(launch_utils.PACKAGE)
    except launch_utils.PackageNotFoundError:
        pytest.skip('molly_bringup is not installed')
    for name in ('robot.launch.py', 'sim.launch.py'):
        result = subprocess.run(
            ['ros2', 'launch', launch_utils.PACKAGE, name, '--show-args'],
            capture_output=True,
            text=True,
            timeout=60,
            env={**os.environ, 'ROS_LOG_DIR': '/tmp'},
            check=False,
        )
        assert result.returncode == 0, result.stderr
        assert 'robot_params' in result.stdout
