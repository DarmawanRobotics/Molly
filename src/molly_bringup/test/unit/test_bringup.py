"""Argument validation and parameter files (no ROS)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from molly_bringup import bringup

BringupError = bringup.BringupError

CONFIG = Path(__file__).resolve().parents[2] / 'config'


def write(path: Path, data) -> str:
    path.write_text(yaml.safe_dump(data))
    return str(path)


def test_truthy():
    assert all(bringup.truthy(v) for v in ('1', 'true', 'Yes', ' on '))
    assert not any(bringup.truthy(v) for v in ('', '0', 'false', 'off', 'maybe'))


def test_namespace():
    assert bringup.resolve_namespace('/robot/') == 'robot'
    assert bringup.resolve_namespace('fleet/r1') == 'fleet/r1'
    assert bringup.resolve_namespace('') == bringup.DEFAULT_ROBOT_NS
    for bad in ('1robot', 'ro bot', 'a//b', 'r-1'):
        with pytest.raises(BringupError, match='robot_ns'):
            bringup.resolve_namespace(bad)


def test_odometry_and_site():
    assert bringup.resolve_odometry('') == 'fast_lio'
    assert bringup.resolve_odometry(' Leg_Kilo ') == 'leg_kilo'
    with pytest.raises(BringupError, match='odometry'):
        bringup.resolve_odometry('slam')
    assert bringup.resolve_site(' lobby_2 ') == 'lobby_2'
    assert bringup.resolve_site('') == ''
    with pytest.raises(BringupError, match='site'):
        bringup.resolve_site('../etc')


def test_driver():
    assert bringup.resolve_driver('') is None
    assert bringup.resolve_driver('my_driver') == ('my_driver', 'bringup.launch.py')
    assert bringup.resolve_driver('my_driver/robot.launch.xml') == (
        'my_driver',
        'robot.launch.xml',
    )
    for bad in ('my driver', 'pkg/../x.launch.py', 'pkg/file.py', '/abs/path.launch.py'):
        with pytest.raises(BringupError, match='robot_driver'):
            bringup.resolve_driver(bad)


def test_existing_file(tmp_path):
    assert bringup.existing_file('params_file', '  ') == ''
    path = write(tmp_path / 'p.yaml', {})
    assert bringup.existing_file('params_file', path) == path
    with pytest.raises(BringupError, match='params_file'):
        bringup.existing_file('params_file', str(tmp_path / 'missing.yaml'))


def test_load_params_rejects_broken_files(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('a: [1, 2')
    with pytest.raises(BringupError):
        bringup.load_params(str(bad))
    with pytest.raises(BringupError, match='mapping'):
        bringup.load_params(write(tmp_path / 'list.yaml', [1, 2]))
    with pytest.raises(BringupError, match='ros__parameters'):
        bringup.load_params(write(tmp_path / 'flat.yaml', {'node': {'a': 1}}))


def test_shipped_robot_profile_is_valid():
    profile = bringup.robot_profile(str(CONFIG / 'robot.yaml'))
    assert profile.kinematics == 'omni'
    assert profile.frames.base == 'base_link'


def profile_file(tmp_path, robot: dict) -> str:
    return write(tmp_path / 'robot.yaml', {'/**': {'ros__parameters': {'robot': robot}}})


def test_robot_profile_errors(tmp_path):
    with pytest.raises(BringupError, match='no robot'):
        bringup.robot_profile(write(tmp_path / 'r.yaml', {'n': {'ros__parameters': {}}}))
    with pytest.raises(BringupError, match=r'footprint\.frnt'):
        bringup.robot_profile(profile_file(tmp_path, {'footprint': {'frnt': 0.3}}))
    with pytest.raises(BringupError, match='max_vx'):
        bringup.robot_profile(profile_file(tmp_path, {'limits': {'max_vx': -1.0}}))
    with pytest.raises(BringupError, match='max_vy'):
        bringup.robot_profile(profile_file(tmp_path, {'kinematics': 'diff'}))


def test_flatten_and_merge():
    nested = {'a': {'b': 1, 'c': {'d': 2}}, 'e': 3}
    assert bringup.flatten(nested) == {'a.b': 1, 'a.c.d': 2, 'e': 3}
    base = {'n': {'ros__parameters': {'a': 1, 'l': [1, 2], 's': {'x': 1, 'y': 2}}}}
    extra = {'n': {'ros__parameters': {'l': [3], 's': {'y': 5}}}, 'm': {'ros__parameters': {}}}
    merged = bringup.merge(base, extra)
    assert merged['n']['ros__parameters'] == {'a': 1, 'l': [3], 's': {'x': 1, 'y': 5}}
    assert 'm' in merged
    assert base['n']['ros__parameters']['l'] == [1, 2]  # inputs untouched


def test_merge_files_later_wins(tmp_path):
    first = write(tmp_path / 'a.yaml', {'n': {'ros__parameters': {'a': 1, 'b': 1}}})
    second = write(tmp_path / 'b.yaml', {'n': {'ros__parameters': {'b': 2}}})
    assert bringup.merge_files([first, '', second]) == {'n': {'ros__parameters': {'a': 1, 'b': 2}}}


def test_expected_nodes_keep_listed_ones():
    data = {'h': {'ros__parameters': {'nodes': {'expected': ['', '/driver']}, 'x': 1}}}
    out = bringup.with_expected_nodes(data, 'h', ['/b', '/a'])
    assert out['h']['ros__parameters'] == {'nodes': {'expected': ['/a', '/b', '/driver']}, 'x': 1}
    assert bringup.with_expected_nodes({}, 'h', ['/a'])['h']['ros__parameters'] == {
        'nodes': {'expected': ['/a']}
    }


def test_unknown_sections():
    data = {'/**': {}, 'planner_node': {}, '/planer_node': {}}
    assert bringup.unknown_sections(data, ['/planner_node']) == ['/planer_node']


def test_shared_parameters_besides_the_robot_profile():
    data = {
        '/**': {'ros__parameters': {'robot': {'name': 'r'}, 'sites_dir': '/s', 'a': {'b': 1}}},
        'planner_node': {'ros__parameters': {'x': 1}},
    }
    assert bringup.shared_parameters(data) == ['a.b', 'sites_dir']
    assert bringup.shared_parameters({'planner_node': {'ros__parameters': {}}}) == []


def test_write_params_round_trip(tmp_path):
    data = {'n': {'ros__parameters': {'a': [1.0, 2.0], 's': 'x'}}}
    path = bringup.write_params(data, str(tmp_path))
    assert Path(path).parent == tmp_path
    assert bringup.load_params(path) == data


def test_shipped_override_files_are_valid():
    for name in ('stack.yaml', 'sim.yaml'):
        assert bringup.load_params(str(CONFIG / name))


def test_route_loader_gets_the_sites_dir():
    data = {'path_loader_node': {'ros__parameters': {'route': 'loop.csv'}}}
    out = bringup.with_sites_dir(data, 'path_loader_node', '/sites')
    assert out['path_loader_node']['ros__parameters'] == {
        'route': 'loop.csv',
        'sites_dir': '/sites',
    }
    assert bringup.with_sites_dir(data, 'path_loader_node', '') is data


def test_route_and_sites_dir_for_several_nodes():
    data = bringup.with_sites_dir({}, ('a_node', 'b_node'), '/sites')
    assert data['b_node']['ros__parameters']['sites_dir'] == '/sites'
    data = bringup.with_route(data, 'a_node', 'hall/routes/x.csv')
    assert data['a_node']['ros__parameters'] == {
        'sites_dir': '/sites',
        'route': 'hall/routes/x.csv',
    }
    assert bringup.with_route(data, 'a_node', '') is data


def test_demo_site_is_seeded_once(tmp_path):
    source = tmp_path / 'demo'
    source.mkdir()
    (source / 'pois.json').write_text('{"pois": []}')
    assert bringup.seed_site(str(tmp_path / 'sites'), 'hall', str(source)) == ['pois.json']
    assert bringup.seed_site(str(tmp_path / 'sites'), 'hall', str(source)) == []
    assert bringup.seed_site(str(tmp_path / 'sites'), '', str(source)) == []


def test_package_version(tmp_path):
    (tmp_path / 'package.xml').write_text('<package><version>2.0.0</version></package>')
    assert bringup.package_version(str(tmp_path / 'package.xml')) == '2.0.0'
    assert bringup.package_version(str(tmp_path / 'missing.xml')) == 'unknown'
