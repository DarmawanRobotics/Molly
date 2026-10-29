"""The state message from typed status, the session, routes and the web whitelist (no ROS)."""

from __future__ import annotations

import os
import types

from molly_bridge.core import args, routes, summaries, web
from molly_bridge.core.session import Session
import pytest

NS = types.SimpleNamespace


def test_control_and_navigation_words():
    control = summaries.control_summary(
        NS(mode=1, estop=True, estop_reasons=['handheld'], field=4)
    )
    assert control == {
        'mode': 'remote',
        'estop': True,
        'estop_reasons': ['handheld'],
        'field': 'stale',
    }
    nav = summaries.navigation_summary(NS(state=5, message='held', pause_reasons=['remote']))
    assert nav == {'nav': 'PAUSED', 'nav_msg': 'held', 'pause': ['remote']}
    assert summaries.word(summaries.FIELDS, 42, 'off') == 'off'


def test_localization_text():
    assert summaries.localization_text(NS(state=0, source='tag:base_map')) == 'none'
    assert summaries.localization_text(NS(state=1, source='tag:base_map')) == 'tag:base_map'
    assert summaries.localization_text(NS(state=2, source='')) == 'uncertain'


def test_system_load_from_the_health_diagnostics():
    statuses = [
        ('health: system:cpu', {'value': '37.6', 'limit': '95'}),
        ('health: system:memory', {'value': '41.2', 'used_gb': '6.43', 'total_gb': '15.3'}),
        ('health: system:temperature', {'value': 'nan'}),
        ('health: sensor:/livox/lidar', {'hz': '10'}),
    ]
    load = summaries.system_load(statuses)
    assert load == {'cpu': 38, 'gpu': None, 'ram': 41, 'ram_used': 6.4, 'ram_total': 15.3,
                    'temp': None}  # fmt: skip
    health = summaries.health_summary(NS(level=1, issues=['a', 'b'], speed_scale=0.5), load)
    assert health['ok'] is False and health['speed_scale'] == 0.5 and health['cpu'] == 38


def test_planner_goal_named_by_its_poi():
    pois = [NS(id='lobby', x=1.0, y=2.0), NS(id='hall', x=4.0, y=0.0)]
    msg = NS(state=4, goal_x=4.0, goal_y=0.01, remaining=0.0, message='arrived')
    assert summaries.planner_summary(msg, pois) == {
        'state': 'arrived',
        'goal': 'hall',
        'remaining': 0.0,
        'msg': 'arrived',
    }
    assert summaries.planner_summary(NS(state=2, goal_x=9.0, goal_y=9.0, remaining=3.21,
                                        message=''), pois)['goal'] == ''  # fmt: skip


def test_session_one_driver_and_acks():
    session = Session('secret', 2.0)
    assert not session.authorized({'k': 'wrong'}) and session.authorized({'k': 'secret'})
    assert session.accept(('10.0.0.2', 5000), 'phone', 0.0)
    assert session.busy_for(('10.0.0.3', 5000), 1.0)
    assert not session.busy_for(('10.0.0.2', 6000), 1.0)  # the same device, a new port
    assert not session.busy_for(('10.0.0.3', 5000), 3.0)  # the first one went silent
    session.cache((1, 'estop'), {'ok': True})
    assert session.accept(('10.0.0.2', 6000), 'phone', 1.0) and session.acks == {}
    assert session.deny(('1.2.3.4', 1), 0.0) and not session.deny(('1.2.3.4', 1), 0.5)
    assert session.expired(5.0) and not session.alive(5.0)
    session.set_streams({'camera': 1, 'bogus': True})
    assert session.streams == {'camera': True, 'lidar': False, 'video': False}


def test_routes_inside_the_sites_directory(tmp_path):
    folder = tmp_path / 'hall' / 'routes'
    folder.mkdir(parents=True)
    for name in ('a.csv', 'b.csv', 'notes.txt'):
        (folder / name).write_text('x,y,yaw_deg,dwell_sec\n')
    os.utime(folder / 'a.csv', (1, 1))
    assert routes.list_routes(str(tmp_path), 'hall') == ['hall/routes/b.csv', 'hall/routes/a.csv']
    assert routes.list_routes(str(tmp_path), '') == ['hall/routes/b.csv', 'hall/routes/a.csv']
    assert routes.route_path(str(tmp_path), 'hall/routes/a.csv') == 'hall/routes/a.csv'
    abs_path = str(folder / 'a.csv')
    assert routes.route_path(str(tmp_path), abs_path) == 'hall/routes/a.csv'
    for bad in ('../x.csv', '/etc/passwd.csv', 'hall/routes/notes.txt', '', 3):
        with pytest.raises(ValueError):
            routes.route_path(str(tmp_path), bad)


def test_command_arguments():
    assert args.dwell(5) == 5.0 and args.speed(0.456, 0.1, 0.8) == 0.46
    for bad in (0, -1, 4000, 'x', True):
        with pytest.raises(ValueError):
            args.dwell(bad)
    with pytest.raises(ValueError):
        args.speed(0.9, 0.1, 0.8)
    assert args.mode('remote') == 'remote'
    with pytest.raises(ValueError):
        args.mode('manual')
    with pytest.raises(ValueError):
        args.enabled({'enabled': 'yes'})


def test_web_whitelist_never_the_driver_cmd_vel():
    globs = web.web_globs('lite3')
    assert (
        '/lite3/battery' in globs['topics_glob'] and '/lite3/cmd_vel' not in globs['topics_glob']
    )
    assert "'/control/remote/dashboard/cmd_vel'" in globs['topics_glob']
    assert globs['params_glob'] == "['/path_follower_node/*']"
