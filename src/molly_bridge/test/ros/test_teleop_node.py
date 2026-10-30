"""teleop_udp_node in-process: a real UDP client against the running node."""

from __future__ import annotations

import json

from bridge_rig import Rig
import pytest
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from dharma_interfaces.msg import ControlStatus, LocalizationStatus, PlannerStatus
from dharma_interfaces.srv import SetMode
from dharma_utils.ros import LATCHED
from molly_interfaces.msg import Poi, PoiArray
from molly_interfaces.srv import GoToPoi


@pytest.fixture
def rig(ros):
    r = Rig('teleop_helper')
    yield r
    r.close()


def hello(rig, sock, **streams):
    rig.send(sock, {'t': 'hello', 'name': 'test', 'streams': streams})
    return rig.received(sock)


def test_hello_is_welcomed_with_the_robot_limits(rig):
    welcome = next(m for m in hello(rig, rig.client()) if m['t'] == 'welcome')
    assert welcome['protocol'] == 'molly-1' and welcome['limits'] == [1.0, 0.5, 1.5]
    assert {'set_mode', 'tour_start', 'go_to_poi', 'dock', 'get_map'} <= set(welcome['commands'])
    assert 'people_set' not in welcome['commands']


def test_wrong_token_is_denied(rig):
    a = rig.client()
    rig.send(a, json.dumps({'t': 'hello'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'hello', 'k': 'wrong'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'joy', 'vx': 0.5, 'k': 'wrong'}).encode(), raw=True)
    assert [m['t'] for m in rig.received(a)] == ['denied']
    assert rig.twists == []


def test_joystick_goes_to_the_mux_clamped_and_garbage_stops(rig):
    a = rig.client()
    hello(rig, a)
    rig.send(a, {'t': 'joy', 'vx': 5.0, 'vy': -5.0, 'wz': 0.3})
    t = rig.twists[-1]
    assert (t.linear.x, t.linear.y, t.angular.z) == (1.0, -0.5, 0.3)
    for bad in (b'NaN', b'Infinity', b'"fast"', b'[1]', b'true'):
        rig.send(a, b'{"k": "secret", "t": "joy", "vx": ' + bad + b'}', raw=True)
        assert rig.twists[-1].linear.x == 0.0, bad


def test_malformed_messages_do_not_kill_the_node(rig):
    a = rig.client()
    hello(rig, a)
    for junk in (
        b'\xff\xfe',
        b'[]',
        b'{"k": "secret"}',
        b'{"k": "secret", "t": "cmd", "id": [1]}',
        b'{"k": "secret", "t": "hello", "streams": 5}',
    ):
        rig.send(a, junk, raw=True)
    rig.send(a, {'t': 'ping', 'ts': 7})
    assert {'t': 'pong', 'ts': 7} in rig.received(a)


def test_second_device_is_busy_while_first_drives(rig):
    a, b = rig.client('127.0.0.1'), rig.client('127.0.0.2')
    hello(rig, a)
    rig.send(b, {'t': 'hello'})
    assert [m['t'] for m in rig.received(b)] == ['busy']
    rig.send(b, {'t': 'joy', 'vx': 0.5})
    assert not rig.twists or rig.twists[-1].linear.x == 0.0
    a2 = rig.client('127.0.0.1')  # app restart: same device, new port
    assert any(m['t'] == 'welcome' for m in hello(rig, a2))


def test_estop_latches_first_and_ids_run_once_per_session(rig):
    a = rig.client()
    hello(rig, a)
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'emergency_stop'})
    assert rig.estop_requests == ['handheld estop', 'handheld emergency_stop']
    acks = {m['id']: m for m in rig.received(a) if m['t'] == 'ack'}
    assert acks[1]['ok'] is False and 'not available' in acks[1]['msg']  # no mux here
    a2 = rig.client('127.0.0.1')  # a new session forgets the acks
    hello(rig, a2)
    rig.send(a2, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    assert rig.estop_requests[-1] == 'handheld estop' and len(rig.estop_requests) == 3


def test_commands_reach_the_stack_and_molly(rig):
    calls = []

    def set_mode(request, response):
        calls.append(('set_mode', request.mode, request.source))
        response.success, response.message = True, 'remote'
        return response

    def go_to_poi(request, response):
        calls.append(('go_to_poi', request.id))
        response.success = True
        return response

    def engage(_request, response):
        calls.append(('engage',))
        response.success = True
        return response

    rig.helper.create_service(SetMode, 'control/set_mode', set_mode)
    rig.helper.create_service(GoToPoi, 'guide/go_to_poi', go_to_poi)
    rig.helper.create_service(Trigger, 'control/estop/engage', engage)
    a = rig.client()
    hello(rig, a)
    assert rig.until(rig.node.commands['set_mode'].client.service_is_ready)
    rig.send(a, {'t': 'cmd', 'id': 10, 'name': 'set_mode', 'args': {'mode': 'remote'}})
    rig.send(a, {'t': 'cmd', 'id': 11, 'name': 'set_mode', 'args': {'mode': 'manual'}})
    rig.send(a, {'t': 'cmd', 'id': 12, 'name': 'go_to_poi', 'args': {'id': 'lobby'}})
    rig.send(a, {'t': 'cmd', 'id': 13, 'name': 'estop'}, spin=0.5)
    acks = {m['id']: m for m in rig.received(a) if m['t'] == 'ack' and m['ok'] is not None}
    assert acks[10]['ok'] and acks[12]['ok'] and acks[13]['ok']
    assert not acks[11]['ok'] and 'auto or remote' in acks[11]['msg']
    assert calls == [('set_mode', 'remote', 'handheld'), ('go_to_poi', 'lobby'), ('engage',)]


def test_state_carries_the_typed_status(rig):
    pub = rig.helper.create_publisher
    pub(ControlStatus, 'control/status', LATCHED).publish(
        ControlStatus(mode=ControlStatus.MODE_REMOTE, estop=True, field=ControlStatus.FIELD_STOP)
    )
    loc = LocalizationStatus(state=LocalizationStatus.LOCALIZED, source='tag:base_map')
    pub(LocalizationStatus, 'localization/status', LATCHED).publish(loc)
    pub(PoiArray, 'site/pois', LATCHED).publish(
        PoiArray(site='hall', pois=[Poi(id='lobby', name='Lobby', x=1.0, y=2.0)])
    )
    pub(PlannerStatus, 'planner/status', LATCHED).publish(
        PlannerStatus(state=PlannerStatus.FOLLOWING, goal_x=1.0, goal_y=2.0, remaining=3.0)
    )
    a = rig.client()
    hello(rig, a)
    assert rig.until(lambda: rig.node.telemetry.pois and rig.node.telemetry.guide['planner'])
    rig.spin(0.3)
    states = [m for m in rig.received(a) if m['t'] == 'state']
    assert states, 'no state message'
    state = states[-1]
    assert (state['mode'], state['estop'], state['field']) == ('remote', True, 'stop')
    assert state['localization'] == 'tag:base_map'
    assert state['planner']['state'] == 'following' and state['planner']['goal'] == 'lobby'


def test_silent_client_is_dropped_with_a_zero(ros):
    rig = Rig('teleop_helper_timeout', [Parameter('client_timeout_sec', value=0.3)])
    try:
        a = rig.client()
        hello(rig, a)
        rig.send(a, {'t': 'joy', 'vx': 0.4})
        rig.spin(0.8)
        assert rig.node.session.client is None
        assert rig.twists[-1].linear.x == 0.0
    finally:
        rig.close()


def test_discovery_beacon_needs_no_token(rig):
    a = rig.client()
    rig.send(a, json.dumps({'t': 'discover'}).encode(), raw=True)
    beacon = next(m for m in rig.received(a) if m['t'] == 'beacon')
    assert beacon['secured'] is True and beacon['in_use_by'] == ''


def test_routes_are_listed_relative_and_loaded_inside(ros, tmp_path):
    (tmp_path / 'hall' / 'routes').mkdir(parents=True)
    (tmp_path / 'hall' / 'routes' / 'loop.csv').write_text('x,y,yaw_deg,dwell_sec\n')
    rig = Rig('teleop_helper_routes', [Parameter('sites_dir', value=str(tmp_path))])
    try:
        a = rig.client()
        hello(rig, a)
        rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'list_routes'})
        rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'load_route', 'args': {'file': '../x.csv'}})
        acks = {m['id']: m for m in rig.received(a) if m['t'] == 'ack'}
        assert acks[1]['data'] == ['hall/routes/loop.csv']
        assert acks[2]['ok'] is False
    finally:
        rig.close()
