"""In-process tests of teleop_udp_node: a real UDP client against the running node."""

from __future__ import annotations

import json
import socket
import time

import pytest

rclpy = pytest.importorskip('rclpy')
Twist = pytest.importorskip('geometry_msgs.msg').Twist
String = pytest.importorskip('std_msgs.msg').String
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
TeleopUdpNode = pytest.importorskip('molly_bridge.teleop_udp_node').TeleopUdpNode

TOKEN = 'secret'


def free_port() -> int:
    """Return a UDP port nobody listens on."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Rig:
    """The node, a listener on bridge/remote_cmd_vel and UDP clients."""

    def __init__(self, params=()):
        self.port = free_port()
        args = ['--ros-args', '-p', f'port:={self.port}', '-p', f'token:={TOKEN}']
        for param in params:
            args += ['-p', param]
        rclpy.init(args=args)
        try:
            self.node = TeleopUdpNode()
        except Exception:
            rclpy.shutdown()  # keep the next test's rclpy.init() working
            raise
        self.listener = rclpy.create_node('teleop_test_listener')
        self.twists, self.estop_requests = [], []
        self.listener.create_subscription(Twist, 'bridge/remote_cmd_vel', self.twists.append, 10)
        self.listener.create_subscription(
            String, 'bridge/estop_request', lambda m: self.estop_requests.append(m.data), 10
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self.clients = []

    def client(self, ip: str = '127.0.0.1') -> socket.socket:
        """Open a client socket bound to ip (127.0.0.2 counts as another device)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, 0))
        sock.settimeout(0.05)
        self.clients.append(sock)
        return sock

    def send(self, sock, message, raw: bool = False):
        """Send one datagram to the node and let it process it."""
        data = message if raw else json.dumps({'k': TOKEN, **message}).encode()
        sock.sendto(data, ('127.0.0.1', self.port))
        self.spin(0.2)

    def spin(self, seconds: float):
        """Spin both nodes for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def received(self, sock) -> list:
        """Return the JSON messages waiting on a client socket."""
        out = []
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except (socket.timeout, BlockingIOError):
                return out
            if data[:1] == b'{':
                out.append(json.loads(data))

    def close(self):
        """Tear everything down."""
        for sock in self.clients:
            sock.close()
        self.executor.shutdown()
        self.listener.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig():
    """Provide a running teleop node."""
    r = Rig()
    yield r
    r.close()


@pytest.fixture
def video_rig():
    """Provide a teleop node on a robot with WebRTC video (MediaMTX on port 8890)."""
    r = Rig(['video_port:=8890'])
    yield r
    r.close()


def test_hello_is_welcomed(rig):
    """A client with the right token gets the welcome message."""
    a = rig.client()
    rig.send(a, {'t': 'hello', 'name': 'test'})
    assert any(m['t'] == 'welcome' for m in rig.received(a))


def test_wrong_token_is_denied(rig):
    """A hello without the right token gets `denied` (once a second); nothing else, no motion."""
    a = rig.client()
    rig.send(a, json.dumps({'t': 'hello'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'hello', 'k': 'wrong'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'joy', 'vx': 0.5, 'k': 'wrong'}).encode(), raw=True)
    assert [m['t'] for m in rig.received(a)] == ['denied']
    assert rig.twists == []


def test_joystick_is_clamped_and_garbage_stops(rig):
    """Speeds are clamped to the limits; NaN, inf and strings become zero."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'joy', 'vx': 5.0, 'vy': -5.0, 'wz': 0.3})
    t = rig.twists[-1]
    assert (t.linear.x, t.linear.y, t.angular.z) == (1.0, -0.5, 0.3)
    for bad in (b'NaN', b'Infinity', b'"fast"', b'[1]', b'true'):
        rig.send(a, b'{"k": "secret", "t": "joy", "vx": ' + bad + b'}', raw=True)
        assert rig.twists[-1].linear.x == 0.0, bad


def test_malformed_messages_do_not_kill_the_node(rig):
    """Broken datagrams are dropped and the node keeps serving."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
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
    """Another IP cannot take over a live session; the same device may reconnect."""
    a, b = rig.client('127.0.0.1'), rig.client('127.0.0.2')
    rig.send(a, {'t': 'hello'})
    rig.received(a)
    rig.send(b, {'t': 'hello'})
    assert [m['t'] for m in rig.received(b)] == ['busy']
    rig.send(b, {'t': 'joy', 'vx': 0.5})
    assert not rig.twists or rig.twists[-1].linear.x == 0.0
    a2 = rig.client('127.0.0.1')  # app restart: same device, new port
    rig.send(a2, {'t': 'hello'})
    assert any(m['t'] == 'welcome' for m in rig.received(a2))


def test_route_outside_routes_dir_is_refused(rig):
    """load_route only accepts files under routes_dir."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.received(a)
    with pytest.raises(ValueError):
        rig.node.route_path('/etc/passwd')
    with pytest.raises(ValueError):
        rig.node.route_path('../../etc/passwd')


def test_handheld_estop_latches_without_waiting_for_services(rig):
    """Handheld estop and emergency_stop publish an e-stop request at once (no mux needed)."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'emergency_stop'})
    rig.spin(0.3)
    assert rig.estop_requests == ['handheld estop', 'handheld emergency_stop']
    acks = {m['id']: m for m in rig.received(a) if m['t'] == 'ack'}
    assert acks[1]['ok'] is False  # no mux in this test: the service is reported unavailable


def test_command_ids_are_per_session(rig):
    """A resent (id, name) runs once; after an app restart the same id runs again."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    assert rig.estop_requests == ['handheld estop']
    a2 = rig.client('127.0.0.1')  # app restart: new port, ids start over
    rig.send(a2, {'t': 'hello'})
    rig.send(a2, {'t': 'cmd', 'id': 1, 'name': 'estop'})
    rig.spin(0.2)
    assert rig.estop_requests == ['handheld estop', 'handheld estop']


def test_state_carries_apriltags_while_the_camera_is_on(rig):
    """Tags the detector sees reach the handheld as image fractions, with the camera stream."""
    tag_msgs = pytest.importorskip('apriltag_msgs.msg')
    CameraInfo = pytest.importorskip('sensor_msgs.msg').CameraInfo
    info_pub = rig.listener.create_publisher(CameraInfo, '/perception/apriltag/camera_info', 10)
    tag_pub = rig.listener.create_publisher(
        tag_msgs.AprilTagDetectionArray, '/perception/apriltag/detections', 10
    )
    a = rig.client()
    rig.send(a, {'t': 'hello', 'streams': {'camera': True}})
    tag = tag_msgs.AprilTagDetection(id=3)
    tag.corners = [
        tag_msgs.Point(x=float(x), y=float(y)) for x, y in ((0, 0), (320, 0), (320, 240), (0, 240))
    ]
    tags, end = [], time.monotonic() + 5.0
    while not tags and time.monotonic() < end:
        info_pub.publish(CameraInfo(width=640, height=480))
        tag_pub.publish(tag_msgs.AprilTagDetectionArray(detections=[tag]))
        rig.spin(0.2)
        states = [m for m in rig.received(a) if m.get('t') == 'state']
        tags = states[-1]['tags'] if states else []
    assert tags == [[3, 0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.0, 0.5]]


def test_streams_are_subscribed_only_while_wanted(rig):
    """The camera and TF are subscribed while a client wants them, and dropped when it leaves."""
    camera = '/camera/camera/color/image_raw/compressed'

    def topics():
        return {s.topic_name for s in rig.node.subscriptions}

    assert camera not in topics() and '/tf' not in topics()
    a = rig.client()
    rig.send(a, {'t': 'hello', 'streams': {'camera': True}})
    assert camera in topics() and '/tf' in topics()
    rig.send(a, {'t': 'streams', 'camera': False})
    assert camera not in topics() and '/tf' in topics()
    rig.send(a, {'t': 'bye'})
    assert '/tf' not in topics()


def test_no_webrtc_video_without_mediamtx(rig):
    """Without MediaMTX (video_port 0) the welcome offers no WebRTC: JPEG frames only."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    welcome = next(m for m in rig.received(a) if m['t'] == 'welcome')
    assert 'video' not in welcome


def test_webrtc_video_is_offered_when_mediamtx_runs(video_rig):
    """The welcome names the WHEP port and path when robot.launch.py started MediaMTX."""
    a = video_rig.client()
    video_rig.send(a, {'t': 'hello'})
    welcome = next(m for m in video_rig.received(a) if m['t'] == 'welcome')
    assert welcome['video'] == {'port': 8890, 'path': 'camera'}


def test_webrtc_video_keeps_the_tags_without_jpeg_frames(video_rig):
    """While the handheld plays WebRTC video the tags still come, the JPEG camera does not."""
    pytest.importorskip('apriltag_msgs.msg')
    camera = '/camera/camera/color/image_raw/compressed'
    tags = '/perception/apriltag/detections'

    def topics():
        return {s.topic_name for s in video_rig.node.subscriptions}

    a = video_rig.client()
    video_rig.send(a, {'t': 'hello', 'streams': {'camera': False, 'video': True}})
    assert tags in topics() and camera not in topics()
    video_rig.send(a, {'t': 'streams', 'video': False, 'camera': True})
    assert tags in topics() and camera in topics()
    video_rig.send(a, {'t': 'streams', 'camera': False})
    assert tags not in topics()


def wait_until(rig, condition, timeout: float = 5.0) -> bool:
    """Spin until condition holds (service discovery, latched messages) or timeout passes."""
    end = time.monotonic() + timeout
    while not condition() and time.monotonic() < end:
        rig.spin(0.05)
    return condition()


def fake_node(rig, name: str, params: dict):
    """Add a node with the given parameters (and so their parameter services) to the rig."""
    node = rclpy.create_node(name)
    for param, value in params.items():
        node.declare_parameter(param, value)
    rig.executor.add_node(node)
    return node


def acks_of(rig, sock) -> dict:
    """Return the acknowledgements waiting on a client socket, by command id."""
    return {m['id']: m for m in rig.received(sock) if m['t'] == 'ack'}


def test_walking_speed_is_read_and_set_within_the_limits(rig):
    """The handheld sees the path follower's speed and sets it, never above its maximum."""
    follower = fake_node(
        rig, 'path_follower_node', {'target_linear_velocity': 0.4, 'max_linear_velocity': 0.6}
    )
    a = rig.client()
    assert wait_until(rig, rig.node._speed_client.service_is_ready)
    rig.send(a, {'t': 'hello'})
    states = []
    wait_until(
        rig,
        lambda: (
            states.extend(m for m in rig.received(a) if m['t'] == 'state')
            or (states and states[-1]['speed'] is not None)
        ),
    )
    assert states[-1]['speed'] == {'target': 0.4, 'min': 0.1, 'max': 0.6}

    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'nav_speed', 'args': {'speed': 0.55}})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'nav_speed', 'args': {'speed': 0.9}})
    rig.send(a, {'t': 'cmd', 'id': 3, 'name': 'nav_speed', 'args': {}})
    acks = {}
    wait_until(rig, lambda: acks.update(acks_of(rig, a)) or len(acks) >= 3)
    assert acks[1]['ok'] and not acks[2]['ok'] and not acks[3]['ok']
    assert '0.6' in acks[2]['msg']
    assert follower.get_parameter('target_linear_velocity').value == 0.55
    rig.executor.remove_node(follower)
    follower.destroy_node()


def test_people_detection_is_switched_and_shown_with_the_system_load(rig):
    """The handheld turns the person detector off; people, CPU, GPU and RAM reach it."""
    detector = fake_node(rig, 'person_detector_node', {'enabled': True})
    latched = rclpy.qos.QoSProfile(
        depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
    )
    people_pub = detector.create_publisher(String, 'perception/people_status', latched)
    health_pub = detector.create_publisher(String, 'health/status', latched)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    welcome = next(m for m in rig.received(a) if m['t'] == 'welcome')
    assert 'people_set' in welcome['commands']
    assert wait_until(rig, rig.node._commands['people_set'][1].service_is_ready)

    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'people_set', 'args': {'enabled': False}})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'people_set', 'args': {'enabled': 'no'}})
    acks = {}
    wait_until(rig, lambda: acks.update(acks_of(rig, a)) or len(acks) >= 2)
    assert acks[1]['ok'] and not acks[2]['ok']
    assert detector.get_parameter('enabled').value is False

    people = {'enabled': True, 'state': 'running', 'count': 3, 'latency_ms': 80, 'at': time.time()}
    people['boxes'] = [[0.1, 0.2, 0.3, 0.9, 0.87], [0, 0, 1.5, 0.5, 0.5], [1, 2], 'x']
    people_pub.publish(String(data=json.dumps(people)))
    health = {
        'ok': True,
        'issues': [],
        'cpu_percent': 23.4,
        'gpu_percent': 41.2,
        'memory_percent': 33.3,
        'memory_used_gb': 5.12,
        'memory_total_gb': 15.3,
        'cpu_temp_c': 52.6,
    }
    health_pub.publish(String(data=json.dumps(health)))
    states = []

    def arrived():
        states.extend(m for m in rig.received(a) if m['t'] == 'state')
        return bool(states) and states[-1]['people'] is not None and states[-1]['health']

    assert wait_until(rig, arrived)
    state = states[-1]
    assert (state['people']['count'], state['people']['state']) == (3, 'running')
    assert state['people']['boxes'] == [[0.1, 0.2, 0.3, 0.9, 0.87], [0.0, 0.0, 1.0, 0.5, 0.5]]
    assert (state['health']['cpu'], state['health']['gpu'], state['health']['ram_used']) == (
        23.0,
        41.0,
        5.1,
    )
    rig.executor.remove_node(detector)
    detector.destroy_node()


def latched_qos():
    """Transient-local QoS like the latched status topics."""
    return rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)


def streams_of(rig, sock) -> dict:
    """Return the binary stream payloads waiting on a client socket, by stream id."""
    from molly_bridge import protocol

    reassembler, out = protocol.Reassembler(), {}
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except (socket.timeout, BlockingIOError):
            return out
        frame = reassembler.push(data)
        if frame is not None:
            out[frame[0]] = frame[2]


def test_welcome_names_the_molly_protocol_and_gestures(rig):
    """The welcome carries the protocol, the gestures, the features and the local commands."""
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    welcome = next(m for m in rig.received(a) if m['t'] == 'welcome')
    assert welcome['protocol'] == 'molly-1'
    assert welcome['gestures'] == ['hello', 'nod', 'bow', 'look_left', 'look_right']
    assert welcome['features'] == []  # nothing of the guide runs in this test
    expected = {'get_map', 'get_site', 'map_start', 'poi_add', 'tour_start', 'go_to_poi', 'say'}
    assert expected <= set(welcome['commands'])
    assert 'inspection_set' not in welcome['commands']


def test_guide_commands_reach_their_services(rig):
    """Mapping, POI, tour, walking and speech commands call Molly's services with their args."""
    from dharma_interfaces.msg import Poi, PoiArray
    from dharma_interfaces.srv import GoTo, SavePoi, StartMapping
    from molly_interfaces.msg import TourCommand
    from molly_interfaces.srv import Say, TourControl

    robot = rclpy.create_node('fake_molly')
    calls = {}

    def serve(kind, name, answer=None):
        def callback(request, response):
            calls.setdefault(name, []).append(request)
            response.success = True
            if answer:
                answer(request, response)
            return response

        robot.create_service(kind, name, callback)

    serve(StartMapping, 'mapping/start')
    serve(SavePoi, 'site/save_poi', lambda q, r: setattr(r, 'poi', q.poi))
    serve(TourControl, 'tour/control')
    serve(GoTo, 'planner/go_to')
    serve(Say, 'interaction/say', lambda q, r: setattr(r, 'id', 7))
    pois = robot.create_publisher(PoiArray, 'site/pois', latched_qos())
    pois.publish(PoiArray(site='hall', pois=[Poi(id='lobby', name='Lobby', x=1.0, y=2.0)]))
    rig.executor.add_node(robot)
    assert wait_until(rig, lambda: rig.node._commands['say'][1].service_is_ready())
    assert wait_until(rig, lambda: len(rig.node._pois) == 1)

    a = rig.client()
    rig.send(a, {'t': 'hello'})
    commands = [
        ('map_start', {'site': 'hall'}),
        ('map_start', {'site': '../etc'}),
        ('poi_add', {'name': '  Pintu   Masuk ', 'narration': 'Selamat datang', 'dwell': 5}),
        ('poi_update', {'poi': {'id': 'lobby', 'narration': 'Ini lobi', 'here': True}}),
        ('poi_update', {'poi': {'id': 'nowhere'}}),
        ('tour_start', {'tour_id': 'main'}),
        ('tour_skip', {}),
        ('go_to_poi', {'id': 'lobby'}),
        ('go_to', {'x': 1.5, 'y': float('nan')}),
        ('say', {'text': 'Halo', 'lang': 'id'}),
        ('say', {'text': '   '}),
    ]
    for cid, (name, args) in enumerate(commands, start=1):
        rig.send(a, {'t': 'cmd', 'id': cid, 'name': name, 'args': args})
    acks = {}
    wait_until(
        rig,
        lambda: (
            acks.update(acks_of(rig, a))
            or sum(1 for m in acks.values() if m['ok'] is not None) >= len(commands)
        ),
    )
    ok = {cid: acks[cid]['ok'] for cid in acks}
    assert ok == {
        1: True,
        2: False,
        3: True,
        4: True,
        5: False,
        6: True,
        7: True,
        8: True,
        9: False,
        10: True,
        11: False,
    }
    start = calls['mapping/start'][0]
    assert (start.site, start.reset) == ('hall', True)
    added, moved = calls['site/save_poi']
    assert (added.poi.name, added.poi.dwell_sec, added.at_robot_pose) == ('Pintu Masuk', 5.0, True)
    assert (moved.poi.id, moved.poi.name, moved.poi.narration, moved.at_robot_pose) == (
        'lobby',
        'Lobby',
        'Ini lobi',
        True,
    )
    assert acks[3]['data']['name'] == 'Pintu Masuk'
    kinds = [(c.command.kind, c.command.tour_id) for c in calls['tour/control']]
    assert kinds == [(TourCommand.KIND_START, 'main'), (TourCommand.KIND_SKIP, '')]
    assert calls['planner/go_to'][0].poi_id == 'lobby'
    assert (calls['interaction/say'][0].text, acks[10]['data']) == ('Halo', {'id': 7})
    rig.executor.remove_node(robot)
    robot.destroy_node()


def test_guide_state_and_streams_reach_the_handheld(rig):
    """Site, tour, planner, speech and visitors are in the state; map and site are streamed."""
    from nav_msgs.msg import OccupancyGrid

    from dharma_interfaces.msg import (
        PlannerStatus,
        Poi,
        PoiArray,
        SiteStatus,
        Tour,
        TourArray,
        Visitors,
    )
    from molly_bridge import map_image, protocol
    from molly_interfaces.msg import SpeechStatus, TourState

    robot = rclpy.create_node('fake_guide')
    latched = latched_qos()
    publishers = {
        name: robot.create_publisher(kind, name, latched)
        for name, kind in (
            ('/map', OccupancyGrid),
            ('site/status', SiteStatus),
            ('site/pois', PoiArray),
            ('site/tours', TourArray),
            ('planner/status', PlannerStatus),
            ('/tour/state', TourState),
            ('interaction/speech_status', SpeechStatus),
        )
    }
    visitors = robot.create_publisher(Visitors, 'perception/visitors', 10)
    rig.executor.add_node(robot)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.received(a)

    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 4, 3, 0.5
    grid.info.origin.position.x, grid.info.origin.position.y = -1.0, -2.0
    grid.data = [0, 0, 100, -1] + [0] * 8  # bottom row: free, free, wall, unknown
    publishers['/map'].publish(grid)
    publishers['site/status'].publish(SiteStatus(site='hall', has_map=True, pois=1, tags=2))
    publishers['site/pois'].publish(PoiArray(site='hall', pois=[Poi(id='lobby', name='Lobby')]))
    publishers['site/tours'].publish(
        TourArray(site='hall', tours=[Tour(id='main', name='Main', poi_ids=['lobby'])])
    )
    publishers['planner/status'].publish(
        PlannerStatus(state='following', goal_id='lobby', remaining=3.26)
    )
    publishers['/tour/state'].publish(
        TourState(
            state=TourState.STATE_RUNNING,
            tour_id='main',
            current_poi_index=0,
            total_pois=1,
            current_poi_id='lobby',
            activity='navigating',
        )
    )
    publishers['interaction/speech_status'].publish(
        SpeechStatus(id=3, state='speaking', text='Hi')
    )
    states, streams = [], {}

    def arrived():
        visitors.publish(Visitors(count=2, front=1, behind=1, nearest=1.24))
        streams.update(streams_of(rig, a))
        rig.send(a, {'t': 'ping'})
        states.extend(m for m in rig.received(a) if m.get('t') == 'state')
        return (
            bool(states)
            and states[-1]['tour'] is not None
            and states[-1]['visitors']
            and (protocol.STREAM_SITE in streams and protocol.STREAM_MAP in streams)
        )

    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'get_map'})
    assert wait_until(rig, arrived)
    state = states[-1]
    assert state['site']['name'] == 'hall' and state['site']['map'] and state['map_rev'] >= 1
    assert state['tour']['state'] == 'running' and state['tour']['poi_id'] == 'lobby'
    assert state['planner'] == {'state': 'following', 'goal': 'lobby', 'remaining': 3.3, 'msg': ''}
    assert state['speech']['state'] == 'speaking'
    assert state['visitors'] == {'count': 2, 'front': 1, 'behind': 1, 'nearest': 1.2}
    site = json.loads(streams[protocol.STREAM_SITE])
    assert site['name'] == 'hall' and site['pois'][0]['id'] == 'lobby'
    assert site['tours'][0]['poi_ids'] == ['lobby']
    header, png = protocol.split_map_payload(streams[protocol.STREAM_MAP])
    assert (header['w'], header['h'], header['ox'], header['oy']) == (4, 3, -1.0, -2.0)
    image = map_image.decode_png(png)
    assert image[-1].tolist() == [254, 254, 0, 205]  # bottom row of the map is the last row
    rig.executor.remove_node(robot)
    robot.destroy_node()
