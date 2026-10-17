#!/usr/bin/env python3
from collections import deque
import hmac
import json
import math
import os
import select
import socket
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from molly_bridge import protocol
from molly_bridge.guide_link import GESTURES, GuideLink
from molly_interfaces.msg import MissionStatus, NavigationStatus, WaypointPath
from molly_interfaces.srv import LoadPath, MarkStopPoint, SetMode
import numpy as np
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import cv2
except ImportError:  # raw images need OpenCV, compressed ones do not
    cv2 = None
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:  # no AprilTag detector here: no tag overlay on the handheld video
    AprilTagDetectionArray = None

# std_srvs/Trigger services every quadruped driver should offer under its namespace; robot-specific
# extras (gait or speed modes, ...) are added per robot with robot_commands in the site file.
ROBOT_COMMANDS = ['stand_up', 'sit_down', 'emergency_stop']

# perception/people_status comes every 2 s; older than this the node is gone (not running)
PEOPLE_STALE_SEC = 6.0
# answered by the gateway itself
LOCAL_COMMANDS = ('get_map', 'get_route', 'get_site', 'list_routes')
# how often the path follower is asked for its walking speed while a handheld is connected
SPEED_POLL_SEC = 3.0


def yaw_of(q) -> float:
    """Return the yaw of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def level_of(status) -> int:
    """Return a DiagnosticStatus level as int (rclpy gives the byte field as bytes)."""
    level = status.level
    return level[0] if isinstance(level, bytes) else int(level)


def finite(value, digits: int = 2):
    """Round a float for JSON, or None when it is NaN/inf."""
    return round(float(value), digits) if math.isfinite(value) else None


def number(value, default=None):
    """Return a finite int/float as float (bools and text are not numbers), else default."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return default
    return float(value)


def typed_parameter(name: str, value) -> Parameter:
    """Return a Parameter; a list is a string array, never empty (rclpy: byte array)."""
    if isinstance(value, list):
        return Parameter(name, Parameter.Type.STRING_ARRAY, [str(v) for v in value] or [''])
    return Parameter(name, value=value)


def set_parameters_request(params: list) -> SetParameters.Request:
    """Return a SetParameters request for (name, value) pairs (types from the values)."""
    return SetParameters.Request(
        parameters=[typed_parameter(name, value).to_parameter_msg() for name, value in params]
    )


def command_outcome(result) -> tuple:
    """Return (ok, message) of a service response: success / message or SetParameters results."""
    if result is None:
        return False, 'no response'
    results = getattr(result, 'results', None)
    if results is not None:
        refused = [r.reason or 'refused' for r in results if not r.successful]
        return not refused, '; '.join(refused)
    return bool(getattr(result, 'success', True)), getattr(result, 'message', '')


def people_parameters(args: dict) -> list:
    """Map people_set args to the person detector's live parameters."""
    if not isinstance(args.get('enabled'), bool):
        raise ValueError('enabled must be true or false')
    return [('enabled', args['enabled'])]


PEOPLE_BOX_SEC = 2.0  # older person boxes are not drawn


def people_summary(status: dict, stale: bool, now: float) -> dict:
    """Compact perception/people_status for the handheld's state message."""
    count, at = number(status.get('count')), number(status.get('at'))
    return {
        'on': status.get('enabled') is True,
        'state': 'offline' if stale else str(status.get('state', ''))[:16],
        'count': None if count is None else int(count),
        'ms': int(number(status.get('latency_ms'), 0.0)),
        'age': None if at is None else max(0, round(now - at)),
        'boxes': [] if stale or at is None or now - at > PEOPLE_BOX_SEC else people_boxes(status),
        'backend': str(status.get('backend', ''))[:24],
        'error': str(status.get('error', ''))[:160],
    }


def people_boxes(status: dict) -> list:
    """Person boxes [x1, y1, x2, y2, score] as fractions of the image, at most 20."""
    boxes = []
    raw = status.get('boxes')
    for box in raw[:20] if isinstance(raw, list) else []:
        if not isinstance(box, list) or len(box) != 5:
            continue
        values = [number(v) for v in box]
        if None in values:
            continue
        boxes.append([round(min(max(v, 0.0), 1.0), 3) for v in values])
    return boxes


def system_load(health: dict) -> dict:
    """CPU, GPU and RAM use and the hottest temperature of health/status, for the handheld."""

    def pick(key: str, digits: int = 0):
        value = number(health.get(key))
        return None if value is None else round(value, digits)

    return {
        'cpu': pick('cpu_percent'),
        'gpu': pick('gpu_percent'),
        'ram': pick('memory_percent'),
        'ram_used': pick('memory_used_gb', 1),
        'ram_total': pick('memory_total_gb', 1),
        'temp': pick('cpu_temp_c'),
    }


def dwell_seconds(value) -> float:
    """Validate a stop-point dwell time from the client (0 < dwell <= 1 h)."""
    dwell = float(value)
    if not math.isfinite(dwell) or not 0.0 < dwell <= 3600.0:
        raise ValueError(f'dwell must be between 0 and 3600 s, got {value!r}')
    return dwell


class TeleopUdpNode(GuideLink, Node):
    """UDP gateway for the handheld: joystick, commands, telemetry, camera, lidar and the guide."""

    def __init__(self):
        """Declare params, open the socket and wire ROS interfaces."""
        super().__init__('teleop_udp_node')
        defaults = {
            'port': protocol.PORT,
            'token': '',
            'robot_name': '',
            'client_timeout_sec': 2.0,
            'max_vx': 1.0,
            'max_vy': 0.5,
            'max_wz': 1.5,
            'telemetry_hz': 10.0,
            'robot_command_prefix': '/robot/',
            'robot_commands': ROBOT_COMMANDS,
            'routes_dir': '~/dev/Molly/map',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'camera_topic': '/camera/camera/color/image_raw/compressed',
            'camera_compressed': True,
            'camera_fps': 8.0,
            'camera_jpeg_quality': 60,
            'camera_max_width': 640,
            'camera_max_bytes': 60000,
            'video_port': 0,
            'video_path': 'camera',
            'tags_topic': '/perception/apriltag/detections',
            'tags_camera_info_topic': '/perception/apriltag/camera_info',
            'tags_max_age_sec': 0.6,
            'lidar_topic': '/cloud_registered_body',
            'lidar_hz': 2.0,
            'lidar_voxel': 0.1,
            'lidar_max_points': 3000,
            'people_node': '/person_detector_node',
            'navigation_node': '/path_follower_node',
            'patrol_speed_min': 0.1,
            'map_max_px': 1024,
            'map_push_sec': 3.0,
            'gestures': GESTURES,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self._token = p('token')
        self._robot_name = p('robot_name') or socket.gethostname()
        self._seq = 0
        self._client_name = ''
        self._client_timeout = float(p('client_timeout_sec'))
        self._limits = (float(p('max_vx')), float(p('max_vy')), float(p('max_wz')))
        self._routes_dir = os.path.expanduser(str(p('routes_dir')))
        self._map_frame, self._base_frame = p('map_frame'), p('base_frame')
        self._client = None
        self._client_seen = 0.0
        self._session = 0
        self._denied: dict = {}
        self._streams = {'camera': False, 'lidar': False, 'video': False}
        self._video = None
        if int(p('video_port')) > 0:
            self._video = {'port': int(p('video_port')), 'path': str(p('video_path'))}
        self._stream_subs: dict = {}
        self._acks: dict = {}
        self._frame_ids = {
            protocol.STREAM_CAMERA: 0,
            protocol.STREAM_LIDAR: 0,
            protocol.STREAM_ROUTE: 0,
            protocol.STREAM_MAP: 0,
            protocol.STREAM_SITE: 0,
        }
        self._state = {
            'mode': 'auto',
            'nav': 'NAV_INACTIVE',
            'nav_msg': '',
            'mission': '',
            'localization': 'none',
            'recording': False,
            'battery': None,
            'driver': None,
            'cmd': [0.0, 0.0, 0.0],
            'health': None,
            'estop': False,
            'field': None,
        }
        self._route: list = []
        self._extrinsics: dict = {}
        self._last_camera = self._last_lidar = 0.0
        self._big_frame_warned = False
        self._tags: list = []
        self._tags_time = 0.0
        self._tags_size = (0, 0)
        self._tags_max_age = float(p('tags_max_age_sec'))
        self._camera_period = 1.0 / max(float(p('camera_fps')), 0.1)
        self._lidar_period = 1.0 / max(float(p('lidar_hz')), 0.1)
        self._people = None  # latest perception/people_status
        self._people_time = 0.0
        self._people_node = str(p('people_node')).rstrip('/')
        self._speed = None  # {target, min, max} of the path follower, None until it answered
        self._speed_asked = 0.0
        self._speed_future = None
        self._speed_min = float(p('patrol_speed_min'))
        self._navigation_node = str(p('navigation_node')).rstrip('/')

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:  # room for a camera frame sent as one burst of datagrams (capped by wmem_max)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        except OSError:
            pass
        try:
            self._sock.bind(('0.0.0.0', int(p('port'))))
        except OSError as error:
            self.get_logger().fatal(
                f'cannot bind UDP port {p("port")}: {error} (another teleop gateway running?)'
            )
            raise
        self._sock.setblocking(False)
        if not self._token:
            self.get_logger().warn(
                'teleop token is empty: anyone on this network can drive the robot, '
                'set teleop_udp_node.token'
            )

        self.init_guide(p('map_max_px'), p('map_push_sec'), list(p('gestures')))
        self._tf_buffer = Buffer()
        self._tf_listener = None  # only while a client is connected, see sync_subscriptions
        self._remote_pub = self.create_publisher(Twist, 'bridge/remote_cmd_vel', 10)
        self.make_clients(p('robot_command_prefix'), p('robot_commands'))
        self.make_subscriptions(
            p('camera_topic'),
            bool(p('camera_compressed')),
            p('lidar_topic'),
            (p('tags_topic'), p('tags_camera_info_topic')),
        )

        # A reader thread wakes the executor per datagram: no polling timer while idle.
        self._inbox = deque(maxlen=256)
        self._wake = self.create_guard_condition(self.drain)
        self._running = True
        self._reader = threading.Thread(target=self.read_socket, daemon=True)
        self._reader.start()
        self.create_timer(1.0 / float(p('telemetry_hz')), self.tick)
        self.get_logger().info(f'teleop UDP on port {p("port")}')

    def destroy_node(self):
        """Stop the reader and close the UDP socket, so the port is free again at once."""
        self._running = False
        self._reader.join(timeout=1.0)
        self._sock.close()
        super().destroy_node()

    # ------------------------------------------------------------------ setup

    def make_clients(self, prefix: str, robot_commands: list):
        """Map command names to (service type, client, request factory)."""

        def trigger(name):
            return Trigger, self.create_client(Trigger, name), lambda a: Trigger.Request()

        def set_bool(name, value):
            return (
                SetBool,
                self.create_client(SetBool, name),
                lambda a, v=value: SetBool.Request(data=v),
            )

        def set_params(node, params_of):
            return (
                SetParameters,
                self.create_client(SetParameters, f'{node}/set_parameters'),
                lambda a: set_parameters_request(params_of(a)),
            )

        estop = self.create_client(SetBool, 'bridge/estop')
        self._estop_request_pub = self.create_publisher(String, 'bridge/estop_request', 10)
        start_nav = self.create_client(SetBool, 'navigation/start_nav')
        pause = self.create_client(SetBool, 'navigation/pause')
        record = self.create_client(SetBool, 'route/record')
        self._commands = {
            'set_mode': (
                SetMode,
                self.create_client(SetMode, 'bridge/set_mode'),
                lambda a: SetMode.Request(mode=str(a.get('mode', 'auto'))),
            ),
            'estop': (SetBool, estop, lambda a: SetBool.Request(data=True)),
            'estop_release': (SetBool, estop, lambda a: SetBool.Request(data=False)),
            'nav_start': (SetBool, start_nav, lambda a: SetBool.Request(data=True)),
            'nav_stop': (SetBool, start_nav, lambda a: SetBool.Request(data=False)),
            'nav_pause': (SetBool, pause, lambda a: SetBool.Request(data=True)),
            'nav_resume': (SetBool, pause, lambda a: SetBool.Request(data=False)),
            'record_start': (SetBool, record, lambda a: SetBool.Request(data=True)),
            'record_stop': (SetBool, record, lambda a: SetBool.Request(data=False)),
            'mark_stop': (
                MarkStopPoint,
                self.create_client(MarkStopPoint, 'route/mark_stop_point'),
                lambda a: MarkStopPoint.Request(dwell_sec=dwell_seconds(a.get('dwell', 10.0))),
            ),
            'load_route': (
                LoadPath,
                self.create_client(LoadPath, 'navigation/load_path'),
                lambda a: LoadPath.Request(waypoints_file=self.route_path(a.get('file', ''))),
            ),
            'localize': trigger('localization/start'),
            'record_tags': trigger('localization/record_tags'),
            'people_set': set_params(self._people_node, people_parameters),
            'nav_speed': set_params(
                self._navigation_node,
                lambda a: [('target_linear_velocity', self.patrol_speed(a.get('speed')))],
            ),
        }
        self._commands.update(self.make_guide_commands())
        self._speed_client = self.create_client(
            GetParameters, f'{self._navigation_node}/get_parameters'
        )
        self._robot_commands = [name for name in robot_commands if name]
        for name in self._robot_commands:
            self._commands[name] = trigger(prefix + name)

    def make_subscriptions(
        self, camera_topic: str, compressed: bool, lidar_topic: str, tag_topics: tuple
    ):
        """Subscribe to telemetry; camera, tags and lidar wait for a client that wants them."""
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavigationStatus, 'navigation/status', self.on_nav_status, 10)
        self.create_subscription(MissionStatus, 'navigation/mission_status', self.on_mission, 10)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        self.create_subscription(String, 'bridge/mode', self.on_mode, latched)
        self.create_subscription(Bool, 'bridge/estop_engaged', self.on_estop, latched)
        self.create_subscription(
            String,
            'bridge/protective_field',
            lambda m: self._state.update(field=m.data),
            latched,
        )
        self.create_subscription(String, 'health/status', self.on_health, latched)
        self.create_subscription(String, 'perception/people_status', self.on_people, latched)
        self.create_subscription(WaypointPath, 'navigation/waypoints', self.on_route, latched)
        self.create_subscription(BatteryState, 'battery', self.on_battery, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.on_diagnostics, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.make_guide_subscriptions()
        # name: (streams that need it, type, topic, callback); the tags go over any video
        video = ('camera', 'video')
        sources = {
            'camera': (
                ('camera',),
                CompressedImage if compressed else Image,
                camera_topic,
                self.on_camera,
            ),
            'lidar': (('lidar',), PointCloud2, lidar_topic, self.on_lidar),
        }
        if AprilTagDetectionArray is not None:
            sources['tags'] = (video, AprilTagDetectionArray, tag_topics[0], self.on_tags)
            sources['tags_info'] = (video, CameraInfo, tag_topics[1], self.on_tags_info)
        self._stream_sources = {name: source for name, source in sources.items() if source[2]}

    def sync_subscriptions(self):
        """
        Hold the camera, lidar and TF subscriptions only while a client uses them.

        Deserializing a 30 Hz camera, a 10 Hz cloud and every TF message cost ~1/3 of a Jetson
        core while nobody was connected.
        """
        live = self._client is not None
        if live and self._tf_listener is None:
            self._tf_listener = TransformListener(self._tf_buffer, self)
        elif not live and self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        for name, (streams, kind, topic, callback) in self._stream_sources.items():
            wanted = live and any(self._streams[stream] for stream in streams)
            subscription = self._stream_subs.get(name)
            if wanted and subscription is None:
                self._stream_subs[name] = self.create_subscription(
                    kind, topic, callback, qos_profile_sensor_data
                )
            elif not wanted and subscription is not None:
                self.destroy_subscription(self._stream_subs.pop(name))

    # ------------------------------------------------------------------ socket

    def send(self, data: bytes, addr=None) -> bool:
        """Send one datagram to the client (or addr); False when it was dropped (socket busy)."""
        addr = addr or self._client
        if addr is None:
            return False
        try:
            self._sock.sendto(data, addr)
        except (BlockingIOError, OSError):
            return False
        return True

    def send_json(self, message: dict, addr=None):
        """Send a JSON message to the client (or addr)."""
        self.send(protocol.encode_json(message), addr)

    def send_stream(self, stream: int, payload: bytes):
        """Send a chunked binary payload to the client; stop at a dropped chunk."""
        self._frame_ids[stream] += 1
        for datagram in protocol.chunk(stream, self._frame_ids[stream], payload):
            if not self.send(datagram):
                return  # the client cannot rebuild this frame any more

    def read_socket(self):
        """Receive datagrams on the reader thread and wake the executor to handle them."""
        while self._running:
            try:
                ready, _, _ = select.select([self._sock], [], [], 0.5)
            except (OSError, ValueError):  # socket closed under us
                return
            if not ready:
                continue
            for _ in range(64):
                try:
                    self._inbox.append(self._sock.recvfrom(65535))
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    if not self._running:
                        return
                    break
            try:
                self._wake.trigger()
            except Exception:  # noqa: B902  rclpy is shutting down
                return

    def drain(self):
        """Handle the received datagrams (bounded inbox, so a flood cannot starve ROS)."""
        for _ in range(len(self._inbox)):
            data, addr = self._inbox.popleft()
            message = protocol.decode_json(data)
            if message is None:
                continue
            if message.get('t') == 'discover':
                self.reply_beacon(addr)  # no token needed, nothing sensitive
                continue
            if not self.authorized(message):
                if message.get('t') == 'hello':
                    self.deny(addr)
                continue
            try:
                self.handle_message(message, addr)
            except (AttributeError, KeyError, TypeError, ValueError) as error:
                self.get_logger().warn(
                    f'ignored malformed teleop message from {addr}: {error}',
                    throttle_duration_sec=5.0,
                )

    def authorized(self, message: dict) -> bool:
        """Check the shared token (constant time) when one is configured."""
        if not self._token:
            return True
        return hmac.compare_digest(str(message.get('k', '')), self._token)

    def deny(self, addr):
        """Tell a handheld its token is wrong, at most once per second per address."""
        now = time.monotonic()
        if now - self._denied.get(addr, -1.0) < 1.0:
            return
        if len(self._denied) > 64:
            self._denied = {a: t for a, t in self._denied.items() if now - t < 1.0}
        self._denied[addr] = now
        self.get_logger().warn(f'teleop hello from {addr[0]} with a wrong token', once=True)
        self.send_json({'t': 'denied', 'msg': 'wrong token'}, addr)

    def accept_client(self, addr, message: dict) -> bool:
        """
        Register addr as the client; another device cannot take over a live session.

        The same device may come back from a new port (app restart). A different device gets
        `busy` until the current client has been silent for client_timeout_sec.
        """
        current = self._client
        if current is not None and addr != current and addr[0] != current[0]:
            if time.monotonic() - self._client_seen <= self._client_timeout:
                self.send_json({'t': 'busy', 'msg': f'robot is driven from {current[0]}'}, addr)
                return False
        if addr != current:
            self.get_logger().info(f'teleop client {addr} ({message.get("name", "?")})')
            self.end_session()
            self._client = addr
            self._session += 1
        return True

    def end_session(self):
        """Forget the client and its acknowledgements, and stop the remote velocity."""
        if self._client is not None:
            self._remote_pub.publish(Twist())
        self._client = None
        self._acks.clear()

    def handle_message(self, message: dict, addr):
        """Dispatch one client message."""
        kind = message['t']
        if kind == 'hello':
            if not self.accept_client(addr, message):
                return
            self._client_name = str(message.get('name', ''))[:32]
            streams = message.get('streams', {})
            if not isinstance(streams, dict):
                streams = {}
            self.set_camera_fps(streams.get('camera_fps'))
            self._streams = {k: bool(streams.get(k, False)) for k in self._streams}
            welcome = {
                't': 'welcome',
                'name': self._robot_name,
                'protocol': protocol.NAME,
                'commands': sorted([*self._commands, *LOCAL_COMMANDS]),
                'robot_commands': self._robot_commands,
                'gestures': self.gestures(),
                'features': self.features(),
                'limits': self._limits,
            }
            if self._video:
                welcome['video'] = self._video  # WebRTC (WHEP) through MediaMTX
            self.send_json(welcome)
            self.send_route()
            self.send_map(force=True)
            self.send_site()
            self.ask_speed()
        elif addr != self._client:
            return
        self._client_seen = time.monotonic()
        if kind == 'joy':
            self.on_joy(message)
        elif kind == 'cmd':
            self.on_command(message)
        elif kind == 'ping':
            self.send_json({'t': 'pong', 'ts': message.get('ts')})
        elif kind == 'streams':
            self._streams.update({k: bool(v) for k, v in message.items() if k in self._streams})
            self.set_camera_fps(message.get('camera_fps'))
        elif kind == 'bye':
            self.end_session()
        self.sync_subscriptions()

    def tick(self):
        """Expire a silent client, then send the telemetry snapshot."""
        if self._client and time.monotonic() - self._client_seen > self._client_timeout:
            self.get_logger().warn(f'teleop client {self._client} timed out')
            self.end_session()
            self.sync_subscriptions()
        if self._client and time.monotonic() - self._speed_asked > SPEED_POLL_SEC:
            self.ask_speed()
        self.send_map()
        self.send_state()

    def on_joy(self, message: dict):
        """Publish a clamped teleop velocity; NaN, inf and non-numbers count as 0."""
        vx_max, vy_max, wz_max = self._limits
        twist = Twist()
        twist.linear.x = protocol.clamp_axis(message.get('vx'), vx_max)
        twist.linear.y = protocol.clamp_axis(message.get('vy'), vy_max)
        twist.angular.z = protocol.clamp_axis(message.get('wz'), wz_max)
        self._remote_pub.publish(twist)

    def on_command(self, message: dict):
        """Run a named command once per (id, name) in this session and acknowledge it."""
        cid, name, args = message.get('id'), str(message.get('name', '')), message.get('args', {})
        if not isinstance(cid, (int, str)) or isinstance(cid, bool):
            return
        if not isinstance(args, dict):
            args = {}
        key = (cid, name)
        if key in self._acks:
            self.send_json(self._acks[key])
            return
        if name in ('estop', 'emergency_stop'):
            # latch the software e-stop first, without waiting for any service
            self._estop_request_pub.publish(String(data=f'handheld {name}'))
        if name == 'list_routes':
            self.ack(key, True, '', self.list_routes())
            return
        if name == 'get_route':
            self.send_route()
            self.ack(key, True, f'{len(self._route)} points')
            return
        if name == 'get_map':
            self.send_map(force=True)
            self.ack(key, self._grid is not None, '' if self._grid is not None else 'no map yet')
            return
        if name == 'get_site':
            self.send_site()
            self.ack(key, True, f'{len(self._pois)} POIs, {len(self._tours)} tours')
            return
        if name not in self._commands:
            self.ack(key, False, f'unknown command {name}')
            return
        _, client, make_request = self._commands[name]
        if not client.service_is_ready():
            self.ack(key, False, f'{client.srv_name} not available')
            return
        try:
            request = make_request(args)
        except (TypeError, ValueError) as error:
            self.ack(key, False, str(error))
            return
        future = client.call_async(request)
        session = self._session
        future.add_done_callback(lambda f: self.on_command_done(key, session, f))
        self._acks[key] = {'t': 'ack', 'id': cid, 'ok': None, 'msg': 'pending'}

    def on_command_done(self, key: tuple, session: int, future):
        """Acknowledge a finished service call, unless another session started meanwhile."""
        name = key[1]
        data = None
        try:
            result = future.result()
        except Exception as error:  # noqa: B902  the service raised, report it to the client
            ok, msg = False, f'{name} failed: {error}'
        else:
            ok, msg = command_outcome(result)
            data = self.command_data(name, result) if result is not None else None
        if ok and name in ('record_start', 'record_stop'):
            self._state['recording'] = name == 'record_start'
        if ok and name == 'nav_speed':
            self._speed_asked = 0.0  # show the new speed with the next state
        if session == self._session:
            self.ack(key, ok, msg, data)

    def ack(self, key: tuple, ok: bool, msg: str, data=None):
        """Send and cache an acknowledgement."""
        reply = {'t': 'ack', 'id': key[0], 'ok': ok, 'msg': msg}
        if data is not None:
            reply['data'] = data
        self._acks[key] = reply
        if len(self._acks) > 200:
            self._acks.pop(next(iter(self._acks)))
        self.send_json(reply)

    def list_routes(self) -> list:
        """Return recorded route files of the active site (else under routes_dir), newest first."""
        found = []
        top = self._routes_dir
        if self._site_name:
            top = os.path.join(self._routes_dir, self._site_name, 'routes')
        for root, _, files in os.walk(top):
            for name in files:
                if name.endswith('.csv'):
                    path = os.path.join(root, name)
                    try:
                        found.append((os.path.getmtime(path), path))
                    except OSError:  # deleted while listing
                        continue
        return [path for _, path in sorted(found, reverse=True)[:50]]

    def route_path(self, file) -> str:
        """Resolve a route file from the client; only files under routes_dir may be loaded."""
        root = os.path.realpath(self._routes_dir)
        path = os.path.realpath(os.path.join(root, str(file)))
        if not str(file) or os.path.commonpath([root, path]) != root:
            raise ValueError(f'route must be a file under {self._routes_dir}')
        return path

    # ------------------------------------------------------------------ telemetry

    def on_nav_status(self, msg: NavigationStatus):
        """Cache the navigation state."""
        self._state['nav'], self._state['nav_msg'] = msg.state, msg.message

    def on_mission(self, msg: MissionStatus):
        """Cache the mission summary."""
        self._state['mission'] = msg.active_behavior

    def on_localization(self, msg: String):
        """Cache the localization source."""
        self._state['localization'] = msg.data

    def on_mode(self, msg: String):
        """Cache the control mode."""
        self._state['mode'] = msg.data

    def on_estop(self, msg: Bool):
        """Cache the software e-stop state."""
        self._state['estop'] = bool(msg.data)

    def on_health(self, msg: String):
        """Cache ok + issues of the health summary."""
        try:
            health = json.loads(msg.data)
        except ValueError:
            return
        self._state['health'] = {
            'ok': bool(health.get('ok')),
            'issues': health.get('issues', [])[:5],
            **system_load(health),
        }

    def on_people(self, msg: String):
        """Cache the person detector's status (on, state, people counted)."""
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(status, dict):
            self._people = status
            self._people_time = time.monotonic()

    def people_state(self):
        """Return the people part of the state message, None when no detector ever ran."""
        if self._people is None:
            return None
        stale = time.monotonic() - self._people_time > PEOPLE_STALE_SEC
        return people_summary(self._people, stale, time.time())

    def patrol_speed(self, value) -> float:
        """Validate a walking speed from the handheld against the path follower's limits."""
        speed = number(value)
        top = number((self._speed or {}).get('max'), 1.0)
        if speed is None or not self._speed_min <= speed <= top:
            raise ValueError(f'speed must be {self._speed_min:g} to {top:g} m/s')
        return round(speed, 2)

    def ask_speed(self):
        """Ask the path follower for its walking speed and cap, one request at a time."""
        self._speed_asked = time.monotonic()
        if self._speed_future is not None and not self._speed_future.done():
            return
        if not self._speed_client.service_is_ready():
            self._speed = None  # no navigation running (teach mode, or it is restarting)
            return
        request = GetParameters.Request(names=['target_linear_velocity', 'max_linear_velocity'])
        self._speed_future = self._speed_client.call_async(request)
        self._speed_future.add_done_callback(self.on_speed)

    def on_speed(self, future):
        """Keep the path follower's answer: target and max linear velocity."""
        try:
            values = future.result().values
        except Exception:  # noqa: B902  the node went away meanwhile: ask again later
            return
        double = ParameterType.PARAMETER_DOUBLE
        if len(values) == 2 and all(v.type == double for v in values):
            target, top = values[0].double_value, values[1].double_value
            self._speed = {
                'target': round(target, 2),
                'min': self._speed_min,
                'max': round(max(top, self._speed_min), 2),
            }

    def on_battery(self, msg: BatteryState):
        """Cache battery readings."""
        self._state['battery'] = {
            'pct': finite(msg.percentage * 100.0, 0),
            'v': finite(msg.voltage, 1),
            'a': finite(msg.current, 1),
            'temp': finite(msg.temperature, 0),
        }

    def on_diagnostics(self, msg: DiagnosticArray):
        """Cache the driver status line."""
        if msg.status:
            status = msg.status[0]
            self._state['driver'] = {'level': level_of(status), 'msg': status.message}

    def on_cmd_vel(self, msg: Twist):
        """Cache the command actually sent to the robot."""
        self._state['cmd'] = [
            round(msg.linear.x, 2),
            round(msg.linear.y, 2),
            round(msg.angular.z, 2),
        ]

    def on_route(self, msg: WaypointPath):
        """Cache the loaded route and push it to the client."""
        self._route = protocol.simplify_route(
            [[round(w.x, 2), round(w.y, 2), round(w.dwell_sec, 1)] for w in msg.waypoints], 600
        )
        self.send_route()

    def send_route(self):
        """Send the loaded route as a chunked JSON payload."""
        if self._client is not None:
            payload = protocol.encode_json({'t': 'route', 'points': self._route})
            self.send_stream(protocol.STREAM_ROUTE, payload)

    def pose(self):
        """Return the robot [x, y, yaw] in the map frame, or None."""
        try:
            t = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        p = t.transform.translation
        return [round(p.x, 3), round(p.y, 3), round(yaw_of(t.transform.rotation), 3)]

    def send_state(self):
        """Send the telemetry snapshot; seq lets the client measure packet loss."""
        if self._client is not None:
            self._seq += 1
            self.send_json(
                {
                    't': 'state',
                    'seq': self._seq,
                    'pose': self.pose(),
                    'tags': self.tags(),
                    'people': self.people_state(),
                    'speed': self._speed,
                    **self.guide_state(),
                    **self._state,
                }
            )

    def tags(self) -> list:
        """Tags in the camera image now, corners as image fractions, for the handheld overlay."""
        if not self._tags or time.monotonic() - self._tags_time > self._tags_max_age:
            return []
        return protocol.tag_quads(self._tags, *self._tags_size)

    def on_tags(self, msg):
        """Keep the latest AprilTag detections (pixel corners of the detector's image)."""
        self._tags = [(d.id, [(c.x, c.y) for c in d.corners]) for d in msg.detections]
        self._tags_time = time.monotonic()

    def on_tags_info(self, msg: CameraInfo):
        """Size of the image the AprilTag detector sees."""
        self._tags_size = (msg.width, msg.height)

    def reply_beacon(self, addr):
        """Answer a LAN discovery probe (no token needed, nothing sensitive)."""
        alive = (
            self._client is not None
            and time.monotonic() - self._client_seen < self._client_timeout
        )
        beacon = {
            't': 'beacon',
            'name': self._robot_name,
            'secured': bool(self._token),
            'in_use_by': self._client_name if alive else '',
            'mode': self._state.get('mode'),
            'battery': (self._state.get('battery') or {}).get('pct'),
            'site': self._site_name,
        }
        try:
            self._sock.sendto(protocol.encode_json(beacon), addr)
        except OSError:
            pass

    def set_camera_fps(self, fps):
        """Let the handheld lower or restore the camera rate, capped by camera_fps."""
        if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps):
            return  # absent, text or NaN (json accepts NaN): keep the current rate
        limit = max(float(self.get_parameter('camera_fps').value), 0.1)
        self._camera_period = 1.0 / min(max(float(fps), 0.5), limit)

    # ------------------------------------------------------------------ streams

    def on_camera(self, msg):
        """Forward a throttled JPEG frame."""
        now = time.monotonic()
        if not self._streams['camera'] or self._client is None:
            return
        if now - self._last_camera < self._camera_period:
            return
        self._last_camera = now
        if isinstance(msg, CompressedImage):
            jpeg = self.shrink(bytes(msg.data))
        else:
            jpeg = self.encode(msg)
        if jpeg:
            self.send_stream(protocol.STREAM_CAMERA, jpeg)

    def shrink(self, jpeg: bytes) -> bytes:
        """
        Re-encode a driver JPEG bigger than camera_max_bytes (drivers default to quality 95).

        A frame travels as a burst of 1400-byte datagrams and one lost datagram loses the frame,
        so a 150 kB JPEG rarely reaches the handheld whole over Wi-Fi.
        """
        if len(jpeg) <= int(self.get_parameter('camera_max_bytes').value):
            return jpeg
        if cv2 is None:
            if not self._big_frame_warned:
                self._big_frame_warned = True
                self.get_logger().warn(
                    f'camera frames of {len(jpeg)} B are too big for the handheld and OpenCV is '
                    'missing to shrink them: install python3-opencv or lower the JPEG quality'
                )
            return jpeg
        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        return self.compress(img) if img is not None else b''

    def encode(self, msg: Image) -> bytes:
        """JPEG-encode a raw rgb8/bgr8 image."""
        if cv2 is None or msg.encoding not in ('rgb8', 'bgr8'):
            return b''
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        img = img[:, : msg.width]
        if msg.encoding == 'rgb8':
            img = np.ascontiguousarray(img[:, :, ::-1])
        return self.compress(img)

    def compress(self, img) -> bytes:
        """JPEG-encode a BGR image at camera_jpeg_quality, no wider than camera_max_width."""
        height, width = img.shape[:2]
        max_width = int(self.get_parameter('camera_max_width').value)
        if width > max_width:
            size = (max_width, int(height * max_width / width))
            img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
        quality = int(self.get_parameter('camera_jpeg_quality').value)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else b''

    def on_lidar(self, msg: PointCloud2):
        """Forward a throttled, voxel-downsampled cloud in the robot frame."""
        now = time.monotonic()
        if not self._streams['lidar'] or self._client is None:
            return
        if now - self._last_lidar < self._lidar_period:
            return
        self._last_lidar = now
        extrinsic = self.extrinsic(msg.header.frame_id)
        if extrinsic is None:
            return
        offsets = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offsets.keys():
            return
        dtype = np.dtype(
            {
                'names': ['x', 'y', 'z'],
                'formats': ['<f4'] * 3,
                'offsets': [offsets['x'], offsets['y'], offsets['z']],
                'itemsize': msg.point_step,
            }
        )
        cloud = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        rotation, translation = extrinsic
        xyz = xyz @ rotation.T + translation
        xyz = protocol.voxel_downsample(
            xyz,
            float(self.get_parameter('lidar_voxel').value),
            int(self.get_parameter('lidar_max_points').value),
        )
        self.send_stream(protocol.STREAM_LIDAR, protocol.pack_points(xyz))

    def extrinsic(self, frame_id: str):
        """Return the cached (rotation, translation) from frame_id to base_frame, or None."""
        if frame_id not in self._extrinsics:
            try:
                t = self._tf_buffer.lookup_transform(self._base_frame, frame_id, Time())
            except TransformException:
                return None
            q, v = t.transform.rotation, t.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=np.float32,
            )
            self._extrinsics[frame_id] = (rotation, np.array([v.x, v.y, v.z], np.float32))
        return self._extrinsics[frame_id]


def main(args=None):
    """Spin the teleop UDP node."""
    rclpy.init(args=args)
    node = TeleopUdpNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.send_json({'t': 'bye'})
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
