"""
teleop_udp_node: the handheld's UDP gateway (Molly protocol v1, PROTOCOL.md).

The joystick goes to the control mux as an operator source (control/remote/handheld/cmd_vel);
the mux alone drives the robot. Commands become service calls, the robot's state a 10 Hz JSON
message, the camera, cloud, route, map and site chunked binary streams.
"""

from __future__ import annotations

import socket
import time

from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from dharma_utils.ros import COMMAND, EVENTS, DharmaNode, run
from molly_bridge.core import protocol, routes
from molly_bridge.core.config import TeleopConfig
from molly_bridge.core.session import Session
from molly_bridge.utils.commands import CommandTable, ack_data, outcome
from molly_bridge.utils.link import UdpLink
from molly_bridge.utils.pusher import Pusher
from molly_bridge.utils.speed import SpeedPoll
from molly_bridge.utils.streams import Streams
from molly_bridge.utils.telemetry import Telemetry

LOCAL_COMMANDS = ('get_map', 'get_route', 'get_site', 'list_routes')


class TeleopUdpNode(DharmaNode):
    """UDP gateway for the handheld: joystick, commands, telemetry and streams."""

    def __init__(self, **kwargs):
        super().__init__('teleop_udp_node', **kwargs)
        self.cfg = cfg = self.declare_config(TeleopConfig).value
        self.robot = self.declare_robot().value
        self.remote_source, self.sites_dir = cfg.remote_source, cfg.sites_dir
        self.navigation_node = cfg.navigation_node.rstrip('/')
        self.robot_name = cfg.robot_name or socket.gethostname()
        self.session = Session(cfg.token, cfg.client_timeout_sec)
        self.seq = 0
        self.video = {'port': cfg.video_port, 'path': cfg.video_path} if cfg.video_port else None
        group = ReentrantCallbackGroup()
        self.remote_pub = self.create_publisher(
            Twist, f'control/remote/{cfg.remote_source}/cmd_vel', COMMAND
        )
        self.estop_pub = self.create_publisher(String, 'control/estop_request', EVENTS)
        self.speed = SpeedPoll(self, self.navigation_node, cfg.patrol_speed_min, group)
        self.telemetry = Telemetry(self, self.on_changed)
        self.table = CommandTable(self, self, group)
        self.commands = self.table.build(cfg.robot_command_prefix, cfg.robot_commands)
        self.robot_commands = [n for n in cfg.robot_commands if n]
        self.pusher = Pusher(self.telemetry, self.send_stream, cfg.map_max_px, cfg.map_push_sec)
        self.streams = Streams(self, cfg, self.robot.frames, self.send_stream,
                               lambda note: self.log.warning(note, once=True))  # fmt: skip
        try:
            self.link = UdpLink(self, cfg.port, self.on_message)
        except OSError as error:
            self.log.critical('cannot bind the UDP port', port=cfg.port, error=str(error))
            raise
        if not cfg.token:
            self.log.warning('teleop token empty: anyone on this network can drive the robot')
        self.every(1.0 / cfg.telemetry_hz, self.tick)
        self.add_shutdown_hook(self.on_shutdown)
        self.log.info('teleop UDP ready', port=cfg.port, source=cfg.remote_source)

    # ---------------------------------------------------------------- what commands need
    def speed_range(self) -> tuple[float, float]:
        """Return the walking speeds the handheld may set (m/s)."""
        return self.speed.range(self.robot.limits.max_vx)

    def gesture_names(self) -> list[str]:
        """Return the gestures the robot runs (interaction_node), else the configured ones."""
        names = self.telemetry.gestures
        return list(names) if names is not None else [g for g in self.cfg.gestures if g]

    # ---------------------------------------------------------------- messages
    def on_message(self, message: dict, addr):
        """Dispatch one datagram from the reader thread's inbox."""
        if message.get('t') == 'discover':
            self.reply_beacon(addr)  # no token needed, nothing sensitive
            return
        now = time.monotonic()
        if not self.session.authorized(message):
            if message.get('t') == 'hello' and self.session.deny(addr, now):
                self.log.warning('teleop hello with a wrong token', address=addr[0], once=True)
                self.link.send_json({'t': 'denied', 'msg': 'wrong token'}, addr)
            return
        try:
            self.dispatch(message, addr, now)
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            self.log.warning('malformed teleop message', address=addr[0], error=str(error),
                             throttle=5.0)  # fmt: skip

    def dispatch(self, message: dict, addr, now: float):
        """Handle one authorized client message."""
        kind = message['t']
        if kind == 'hello':
            if self.session.busy_for(addr, now):
                busy = {'t': 'busy', 'msg': f'robot is driven from {self.session.client[0]}'}
                self.link.send_json(busy, addr)
                return
            self.hello(message, addr, now)
        elif addr != self.session.client:
            return
        self.session.seen = now
        if kind == 'joy':
            self.on_joy(message)
        elif kind == 'cmd':
            self.on_command(message)
        elif kind == 'ping':
            self.send_json({'t': 'pong', 'ts': message.get('ts')})
        elif kind == 'streams':
            self.session.set_streams(message)
            self.streams.set_camera_fps(message.get('camera_fps'))
        elif kind == 'bye':
            self.end_session()
        self.streams.sync(self.session.client is not None, self.session.streams)

    def hello(self, message: dict, addr, now: float):
        """Register the client and send the welcome, the route, the map and the site."""
        if self.session.accept(addr, message.get('name', ''), now):
            self.log.info('teleop client', address=addr, name=self.session.name)
        streams = message.get('streams') if isinstance(message.get('streams'), dict) else {}
        self.session.streams = dict.fromkeys(self.session.streams, False)
        self.session.set_streams(streams)
        self.streams.set_camera_fps(streams.get('camera_fps'))
        welcome = {
            't': 'welcome',
            'name': self.robot_name,
            'protocol': protocol.NAME,
            'commands': sorted([*self.commands, *LOCAL_COMMANDS]),
            'robot_commands': self.robot_commands,
            'gestures': self.gesture_names(),
            'features': self.table.features(),
            'limits': [self.robot.limits.max_vx, self.robot.limits.max_vy,
                       self.robot.limits.max_wz],
        }  # fmt: skip
        if self.video:
            welcome['video'] = self.video  # WebRTC (WHEP) through MediaMTX
        self.send_json(welcome)
        for kind in ('route', 'map', 'site'):
            self.pusher.push(kind, force=True)
        self.speed.ask()

    def on_joy(self, message: dict):
        """Publish a clamped operator velocity; NaN, inf and non-numbers count as 0."""
        limits, twist = self.robot.limits, Twist()
        twist.linear.x = protocol.clamp_axis(message.get('vx'), limits.max_vx)
        twist.linear.y = protocol.clamp_axis(message.get('vy'), limits.max_vy)
        twist.angular.z = protocol.clamp_axis(message.get('wz'), limits.max_wz)
        self.remote_pub.publish(twist)

    def end_session(self):
        """Forget the client and stop the operator velocity."""
        if self.session.client is not None:
            self.remote_pub.publish(Twist())
        self.session.end()

    # ---------------------------------------------------------------- commands
    def on_command(self, message: dict):
        """Run a named command once per (id, name) in this session and acknowledge it."""
        cid, name, a = message.get('id'), str(message.get('name', '')), message.get('args', {})
        if not isinstance(cid, (int, str)) or isinstance(cid, bool):
            return
        key, a = (cid, name), a if isinstance(a, dict) else {}
        if key in self.session.acks:
            self.send_json(self.session.acks[key])
            return
        if name in ('estop', 'emergency_stop'):  # latch first, without waiting for a service
            self.estop_pub.publish(String(data=f'handheld {name}'))
        if name in LOCAL_COMMANDS:
            self.local_command(key)
            return
        command = self.commands.get(name)
        if command is None:
            self.ack(key, False, f'unknown command {name}')
            return
        if not command.client.service_is_ready():
            self.ack(key, False, f'{command.client.srv_name} not available')
            return
        try:
            request = command.request(a)
        except (TypeError, ValueError) as error:
            self.ack(key, False, str(error))
            return
        session = self.session.number
        future = command.client.call_async(request)
        future.add_done_callback(lambda f: self.command_done(key, session, f))
        self.session.cache(key, {'t': 'ack', 'id': cid, 'ok': None, 'msg': 'pending'})

    def local_command(self, key: tuple):
        """Answer get_map, get_route, get_site and list_routes from the gateway itself."""
        name, telemetry = key[1], self.telemetry
        if name == 'list_routes':
            self.ack(key, True, '', routes.list_routes(self.sites_dir, telemetry.site))
            return
        kind = name[len('get_') :]
        self.pusher.push(kind, force=True)
        if kind == 'map' and telemetry.grid is None:
            self.ack(key, False, 'no map yet')
        else:
            counts = {
                'route': f'{len(telemetry.route)} points',
                'map': '',
                'site': f'{len(telemetry.pois)} POIs, {len(telemetry.tours)} tours',
            }
            self.ack(key, True, counts[kind])

    def command_done(self, key: tuple, session: int, future):
        """Acknowledge a finished service call, unless another session started meanwhile."""
        try:
            result = future.result()
        except Exception as error:
            ok, msg, data = False, f'{key[1]} failed: {error}', None
        else:
            ok, msg = outcome(result)
            data = ack_data(key[1], result) if result is not None else None
        if ok and key[1] in ('record_start', 'record_stop'):
            self.telemetry.state['recording'] = key[1] == 'record_start'
        if ok and key[1] == 'nav_speed':
            self.speed.asked = 0.0  # show the new speed with the next state
        if session == self.session.number:
            self.ack(key, ok, msg, data)

    def ack(self, key: tuple, ok: bool, msg: str, data=None):
        """Send and cache an acknowledgement."""
        reply = {'t': 'ack', 'id': key[0], 'ok': ok, 'msg': msg}
        if data is not None:
            reply['data'] = data
        self.session.cache(key, reply)
        self.send_json(reply)

    # ---------------------------------------------------------------- telemetry and streams
    def send_json(self, message: dict):
        """Send a JSON message to the client."""
        self.link.send_json(message, self.session.client)

    def send_stream(self, stream: int, payload: bytes):
        """Send a binary stream frame to the client."""
        self.link.send_stream(stream, payload, self.session.client)

    def on_changed(self, kind: str):
        """Push a changed route, map or site to a connected client."""
        if kind in ('route', 'map', 'site') and self.session.client is not None:
            self.pusher.push(kind)

    def tick(self):
        """Expire a silent client, refresh the speed, push the map, send the state."""
        now = time.monotonic()
        if self.session.expired(now):
            self.log.warning('teleop client timed out', address=self.session.client)
            self.end_session()
            self.streams.sync(False, self.session.streams)
        if self.session.client is None:
            return
        if self.speed.due(now):
            self.speed.ask()
        self.pusher.push('map')
        self.seq += 1
        state = {'t': 'state', 'seq': self.seq, 'pose': self.streams.pose(),
                 'tags': self.streams.tag_quads(), 'speed': self.speed.value}  # fmt: skip
        self.send_json({**state, **self.telemetry.snapshot()})

    def reply_beacon(self, addr):
        """Answer a LAN discovery probe."""
        alive = self.session.alive(time.monotonic())
        beacon = {
            't': 'beacon',
            'name': self.robot_name,
            'secured': bool(self.cfg.token),
            'in_use_by': self.session.name if alive else '',
            'mode': self.telemetry.state.get('mode'),
            'battery': (self.telemetry.state.get('battery') or {}).get('pct'),
            'site': self.telemetry.site,
        }
        self.link.send_json(beacon, addr)

    def on_shutdown(self):
        """Tell the handheld and stop the operator velocity."""
        self.send_json({'t': 'bye'})
        self.end_session()

    def destroy_node(self):
        """Close the UDP socket, so the port is free again at once."""
        if hasattr(self, 'link'):
            self.link.close()
        super().destroy_node()


def main(argv=None):
    """Run the node (one thread: nothing in it blocks)."""
    run(TeleopUdpNode, args=argv)


if __name__ == '__main__':
    main()
