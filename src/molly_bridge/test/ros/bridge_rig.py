"""A teleop_udp_node, a helper node with fake services, and UDP clients."""

from __future__ import annotations

import json
import socket
import time

from geometry_msgs.msg import Twist
from molly_bridge.teleop_udp_node import TeleopUdpNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

TOKEN = 'secret'


def free_port() -> int:
    """Return a UDP port nobody listens on."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Rig:
    """The node, a helper node listening to what it publishes, and handheld sockets."""

    def __init__(self, name: str, params=()):
        self.port = free_port()
        overrides = [
            Parameter('port', value=self.port),
            Parameter('token', value=TOKEN),
            Parameter('robot.limits.max_vx', value=1.0),
            Parameter('robot.limits.max_vy', value=0.5),
            Parameter('robot.limits.max_wz', value=1.5),
            *params,
        ]
        self.node = TeleopUdpNode(parameter_overrides=overrides)
        self.helper = rclpy.create_node(name)
        self.twists, self.estop_requests = [], []
        self.helper.create_subscription(
            Twist, 'control/remote/handheld/cmd_vel', self.twists.append, 10
        )
        self.helper.create_subscription(
            String, 'control/estop_request', lambda m: self.estop_requests.append(m.data), 10
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.clients = []

    def client(self, ip: str = '127.0.0.1') -> socket.socket:
        """Open a client socket bound to ip (127.0.0.2 counts as another device)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, 0))
        sock.settimeout(0.05)
        self.clients.append(sock)
        return sock

    def send(self, sock, message, raw: bool = False, spin: float = 0.2):
        """Send one datagram to the node and let it process it."""
        data = message if raw else json.dumps({'k': TOKEN, **message}).encode()
        sock.sendto(data, ('127.0.0.1', self.port))
        self.spin(spin)

    def spin(self, seconds: float):
        """Spin both nodes for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def until(self, condition, timeout: float = 5.0) -> bool:
        """Spin until condition() holds."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if condition():
                return True
        return False

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
        self.helper.destroy_node()
        self.node.destroy_node()
