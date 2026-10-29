"""
End to end on dharma_sim: the demo tour of three POIs driven to the end, with a handheld
taking over through the control mux on the way and handing back to AUTO.

Starts `ros2 launch molly_bringup sim.launch.py` (the workspace installed, as in colcon test)
with a fresh sites directory, its own ROS domain and teleop port. Set MOLLY_SKIP_SIM=1 to skip.
"""

from __future__ import annotations

import json
import math
import os
import random
import signal
import socket
import subprocess
import tempfile
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped  # noqa: E402

from dharma_interfaces.msg import ControlStatus, LocalizationStatus, NavigationStatus  # noqa: E402
from dharma_utils.ros import LATCHED  # noqa: E402
from dharma_utils.ros.compat import try_shutdown  # noqa: E402
from molly_interfaces.msg import SpeechStatus, TourCommand, TourState  # noqa: E402
from molly_interfaces.srv import TourControl  # noqa: E402

pytestmark = pytest.mark.skipif(os.environ.get('MOLLY_SKIP_SIM') == '1', reason='MOLLY_SKIP_SIM')
DOMAIN = 120 + os.getpid() % 40
WEST_HALL = (-4.5, 1.0, -math.pi / 2)  # the last POI of the demo tour (config/demo_site)


def free_port() -> int:
    """Return a free UDP port below the ephemeral range: no socket the kernel numbers takes it."""
    rng = random.Random(os.getpid())
    for _ in range(100):
        port = rng.randrange(20000, 32000)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            try:
                s.bind(('0.0.0.0', port))
            except OSError:
                continue
            return port
    raise RuntimeError('no free UDP port')


class Watcher:
    """Talks to the stack like the dashboard and keeps what it reports."""

    def __init__(self, node):
        self.node = node
        self.tour, self.speech, self.poses, self.nav = [], [], [], []
        self.control = self.localization = None
        sub = node.create_subscription
        sub(TourState, 'tour/state', self.tour.append, LATCHED)
        sub(SpeechStatus, 'interaction/speech_status', self.speech.append, LATCHED)
        sub(NavigationStatus, 'navigation/status', self.nav.append, LATCHED)
        sub(ControlStatus, 'control/status', lambda m: setattr(self, 'control', m), LATCHED)
        sub(LocalizationStatus, 'localization/status',
            lambda m: setattr(self, 'localization', m), LATCHED)  # fmt: skip
        sub(PoseStamped, 'sim/ground_truth', self.on_pose, 10)

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.poses.append((msg.pose.position.x, msg.pose.position.y, yaw))

    def spin_until(self, condition, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return True
        return False

    def call(self, kind, name: str, request, timeout: float = 30.0):
        client = self.node.create_client(kind, name)
        if not self.spin_until(client.service_is_ready, 60.0):
            return None
        future = client.call_async(request)
        self.spin_until(future.done, timeout)
        return future.result() if future.done() else None


class Handheld:
    """A UDP handheld (Molly protocol v1)."""

    def __init__(self, port: int, token: str):
        self.addr, self.token, self.id = ('127.0.0.1', port), token, 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.05)

    def send(self, message: dict):
        self.sock.sendto(json.dumps({'k': self.token, **message}).encode(), self.addr)

    def command(self, name: str, watcher, **args) -> dict | None:
        """Send a command until its final ack arrives."""
        self.id += 1
        end = time.monotonic() + 20.0
        while time.monotonic() < end:
            self.send({'t': 'cmd', 'id': self.id, 'name': name, 'args': args})
            watcher.spin_until(lambda: False, 0.2)
            for message in self.messages():
                if (
                    message.get('t') == 'ack'
                    and message['id'] == self.id
                    and message['ok'] is not None
                ):
                    return message
        return None

    def messages(self) -> list:
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, BlockingIOError):
                return out
            if data[:1] == b'{':
                out.append(json.loads(data))


@pytest.fixture(scope='module')
def sim():
    """Run the simulation; check the log for dead nodes and tracebacks at the end."""
    sites = tempfile.mkdtemp(prefix='molly_sites_')
    port = free_port()
    extra = os.path.join(sites, 'e2e_params.yaml')
    with open(extra, 'w', encoding='utf-8') as f:
        f.write(f'teleop_udp_node:\n  ros__parameters:\n    port: {port}\n    token: sim\n')
    log_path = os.path.join(sites, 'e2e.log')
    log = open(log_path, 'w+', encoding='utf-8')  # noqa: SIM115  (held by the launch)
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    command = ['ros2', 'launch', 'molly_bringup', 'sim.launch.py', f'sites_dir:={sites}',
               f'params_file:={extra}']  # fmt: skip
    proc = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL, stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)  # fmt: skip
    rclpy.init(domain_id=DOMAIN)
    node = rclpy.create_node('molly_e2e')
    watcher = Watcher(node)
    try:
        yield watcher, Handheld(port, 'sim')
    finally:
        node.destroy_node()
        try_shutdown()
        proc.send_signal(signal.SIGINT)  # the launch stops its nodes, once each
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        log.seek(0)
        output = log.read()
        log.close()
        problems = [
            ln for ln in output.splitlines() if 'process has died' in ln or 'Traceback' in ln
        ]
        assert not problems, '\n'.join(problems) + f'\nfull log: {log.name}'


def moved(poses, since: int) -> float:
    """Return how far the robot moved after pose number since."""
    if len(poses) <= since:
        return 0.0
    (x0, y0, _), (x1, y1, _) = poses[since], poses[-1]
    return math.hypot(x1 - x0, y1 - y0)


def test_tour_with_a_handheld_takeover(sim):
    watcher, handheld = sim
    localized = LocalizationStatus.NONE
    assert watcher.spin_until(
        lambda: watcher.localization is not None and watcher.localization.state != localized, 120.0
    ), 'not localized'
    request = TourControl.Request(command=TourCommand(kind=TourCommand.START, tour_id='demo'))
    answer = None
    for _ in range(30):  # the POIs and the control status reach the guide a moment later
        answer = watcher.call(TourControl, 'tour/control', request)
        if answer is not None and answer.success:
            break
        watcher.spin_until(lambda: False, 1.0)
    assert answer is not None and answer.success, answer.message if answer else 'no answer'
    walking = TourState.ACTIVITY_NAVIGATING
    assert watcher.spin_until(lambda: watcher.tour[-1].activity == walking and moved(
        watcher.poses, 0) > 1.0, 120.0), 'the tour does not walk'  # fmt: skip

    # the handheld takes over through the control mux
    handheld.send({'t': 'hello', 'name': 'e2e', 'streams': {}})
    ack = handheld.command('set_mode', watcher, mode='remote')
    assert ack is not None and ack['ok'], ack
    assert watcher.spin_until(
        lambda: (
            watcher.control.mode == ControlStatus.MODE_REMOTE
            and watcher.control.remote_source == 'handheld'
        ),
        10.0,
    )
    paused = NavigationStatus.PAUSED
    assert watcher.spin_until(
        lambda: watcher.nav[-1].state == paused and 'remote' in watcher.nav[-1].pause_reasons, 10.0
    ), watcher.nav[-1]
    for _ in range(10):  # neutral commands arm the source
        handheld.send({'t': 'joy', 'vx': 0.0, 'vy': 0.0, 'wz': 0.0})
        watcher.spin_until(lambda: False, 0.05)
    start = len(watcher.poses)
    for _ in range(40):
        handheld.send({'t': 'joy', 'vx': 0.2, 'vy': 0.0, 'wz': 0.0})
        watcher.spin_until(lambda: False, 0.05)
    for _ in range(10):
        handheld.send({'t': 'joy', 'vx': 0.0, 'vy': 0.0, 'wz': 0.0})
        watcher.spin_until(lambda: False, 0.05)
    assert moved(watcher.poses, start) > 0.1, 'the handheld did not drive'
    ack = handheld.command('set_mode', watcher, mode='auto')
    assert ack is not None and ack['ok'], ack
    handheld.send({'t': 'bye'})

    # back in AUTO the tour goes on to the end
    ended = watcher.spin_until(
        lambda: watcher.tour[-1].state in (TourState.IDLE, TourState.ERROR)
        and watcher.tour[-1].tour_id == '', 600.0,
    )  # fmt: skip
    states = [(s.state, s.current_poi_id, s.activity) for s in watcher.tour]
    assert ended and watcher.tour[-1].state == TourState.IDLE, states[-10:]
    narrated = {
        s.current_poi_id for s in watcher.tour if s.activity == TourState.ACTIVITY_NARRATING
    }
    assert {'east_gate', 'fountain', 'west_hall'} <= narrated, states
    dwelled = {s.current_poi_id for s in watcher.tour if s.activity == TourState.ACTIVITY_DWELLING}
    assert dwelled == {'east_gate', 'fountain', 'west_hall'}, states
    said = ' '.join(s.text for s in watcher.speech)
    assert 'east gate' in said and 'fountain' in said and 'aula barat' in said, said
    x, y, yaw = watcher.poses[-1]
    assert math.hypot(x - WEST_HALL[0], y - WEST_HALL[1]) < 0.3, watcher.poses[-1]
    assert abs(math.remainder(yaw - WEST_HALL[2], math.tau)) < math.radians(15.0)
