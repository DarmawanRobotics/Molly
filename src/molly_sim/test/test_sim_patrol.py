"""
End to end: the whole stack patrols the simulated mall.

Starts `ros2 launch molly_sim sim.launch.py autostart:=true` (needs the workspace installed, as in
`colcon test`). A person steps onto the route: the robot slides around them without touching them
or straying far, and completes the inspection stop. A crowd blocks the corridor: it stops short
and drives on once they are gone. Then the handheld drives straight at a person, where only the
protective field stands in the way. Finally no node may have died or printed a traceback. Set
MOLLY_SKIP_SIM=1 to skip it.
"""

import math
import os
import signal
import subprocess
import tempfile
import time

import pytest

rclpy = pytest.importorskip('rclpy')
qos = pytest.importorskip('rclpy.qos')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
std_msgs = pytest.importorskip('std_msgs.msg')
StopPointEvent = pytest.importorskip('molly_interfaces.msg').StopPointEvent
SetMode = pytest.importorskip('molly_interfaces.srv').SetMode

pytestmark = pytest.mark.skipif(
    os.environ.get('MOLLY_SKIP_SIM') == '1', reason='MOLLY_SKIP_SIM=1'
)
DOMAIN = 70 + os.getpid() % 30
PERSON_RADIUS = 0.25
ROBOT_FRONT = 0.40
ROBOT_HALF_WIDTH = 0.28
MAX_OFFSET = 0.8      # path_shift.max_offset
ROUTE_Y = -3.0        # the bottom corridor of the mall_loop route


class Recorder:
    """Collects what the stack reports while the simulation runs."""

    def __init__(self, node):
        """Subscribe to stop events, the ground truth and the protective field."""
        self.node = node
        self.events, self.poses, self.field = [], [], []
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            StopPointEvent, 'navigation/stop_point_event', self.events.append, 10
        )
        node.create_subscription(
            geometry_msgs.PoseStamped, 'sim/ground_truth', self.on_pose, 10
        )
        node.create_subscription(
            std_msgs.String,
            'bridge/protective_field',
            lambda m: self.field.append(m.data),
            latched,
        )
        self.obstacles = node.create_publisher(geometry_msgs.PoseArray, 'sim/obstacles', 10)
        self.remote = node.create_publisher(geometry_msgs.Twist, 'bridge/remote_cmd_vel', 10)
        self.mode_client = node.create_client(SetMode, 'bridge/set_mode')

    def on_pose(self, msg):
        """Keep (time, x, y, yaw) of the true robot pose."""
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.poses.append((time.monotonic(), msg.pose.position.x, msg.pose.position.y, yaw))

    def spin_until(self, condition, timeout: float) -> bool:
        """Spin until condition() is true or the timeout passes."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return True
        return False

    def place(self, points):
        """Put people (circles) at these map positions; [] clears them."""
        msg = geometry_msgs.PoseArray()
        for x, y in points:
            pose = geometry_msgs.Pose()
            pose.position.x, pose.position.y = x, y
            msg.poses.append(pose)
        self.obstacles.publish(msg)

    def set_mode(self, mode: str) -> bool:
        """Switch the control mux to auto or remote, like the handheld does."""
        if not self.mode_client.wait_for_service(timeout_sec=10.0):
            return False
        future = self.mode_client.call_async(SetMode.Request(mode=mode))
        self.spin_until(future.done, 10.0)
        return future.done() and future.result().success

    def drive_remote(self, vx: float, seconds: float):
        """Hold the handheld stick: send a forward command at 20 Hz for this long."""
        cmd = geometry_msgs.Twist()
        cmd.linear.x = vx
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.remote.publish(cmd)
            self.spin_until(lambda: False, 0.05)


def default_sigint():
    """Give the launch the default SIGINT action, even if this test runs as a background job."""
    signal.signal(signal.SIGINT, signal.SIG_DFL)


def session_processes(sid: int) -> str:
    """List the processes of session sid as 'pid state command' lines, read from /proc."""
    rows = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/stat') as f:
                stat = f.read()
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:  # gone meanwhile
            continue
        fields = stat[stat.rfind(')') + 2:].split()  # state ppid pgrp session ...
        if len(fields) > 3 and fields[3] == str(sid):
            rows.append(f'{entry} {fields[0]} {cmd[:160]}')
    return '\n'.join(rows) or '(none)'


@pytest.fixture(scope='module')
def sim():
    """Run the simulation for the module; yields the recorder and checks the logs at the end."""
    log = tempfile.NamedTemporaryFile('w+', prefix='molly_sim_', suffix='.log', delete=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    proc = subprocess.Popen(
        ['ros2', 'launch', 'molly_sim', 'sim.launch.py', 'autostart:=true'],
        env=env,
        stdin=subprocess.DEVNULL,  # non-interactive: launch itself stops every node, like systemd
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        preexec_fn=default_sigint,
    )
    rclpy.init(domain_id=DOMAIN)
    node = rclpy.create_node('sim_e2e_test')
    recorder = Recorder(node)
    try:
        yield recorder
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # SIGINT to ros2 launch only, as `systemctl stop` does (KillMode=mixed); signalling the
        # whole group as well would interrupt every node twice
        os.kill(proc.pid, signal.SIGINT)
        still = ''
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            still = session_processes(proc.pid)  # what keeps the launch from stopping
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        log.seek(0)
        output = log.read()
        log.close()
        problems = [
            line for line in output.splitlines()
            if 'process has died' in line or 'Traceback' in line
        ]
        tail = '\n'.join(output.splitlines()[-40:])
        assert not still, (
            f'ros2 launch did not stop within 30 s of SIGINT; still running:\n{still}\n'
            f'end of the log:\n{tail}\nfull log: {log.name}'
        )
        assert not problems, '\n'.join(problems) + f'\nfull log: {log.name}'


def test_patrol_slides_around_a_person_and_inspects(sim):
    """A person on the route is passed sideways within max_offset; the stop is still done."""
    assert sim.spin_until(lambda: len(sim.poses) > 10, 60.0), 'simulator not publishing'
    start_x = sim.poses[-1][1]
    assert sim.spin_until(lambda: sim.poses[-1][1] > start_x + 0.2, 60.0), 'patrol did not start'

    # someone stands on the route between the robot and the stop point (x = 0.1)
    px, py = -2.0, ROUTE_Y
    sim.place([(px, py)])
    start = len(sim.poses)
    assert sim.spin_until(
        lambda: any(e.event == 'departed' for e in sim.events), 90.0
    ), f'stop point not completed, events: {[(e.index, e.event) for e in sim.events]}'
    track = sim.poses[start:]
    closest = min(math.hypot(p[1] - px, p[2] - py) for p in track)
    assert closest > PERSON_RADIUS + ROBOT_HALF_WIDTH, f'robot came within {closest:.2f} m'
    deviation = max(abs(p[2] - ROUTE_Y) for p in track if p[1] < 0.0)
    assert deviation > 0.3, 'robot did not move aside (was the person on the route?)'
    assert deviation < MAX_OFFSET + 0.1, f'robot strayed {deviation:.2f} m from the route'

    assert [e.event for e in sim.events][:3] == ['arrived', 'inspected', 'departed']
    assert 'timeout' not in sim.events[0].message, sim.events[0].message
    assert sim.events[1].message.startswith('/sim/inspect: ok clear'), sim.events[1].message
    sim.place([])


def test_patrol_stops_for_a_crowd_and_drives_on(sim):
    """People across the whole corridor: no gap within max_offset, the robot stops short."""
    _, x, _, _ = sim.poses[-1]
    crowd = [(x + 2.2, y) for y in (-4.2, -3.6, -3.0, -2.4, -1.8)]
    sim.place(crowd)
    start = len(sim.poses)
    sim.spin_until(lambda: False, 10.0)
    track = sim.poses[start:]
    closest = min(math.hypot(p[1] - cx, p[2] - cy) for p in track for cx, cy in crowd)
    assert closest > PERSON_RADIUS + ROBOT_FRONT, f'robot came within {closest:.2f} m'
    speed = math.hypot(track[-1][1] - track[-20][1], track[-1][2] - track[-20][2])
    assert speed < 0.05, 'robot still moving toward the crowd'

    # they walk away, the patrol goes on
    sim.place([])
    _, x0, y0, _ = sim.poses[-1]
    assert sim.spin_until(
        lambda: math.hypot(sim.poses[-1][1] - x0, sim.poses[-1][2] - y0) > 0.5, 15.0
    ), 'robot did not drive on after the crowd left'


def test_handheld_cannot_drive_into_a_person(sim):
    """With field_in_remote (sim_site.yaml) the protective field keeps the stick off a person."""
    assert sim.set_mode('remote'), 'control mux did not switch to remote'
    sim.spin_until(lambda: False, 1.5)  # navigation pauses, the robot comes to rest

    # a person 1.5 m straight ahead (the bottom corridor is clear that far), stick full forward
    _, x, y, yaw = sim.poses[-1]
    px, py = x + 1.5 * math.cos(yaw), y + 1.5 * math.sin(yaw)
    sim.place([(px, py)])
    sim.spin_until(lambda: False, 0.5)
    start = len(sim.poses)
    sim.drive_remote(0.4, 8.0)

    track = sim.poses[start:]
    moved = math.hypot(track[-1][1] - track[0][1], track[-1][2] - track[0][2])
    closest = min(math.hypot(p[1] - px, p[2] - py) for p in track)
    assert moved > 0.3, f'the stick did not move the robot ({moved:.2f} m)'
    assert closest > PERSON_RADIUS + ROBOT_FRONT, f'robot came within {closest:.2f} m'
    assert 'stop' in sim.field, f'protective field never stopped the robot: {sim.field}'
    speed = math.hypot(track[-1][1] - track[-20][1], track[-1][2] - track[-20][2])
    assert speed < 0.05, 'robot still creeping toward the person'

    sim.place([])
    assert sim.set_mode('auto'), 'control mux did not switch back to auto'
