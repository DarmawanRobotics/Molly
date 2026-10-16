"""In-process tests of control_mux_node: e-stop, forwarding and the protective field."""

import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
Twist = pytest.importorskip('geometry_msgs.msg').Twist
std_msgs = pytest.importorskip('std_msgs.msg')
SetBool = pytest.importorskip('std_srvs.srv').SetBool
SectorScan = pytest.importorskip('molly_interfaces.msg').SectorScan
SetMode = pytest.importorskip('molly_interfaces.srv').SetMode
qos = pytest.importorskip('rclpy.qos')
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
ControlMuxNode = pytest.importorskip('molly_bridge.control_mux_node').ControlMuxNode


class Rig:
    """The mux, a fake navigation pause service and recorders for its outputs."""

    def __init__(self):
        rclpy.init()
        self.mux = ControlMuxNode()
        self.helper = rclpy.create_node('mux_test_helper')
        self.out, self.engaged, self.pauses = [], [], []
        self.helper.create_subscription(Twist, 'cmd_vel', self.out.append, 50)
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.helper.create_subscription(
            std_msgs.Bool, 'bridge/estop_engaged', lambda m: self.engaged.append(m.data), latched
        )
        self.helper.create_service(SetBool, 'navigation/pause', self.on_pause)
        self.nav = self.helper.create_publisher(Twist, 'bridge/nav_cmd_vel', 10)
        self.remote = self.helper.create_publisher(Twist, 'bridge/remote_cmd_vel', 10)
        self.set_mode = self.helper.create_client(SetMode, 'bridge/set_mode')
        self.request = self.helper.create_publisher(std_msgs.String, 'bridge/estop_request', 10)
        self.estop = self.helper.create_client(SetBool, 'bridge/estop')
        self.field_states = []
        self.helper.create_subscription(
            std_msgs.String,
            'bridge/protective_field',
            lambda m: self.field_states.append(m.data),
            latched,
        )
        self.scan_pub = self.helper.create_publisher(SectorScan, 'perception/safety_scan', 10)
        self.obstacles = {}  # sector index -> range
        self.scanning = True
        self.helper.create_timer(0.1, self.publish_scan)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mux)
        self.executor.add_node(self.helper)

    def publish_scan(self):
        """Publish a 72-sector safety scan with the configured obstacles."""
        if not self.scanning:
            return
        msg = SectorScan()
        msg.num_sectors = 72
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / 72
        msg.range_max = 4.0
        msg.ranges = [self.obstacles.get(i, 4.0) for i in range(72)]
        self.scan_pub.publish(msg)

    def on_pause(self, request, response):
        """Record navigation/pause calls."""
        self.pauses.append(request.data)
        response.success = True
        return response

    def spin(self, seconds: float):
        """Spin both nodes."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def call(self, data: bool):
        """Call bridge/estop and wait for the answer."""
        return self.wait(self.estop.call_async(SetBool.Request(data=data)))

    def mode(self, mode: str):
        """Call bridge/set_mode and wait for the answer."""
        return self.wait(self.set_mode.call_async(SetMode.Request(mode=mode)))

    def wait(self, future):
        """Spin until a service answer arrives (3 s at most)."""
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
        return future.result()

    def drive(self, vx: float, seconds: float = 0.3, vy: float = 0.0):
        """Publish a navigation command and spin."""
        cmd = Twist()
        cmd.linear.x, cmd.linear.y = vx, vy
        self.nav.publish(cmd)
        self.spin(seconds)

    def close(self):
        """Tear down."""
        self.executor.shutdown()
        self.helper.destroy_node()
        self.mux.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig():
    """Provide a running mux."""
    r = Rig()
    r.spin(0.5)  # discovery
    yield r
    r.close()


def test_estop_service_holds_the_robot_still(rig):
    """Engaged: nav is blocked, zero is streamed, navigation paused; release lets nav through."""
    rig.drive(0.4)
    assert rig.out and rig.out[-1].linear.x == 0.4
    assert rig.call(True).success
    rig.spin(0.3)
    assert rig.engaged[-1] is True and rig.pauses == [True]
    rig.out.clear()
    rig.drive(0.4, 0.5)
    assert rig.out and all(t.linear.x == 0.0 for t in rig.out)  # only the hold stream
    assert rig.call(False).success
    rig.spin(0.2)
    assert rig.engaged[-1] is False and rig.pauses == [True]  # release never resumes
    rig.drive(0.3)
    assert rig.out[-1].linear.x == 0.3


def test_estop_request_topic_engages(rig):
    """Any node can engage the e-stop with a reason on bridge/estop_request."""
    rig.request.publish(std_msgs.String(data='unit test'))
    rig.spin(0.3)
    assert rig.engaged[-1] is True
    assert rig.mux._mux.estop_reason == 'unit test'


def test_field_stops_forward_motion_toward_an_obstacle(rig):
    """An obstacle 0.5 m ahead stops forward motion, sideways motion still passes."""
    rig.obstacles = {36: 0.5}  # sector 36 is straight ahead (0 deg)
    rig.spin(0.3)
    rig.drive(0.4)
    assert rig.out[-1].linear.x == 0.0 and rig.field_states[-1] == 'stop'
    rig.drive(0.0, vy=0.3)
    assert abs(rig.out[-1].linear.y - 0.3) < 1e-6
    rig.obstacles = {}
    rig.spin(0.3)
    rig.drive(0.4)
    assert abs(rig.out[-1].linear.x - 0.4) < 1e-6 and rig.field_states[-1] == 'clear'


def test_remote_is_not_limited_by_the_field(rig):
    """REMOTE: an obstacle ahead limits nothing, the operator watches the robot."""
    rig.obstacles = {36: 0.5}
    rig.spin(0.3)
    assert rig.mode('remote').success
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = 0.4, 0.8
    rig.remote.publish(cmd)
    rig.spin(0.15)  # within remote_timeout_sec, before the watchdog zeroes the robot
    moving = [t for t in rig.out if t.linear.x > 0.0]
    assert moving and abs(moving[-1].linear.x - 0.4) < 1e-6
    assert abs(moving[-1].angular.z - 0.8) < 1e-6
    assert rig.field_states[-1] == 'off'


def test_field_stops_navigation_without_a_fresh_scan(rig):
    """No safety scan: navigation never drives blind."""
    rig.scanning = False
    rig.spin(0.8)
    rig.drive(0.4)
    assert rig.out[-1].linear.x == 0.0 and rig.field_states[-1] == 'stale'


def test_non_finite_commands_never_reach_the_robot(rig):
    """A NaN or inf from any source becomes zero, even with nothing in the way."""
    rig.drive(0.3)
    assert abs(rig.out[-1].linear.x - 0.3) < 1e-6
    for bad in (float('nan'), float('inf')):
        rig.drive(bad)
        out = rig.out[-1]
        assert (out.linear.x, out.linear.y, out.angular.z) == (0.0, 0.0, 0.0)
    cmd = Twist()
    cmd.linear.x, cmd.angular.y = 0.3, float('nan')  # a field the gate does not limit
    rig.nav.publish(cmd)
    rig.spin(0.3)
    assert rig.out[-1].linear.x == 0.0


def test_commands_are_capped_at_the_speed_envelope(rig):
    """2 m/s forward + 1 m/s left becomes 1.0 / 0.5 m/s: the same heading, within the limits."""
    rig.drive(2.0, vy=1.0)
    out = rig.out[-1]
    assert abs(out.linear.x - 1.0) < 1e-6 and abs(out.linear.y - 0.5) < 1e-6
