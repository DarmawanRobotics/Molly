"""In-process tests of the odometry gate, monitor and route recorder against fake sensor data."""

import csv
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
qos = pytest.importorskip('rclpy.qos')
Imu = pytest.importorskip('sensor_msgs.msg').Imu
Odometry = pytest.importorskip('nav_msgs.msg').Odometry
std_msgs = pytest.importorskip('std_msgs.msg')
std_srvs = pytest.importorskip('std_srvs.srv')
TransformStamped = pytest.importorskip('geometry_msgs.msg').TransformStamped
tf2_ros = pytest.importorskip('tf2_ros')
MarkStopPoint = pytest.importorskip('molly_interfaces.srv').MarkStopPoint
StillGate = pytest.importorskip('molly_odometry.wait_still').StillGate
OdometryMonitorNode = pytest.importorskip(
    'molly_odometry.odometry_monitor_node'
).OdometryMonitorNode
RouteRecorderNode = pytest.importorskip('molly_odometry.route_recorder_node').RouteRecorderNode


@pytest.fixture
def ros(request):
    """Init rclpy with test parameters; yields (executor, helper node)."""
    params = getattr(request, 'param', [])
    rclpy.init(args=['--ros-args', *[a for p in params for a in ('-p', p)]])
    helper = rclpy.create_node('odometry_test_helper')
    executor = SingleThreadedExecutor()
    executor.add_node(helper)
    yield executor, helper
    executor.shutdown()
    helper.destroy_node()
    rclpy.shutdown()


def spin(executor, seconds: float, every=None, period: float = 0.0):
    """Spin for a while, calling every() at most once per period."""
    end, last = time.monotonic() + seconds, 0.0
    while time.monotonic() < end:
        if every and time.monotonic() - last >= period:
            last = time.monotonic()
            every()
        executor.spin_once(timeout_sec=0.002)


def imu(tilt_deg: float = 0.0, gyro: float = 0.0) -> Imu:
    """Return a Livox-like IMU sample (acceleration in g)."""
    msg = Imu()
    t = math.radians(tilt_deg)
    msg.linear_acceleration.x, msg.linear_acceleration.z = math.sin(t), math.cos(t)
    msg.angular_velocity.z = gyro
    return msg


@pytest.mark.parametrize('ros', [['still_sec:=0.5', 'timeout_sec:=0.0']], indirect=True)
def test_gate_opens_when_still_and_level(ros):
    """A still, level IMU releases the gate after still_sec."""
    executor, helper = ros
    gate = StillGate()
    executor.add_node(gate)
    pub = helper.create_publisher(Imu, '/livox/imu', 50)
    spin(executor, 1.5, lambda: pub.publish(imu()), 0.005)
    assert gate.done
    executor.remove_node(gate)
    gate.destroy_node()


@pytest.mark.parametrize('ros', [['still_sec:=0.5', 'timeout_sec:=0.0']], indirect=True)
def test_gate_holds_while_tilted_or_turning(ros):
    """A sitting (tilted) or rotating robot keeps the gate closed."""
    executor, helper = ros
    gate = StillGate()
    executor.add_node(gate)
    pub = helper.create_publisher(Imu, '/livox/imu', 50)
    spin(executor, 1.2, lambda: pub.publish(imu(tilt_deg=15.0)), 0.005)
    assert not gate.done
    spin(executor, 1.2, lambda: pub.publish(imu(gyro=0.2)), 0.005)
    assert not gate.done
    executor.remove_node(gate)
    gate.destroy_node()


@pytest.mark.parametrize('ros', [['leg_odom_topic:=/test/no_leg_odom']], indirect=True)
def test_monitor_reports_ok_jump_restart_and_stale(ros):
    """Smooth motion is ok; a jump degrades; snapping back to the origin is a restart."""
    executor, helper = ros
    monitor = OdometryMonitorNode()
    executor.add_node(monitor)
    statuses, restarts = [], []
    latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
    helper.create_subscription(
        std_msgs.String, 'odometry/status', lambda m: statuses.append(m.data), latched
    )
    helper.create_subscription(
        std_msgs.Empty, 'odometry/restarted', lambda m: restarts.append(m), 10
    )
    pub = helper.create_publisher(Odometry, '/Odometry', 10)
    state = {'x': 0.0, 't': 0.0}

    def odom(x=None):
        state['t'] += 0.1
        state['x'] = state['x'] + 0.05 if x is None else x
        msg = Odometry()
        msg.header.stamp.sec = int(state['t'])
        msg.header.stamp.nanosec = int((state['t'] % 1.0) * 1e9)
        msg.pose.pose.position.x = state['x']
        msg.pose.pose.orientation.w = 1.0
        pub.publish(msg)

    spin(executor, 1.5, odom, 0.1)
    assert statuses[-1] == 'ok'
    state['t'] -= 0.1
    odom(x=state['x'])  # the same stamp again: a duplicate, not a restart
    spin(executor, 0.3)
    assert statuses[-1] == 'ok' and not restarts
    odom(x=state['x'] + 2.0)  # 2 m in one 0.1 s frame
    spin(executor, 0.3)
    assert statuses[-1].startswith('degraded: jumped')
    odom(x=0.0)  # FAST-LIO came back at the origin
    spin(executor, 0.3)
    assert restarts and statuses[-1].startswith('lost: odometry restarted')
    spin(executor, 1.0)  # nothing published any more
    assert statuses[-1].startswith('lost: no odometry')
    executor.remove_node(monitor)
    monitor.destroy_node()


def test_recorder_writes_route_with_stop_point(tmp_path):
    """Recording samples map->base_link, marks a stop and writes the CSV."""
    rclpy.init(args=['--ros-args', '-p', f'output_path:={tmp_path}', '-p', 'record_rate_hz:=20.0'])
    try:
        helper = rclpy.create_node('recorder_test_helper')
        recorder = RouteRecorderNode()
        executor = SingleThreadedExecutor()
        executor.add_node(helper)
        executor.add_node(recorder)
        broadcaster = tf2_ros.TransformBroadcaster(helper)
        saved = []
        helper.create_subscription(std_msgs.String, 'route/saved', saved.append, 10)
        record = helper.create_client(std_srvs.SetBool, 'route/record')
        mark = helper.create_client(MarkStopPoint, 'route/mark_stop_point')
        pose = {'x': 0.0}

        def move():
            pose['x'] += 0.02
            t = TransformStamped()
            t.header.stamp = helper.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = 'map', 'base_link'
            t.transform.translation.x = pose['x']
            t.transform.rotation.w = 1.0
            broadcaster.sendTransform(t)

        def call(client, request):
            future = client.call_async(request)
            end = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < end:
                move()
                executor.spin_once(timeout_sec=0.01)
            return future.result()

        assert call(record, std_srvs.SetBool.Request(data=True)).success
        spin(executor, 1.5, move, 0.01)
        assert call(mark, MarkStopPoint.Request(dwell_sec=5.0)).success
        spin(executor, 0.5, move, 0.01)
        assert call(record, std_srvs.SetBool.Request(data=False)).success
        spin(executor, 0.2)
        assert saved, 'route/saved not published'
        with open(saved[-1].data) as f:
            rows = [r for r in csv.reader(f) if r and not r[0].startswith('#')]
        assert len(rows) >= 3
        assert sum(1 for r in rows if float(r[3]) > 0.0) == 1
        xs = [float(r[0]) for r in rows]
        assert all(b >= a for a, b in zip(xs, xs[1:]))
        executor.shutdown()
        recorder.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.shutdown()
