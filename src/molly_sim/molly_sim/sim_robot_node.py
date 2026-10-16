#!/usr/bin/env python3
"""
Software-in-the-loop quadruped: replaces the robot driver, LiDAR and FAST-LIO for the real stack.

Consumes /<ns>/cmd_vel like the driver and publishes what the stack expects from the robot:
FAST-LIO's /Odometry, camera_init->body and /cloud_registered_body, the URDF transforms of the
LiDAR, /<ns>/odom and /<ns>/battery. It localizes the stack once with /initialpose at the world's
start pose. People or boxes can be dropped into the world on sim/obstacles at runtime, and
sim/inspect stands in for the VLM inspection at stop points.
"""

import math
import time

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from molly_sim.world import Body, lio_pose, scan_points, World
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def quaternion_z(yaw: float):
    """Return (x, y, z, w) of a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SimRobotNode(Node):
    """Kinematic quadruped with a ray-cast Mid-360 in a 2D world."""

    def __init__(self):
        """Load the world, declare parameters and start the simulation timers."""
        super().__init__('sim_robot_node')
        defaults = {
            'world_file': '',
            'robot_ns': 'robot',
            'rate_hz': 50.0,
            'odom_rate_hz': 20.0,
            'lidar_rate_hz': 10.0,
            'lidar_rays': 360,
            'lidar_range': 12.0,
            'lidar_heights': [-0.1, 0.2, 0.6],
            'lidar_noise': 0.01,
            'lidar_xyz': [0.13011, -0.02329, 0.17598],     # base_link -> livox_frame (URDF)
            'imu_in_lidar': [-0.011, -0.02329, 0.04412],   # livox_frame -> livox_imu_frame
            'cmd_timeout_sec': 0.5,
            'accel': 1.0,
            'yaw_accel': 2.0,
            'obstacle_radius': 0.25,
            'battery_start': 0.9,
            'battery_drain_per_hour': 0.2,
            'autostart_nav': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        path = p('world_file')
        self._world = World.from_yaml(path) if path else World()
        sx, sy, syaw = self._world.start
        self._body = Body(sx, sy, syaw, accel=float(p('accel')), yaw_accel=float(p('yaw_accel')))
        self._ns = p('robot_ns').strip('/')
        self._lidar = [float(v) for v in p('lidar_xyz')]
        imu = [float(v) for v in p('imu_in_lidar')]
        self._imu = [self._lidar[i] + imu[i] for i in range(3)]  # base_link -> imu (= body)
        self._rays, self._range = int(p('lidar_rays')), float(p('lidar_range'))
        self._heights = [float(h) for h in p('lidar_heights')]
        self._noise = float(p('lidar_noise'))
        self._cmd_timeout = float(p('cmd_timeout_sec'))
        self._obstacle_radius = float(p('obstacle_radius'))
        self._battery = float(p('battery_start'))
        self._drain = float(p('battery_drain_per_hour')) / 3600.0
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = 0.0
        self._dynamic = np.empty((0, 3))
        self._localized = False
        self._rng = np.random.default_rng(0)
        self._dt = 1.0 / float(p('rate_hz'))
        self._autostart = bool(p('autostart_nav'))
        self._start_nav = self.create_client(SetBool, 'navigation/start_nav')
        self._start_pending = False

        topic = lambda name: f'/{self._ns}/{name}'  # noqa: E731
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._lio_pub = self.create_publisher(Odometry, '/Odometry', 20)
        self._cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_body', 5)
        self._odom_pub = self.create_publisher(Odometry, topic('odom'), 20)
        self._battery_pub = self.create_publisher(BatteryState, topic('battery'), 5)
        self._truth_pub = self.create_publisher(PoseStamped, 'sim/ground_truth', 10)
        self._initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, topic('cmd_vel'), self.on_cmd, 10)
        self.create_subscription(PoseArray, 'sim/obstacles', self.on_obstacles, 10)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        self.create_service(Trigger, 'sim/inspect', self.on_inspect)

        self.publish_static()
        self.create_timer(self._dt, self.step)
        self.create_timer(1.0 / float(p('odom_rate_hz')), self.publish_odometry)
        self.create_timer(1.0 / float(p('lidar_rate_hz')), self.publish_cloud)
        self.create_timer(1.0, self.publish_slow)
        self.get_logger().info(
            f'sim ready: {len(self._world.segments)} wall segments, '
            f'{len(self._world.circles)} pillars, start {sx:.1f} {sy:.1f} {math.degrees(syaw):.0f}'
        )

    # ------------------------------------------------------------------ inputs

    def on_cmd(self, msg: Twist):
        """Take a velocity command like the robot driver does."""
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd = tuple(v if math.isfinite(v) else 0.0 for v in values)
        self._cmd_time = time.monotonic()

    def on_obstacles(self, msg: PoseArray):
        """Replace the dynamic obstacles (people, boxes) with circles at these map positions."""
        self._dynamic = np.array(
            [(p.position.x, p.position.y, self._obstacle_radius) for p in msg.poses]
        ).reshape(-1, 3)
        self.get_logger().info(f'{len(msg.poses)} dynamic obstacle(s)')

    def on_localization(self, msg: String):
        """Stop sending /initialpose once the stack is localized."""
        self._localized = msg.data not in ('', 'none')

    def on_inspect(self, request, response):
        """Answer a stop-point inspection like anomaly_detector_node/check_now does."""
        b = self._body
        response.success = True
        response.message = f'clear (sim at {b.x:.2f} {b.y:.2f})'
        return response

    def autostart(self):
        """Start the patrol once the stack is localized (autostart_nav), retrying until it is."""
        if not self._autostart or not self._localized or self._start_pending:
            return
        if not self._start_nav.service_is_ready():
            return
        self._start_pending = True
        future = self._start_nav.call_async(SetBool.Request(data=True))
        future.add_done_callback(self.autostart_done)

    def autostart_done(self, future):
        """Stop retrying after navigation accepted the start."""
        self._start_pending = False
        result = future.result()
        if result is not None and result.success:
            self._autostart = False
            self.get_logger().info('patrol started (autostart_nav)')

    # ------------------------------------------------------------------ simulation

    def step(self):
        """Advance the robot; a stale command means stop, like the real driver."""
        fresh = time.monotonic() - self._cmd_time < self._cmd_timeout
        self._body.step(self._cmd if fresh else (0.0, 0.0, 0.0), self._dt)
        self._battery = max(0.0, self._battery - self._drain * self._dt)

    def lio_pose(self):
        """Return the FAST-LIO pose camera_init->body (x, y, yaw) for the current base pose."""
        b = self._body
        return lio_pose(self._world.start, (b.x, b.y, b.yaw), (self._imu[0], self._imu[1]))

    def publish_static(self):
        """Publish the URDF transforms the stack needs (base_link -> LiDAR -> IMU)."""
        stamp = self.get_clock().now().to_msg()
        imu_in_lidar = [self._imu[i] - self._lidar[i] for i in range(3)]
        out = []
        for parent, child, xyz in (
            ('base_link', 'livox_frame', self._lidar),
            ('livox_frame', 'livox_imu_frame', imu_in_lidar),
        ):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
            t.transform.translation.x, t.transform.translation.y = xyz[0], xyz[1]
            t.transform.translation.z = xyz[2]
            t.transform.rotation.w = 1.0
            out.append(t)
        self._static_tf.sendTransform(out)

    def publish_odometry(self):
        """Publish FAST-LIO's odometry + TF, the robot's own odometry and the ground truth."""
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.lio_pose()
        qx, qy, qz, qw = quaternion_z(yaw)
        lio = Odometry()
        lio.header.stamp, lio.header.frame_id, lio.child_frame_id = stamp, 'camera_init', 'body'
        lio.pose.pose.position.x, lio.pose.pose.position.y = x, y
        lio.pose.pose.position.z = 0.0
        o = lio.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self._lio_pub.publish(lio)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'camera_init', 'body'
        t.transform.translation.x, t.transform.translation.y = x, y
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = qx, qy, qz, qw
        self._tf.sendTransform(t)

        b = self._body
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = b.vx, b.vy
        odom.twist.twist.angular.z = b.wz
        self._odom_pub.publish(odom)

        truth = PoseStamped()
        truth.header.stamp, truth.header.frame_id = stamp, 'map'
        truth.pose.position.x, truth.pose.position.y = b.x, b.y
        qx, qy, qz, qw = quaternion_z(b.yaw)
        o = truth.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self._truth_pub.publish(truth)

    def publish_cloud(self):
        """Ray-cast the LiDAR and publish the scan in the body (IMU) frame."""
        points = scan_points(
            self._world,
            self._body,
            (self._lidar[0], self._lidar[1]),
            self._rays,
            self._range,
            self._heights,
            self._noise,
            self._rng,
            self._dynamic,
        )
        points -= np.array(self._imu, dtype=np.float32)  # base_link -> body (pure offset)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body'
        msg.height, msg.width = 1, points.shape[0]
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate('xyz')
        ]
        msg.point_step, msg.row_step = 12, 12 * points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()
        self._cloud_pub.publish(msg)

    def publish_slow(self):
        """Battery once a second; /initialpose until the stack is localized; autostart."""
        self.autostart()
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.percentage = float(self._battery)
        battery.voltage = 42.0 + 8.0 * float(self._battery)
        battery.present = True
        self._battery_pub.publish(battery)
        if not self._localized:
            sx, sy, syaw = self._world.start
            pose = PoseWithCovarianceStamped()
            pose.header.stamp, pose.header.frame_id = battery.header.stamp, 'map'
            pose.pose.pose.position.x, pose.pose.pose.position.y = sx, sy
            qx, qy, qz, qw = quaternion_z(syaw)
            o = pose.pose.pose.orientation
            o.x, o.y, o.z, o.w = qx, qy, qz, qw
            self._initial_pub.publish(pose)


def main(args=None):
    """Spin the simulator."""
    rclpy.init(args=args)
    node = SimRobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
