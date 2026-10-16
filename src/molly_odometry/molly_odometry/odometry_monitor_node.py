#!/usr/bin/env python3
"""Watch FAST-LIO and report when its odometry can no longer be trusted."""

import math
import time

from molly_odometry.checks import looks_like_restart, yaw_from_quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String


class OdometryMonitorNode(Node):
    """
    Publish odometry/status and odometry/restarted from FAST-LIO (/Odometry) and leg odometry.

    Status: initializing | ok | degraded: <why> | lost: <why>
    """

    def __init__(self):
        """Declare parameters, subscribe and start the watchdog."""
        super().__init__('odometry_monitor_node')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('leg_odom_topic', '/robot/odom')   # empty: no cross-check
        self.declare_parameter('stale_sec', 0.5)
        self.declare_parameter('max_speed_mps', 3.0)
        self.declare_parameter('max_z_drift_m', 0.5)            # 0: off (ramps, other floors)
        self.declare_parameter('max_speed_diff_mps', 0.35)
        self.declare_parameter('speed_diff_sec', 1.5)
        self.declare_parameter('min_check_speed_mps', 0.1)
        self.declare_parameter('restart_origin_radius_m', 0.05)
        self.declare_parameter('restart_min_offset_m', 0.3)

        p = self.get_parameter
        self._stale = float(p('stale_sec').value)
        self._max_speed = float(p('max_speed_mps').value)
        self._max_z = float(p('max_z_drift_m').value)
        self._max_diff = float(p('max_speed_diff_mps').value)
        self._diff_sec = float(p('speed_diff_sec').value)
        self._min_check = float(p('min_check_speed_mps').value)
        self._origin_r = float(p('restart_origin_radius_m').value)
        self._min_offset = float(p('restart_min_offset_m').value)

        self._last = None            # (stamp_sec, x, y, z, yaw)
        self._last_rx = None         # monotonic receipt time of the last /Odometry
        self._lio_speed = 0.0        # smoothed planar speed
        self._leg_speed = None
        self._leg_rx = 0.0
        self._diff_since = None
        self._event = None           # (message, until monotonic) for one-off problems
        self._status = None

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, 'odometry/status', latched)
        self._restart_pub = self.create_publisher(Empty, 'odometry/restarted', 10)
        self.create_subscription(Odometry, p('odom_topic').value, self.on_odometry, 20)
        leg_topic = p('leg_odom_topic').value
        if leg_topic:
            self.create_subscription(Odometry, leg_topic, self.on_leg_odometry, 20)
        self.create_timer(0.2, self.update_status)
        self.publish_status('initializing')

    def on_leg_odometry(self, msg: Odometry):
        """Keep the planar body speed reported by the robot (wheels + legs)."""
        v = msg.twist.twist.linear
        self._leg_speed = math.hypot(v.x, v.y)
        self._leg_rx = time.monotonic()

    def on_odometry(self, msg: Odometry):
        """Check every FAST-LIO pose for restarts, jumps, z drift and speed mismatch."""
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        current = (stamp, pos.x, pos.y, pos.z, yaw)
        self._last_rx = now

        if self._last is not None:
            t0, x0, y0, _, yaw0 = self._last
            dt = stamp - t0
            if dt == 0.0:  # the same pose again (republished), nothing new to check
                return
            step = math.hypot(pos.x - x0, pos.y - y0)
            restarted = dt < 0.0 or looks_like_restart(
                math.hypot(x0, y0), yaw0, math.hypot(pos.x, pos.y), yaw,
                self._origin_r, self._min_offset,
            )
            if restarted:
                self.get_logger().error('odometry restarted: odom frame reset, map->odom invalid')
                self._restart_pub.publish(Empty())
                self.flag('lost: odometry restarted, relocalize', 5.0)
                self._lio_speed, self._diff_since = 0.0, None
            elif dt < 0.5:
                speed = step / dt
                if speed > self._max_speed:
                    self.get_logger().error(f'odometry jumped {step:.2f} m in {dt:.2f} s')
                    self.flag(f'degraded: jumped {step:.2f} m', 5.0)
                else:
                    self._lio_speed += 0.3 * (speed - self._lio_speed)
                    self.check_speed_mismatch(now)
        self._last = current

    def check_speed_mismatch(self, now: float):
        """Track how long FAST-LIO and the robot's own speed disagree."""
        fresh = self._leg_speed is not None and now - self._leg_rx < 0.3
        moving = fresh and max(self._lio_speed, self._leg_speed) > self._min_check
        if moving and abs(self._lio_speed - self._leg_speed) > self._max_diff:
            if self._diff_since is None:
                self._diff_since = now
        else:
            self._diff_since = None

    def flag(self, message: str, hold_sec: float):
        """Show a one-off problem for hold_sec seconds."""
        self._event = (message, time.monotonic() + hold_sec)
        self.update_status()

    def update_status(self):
        """Derive the status from the latest checks; publish only when it changes."""
        now = time.monotonic()
        if self._last_rx is None:
            status = 'initializing'
        elif now - self._last_rx > self._stale:
            status = f'lost: no odometry for {now - self._last_rx:.1f} s'
        elif self._event is not None and now < self._event[1]:
            status = self._event[0]
        elif self._max_z > 0.0 and self._last is not None and abs(self._last[3]) > self._max_z:
            status = f'degraded: z drift {self._last[3]:+.2f} m (restart standing still)'
        elif self._diff_since is not None and now - self._diff_since > self._diff_sec:
            status = (
                f'degraded: LIO {self._lio_speed:.2f} m/s vs robot {self._leg_speed:.2f} m/s'
            )
        else:
            status = 'ok'
        self.publish_status(status)

    def publish_status(self, status: str):
        """Publish the status (latched) and log transitions."""
        if status == self._status:
            return
        if self._status is not None:
            # one call site per severity: rclpy refuses to change it between calls
            if status == 'ok':
                self.get_logger().info(f'odometry: {status}')
            else:
                self.get_logger().warn(f'odometry: {status}')
        self._status = status
        self._status_pub.publish(String(data=status))


def main(args=None):
    """Spin the odometry monitor."""
    rclpy.init(args=args)
    node = OdometryMonitorNode()
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
