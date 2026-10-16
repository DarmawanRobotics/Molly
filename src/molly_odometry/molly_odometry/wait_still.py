#!/usr/bin/env python3
"""
Launch prefix: wait until the IMU is still and level, then exec the real command.

FAST-LIO estimates gravity and the IMU biases from its first ~10 IMU samples and assumes the robot
is not moving. Starting it while the robot is standing up or being carried bakes a tilted gravity
into the whole run. Used as a launch prefix, so a respawned FAST-LIO waits again:

    wait_still [--params-file odometry_params.yaml] -- fastlio_mapping --ros-args ...
"""

from collections import deque
import os
import sys
import time

from molly_odometry.checks import stillness
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class StillGate(Node):
    """Collect IMU samples and decide when the robot is still enough to start odometry."""

    def __init__(self):
        """Declare parameters and subscribe to the IMU."""
        super().__init__('odometry_gate')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('still_sec', 2.0)
        self.declare_parameter('max_gyro_rad_s', 0.03)
        self.declare_parameter('max_acc_rel_std', 0.01)
        self.declare_parameter('max_tilt_deg', 8.0)
        self.declare_parameter('timeout_sec', 60.0)

        p = self.get_parameter
        self._still_sec = float(p('still_sec').value)
        self._max_gyro = float(p('max_gyro_rad_s').value)
        self._max_acc_std = float(p('max_acc_rel_std').value)
        self._max_tilt = float(p('max_tilt_deg').value)
        self._timeout = float(p('timeout_sec').value)

        self.done = False
        self._start = time.monotonic()
        self._last_log = 0.0
        self._samples = deque()
        topic = p('imu_topic').value
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.2, self.evaluate)
        self.get_logger().info(
            f'waiting for {self._still_sec:.1f} s of stillness on {topic} before starting odometry'
        )

    def on_imu(self, msg: Imu):
        """Keep the samples of the last still_sec seconds."""
        now = time.monotonic()
        w, a = msg.angular_velocity, msg.linear_acceleration
        self._samples.append((now, (w.x, w.y, w.z), (a.x, a.y, a.z)))
        while self._samples and now - self._samples[0][0] > self._still_sec:
            self._samples.popleft()

    def evaluate(self):
        """Release the gate when the window is still and level, or when the timeout passes."""
        if self.done:
            return
        now = time.monotonic()
        elapsed = now - self._start
        window_full = (
            len(self._samples) >= 20
            and now - self._samples[0][0] >= 0.9 * self._still_sec
        )
        reason = 'no IMU data yet'
        if window_full:
            gyro = np.array([s[1] for s in self._samples])
            acc = np.array([s[2] for s in self._samples])
            gyro_max, acc_std, tilt = stillness(gyro, acc)
            problems = []
            if gyro_max > self._max_gyro:
                problems.append(f'rotating {gyro_max:.3f} rad/s')
            if acc_std > self._max_acc_std:
                problems.append(f'shaking {acc_std * 100:.1f} %')
            if self._max_tilt > 0.0 and tilt > self._max_tilt:
                problems.append(f'tilted {tilt:.1f} deg (sitting?)')
            if not problems:
                self.get_logger().info(
                    f'IMU still (gyro {gyro_max:.3f} rad/s, acc {acc_std * 100:.2f} %, '
                    f'tilt {tilt:.1f} deg) after {elapsed:.1f} s, starting odometry'
                )
                self.done = True
                return
            reason = ', '.join(problems)
        if self._timeout > 0.0 and elapsed > self._timeout:
            self.get_logger().error(
                f'not still after {self._timeout:.0f} s ({reason}), starting odometry anyway; '
                'expect drift, restart it with the robot standing still'
            )
            self.done = True
            return
        if now - self._last_log > 5.0:
            self._last_log = now
            self.get_logger().warn(f'odometry start held: {reason}')


def split_args(argv: list[str]) -> tuple[list[str], list[str]]:
    """Split `[--params-file F]... -- command...` into (rclpy args, command)."""
    if '--' not in argv:
        raise SystemExit('usage: wait_still [--params-file FILE]... -- COMMAND [ARGS...]')
    i = argv.index('--')
    own, command = argv[:i], argv[i + 1:]
    if not command:
        raise SystemExit('wait_still: no command after --')
    files = [own[k + 1] for k in range(0, len(own) - 1, 2) if own[k] == '--params-file']
    files = [f for f in files if f]
    if not files:
        return [], command
    return ['--ros-args', *[a for f in files for a in ('--params-file', f)]], command


def main(argv=None):
    """Wait for stillness, then replace this process with the command."""
    ros_args, command = split_args(sys.argv[1:] if argv is None else argv)
    # Only our own args: the command's --ros-args (node name, params) must not reach the gate.
    rclpy.init(args=['wait_still', *ros_args])
    node = StillGate()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass
    if not node.done:
        return 1
    sys.stdout.flush()
    sys.stderr.flush()
    os.execvp(command[0], command)


if __name__ == '__main__':
    sys.exit(main())
