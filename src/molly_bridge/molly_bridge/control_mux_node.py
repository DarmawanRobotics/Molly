#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import Twist
from molly_bridge import protective_field
from molly_bridge.mux import ControlMux, envelope, MODES
from molly_interfaces.msg import SectorScan
from molly_interfaces.srv import SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class ControlMuxNode(Node):
    """
    The single gate between every velocity source and the robot.

    Forwards navigation (auto) or teleop (remote) velocity, zeroes the robot when the active
    source goes silent, holds it still while the software e-stop is engaged, and limits every
    command to a speed the robot can still stop at before the nearest obstacle in its path
    (protective field on perception/safety_scan). Nothing non-finite or outside the robot's
    speed envelope (max_vx, max_vy, max_wz) is ever forwarded, whichever source sent it.
    """

    def __init__(self):
        """Declare params, wire topics and the mode / e-stop services."""
        super().__init__('control_mux_node')
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('remote_timeout_sec', 0.3)
        self.declare_parameter('nav_timeout_sec', 0.5)
        self.declare_parameter('pause_nav_on_remote', True)
        self.declare_parameter('estop_hold_hz', 10.0)
        self.declare_parameter('max_vx', 1.0)
        self.declare_parameter('max_vy', 0.5)
        self.declare_parameter('max_wz', 1.5)
        self.declare_parameter('field_in_auto', True)
        self.declare_parameter('field_in_remote', False)
        self.declare_parameter('safety_scan_timeout_sec', 0.5)
        self.declare_parameter('stale_remote_speed', 0.15)
        defaults = protective_field.FieldConfig()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(f'field.{name}', float(getattr(defaults, name)))
        self._field = protective_field.FieldConfig(
            **{
                name: float(self.get_parameter(f'field.{name}').value)
                for name in defaults.__dataclass_fields__
            }
        )
        self._field_modes = {
            'auto': bool(self.get_parameter('field_in_auto').value),
            'remote': bool(self.get_parameter('field_in_remote').value),
        }
        self._scan_timeout = float(self.get_parameter('safety_scan_timeout_sec').value)
        self._stale_remote_speed = float(self.get_parameter('stale_remote_speed').value)
        self._envelope = tuple(
            abs(float(self.get_parameter(name).value)) for name in ('max_vx', 'max_vy', 'max_wz')
        )
        self._obstacles = []
        self._scan_time = None
        self._field_state = None
        self._mux = ControlMux(
            self.get_parameter('mode').value,
            float(self.get_parameter('remote_timeout_sec').value),
            float(self.get_parameter('nav_timeout_sec').value),
        )
        self._pause_nav = bool(self.get_parameter('pause_nav_on_remote').value)
        self._paused_by_us = False

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._mode_pub = self.create_publisher(String, 'bridge/mode', latched)
        self._estop_pub = self.create_publisher(Bool, 'bridge/estop_engaged', latched)
        self._estop_reason_pub = self.create_publisher(String, 'bridge/estop_reason', latched)
        self._field_pub = self.create_publisher(String, 'bridge/protective_field', latched)
        self.create_subscription(SectorScan, 'perception/safety_scan', self.scan_callback, 10)
        self.create_subscription(Twist, 'bridge/nav_cmd_vel', self.nav_callback, 10)
        self.create_subscription(Twist, 'bridge/remote_cmd_vel', self.remote_callback, 10)
        self.create_subscription(String, 'bridge/estop_request', self.estop_request_callback, 10)
        self.create_service(SetMode, 'bridge/set_mode', self.set_mode_callback)
        self.create_service(SetBool, 'bridge/estop', self.estop_callback)
        self._pause_client = self.create_client(SetBool, 'navigation/pause')
        self.create_timer(0.05, self.watchdog)
        self.create_timer(1.0 / float(self.get_parameter('estop_hold_hz').value), self.estop_hold)
        self.publish_mode()
        self.publish_estop()

    def now(self) -> float:
        """Return seconds from the node clock."""
        return self.get_clock().now().nanoseconds / 1e9

    def publish_mode(self):
        """Publish the current mode (latched)."""
        self._mode_pub.publish(String(data=self._mux.mode))

    def nav_callback(self, msg: Twist):
        """Forward navigation commands in auto mode."""
        out = self._mux.on_nav(msg, self.now())
        if out is not None:
            self._cmd_pub.publish(self.gate(out))

    def remote_callback(self, msg: Twist):
        """Forward teleop commands in remote mode."""
        out = self._mux.on_remote(msg, self.now())
        if out is not None:
            self._cmd_pub.publish(self.gate(out))

    def gate(self, cmd: Twist) -> Twist:
        """Last checks before the robot: finite values, speed envelope, protective field."""
        rest = (cmd.linear.z, cmd.angular.x, cmd.angular.y)
        limited = envelope(cmd.linear.x, cmd.linear.y, cmd.angular.z, *self._envelope)
        if limited is None or not all(math.isfinite(v) for v in rest):
            self.get_logger().error(
                f'dropped a non-finite {self._mux.mode} command', throttle_duration_sec=1.0
            )
            return Twist()
        if limited != (cmd.linear.x, cmd.linear.y, cmd.angular.z):
            self.get_logger().warn(
                f'{self._mux.mode} command above the speed envelope, scaled down',
                throttle_duration_sec=5.0,
            )
            cmd.linear.x, cmd.linear.y, cmd.angular.z = limited
        return self.apply_field(cmd)

    def scan_callback(self, msg: SectorScan):
        """Keep the obstacle points of the latest 360 deg safety scan."""
        self._obstacles = protective_field.scan_points(
            msg.angle_min, msg.angle_increment, msg.ranges, msg.range_max
        )
        self._scan_time = time.monotonic()

    def apply_field(self, cmd: Twist) -> Twist:
        """Scale the linear velocity so the robot can stop before what is in its path."""
        if not self._field_modes.get(self._mux.mode, False):
            self.set_field_state('off')
            return cmd
        stale = self._scan_time is None or time.monotonic() - self._scan_time > self._scan_timeout
        if stale:
            self.set_field_state('stale')
            if self._mux.mode != 'remote':
                return Twist()  # navigation never drives blind
            speed = math.hypot(cmd.linear.x, cmd.linear.y)
            scale = min(1.0, self._stale_remote_speed / speed) if speed > 0.0 else 1.0
        else:
            result = protective_field.limit(
                self._field, cmd.linear.x, cmd.linear.y, self._obstacles
            )
            self.set_field_state(result.state)
            scale = result.scale
        if scale < 1.0:
            cmd.linear.x *= scale
            cmd.linear.y *= scale
        return cmd

    def set_field_state(self, state: str):
        """Publish and log protective-field state changes (latched)."""
        if state == self._field_state:
            return
        if state in ('stop', 'stale'):
            self.get_logger().warn(f'protective field: {state}')
        else:
            self.get_logger().info(f'protective field: {state}')
        self._field_state = state
        self._field_pub.publish(String(data=state))

    def watchdog(self):
        """Send one zero command when the active source goes silent or the mode just changed."""
        if self._mux.watchdog(self.now()):
            self._cmd_pub.publish(Twist())

    def estop_hold(self):
        """Keep sending zero while the e-stop is engaged (drivers with a long command timeout)."""
        if self._mux.estop:
            self._cmd_pub.publish(Twist())

    def publish_estop(self):
        """Publish the e-stop state and reason (latched)."""
        self._estop_pub.publish(Bool(data=self._mux.estop))
        self._estop_reason_pub.publish(String(data=self._mux.estop_reason))

    def engage(self, reason: str) -> bool:
        """Engage the e-stop: zero now, pause navigation, latch until released."""
        changed = self._mux.engage_estop(reason)
        self._cmd_pub.publish(Twist())
        if changed:
            self.get_logger().error(f'E-STOP engaged ({self._mux.estop_reason})')
            self.call_pause(True)
            self._paused_by_us = False  # a mode switch must not resume after an e-stop
            self.publish_estop()
        return changed

    def estop_callback(self, request, response):
        """Service: data=true engages the e-stop, data=false releases it."""
        if request.data:
            self.engage('service')
            response.message = f'e-stop engaged ({self._mux.estop_reason})'
        elif self._mux.release_estop():
            self._cmd_pub.publish(Twist())
            self.publish_estop()
            self.get_logger().warn('E-STOP released, navigation stays paused until resumed')
            response.message = 'e-stop released; resume navigation with navigation/pause false'
        else:
            response.message = 'e-stop was not engaged'
        response.success = True
        return response

    def estop_request_callback(self, msg: String):
        """Topic for any node (teleop, fleet) to engage the e-stop with a reason, no reply."""
        self.engage(msg.data or 'topic')

    def set_mode_callback(self, request, response):
        """Switch mode, pausing navigation while the remote drives."""
        if request.mode not in MODES:
            response.success = False
            response.message = f'unknown mode {request.mode!r}, use {list(MODES)}'
            return response
        changed = self._mux.set_mode(request.mode)
        self._cmd_pub.publish(Twist())
        if changed and self._pause_nav:
            if request.mode == 'remote':
                self._paused_by_us = self.call_pause(True)
            elif self._paused_by_us:
                self.call_pause(False)
                self._paused_by_us = False
        self.publish_mode()
        self.get_logger().info(f'control mode: {request.mode}')
        response.success = True
        response.message = f'mode {request.mode}'
        return response

    def call_pause(self, pause: bool) -> bool:
        """Ask navigation to pause or resume without waiting; False when it is not running."""
        if not self._pause_client.service_is_ready():
            return False
        self._pause_client.call_async(SetBool.Request(data=pause))
        return True


def main(args=None):
    """Spin the control mux node."""
    rclpy.init(args=args)
    node = ControlMuxNode()
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
