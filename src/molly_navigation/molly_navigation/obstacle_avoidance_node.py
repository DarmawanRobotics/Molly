#!/usr/bin/env python3
import math

from molly_interfaces.msg import AvoidanceCommand, SectorScan
from molly_navigation.utils import visualization as viz
from molly_navigation.utils.avoidance.registry import create_avoidance
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

ALGORITHM_PARAMS = {
    'braitenberg': {
        'steering_gain': 1.5,
        'velocity_gain': 1.0,
        'steering_zone_center_deg': 45.0,
        'steering_zone_width_deg': 60.0,
        'velocity_zone_center_deg': 60.0,
        'velocity_zone_width_deg': 90.0,
        'following_zone_width_deg': 30.0,
        'smoothing': 0.85,
    },
    'vfh': {
        'min_gap_width_deg': 20.0,
        'steering_gain': 1.0,
        'velocity_gain': 1.0,
        'smoothing': 0.85,
    },
    'potential_field': {
        'repulsion_gain': 0.5,
        'steering_gain': 0.8,
        'lateral_gain': 0.3,
        'max_lateral': 0.2,
        'velocity_gain': 1.0,
        'smoothing': 0.8,
    },
    'follow_gap': {
        'bubble_radius': 0.45,
        'steering_gain': 1.0,
        'velocity_gain': 1.0,
        'smoothing': 0.85,
    },
}


class ObstacleAvoidanceNode(Node):
    """Runs a selectable reactive avoidance algorithm on SectorScan."""

    def __init__(self):
        """Declare params, build the initial algorithm, and wire the scan subscription/output."""
        super().__init__('obstacle_avoidance_node')
        self.declare_parameter('algorithm', 'vfh')
        self.declare_parameter('safe_distance', 1.0)
        self.declare_parameter('emergency_distance', 0.75)
        self.declare_parameter('emergency_cone_deg', 60.0)
        for algorithm, params in ALGORITHM_PARAMS.items():
            for name, value in params.items():
                self.declare_parameter(f'{algorithm}.{name}', value)

        self._pending = False
        self._algorithm_name = self.get_parameter('algorithm').value
        self._avoidance = create_avoidance(
            self._algorithm_name, self.build_params(self._algorithm_name)
        )

        self.create_subscription(SectorScan, 'perception/sector_scan', self.scan_callback, 10)
        self._avoidance_pub = self.create_publisher(AvoidanceCommand, 'navigation/avoidance', 10)
        self._markers_pub = self.create_publisher(MarkerArray, 'navigation/avoidance_markers', 10)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

    def build_params(self, name: str) -> dict:
        """Build constructor kwargs: shared distances plus the algorithm's namespaced params."""
        if name not in ALGORITHM_PARAMS:
            raise ValueError(
                f'unknown avoidance algorithm {name!r}, options are {list(ALGORITHM_PARAMS)}'
            )
        params = {
            key: float(self.get_parameter(key).value)
            for key in ('safe_distance', 'emergency_distance', 'emergency_cone_deg')
        }
        for key in ALGORITHM_PARAMS[name]:
            params[key] = float(self.get_parameter(f'{name}.{key}').value)
        return params

    def on_parameters_changed(self, params):
        """Validate a parameter change; the algorithm is rebuilt on the next scan."""
        for param in params:
            if param.name == 'algorithm' and param.value not in ALGORITHM_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f'unknown algorithm {param.value!r}, use {list(ALGORITHM_PARAMS)}',
                )
        self._pending = True
        return SetParametersResult(successful=True)

    def rebuild(self):
        """Recreate the avoidance algorithm from the current parameters."""
        name = self.get_parameter('algorithm').value
        self._avoidance = create_avoidance(name, self.build_params(name))
        if name != self._algorithm_name:
            self.get_logger().info(f'switched avoidance algorithm to {name}')
        else:
            self.get_logger().info(f'applied {name} parameters')
        self._algorithm_name = name

    def scan_callback(self, msg: SectorScan):
        """Run one avoidance update from the sector scan and publish the result."""
        if self._pending:
            self._pending = False
            self.rebuild()
        result = self._avoidance.update(
            list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_max
        )
        out = AvoidanceCommand()
        out.header = msg.header
        out.steering_bias = float(result.steering_bias)
        out.lateral_bias = float(result.lateral_bias)
        out.velocity_scale = float(result.velocity_scale)
        out.emergency = bool(result.emergency)
        self._avoidance_pub.publish(out)
        self.publish_markers(msg, result)

    def publish_markers(self, scan: SectorScan, result):
        """Draw the emergency cone, safe range and avoidance push in the robot frame."""
        frame, stamp, ns = scan.header.frame_id or 'base_link', scan.header.stamp, 'avoidance'
        half_cone = math.radians(float(self.get_parameter('emergency_cone_deg').value)) / 2.0
        emergency = float(self.get_parameter('emergency_distance').value)
        safe = float(self.get_parameter('safe_distance').value)
        markers = MarkerArray()
        markers.markers.append(
            viz.cone(
                frame,
                stamp,
                ns,
                0,
                half_cone,
                emergency,
                viz.ALARM,
                alpha=0.6 if result.emergency else 0.15,
            )
        )
        markers.markers.append(
            viz.circle(
                frame,
                stamp,
                ns,
                1,
                0.0,
                0.0,
                safe,
                (1.0, 0.6, 0.1),
                width=0.015,
                alpha=0.5,
                start=-math.pi / 2,
                sweep=math.pi,
            )
        )
        if abs(result.steering_bias) > 0.02:
            markers.markers.append(
                viz.arrow(
                    frame,
                    stamp,
                    ns,
                    2,
                    0.0,
                    0.0,
                    max(-1.2, min(1.2, result.steering_bias)),
                    0.3 + 0.5 * result.velocity_scale,
                    (1.0, 0.6, 0.1),
                )
            )
        if abs(result.lateral_bias) > 0.01:
            markers.markers.append(
                viz.arrow(
                    frame,
                    stamp,
                    ns,
                    3,
                    0.0,
                    0.0,
                    math.copysign(math.pi / 2, result.lateral_bias),
                    3.0 * abs(result.lateral_bias),
                    (0.8, 0.3, 1.0),
                )
            )
        lifetime = Duration(seconds=0.3).to_msg()
        for m in markers.markers:
            m.lifetime = lifetime
        self._markers_pub.publish(markers)


def main(args=None):
    """Spin the obstacle avoidance node."""
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
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
