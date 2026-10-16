#!/usr/bin/env python3
import csv
import math
import os

from geometry_msgs.msg import PoseStamped
from molly_interfaces.msg import Waypoint, WaypointPath
from molly_interfaces.srv import LoadPath
from molly_navigation.utils.path_processing import parse_route_rows, process_route
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class PathLoaderNode(Node):
    """Loads a recorded route CSV, smooths it and publishes it latched."""

    def __init__(self):
        """Declare params, load and process the route, and publish it."""
        super().__init__('path_loader_node')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('route_state_file', '~/.molly/last_route')
        self.declare_parameter('republish_period', 2.0)
        self.declare_parameter('smoothing_enabled', True)
        self.declare_parameter('resample_spacing', 0.1)
        self.declare_parameter('smooth_weight_data', 0.05)
        self.declare_parameter('smooth_weight_smooth', 0.4)
        self.declare_parameter('max_smoothing_deviation', 0.3)

        p = self.get_parameter
        self._smoothing = bool(p('smoothing_enabled').value)
        self._spacing = float(p('resample_spacing').value)
        self._weight_data = float(p('smooth_weight_data').value)
        self._weight_smooth = float(p('smooth_weight_smooth').value)
        self._max_deviation = float(p('max_smoothing_deviation').value)

        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._path_pub = self.create_publisher(Path, 'navigation/path', latched)
        self._raw_path_pub = self.create_publisher(Path, 'navigation/path_raw', latched)
        self._waypoints_pub = self.create_publisher(WaypointPath, 'navigation/waypoints', latched)

        self._raw: list = []
        self._route: list = []
        self._state_file = os.path.expanduser(p('route_state_file').value)
        if not self.load(p('waypoints_file').value or self.remembered_route()):
            self.get_logger().info('no route loaded yet, call navigation/load_path')

        period = float(p('republish_period').value)
        if period > 0.0:
            self.create_timer(period, self.publish_viz)
        self.create_service(LoadPath, 'navigation/load_path', self.load_path_callback)

    def read_csv(self, filepath: str) -> list:
        """Read x,y,yaw_deg[,dwell_sec] rows; bad rows are skipped and logged."""
        if not filepath or not os.path.isfile(filepath):
            self.get_logger().warn(f'waypoints_file not found: "{filepath}"')
            return []
        try:
            with open(filepath, 'r', newline='') as f:
                waypoints, problems = parse_route_rows(csv.reader(f))
        except (OSError, UnicodeDecodeError, csv.Error) as error:
            self.get_logger().error(f'cannot read {filepath}: {error}')
            return []
        for problem in problems:
            self.get_logger().warn(f'{filepath}: {problem}')
        return waypoints

    def load(self, filepath: str) -> bool:
        """Load, process and publish a route; returns False when nothing was loaded."""
        raw = self.read_csv(filepath)
        if not raw:
            return False
        route, deviation = (
            process_route(raw, self._spacing, self._weight_data, self._weight_smooth)
            if self._smoothing
            else (raw, 0.0)
        )
        if deviation > self._max_deviation:
            self.get_logger().warn(
                f'smoothing moved the route up to {deviation:.2f} m '
                f'(> {self._max_deviation:.2f} m), '
                'lower smooth_weight_smooth near walls'
            )
        self._raw, self._route = raw, route
        stops = sum(1 for w in route if w[3] > 0.0)
        self.get_logger().info(
            f'loaded {filepath}: {len(raw)} -> {len(route)} points, {stops} stop points, '
            f'max deviation {deviation:.3f} m'
        )
        self.publish_viz()
        self.publish_waypoints()
        return True

    def load_path_callback(self, request, response):
        """Service: load another route file without relaunching."""
        response.success = self.load(request.waypoints_file)
        if response.success:
            self.remember_route(request.waypoints_file)
        response.message = (
            f'loaded {len(self._route)} points from {request.waypoints_file}'
            if response.success
            else f'no waypoints loaded from "{request.waypoints_file}"'
        )
        return response

    def remembered_route(self) -> str:
        """Return the last route loaded through the service (survives restarts), or ''."""
        try:
            with open(self._state_file) as f:
                return f.read().strip()
        except OSError:
            return ''

    def remember_route(self, filepath: str):
        """Store the route path so a restarted robot comes back with the same route."""
        try:
            os.makedirs(os.path.dirname(self._state_file), exist_ok=True)
            with open(self._state_file, 'w') as f:
                f.write(os.path.abspath(filepath))
        except OSError as error:
            self.get_logger().warn(f'cannot remember route: {error}')

    def to_path(self, waypoints: list) -> Path:
        """Build a nav_msgs/Path in the map frame."""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw_deg, _ in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
            pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
            msg.poses.append(pose)
        return msg

    def publish_viz(self):
        """Publish the processed and raw routes for RViz."""
        self._path_pub.publish(self.to_path(self._route))
        self._raw_path_pub.publish(self.to_path(self._raw))

    def publish_waypoints(self):
        """Publish the latched WaypointPath consumed by path_follower_node."""
        msg = WaypointPath()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw_deg, dwell_sec in self._route:
            msg.waypoints.append(
                Waypoint(x=float(x), y=float(y), yaw=float(yaw_deg), dwell_sec=float(dwell_sec))
            )
        self._waypoints_pub.publish(msg)


def main(args=None):
    """Spin the path loader node."""
    rclpy.init(args=args)
    node = PathLoaderNode()
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
