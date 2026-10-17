#!/usr/bin/env python3
import csv
from datetime import datetime
import math
import os

from molly_interfaces.msg import SiteStatus
from molly_interfaces.srv import MarkStopPoint
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
import tf_transformations


class RouteRecorderNode(Node):
    """
    Teach: record the robot path from TF and save it as a route CSV with stop points.

    Routes go into the active site (<sites_dir>/<site>/routes/<date>_<time>_path.csv), or into
    <output_path>/<date>/<time>_path.csv while no site is active.
    """

    def __init__(self):
        """Initialize parameters and services; TF and sampling start with a recording."""
        super().__init__('route_recorder_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_waypoint_spacing_m', 0.2)
        self.declare_parameter('record_rate_hz', 10.0)
        self.declare_parameter('output_path', '/home/robot/dev/Molly/map')
        self.declare_parameter('sites_dir', '~/dev/Molly/map')

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._min_spacing = float(self.get_parameter('min_waypoint_spacing_m').value)
        self._record_period = 1.0 / float(self.get_parameter('record_rate_hz').value)
        self._output_path = self.get_parameter('output_path').value
        self._sites_dir = os.path.expanduser(str(self.get_parameter('sites_dir').value))
        self._site = ''

        self._recording = False
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._recording_file = None

        self._tf_buffer = Buffer()
        self._tf_listener = None  # only while recording, see start_sampling()
        self._sample_timer = None

        self._saved_pub = self.create_publisher(String, 'route/saved', 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SiteStatus, 'site/status', self.on_site_status, latched)
        self.create_service(SetBool, 'route/record', self.recording_callback)
        self.create_service(
            MarkStopPoint, 'route/mark_stop_point', self.mark_stop_point_callback
        )

    def on_site_status(self, msg: SiteStatus):
        """Remember the active site: its routes/ directory takes new recordings."""
        self._site = msg.site

    def recording_path(self, timestamp: datetime) -> str:
        """Return the CSV for a recording started now."""
        if self._site:
            name = timestamp.strftime('%Y-%m-%d_%H-%M-%S_path.csv')
            return os.path.join(self._sites_dir, self._site, 'routes', name)
        date_dir = timestamp.strftime('%Y-%m-%d')
        time_name = timestamp.strftime('%H-%M-%S')
        return os.path.join(self._output_path, date_dir, f'{time_name}_path.csv')

    def recording_callback(self, request, response):
        """Start or stop path recording."""
        if request.data:
            if self._recording:
                response.success = False
                response.message = 'already recording'
                return response

            self._waypoints = []
            self._recording = True

            self._recording_file = self.recording_path(datetime.now())
            os.makedirs(os.path.dirname(self._recording_file), exist_ok=True)
            self.start_sampling()

            response.success = True
            response.message = 'recording started'
            self.get_logger().info(f'path recording started: {self._recording_file}')
            return response

        if not self._recording:
            response.success = False
            response.message = 'not recording'
            return response

        self._recording = False
        self.stop_sampling()
        self.write_csv()
        self._saved_pub.publish(String(data=self._recording_file))

        response.success = True
        response.message = (
            f'recording stopped, {len(self._waypoints)} waypoints saved to {self._recording_file}'
        )
        self.get_logger().info(response.message)
        return response

    def start_sampling(self):
        """Listen to TF and sample the pose while a route is recorded."""
        if self._tf_listener is None:
            self._tf_listener = TransformListener(self._tf_buffer, self)
        if self._sample_timer is None:
            self._sample_timer = self.create_timer(self._record_period, self.record_tick)

    def stop_sampling(self):
        """Release TF and the sampling timer between recordings (nothing to do then)."""
        if self._sample_timer is not None:
            self.destroy_timer(self._sample_timer)
            self._sample_timer = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None

    def lookup_current_pose(self):
        """Look up the current map->base_link pose."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception as error:
            self.get_logger().warning(
                f'TF lookup {self._map_frame}->{self._base_frame} failed: {error}',
                throttle_duration_sec=2.0,
            )
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        return x, y, math.degrees(yaw)

    def record_tick(self):
        """Sample and record the robot pose when it moved far enough."""
        if not self._recording:
            return

        pose = self.lookup_current_pose()
        if pose is None:
            return

        x, y, yaw_deg = pose

        if self._waypoints:
            last_x, last_y, _, _ = self._waypoints[-1]
            if math.hypot(x - last_x, y - last_y) < self._min_spacing:
                return

        self._waypoints.append((x, y, yaw_deg, 0.0))

    def mark_stop_point_callback(self, request, response):
        """Append the current pose as an inspection stop point."""
        if not self._recording:
            response.success = False
            response.message = 'not recording, call route/record(true) first'
            return response

        pose = self.lookup_current_pose()
        if pose is None:
            response.success = False
            response.message = 'TF not available, could not sample current pose'
            return response

        x, y, yaw_deg = pose
        self._waypoints.append((x, y, yaw_deg, request.dwell_sec))
        self.write_csv()

        response.success = True
        response.message = (
            f'stop point marked at ({x:.2f}, {y:.2f}), dwell {request.dwell_sec:.1f}s'
        )
        self.get_logger().info(response.message)
        return response

    def write_csv(self):
        """Write recorded waypoints to the current recording CSV file."""
        if not self._recording_file:
            self.get_logger().error('No recording file configured.')
            return

        os.makedirs(os.path.dirname(self._recording_file), exist_ok=True)

        with open(self._recording_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['# x', 'y', 'yaw_deg', 'dwell_sec'])

            for x, y, yaw_deg, dwell_sec in self._waypoints:
                writer.writerow([f'{x:.4f}', f'{y:.4f}', f'{yaw_deg:.2f}', f'{dwell_sec:.1f}'])


def main(args=None):
    """Initialize ROS 2 and run the route recorder node."""
    rclpy.init(args=args)
    node = RouteRecorderNode()

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
