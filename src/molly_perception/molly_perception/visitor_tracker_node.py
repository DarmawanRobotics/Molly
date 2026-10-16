#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import Point, PoseArray
from molly_interfaces.msg import Visitors
from molly_perception.livox_cdr import parse_custom_msg
from molly_perception.visitors import (
    band_points,
    drop_static,
    from_frame,
    fuse_camera,
    lidar_confidence,
    person_clusters,
    StaticMap,
    summarize,
    VisitorTracker,
)
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray

FRONT = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.8)
BEHIND = ColorRGBA(r=0.3, g=1.0, b=0.4, a=0.8)
SIDE = ColorRGBA(r=1.0, g=0.8, b=0.2, a=0.8)


def yaw_of(q) -> float:
    """Return the yaw of a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VisitorTrackerNode(Node):
    """Finds the people around the robot in the 360 deg LiDAR and publishes perception/visitors."""

    def __init__(self):
        """Declare params and wire the cloud, map, camera people and outputs."""
        super().__init__('visitor_tracker_node')
        defaults = {
            'input_type': 'livox',
            'pointcloud_topic': '/cloud_registered_body',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'lidar_frame': 'livox_frame',
            'z_min': -0.15,
            'z_max': 1.0,
            'range_min': 0.3,
            'max_range': 5.0,
            'self_filter_x_min': -0.40,
            'self_filter_x_max': 0.40,
            'self_filter_y_min': -0.28,
            'self_filter_y_max': 0.28,
            'cluster_tolerance': 0.25,
            'min_points': 4,
            'min_width': 0.1,
            'max_width': 1.0,
            'use_map': True,
            'map_margin': 0.25,
            'use_camera_people': False,
            'people_max_age_sec': 1.0,
            'fuse_distance': 0.6,
            'front_angle_deg': 90.0,
            'behind_angle_deg': 150.0,
            'confirm_scans': 2,
            'max_misses': 2,
            'assoc_distance': 0.6,
            'rate_hz': 5.0,
            'publish_markers': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._base_frame, self._map_frame = str(p('base_frame')), str(p('map_frame'))
        self._fallback_frame = str(p('lidar_frame'))
        self._z = (float(p('z_min')), float(p('z_max')))
        self._range = (float(p('range_min')), float(p('max_range')))
        self._self_box = [
            float(p(f'self_filter_{k}')) for k in ('x_min', 'x_max', 'y_min', 'y_max')
        ]
        self._tolerance = float(p('cluster_tolerance'))
        self._min_points = int(p('min_points'))
        self._width = (float(p('min_width')), float(p('max_width')))
        self._use_map, self._map_margin = bool(p('use_map')), float(p('map_margin'))
        self._use_camera = bool(p('use_camera_people'))
        self._people_max_age = float(p('people_max_age_sec'))
        self._fuse_distance = float(p('fuse_distance'))
        self._front = math.radians(float(p('front_angle_deg')))
        self._behind = math.radians(float(p('behind_angle_deg')))
        rate = float(p('rate_hz'))
        self._period = 1.0 / rate if rate > 0.0 else 0.0
        self._markers = bool(p('publish_markers'))
        self._tracker = VisitorTracker(
            int(p('confirm_scans')), int(p('max_misses')), float(p('assoc_distance'))
        )
        self._last_run = -math.inf
        self._static_map = None
        self._people = []  # (x, y, score) in map
        self._people_time = -math.inf
        self._extrinsics = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._visitors_pub = self.create_publisher(Visitors, 'perception/visitors', 10)
        self._markers_pub = (
            self.create_publisher(MarkerArray, 'perception/visitor_markers', 10)
            if self._markers
            else None
        )
        if self._use_map:
            latched = QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        if self._use_camera:
            self.create_subscription(PoseArray, 'perception/people', self.on_people, 10)

        input_type = str(p('input_type'))
        if input_type == 'pointcloud2':
            self.create_subscription(
                PointCloud2, str(p('pointcloud_topic')), self.on_pointcloud, 10
            )
        elif input_type == 'livox':
            from livox_ros_driver2.msg import CustomMsg  # only needed for raw Livox input

            self._custom_msg = CustomMsg
            self.create_subscription(CustomMsg, 'livox/lidar', self.on_livox, 10, raw=True)
        else:
            raise ValueError(f"input_type must be 'livox' or 'pointcloud2', got {input_type!r}")

    # ---------------------------------------------------------------- inputs
    def on_map(self, msg: OccupancyGrid):
        """Keep the static obstacles of the site map, grown by map_margin."""
        info = msg.info
        if info.width == 0 or info.height == 0 or info.resolution <= 0.0:
            self._static_map = None
            return
        try:
            self._static_map = StaticMap.from_grid(
                msg.data, info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y,
                yaw_of(info.origin.orientation), self._map_margin,
            )
        except ValueError as error:
            self.get_logger().warn(f'ignoring /map: {error}')
            self._static_map = None

    def on_people(self, msg: PoseArray):
        """Keep the latest camera people (map frame, score in orientation.w)."""
        self._people = [(p.position.x, p.position.y, p.orientation.w) for p in msg.poses]
        self._people_time = time.monotonic()

    def on_livox(self, data: bytes):
        """Read a serialized Livox CustomMsg as numpy."""
        if not self.due():
            return
        parsed = parse_custom_msg(data)
        if parsed is None:
            msg = deserialize_message(data, self._custom_msg)
            xyz = np.array([(q.x, q.y, q.z) for q in msg.points], dtype=np.float32).reshape(-1, 3)
            self.process(msg.header.frame_id, xyz[np.isfinite(xyz).all(axis=1)])
            return
        _, _, frame_id, xyz = parsed
        self.process(frame_id, xyz)

    def on_pointcloud(self, msg: PointCloud2):
        """Read x/y/z straight from the PointCloud2 buffer."""
        if not self.due():
            return
        offsets = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offsets.keys():
            self.get_logger().error('PointCloud2 has no x/y/z fields', throttle_duration_sec=5.0)
            return
        dtype = np.dtype(
            {
                'names': ['x', 'y', 'z'],
                'formats': ['<f4' if not msg.is_bigendian else '>f4'] * 3,
                'offsets': [offsets['x'], offsets['y'], offsets['z']],
                'itemsize': msg.point_step,
            }
        )
        cloud = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(np.float32)
        self.process(msg.header.frame_id, xyz[np.isfinite(xyz).all(axis=1)])

    def due(self) -> bool:
        """Return True when the next scan should be processed (rate_hz)."""
        now = time.monotonic()
        if now - self._last_run < self._period:
            return False
        self._last_run = now
        return True

    # ---------------------------------------------------------------- processing
    def extrinsic(self, frame_id: str):
        """Return the (rotation, translation) of the LiDAR in base_frame, cached once known."""
        if frame_id not in self._extrinsics:
            try:
                tf = self._tf_buffer.lookup_transform(self._base_frame, frame_id, Time())
            except TransformException as error:
                self.get_logger().warn(
                    f'waiting for TF {frame_id} -> {self._base_frame}: {error}',
                    throttle_duration_sec=2.0,
                )
                return None
            q, t = tf.transform.rotation, tf.transform.translation
            rotation = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            self._extrinsics[frame_id] = (rotation, np.array([t.x, t.y, t.z]))
        return self._extrinsics[frame_id]

    def map_base(self):
        """Return the robot's planar pose (x, y, yaw) in the map, or None when not localized."""
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def process(self, frame_id: str, xyz: np.ndarray):
        """Cluster one scan into visitors and publish them."""
        extrinsic = self.extrinsic(frame_id or self._fallback_frame)
        if extrinsic is None:
            return
        rotation, translation = extrinsic
        points = xyz @ rotation.T + translation if xyz.shape[0] else np.empty((0, 3))
        xy = band_points(points, *self._z, self._self_box, *self._range)
        clusters = person_clusters(xy, self._tolerance, self._min_points, *self._width)
        for c in clusters:
            c.confidence = lidar_confidence(c.size)
        map_base = self.map_base()
        clusters = drop_static(clusters, self._static_map, map_base)
        if (
            self._use_camera
            and map_base is not None
            and time.monotonic() - self._people_time <= self._people_max_age
        ):
            people = [(*from_frame(x, y, map_base), s) for x, y, s in self._people]
            people = [(x, y, s) for x, y, s in people if math.hypot(x, y) <= self._range[1]]
            clusters = fuse_camera(clusters, people, self._fuse_distance)
        confirmed = self._tracker.update(clusters)
        self.publish(confirmed)

    def publish(self, tracks: list):
        """Publish perception/visitors and the RViz markers."""
        positions = [(t.x, t.y) for t in tracks]
        summary = summarize(positions, self._front, self._behind)
        msg = Visitors()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.positions = [Point(x=float(x), y=float(y), z=0.0) for x, y in positions]
        msg.confidences = [float(t.confidence) for t in tracks]
        msg.count, msg.front, msg.behind = summary['count'], summary['front'], summary['behind']
        msg.nearest = float(summary['nearest'])
        msg.nearest_front = float(summary['nearest_front'])
        msg.nearest_behind = float(summary['nearest_behind'])
        self._visitors_pub.publish(msg)
        if self._markers_pub is not None:
            self.publish_markers(msg)

    def publish_markers(self, msg: Visitors):
        """Draw every visitor as a cylinder, coloured by sector."""
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for i, point in enumerate(msg.positions):
            item = Marker()
            item.header.frame_id = self._base_frame
            item.ns = 'visitors'
            item.id = i
            item.type = Marker.CYLINDER
            item.action = Marker.ADD
            item.pose.position.x, item.pose.position.y = point.x, point.y
            item.pose.position.z = 0.0
            item.pose.orientation.w = 1.0
            item.scale.x = item.scale.y = 0.4
            item.scale.z = 0.1
            angle = abs(math.atan2(point.y, point.x))
            item.color = (
                FRONT if angle <= self._front / 2.0
                else BEHIND if angle >= math.pi - self._behind / 2.0
                else SIDE
            )
            item.lifetime = Duration(seconds=1.0).to_msg()
            markers.markers.append(item)
        self._markers_pub.publish(markers)


def main(args=None):
    """Spin the visitor tracker node."""
    rclpy.init(args=args)
    node = VisitorTrackerNode()
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
