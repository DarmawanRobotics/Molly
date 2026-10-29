"""
visitor_tracker_node: the people around the robot in the 360 deg LiDAR (perception/visitors).

Points in base_link are cut to a leg / torso band above the floor (robot.base_height), off the
body (robot.footprint), clustered, kept when person-sized and not on the site map's walls, and
confirmed over a few scans. The guide asks: is anybody following? The greeter: somebody in front?
"""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from dharma_utils.math import transform_points, yaw_from_quat
from dharma_utils.ros import LATCHED, SENSOR, DharmaNode, run
from dharma_utils.ros.markers import color, delete_all, marker
from dharma_utils.ros.pointcloud import CloudError, read_xyz
from molly_interaction.core.config import VisitorConfig
from molly_interaction.core.visitors import (
    StaticMap,
    VisitorTracker,
    band_points,
    drop_static,
    lidar_confidence,
    person_clusters,
    summarize,
)
from molly_interfaces.msg import Visitors


def _xyzw(q) -> tuple[float, float, float, float]:
    return q.x, q.y, q.z, q.w


class VisitorTrackerNode(DharmaNode):
    """perception/visitors (and RViz markers) from the LiDAR, the site map and TF."""

    def __init__(self, **kwargs):
        super().__init__('visitor_tracker_node', **kwargs)
        self.cfg = self.declare_config(VisitorConfig).value
        self.robot = self.declare_robot().value
        cfg = self.cfg
        self.tracker = VisitorTracker(cfg.confirm_scans, cfg.max_misses, cfg.assoc_distance)
        self.period = 1.0 / cfg.rate_hz
        self.last_run = -math.inf
        self.static_map: StaticMap | None = None
        self.base = self.robot.frames.base
        self.pub = self.create_publisher(Visitors, 'perception/visitors', 10)
        self.markers = None
        if cfg.publish_markers:
            self.markers = self.create_publisher(MarkerArray, 'perception/visitor_markers', 1)
        if cfg.use_map:
            self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.subscribe()
        self.log.info('visitor tracker ready', input=cfg.input_type)

    def subscribe(self):
        """Subscribe to the LiDAR: raw Livox CustomMsg bytes or a PointCloud2."""
        if self.cfg.input_type == 'pointcloud2':
            self.create_subscription(PointCloud2, self.cfg.pointcloud_topic, self.on_cloud, SENSOR)
            return
        try:
            from livox_ros_driver2.msg import CustomMsg
        except ImportError:
            self.log.error('livox_ros_driver2 is not installed: no visitors from the Livox')
            return
        self.create_subscription(CustomMsg, self.cfg.livox_topic, self.on_livox, SENSOR, raw=True)

    # ---------------------------------------------------------------- inputs
    def on_map(self, msg: OccupancyGrid):
        """Keep the static obstacles of the site map, grown by map_margin."""
        info = msg.info
        if info.width == 0 or info.height == 0 or info.resolution <= 0.0:
            self.static_map = None
            return
        try:
            self.static_map = StaticMap.from_grid(
                msg.data, info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y,
                yaw_from_quat(*_xyzw(info.origin.orientation)), self.cfg.map_margin,
            )  # fmt: skip
        except ValueError as error:
            self.log.warning('map not usable for visitors', error=str(error))
            self.static_map = None

    def due(self) -> bool:
        """Return True when the next scan should be processed (rate_hz)."""
        now = time.monotonic()
        if now - self.last_run < self.period:
            return False
        self.last_run = now
        return True

    def on_livox(self, data: bytes):
        """Process one serialized Livox CustomMsg."""
        from dharma_perception.core.livox import LivoxError, parse_custom_msg

        if not self.due():
            return
        try:
            scan = parse_custom_msg(data)
        except LivoxError as error:
            self.log.error('Livox scan dropped', reason=str(error), throttle=5.0)
            return
        self.process(scan.frame_id, scan.xyz)

    def on_cloud(self, msg: PointCloud2):
        """Process one PointCloud2."""
        if not self.due():
            return
        try:
            xyz = read_xyz(msg)
        except CloudError as error:
            self.log.error('cloud dropped', reason=str(error), throttle=5.0)
            return
        self.process(msg.header.frame_id, xyz)

    # ---------------------------------------------------------------- processing
    def self_box(self) -> tuple[float, float, float, float]:
        """Return the robot body in base_link grown by self_margin: (x0, x1, y0, y1)."""
        f, m = self.robot.footprint, self.cfg.self_margin
        return (-f.rear - m, f.front + m, -f.half_width - m, f.half_width + m)

    def process(self, frame_id: str, xyz: np.ndarray):
        """Cluster one scan into visitors and publish them."""
        frame = frame_id or self.cfg.lidar_frame
        mount = self.tf.static(self.base, frame)
        if mount is None:
            self.log.warning('waiting for the LiDAR mount TF', frame=frame, throttle=5.0)
            return
        cfg = self.cfg
        points = transform_points(mount, xyz) if len(xyz) else np.empty((0, 3))
        floor = -self.robot.base_height
        xy = band_points(
            points, floor + cfg.height_min, floor + cfg.height_max, self.self_box(),
            cfg.range_min, cfg.max_range,
        )  # fmt: skip
        clusters = person_clusters(xy, cfg.cluster_tolerance, cfg.min_points, cfg.min_width,
                                   cfg.max_width)  # fmt: skip
        for cluster in clusters:
            cluster.confidence = lidar_confidence(cluster.size)
        clusters = drop_static(clusters, self.static_map, self.map_pose())
        self.publish(self.tracker.update(clusters))

    def map_pose(self) -> tuple[float, float, float] | None:
        """Return the robot pose (x, y, yaw) in the map, or None when not localized."""
        result = self.tf.lookup(self.robot.frames.map, self.base)
        if result is None:
            return None
        pose = result.pose2d
        return pose.x, pose.y, pose.yaw

    def publish(self, tracks: list):
        """Publish perception/visitors and the RViz markers."""
        positions = [(t.x, t.y) for t in tracks]
        front = math.radians(self.cfg.front_angle_deg)
        behind = math.radians(self.cfg.behind_angle_deg)
        summary = summarize(positions, front, behind)
        msg = Visitors(
            positions=[Point(x=float(x), y=float(y), z=0.0) for x, y in positions],
            confidences=[float(t.confidence) for t in tracks],
            count=summary['count'],
            front=summary['front'],
            behind=summary['behind'],
            nearest=float(summary['nearest']),
            nearest_front=float(summary['nearest_front']),
            nearest_behind=float(summary['nearest_behind']),
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base
        self.pub.publish(msg)
        if self.markers is not None:
            self.publish_markers(msg, front, behind)

    def publish_markers(self, msg: Visitors, front: float, behind: float):
        """Draw every visitor as a disc, coloured by sector."""
        out = MarkerArray(markers=[delete_all(msg.header, 'visitors')])
        for i, point in enumerate(msg.positions):
            item = marker(msg.header, 'visitors', i, Marker.CYLINDER, lifetime=1.0)
            item.pose.position.x, item.pose.position.y = point.x, point.y
            item.scale.x = item.scale.y = 0.4
            item.scale.z = 0.1
            angle = abs(math.atan2(point.y, point.x))
            name = 'yellow'
            if angle <= front / 2:
                name = 'blue'
            elif angle >= math.pi - behind / 2:
                name = 'green'
            item.color = color(name, 0.8)
            out.markers.append(item)
        self.markers.publish(out)


def main(args=None):
    """Run the node."""
    run(VisitorTrackerNode, args=args)


if __name__ == '__main__':
    main()
