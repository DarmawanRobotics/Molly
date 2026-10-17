#!/usr/bin/env python3
"""
2D occupancy grid from the LiDAR-inertial odometry's deskewed scans, while a site is mapped.

Each processed scan (at most rate_hz) is voxel-downsampled, cut to the obstacle height band
around the robot (measured in the gravity-aligned map frame, so a pitching body does not turn
the floor into walls) and the robot's own footprint, and integrated into a log-odds grid in the
map frame. The cloud is only subscribed while mapping.
"""

import array
from collections import deque

from molly_interfaces.msg import MappingStatus, SiteStatus
from molly_interfaces.srv import LoadFile
from molly_mapping.cloud import apply, cloud_xyz, transform_matrix, voxel_downsample
from molly_mapping.grid import LogOddsGrid
from molly_mapping.map_io import crop_known, load_map, MapError, save_map
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULTS = {
    'cloud_topic': '/cloud_registered_body',
    'map_frame': 'map',
    'base_frame': 'base_link',
    'resolution': 0.05,
    'initial_size_m': 20.0,
    'grow_m': 5.0,
    'rate_hz': 2.0,
    'publish_hz': 1.0,
    'voxel': 0.05,
    'obstacle_z_min': -0.25,
    'obstacle_z_max': 1.2,
    'self_filter_x_min': -0.40,
    'self_filter_x_max': 0.40,
    'self_filter_y_min': -0.28,
    'self_filter_y_max': 0.28,
    'min_range': 0.3,
    'max_range': 15.0,
    'angular_bins': 720,
    'log_odds_hit': 0.85,
    'log_odds_miss': -0.4,
    'log_odds_min': -2.0,
    'log_odds_max': 3.5,
    'occupied_threshold': 0.65,
    'free_threshold': 0.35,
    'save_margin_m': 1.0,
    'max_cloud_age_sec': 1.0,
}


class GridMapperNode(Node):
    """Integrate scans into an occupancy grid while mapping; save and load it per site."""

    def __init__(self):
        """Declare parameters and the interfaces; the cloud is subscribed only while mapping."""
        super().__init__('grid_mapper_node')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._map_frame, self._base_frame = str(p('map_frame')), str(p('base_frame'))
        self._cloud_topic = str(p('cloud_topic'))
        self._voxel = float(p('voxel'))
        self._z_band = (float(p('obstacle_z_min')), float(p('obstacle_z_max')))
        self._self_box = tuple(
            float(p(f'self_filter_{k}')) for k in ('x_min', 'x_max', 'y_min', 'y_max')
        )
        self._range = (float(p('min_range')), float(p('max_range')))
        self._bins = int(p('angular_bins'))
        self._save_margin = float(p('save_margin_m'))
        self._max_age = Duration(seconds=float(p('max_cloud_age_sec')))
        self._grid = self.new_grid()
        self._active = False
        self._dirty = False
        self._site = ''
        self._message = 'idle'
        self._clouds = deque(maxlen=4)
        self._static = {}
        self._cloud_sub = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._grid_pub = self.create_publisher(OccupancyGrid, 'mapping/grid', latched)
        self._status_pub = self.create_publisher(MappingStatus, 'mapping/status', latched)
        self.create_subscription(SiteStatus, 'site/status', self.on_site_status, latched)
        self.create_service(SetBool, 'grid_mapper/start', self.start_callback)
        self.create_service(Trigger, 'grid_mapper/stop', self.stop_callback)
        self.create_service(LoadFile, 'grid_mapper/save', self.save_callback)
        self.create_service(LoadFile, 'grid_mapper/load', self.load_callback)
        self.create_timer(1.0 / max(float(p('rate_hz')), 0.1), self.process_tick)
        self.create_timer(1.0 / max(float(p('publish_hz')), 0.1), self.publish_tick)
        self.publish_status()

    def new_grid(self) -> LogOddsGrid:
        """Return an empty grid with the configured resolution and log-odds."""
        p = lambda name: float(self.get_parameter(name).value)  # noqa: E731
        return LogOddsGrid(
            resolution=p('resolution'),
            size_m=p('initial_size_m'),
            hit=p('log_odds_hit'),
            miss=p('log_odds_miss'),
            clamp=(p('log_odds_min'), p('log_odds_max')),
            occupied_threshold=p('occupied_threshold'),
            free_threshold=p('free_threshold'),
            grow_m=p('grow_m'),
        )

    def on_site_status(self, msg: SiteStatus):
        """Remember the active site for the status."""
        self._site = msg.site

    def on_cloud(self, msg: PointCloud2):
        """Queue a scan; process_tick takes the newest one with a transform."""
        self._clouds.append(msg)

    def subscribe_cloud(self, on: bool):
        """Subscribe to the scans only while mapping."""
        if on and self._cloud_sub is None:
            self._cloud_sub = self.create_subscription(
                PointCloud2, self._cloud_topic, self.on_cloud, qos_profile_sensor_data
            )
        elif not on and self._cloud_sub is not None:
            self.destroy_subscription(self._cloud_sub)
            self._cloud_sub = None
            self._clouds.clear()

    def lookup(self, target: str, source: str, stamp: Time):
        """Return the 4x4 target<-source at stamp, or None when TF has no answer yet."""
        try:
            return transform_matrix(self._tf_buffer.lookup_transform(target, source, stamp))
        except TransformException:
            return None

    def static_matrix(self, source: str):
        """Return base_link<-source (static, cached once found), or None."""
        if source not in self._static:
            m = self.lookup(self._base_frame, source, Time())
            if m is None:
                return None
            self._static[source] = m
        return self._static[source]

    def process_tick(self):
        """Integrate the newest queued scan whose map transform is available."""
        if not self._active or not self._clouds:
            return
        for msg in reversed(list(self._clouds)):
            frame = msg.header.frame_id
            m_base_cloud = self.static_matrix(frame)
            m_map_cloud = self.lookup(self._map_frame, frame, Time.from_msg(msg.header.stamp))
            if m_base_cloud is None or m_map_cloud is None:
                continue
            self._clouds.clear()
            self.integrate(msg, m_base_cloud, m_map_cloud)
            return
        cutoff = self.get_clock().now() - self._max_age
        dropped = 0
        while self._clouds and Time.from_msg(self._clouds[0].header.stamp) < cutoff:
            self._clouds.popleft()
            dropped += 1
        if dropped:
            self.get_logger().warn(
                f'scans dropped: no transform {self._map_frame} <- scan (odometry running?)',
                throttle_duration_sec=10.0,
            )

    def integrate(self, msg: PointCloud2, m_base_cloud: np.ndarray, m_map_cloud: np.ndarray):
        """Filter one scan to obstacle hits in the map and add it to the grid."""
        points = voxel_downsample(cloud_xyz(msg), self._voxel)
        if points.shape[0] == 0:
            return
        in_base = apply(m_base_cloud, points)
        in_map = apply(m_map_cloud, points)
        base_z = (m_map_cloud @ np.linalg.inv(m_base_cloud))[2, 3]
        height = in_map[:, 2] - base_z
        x0, x1, y0, y1 = self._self_box
        own = (
            (in_base[:, 0] >= x0) & (in_base[:, 0] <= x1)
            & (in_base[:, 1] >= y0) & (in_base[:, 1] <= y1)
        )
        keep = (height >= self._z_band[0]) & (height <= self._z_band[1]) & ~own
        used = self._grid.integrate(
            m_map_cloud[:2, 3], in_map[keep, :2], self._bins, self._range[1], self._range[0]
        )
        if used:
            self._dirty = True

    def publish_tick(self):
        """Publish the grid and the status while mapping, when something changed."""
        if self._active and self._dirty:
            self._dirty = False
            self._message = f'mapping, {self._grid.scans} scans'
            self.publish_grid()
            self.publish_status()

    def grid_msg(self) -> OccupancyGrid:
        """Return the grid as an OccupancyGrid in the map frame."""
        data = self._grid.occupancy()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(self._grid.resolution)
        msg.info.width, msg.info.height = int(data.shape[1]), int(data.shape[0])
        msg.info.origin.position.x = float(self._grid.origin_x)
        msg.info.origin.position.y = float(self._grid.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', data.tobytes())
        return msg

    def publish_grid(self):
        """Publish the grid on mapping/grid (latched)."""
        self._grid_pub.publish(self.grid_msg())

    def publish_status(self):
        """Publish mapping/status (latched)."""
        msg = MappingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = self._active
        msg.site = self._site
        msg.scans = int(self._grid.scans)
        msg.resolution = float(self._grid.resolution)
        msg.width, msg.height = self._grid.width, self._grid.height
        msg.known_area_m2 = self._grid.known_area()
        msg.message = self._message
        self._status_pub.publish(msg)

    def start_callback(self, request, response):
        """Service: start (or keep) mapping; data=true starts the grid from scratch."""
        if request.data:
            self._grid = self.new_grid()
        self._active = True
        self.subscribe_cloud(True)
        self._dirty = True
        self._message = 'mapping from scratch' if request.data else 'mapping, map continued'
        self.publish_status()
        response.success = True
        response.message = self._message
        self.get_logger().info(f'{self._message} (site {self._site or "?"})')
        return response

    def stop_callback(self, request, response):
        """Service: stop integrating scans; the grid stays for save."""
        self._active = False
        self.subscribe_cloud(False)
        self._message = f'stopped after {self._grid.scans} scans'
        self.publish_grid()
        self.publish_status()
        response.success = True
        response.message = self._message
        self.get_logger().info(self._message)
        return response

    def save_callback(self, request, response):
        """Service: write map.pgm + map.yaml into the directory request.path."""
        directory = request.path.strip()
        if not directory:
            response.success, response.message = False, 'empty path'
            return response
        grid = crop_known(self._grid.to_grid_map(), self._save_margin)
        if not (grid.data >= 0).any():
            response.success, response.message = False, 'nothing mapped yet'
            return response
        try:
            path = save_map(directory, grid)
        except OSError as error:
            response.success, response.message = False, f'cannot save in {directory}: {error}'
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = (
            f'saved {path}: {grid.width}x{grid.height} cells, {self._grid.known_area():.0f} m2'
        )
        self.get_logger().info(response.message)
        return response

    def load_callback(self, request, response):
        """Service: load a saved map (directory or map.yaml) as the grid to continue."""
        try:
            grid = load_map(request.path.strip())
            self._grid.load_prior(grid)
        except (MapError, ValueError) as error:
            response.success, response.message = False, str(error)
            self.get_logger().error(f'grid_mapper/load: {error}')
            return response
        self._message = f'continuing {request.path.strip()}'
        self._dirty = True
        self.publish_status()
        response.success = True
        response.message = f'loaded {grid.width}x{grid.height} cells as the prior'
        return response


def main(args=None):
    """Spin the grid mapper."""
    rclpy.init(args=args)
    node = GridMapperNode()
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
