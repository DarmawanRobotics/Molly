#!/usr/bin/env python3
import json
import math
import os
import time

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from molly_localization.utils.pose_localization import (
    map_base_from_tag,
    map_odom_from_map_base,
    matrix_to_transform,
    planar_delta,
    pose_msg_to_matrix,
    pose_values_to_matrix,
    transform_to_matrix,
)
from molly_localization.utils.tag_recording import average_poses, describe_pose, merge_into_file
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)
import tf_transformations as tft
from visualization_msgs.msg import Marker, MarkerArray


class LocalizationNode(Node):
    """Bridges FAST-LIO frames into map->odom->base_link and corrects map->odom from AprilTags."""

    def __init__(self):
        """Declare params, load tag poses, and start the frame bridge."""
        super().__init__('localization_node')
        self.declare_parameter('tags_config_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lio_odom_frame', 'camera_init')
        self.declare_parameter('lio_body_frame', 'body')
        self.declare_parameter('imu_frame', 'livox_imu_frame')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_max_age_sec', 0.5)
        self.declare_parameter('auto_localize_on_start', True)
        self.declare_parameter('assume_start_origin', False)
        self.declare_parameter('tag_frames', ['base_map', 'charging_dock'])
        self.declare_parameter('tag_record_duration_sec', 2.0)
        self.declare_parameter('tags_record_file', '')
        self.declare_parameter('tag_marker_size', 0.15)
        self.declare_parameter('auto_correct_period_sec', 0.0)
        self.declare_parameter('max_correction_m', 0.5)
        self.declare_parameter('max_correction_deg', 15.0)
        self.declare_parameter('map_odom_hz', 20.0)

        p = self.get_parameter
        self._map = p('map_frame').value
        self._odom = p('odom_frame').value
        self._base = p('base_frame').value
        self._lio_odom = p('lio_odom_frame').value
        self._lio_body = p('lio_body_frame').value
        self._imu = p('imu_frame').value
        self._camera = p('camera_frame').value
        self._tag_max_age = Duration(seconds=float(p('tag_max_age_sec').value))
        self._max_corr_m = float(p('max_correction_m').value)
        self._max_corr_rad = math.radians(float(p('max_correction_deg').value))

        self._tag_poses = self.load_tag_poses(p('tags_config_file').value)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._m_base_imu = None
        self._m_map_odom = np.eye(4)
        self._localized = False
        self._active_source = 'none'

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, 'localization/status', latched)
        self._assume_start_origin = bool(p('assume_start_origin').value)
        self.publish_status()

        self._tags_file = p('tags_config_file').value
        self._tag_markers_pub = self.create_publisher(
            MarkerArray, 'localization/tag_markers', latched
        )
        self.publish_tag_markers()

        self.create_service(Trigger, 'localization/start', self.localization_callback)
        self.create_service(Trigger, 'localization/record_tags', self.record_tags_callback)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_estimate_callback, 10
        )
        self.create_subscription(Empty, 'odometry/restarted', self.odometry_restarted_callback, 10)
        self._auto_timer = None

        self._bridge_timer = self.create_timer(0.5, self.try_start_bridge)
        self.create_timer(1.0 / float(p('map_odom_hz').value), self.publish_map_odom)
        self._auto_start = bool(p('auto_localize_on_start').value)
        period = float(p('auto_correct_period_sec').value)
        if period > 0.0:
            self.create_timer(period, self.auto_correct)

        self.get_logger().info(f'localization_node ready with {len(self._tag_poses)} tag(s)')

    def load_tag_poses(self, filepath: str) -> dict:
        """
        Load a {tag_frame: {x,y,z,qx,qy,qz,qw}} JSON file into tag_frame -> 4x4 matrix.

        A missing file is normal before the first teach (localization/record_tags creates it);
        a broken file or entry is reported and skipped instead of crashing the node.
        """
        if not filepath:
            self.get_logger().warn('tags_config_file not set, AprilTag localization disabled')
            return {}
        try:
            with open(filepath, 'r') as f:
                raw = json.load(f)
        except FileNotFoundError:
            self.get_logger().warn(
                f'{filepath} does not exist yet, record tags with localization/record_tags'
            )
            return {}
        except (OSError, ValueError) as error:
            self.get_logger().error(f'cannot read tag poses from {filepath}: {error}')
            return {}
        if not isinstance(raw, dict):
            self.get_logger().error(f'{filepath} must be a JSON object of tag poses')
            return {}
        poses = {}
        for frame, v in raw.items():
            try:
                poses[frame] = pose_values_to_matrix(
                    [float(v[k]) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
                )
            except (KeyError, TypeError, ValueError) as error:
                self.get_logger().error(f'{filepath}: tag "{frame}" skipped ({error})')
        return poses

    def try_start_bridge(self):
        """Wait for base_link->imu from the URDF, then publish the static frame bridge once."""
        try:
            tf = self._tf_buffer.lookup_transform(self._base, self._imu, Time())
        except TransformException as error:
            self.get_logger().warn(
                f'waiting for URDF TF {self._base}->{self._imu}: {error}',
                throttle_duration_sec=5.0,
            )
            return
        self._m_base_imu = transform_to_matrix(tf)
        self._bridge_timer.cancel()
        self.broadcast_static()
        self.get_logger().info('frame bridge up: map->odom->camera_init, body->base_link')
        if self._assume_start_origin:
            self._localized = True
            self._active_source = 'start position'
            self.publish_status()
        if self._auto_start and self._tag_poses:
            self.start_auto_localize()

    def start_auto_localize(self):
        """(Re)start retrying tag localization every second."""
        if self._auto_timer is not None:
            self._auto_timer.cancel()
        self._auto_timer = self.create_timer(1.0, self.auto_start_tick)

    def odometry_restarted_callback(self, _msg: Empty):
        """FAST-LIO restarted at a new origin: the old map->odom no longer holds."""
        if not self._localized:
            return
        self._localized = False
        self.publish_status()
        self.get_logger().error(
            'odometry restarted, map->odom invalid: show a tag or set a 2D Pose Estimate'
        )
        if self._tag_poses:
            self.start_auto_localize()

    def publish_status(self):
        """Publish the active map->odom source ('none' until localized), latched."""
        self._status_pub.publish(String(data=self._active_source if self._localized else 'none'))

    def broadcast_static(self):
        """Publish the fixed bridge: odom->camera_init (= base->imu), body->base_link (inverse)."""
        stamp = self.get_clock().now().to_msg()
        self._static_broadcaster.sendTransform(
            [
                matrix_to_transform(self._m_base_imu, self._odom, self._lio_odom, stamp),
                matrix_to_transform(
                    np.linalg.inv(self._m_base_imu), self._lio_body, self._base, stamp
                ),
            ]
        )
        self.publish_map_odom()

    def publish_map_odom(self):
        """
        Publish map->odom on /tf (REP-105), at map_odom_hz and right after every change.

        It used to be a static transform, but it changes on every (re)localization and
        StaticTransformBroadcaster keeps the first transform per child frame on newer ROS 2,
        so corrections never reached the other nodes there.
        """
        if self._m_base_imu is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(
            matrix_to_transform(self._m_map_odom, self._map, self._odom, stamp)
        )

    def set_map_odom(self, m_map_odom: np.ndarray, source: str):
        """Store and broadcast a new map->odom."""
        self._m_map_odom = m_map_odom
        self._localized = True
        self._active_source = source
        self.broadcast_static()
        self.publish_status()
        t = m_map_odom[:3, 3]
        yaw = math.degrees(math.atan2(m_map_odom[1, 0], m_map_odom[0, 0]))
        self.get_logger().info(f'map->odom from {source}: x={t[0]:.3f} y={t[1]:.3f} yaw={yaw:.1f}')

    def fresh_tag(self):
        """Return (tag_frame, cam->tag TF) for the first tag seen within tag_max_age_sec."""
        now = self.get_clock().now()
        for frame in self._tag_poses:
            try:
                tf = self._tf_buffer.lookup_transform(self._camera, frame, Time())
            except TransformException:
                continue
            if now - Time.from_msg(tf.header.stamp) <= self._tag_max_age:
                return frame, tf
        return None, None

    def map_odom_from_visible_tag(self):
        """Compute map->odom from a fresh tag: (matrix, tag_frame) or (None, reason)."""
        if self._m_base_imu is None:
            return None, 'frame bridge not ready'
        if not self._tag_poses:
            return None, 'no tags configured'
        frame, cam_tag = self.fresh_tag()
        if frame is None:
            return None, f'no fresh detection of {list(self._tag_poses)}'
        stamp = Time.from_msg(cam_tag.header.stamp)
        try:
            base_cam = self._tf_buffer.lookup_transform(self._base, self._camera, Time())
            odom_base = self._tf_buffer.lookup_transform(
                self._odom, self._base, stamp, Duration(seconds=0.2)
            )
        except TransformException as error:
            return None, f'TF lookup failed: {error}'
        m_map_base = map_base_from_tag(
            self._tag_poses[frame], transform_to_matrix(base_cam), transform_to_matrix(cam_tag)
        )
        return map_odom_from_map_base(m_map_base, transform_to_matrix(odom_base)), frame

    def localization_callback(self, request, response):
        """Service: set map->odom from whichever configured tag is currently visible."""
        m, info = self.map_odom_from_visible_tag()
        if m is None:
            response.success = False
            response.message = info
            self.get_logger().error(info)
            return response
        self.set_map_odom(m, f'apriltag "{info}"')
        response.success = True
        response.message = f'map->odom set from tag "{info}"'
        return response

    def auto_start_tick(self):
        """Retry tag localization every second until the first success."""
        m, info = self.map_odom_from_visible_tag()
        if m is None:
            self.get_logger().info(f'auto localize waiting: {info}', throttle_duration_sec=10.0)
            return
        self.set_map_odom(m, f'apriltag "{info}" (auto start)')
        self._auto_timer.cancel()
        self._auto_timer = None

    def auto_correct(self):
        """Periodically correct drift from any visible tag, rejecting implausible jumps."""
        if not self._localized:
            return
        m, info = self.map_odom_from_visible_tag()
        if m is None:
            return
        dist, dyaw = planar_delta(m, self._m_map_odom)
        if dist > self._max_corr_m or dyaw > self._max_corr_rad:
            self.get_logger().warn(
                f'rejected correction from "{info}": {dist:.2f} m / {math.degrees(dyaw):.1f} deg'
            )
            return
        self.set_map_odom(m, f'apriltag "{info}" (auto correct)')

    def record_tags_callback(self, request, response):
        """Average the map pose of every visible tag in tag_frames and save it to the tag file."""
        duration = float(self.get_parameter('tag_record_duration_sec').value)
        frames = [f for f in self.get_parameter('tag_frames').value if f]
        samples = {frame: [] for frame in frames}
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for frame in frames:
                m = self.fresh_map_tag(frame)
                if m is not None:
                    samples[frame].append(m)
            time.sleep(0.1)

        poses, lines = {}, []
        for frame, values in samples.items():
            if len(values) < 3:
                lines.append(f'{frame}: not seen')
                continue
            mean, pos_std, yaw_std = average_poses(values)
            poses[frame] = mean
            lines.append(
                f'{frame}: {describe_pose(mean)} n={len(values)} '
                f'std={pos_std * 100:.1f}cm/{yaw_std:.1f}deg'
            )
        if not poses:
            response.success = False
            response.message = 'no tag seen: ' + '; '.join(lines)
            return response

        path = self.get_parameter('tags_record_file').value or self._tags_file
        path = path or os.path.join(os.getcwd(), 'tag_config.json')
        merged = merge_into_file(path, poses)
        self._tag_poses = {
            f: pose_values_to_matrix([v['x'], v['y'], v['z'], v['qx'], v['qy'], v['qz'], v['qw']])
            for f, v in merged.items()
        }
        self.publish_tag_markers()
        response.success = True
        response.message = f'saved {len(poses)} tag(s) to {path}\n' + '\n'.join(lines)
        self.get_logger().info(response.message)
        return response

    def fresh_map_tag(self, frame: str):
        """Return map->tag as a 4x4 matrix if the tag was detected within tag_max_age_sec."""
        try:
            cam_tag = self._tf_buffer.lookup_transform(self._camera, frame, Time())
            if self.get_clock().now() - Time.from_msg(cam_tag.header.stamp) > self._tag_max_age:
                return None
            return transform_to_matrix(self._tf_buffer.lookup_transform(self._map, frame, Time()))
        except TransformException:
            return None

    def publish_tag_markers(self):
        """Publish known tags as plates with x/y/z axes, a label and a ground line (latched)."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        size = float(self.get_parameter('tag_marker_size').value)
        axes = (
            (np.eye(4), (1.0, 0.1, 0.1)),
            (tft.euler_matrix(0.0, 0.0, math.pi / 2), (0.1, 1.0, 0.1)),
            (tft.euler_matrix(0.0, -math.pi / 2, 0.0), (0.2, 0.4, 1.0)),
        )
        for i, (frame, m) in enumerate(sorted(self._tag_poses.items())):
            mid = 10 * i
            plate = self.tag_marker(Marker.CUBE, mid, m, (1.0, 1.0, 1.0, 0.9))
            plate.scale.x, plate.scale.y, plate.scale.z = size, size, 0.005
            markers.markers.append(plate)
            for k, (rot, rgb) in enumerate(axes):
                axis = self.tag_marker(Marker.ARROW, mid + 1 + k, m @ rot, (*rgb, 1.0))
                axis.scale.x, axis.scale.y, axis.scale.z = 0.25, 0.02, 0.02
                markers.markers.append(axis)
            text = self.tag_marker(Marker.TEXT_VIEW_FACING, mid + 4, m, (1.0, 0.3, 0.9, 1.0))
            text.pose.position.z += 0.25
            text.scale.z = 0.15
            text.text = frame
            markers.markers.append(text)
            drop = self.tag_marker(Marker.LINE_LIST, mid + 5, np.eye(4), (1.0, 0.3, 0.9, 0.6))
            drop.scale.x = 0.01
            x, y, z = (float(v) for v in m[:3, 3])
            drop.points = [Point(x=x, y=y, z=z), Point(x=x, y=y, z=0.0)]
            markers.markers.append(drop)
        self._tag_markers_pub.publish(markers)

    def tag_marker(self, kind: int, mid: int, m: np.ndarray, rgba: tuple) -> Marker:
        """Return a map-frame marker posed at the 4x4 matrix m."""
        mk = Marker()
        mk.header.frame_id = self._map
        mk.ns = 'tags'
        mk.id = mid
        mk.type = kind
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = (float(v) for v in m[:3, 3])
        q = tft.quaternion_from_matrix(m)
        mk.pose.orientation.x, mk.pose.orientation.y = float(q[0]), float(q[1])
        mk.pose.orientation.z, mk.pose.orientation.w = float(q[2]), float(q[3])
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = rgba
        return mk

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped):
        """Set map->odom so base_link lands on an RViz 2D Pose Estimate."""
        if self._m_base_imu is None:
            self.get_logger().error('2D Pose Estimate ignored, frame bridge not ready')
            return
        try:
            odom_base = self._tf_buffer.lookup_transform(self._odom, self._base, Time())
        except TransformException as error:
            self.get_logger().error(f'2D Pose Estimate ignored, TF not ready: {error}')
            return
        m = map_odom_from_map_base(
            pose_msg_to_matrix(msg.pose.pose), transform_to_matrix(odom_base)
        )
        self.set_map_odom(m, 'RViz 2D Pose Estimate')


def main(args=None):
    """Spin the localization node; a second thread keeps TF flowing during blocking lookups."""
    rclpy.init(args=args)
    node = LocalizationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
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
