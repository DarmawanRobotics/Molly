"""
The camera, LiDAR and AprilTag streams and the robot pose, while a handheld uses them.

Deserialising a 30 Hz camera, a 10 Hz cloud and every TF message cost a third of a Jetson core
while nobody was connected: the subscriptions exist only while a client wants them.
"""

from __future__ import annotations

import time

import numpy as np
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2

from dharma_utils.math import transform_points
from dharma_utils.ros import SENSOR, TfCache
from dharma_utils.ros.pointcloud import CloudError, read_xyz
from molly_bridge.core import protocol
from molly_bridge.core.config import TeleopConfig
from molly_bridge.utils import camera

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:  # no AprilTag detector here: no tag overlay on the handheld video
    AprilTagDetectionArray = None


class Streams:
    """On-demand subscriptions; `send(stream, payload)` delivers a binary frame."""

    def __init__(self, node, cfg: TeleopConfig, frames, send, warn):
        self.node, self.cfg, self.frames, self.send, self.warn = node, cfg, frames, send, warn
        self.subs: dict = {}
        self.tf: TfCache | None = None
        self.last_camera = self.last_lidar = 0.0
        self.camera_period = 1.0 / cfg.camera_fps
        self.lidar_period = 1.0 / cfg.lidar_hz
        self.tags: list = []
        self.tags_time = 0.0
        self.tags_size = (0, 0)
        kind = CompressedImage if cfg.camera_compressed else Image
        sources = {
            'camera': (('camera',), kind, cfg.camera_topic, self.on_camera),
            'lidar': (('lidar',), PointCloud2, cfg.lidar_topic, self.on_lidar),
        }
        if AprilTagDetectionArray is not None and cfg.tags_topic:
            video = ('camera', 'video')
            sources['tags'] = (video, AprilTagDetectionArray, cfg.tags_topic, self.on_tags)
            sources['tags_info'] = (video, CameraInfo, cfg.tags_camera_info_topic, self.on_info)
        self.sources = {name: src for name, src in sources.items() if src[2]}

    def sync(self, live: bool, streams: dict):
        """Hold the subscriptions (and TF) a connected client uses, drop the others."""
        if live and self.tf is None:
            self.tf = TfCache(self.node)
        elif not live and self.tf is not None:
            self.tf.listener.unregister()
            self.tf = None
        for name, (wanted_by, kind, topic, callback) in self.sources.items():
            wanted = live and any(streams.get(stream) for stream in wanted_by)
            subscription = self.subs.get(name)
            if wanted and subscription is None:
                self.subs[name] = self.node.create_subscription(kind, topic, callback, SENSOR)
            elif not wanted and subscription is not None:
                self.node.destroy_subscription(self.subs.pop(name))

    def set_camera_fps(self, fps):
        """Let the handheld lower or restore the camera rate, capped by camera_fps."""
        value = protocol.clamp_axis(fps, 1000.0)
        if value > 0.0:
            self.camera_period = 1.0 / min(max(value, 0.5), self.cfg.camera_fps)

    def pose(self) -> list | None:
        """Return the robot [x, y, yaw] in the map, or None."""
        if self.tf is None:
            return None
        result = self.tf.lookup(self.frames.map, self.frames.base)
        if result is None:
            return None
        p = result.pose2d
        return [round(p.x, 3), round(p.y, 3), round(p.yaw, 3)]

    def tag_quads(self) -> list:
        """Tags in the camera image now, corners as image fractions, for the overlay."""
        if not self.tags or time.monotonic() - self.tags_time > self.cfg.tags_max_age_sec:
            return []
        return protocol.tag_quads(self.tags, *self.tags_size)

    # ---------------------------------------------------------------- callbacks
    def on_camera(self, msg):
        """Forward a throttled JPEG frame."""
        now = time.monotonic()
        if now - self.last_camera < self.camera_period:
            return
        self.last_camera = now
        cfg = self.cfg
        if isinstance(msg, CompressedImage):
            jpeg, note = camera.shrink(bytes(msg.data), cfg.camera_max_bytes,
                                       cfg.camera_max_width, cfg.camera_jpeg_quality)  # fmt: skip
        else:
            jpeg, note = camera.encode(msg, cfg.camera_max_width, cfg.camera_jpeg_quality)
        if note:
            self.warn(note)
        if jpeg:
            self.send(protocol.STREAM_CAMERA, jpeg)

    def on_lidar(self, msg: PointCloud2):
        """Forward a throttled, voxel-downsampled cloud in the robot frame."""
        now = time.monotonic()
        if now - self.last_lidar < self.lidar_period or self.tf is None:
            return
        self.last_lidar = now
        mount = self.tf.static(self.frames.base, msg.header.frame_id)
        if mount is None:
            return
        try:
            xyz = read_xyz(msg)
        except CloudError:
            return
        xyz = transform_points(mount, xyz).astype(np.float32)
        xyz = protocol.voxel_downsample(xyz, self.cfg.lidar_voxel, self.cfg.lidar_max_points)
        self.send(protocol.STREAM_LIDAR, protocol.pack_points(xyz))

    def on_tags(self, msg):
        """Keep the latest AprilTag detections (pixel corners of the detector's image)."""
        self.tags = [(d.id, [(c.x, c.y) for c in d.corners]) for d in msg.detections]
        self.tags_time = time.monotonic()

    def on_info(self, msg: CameraInfo):
        """Keep the size of the image the AprilTag detector sees."""
        self.tags_size = (msg.width, msg.height)
