"""Configuration of teleop_udp_node and video_stream_node (typed parameters)."""

from __future__ import annotations

from dataclasses import dataclass

from dharma_utils.config import param
from molly_bridge.core import protocol

ROBOT_COMMANDS = ('stand_up', 'sit_down', 'emergency_stop')
GESTURES = ('hello', 'nod', 'bow', 'look_left', 'look_right')
ENCODERS = ('auto', 'nvv4l2h264enc', 'x264enc', 'openh264enc')


@dataclass(frozen=True)
class TeleopConfig:
    """The handheld's UDP gateway (read at start). Speed limits come from robot.limits."""

    port: int = param(protocol.PORT, min=1, max=65535, live=False, doc='UDP port')
    token: str = param('', live=False, doc='shared secret; when set every message carries k')
    robot_name: str = param('', live=False, doc='name in discovery beacons; empty: hostname')
    client_timeout_sec: float = param(
        2.0, min=0.2, max=60.0, unit='s', live=False, doc='a silent client is dropped'
    )
    telemetry_hz: float = param(10.0, min=0.5, max=50.0, unit='Hz', live=False, doc='state')
    remote_source: str = param(
        'handheld', live=False, doc='control mux source: control/remote/<source>/cmd_vel'
    )
    robot_command_prefix: str = param(
        '/robot/', live=False, doc='driver services (the bringup sets /<robot_ns>/)'
    )
    robot_commands: tuple[str, ...] = param(
        ROBOT_COMMANDS, live=False, doc="the driver's Trigger services in the action menu"
    )
    sites_dir: str = param('~/dev/Molly/map', live=False, doc='routes are listed from here')
    camera_topic: str = param(
        '/camera/camera/color/image_raw/compressed', live=False, doc='JPEG (or raw) camera'
    )
    camera_compressed: bool = param(True, live=False, doc='false: raw rgb8 / bgr8 Image')
    camera_fps: float = param(8.0, min=0.5, max=30.0, unit='Hz', live=False, doc='at most')
    camera_jpeg_quality: int = param(60, min=10, max=100, live=False, doc='re-encoded JPEGs')
    camera_max_width: int = param(640, min=64, max=4096, live=False, doc='px, wider scaled')
    camera_max_bytes: int = param(
        60000, min=5000, max=1000000, live=False, doc='bigger driver JPEGs are re-encoded'
    )
    video_port: int = param(
        0, min=0, max=65535, live=False, doc='WebRTC (WHEP) port of MediaMTX; 0: JPEG only'
    )
    video_path: str = param('camera', live=False, doc='MediaMTX path')
    tags_topic: str = param(
        '/perception/apriltag/detections', live=False, doc='tags over the video; empty: off'
    )
    tags_camera_info_topic: str = param(
        '/perception/apriltag/camera_info', live=False, doc='size of the tag image'
    )
    tags_max_age_sec: float = param(0.6, min=0.05, max=10.0, unit='s', live=False)
    lidar_topic: str = param('/cloud_registered_body', live=False, doc='cloud for the handheld')
    lidar_hz: float = param(2.0, min=0.1, max=20.0, unit='Hz', live=False, doc='at most')
    lidar_voxel: float = param(0.1, min=0.01, max=1.0, unit='m', live=False, doc='downsampling')
    lidar_max_points: int = param(3000, min=100, max=20000, live=False, doc='per frame')
    navigation_node: str = param(
        '/path_follower_node', live=False, doc='speed.cruise set from the handheld'
    )
    patrol_speed_min: float = param(
        0.1, min=0.01, max=2.0, unit='m/s', live=False, doc='slowest speed the handheld sets'
    )
    map_max_px: int = param(1024, min=64, max=8192, live=False, doc='map PNG per side')
    map_push_sec: float = param(
        3.0, min=0.5, max=60.0, unit='s', live=False, doc='between map pushes while it changes'
    )
    gestures: tuple[str, ...] = param(
        GESTURES, live=False, doc='offered until interaction/greeter names its own'
    )


@dataclass(frozen=True)
class VideoConfig:
    """H.264 for the handheld's WebRTC video (MediaMTX starts the node)."""

    image_topic: str = param(
        '/camera/camera/color/image_raw', live=False, doc='raw rgb8 / bgr8 / mono8 / yuv422'
    )
    rtsp_url: str = param(
        'rtsp://127.0.0.1:8654/camera', live=False, doc='MediaMTX publish URL (the launch sets it)'
    )
    fps: float = param(15.0, min=1.0, max=60.0, unit='Hz', live=False, doc='frames encoded')
    max_width: int = param(1280, min=64, max=4096, live=False, doc='px, wider is scaled down')
    bitrate_kbps: int = param(1500, min=100, max=20000, live=False, doc='H.264 bitrate')
    keyframe_sec: float = param(
        1.0, min=0.1, max=10.0, unit='s', live=False, doc='a new viewer starts at a key frame'
    )
    encoder: str = param('auto', choices=ENCODERS, live=False, doc='auto: the first that works')
