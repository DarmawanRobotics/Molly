#!/usr/bin/env python3
"""
H.264 video of the camera for the handheld, published to MediaMTX (WebRTC).

MediaMTX starts this node when the handheld opens the WebRTC video (runOnDemand in the
configuration molly robot.launch.py writes) and stops it with SIGINT once nobody watches.
Frames go through GStreamer: the Jetson hardware encoder when it works, else x264 or OpenH264,
then over RTSP to MediaMTX on the loopback interface.
"""

from fractions import Fraction
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import gi

    gi.require_version('Gst', '1.0')
    from gi.repository import GLib, Gst
except (ImportError, ValueError):  # python3-gi or gir1.2-gstreamer-1.0 missing
    GLib = Gst = None

# Tried in this order with encoder: auto. nvv4l2h264enc is the Jetson hardware encoder (not on
# the Orin Nano, which has none).
ENCODERS = ('nvv4l2h264enc', 'x264enc', 'openh264enc')

# Encoder properties set only when this GStreamer/JetPack version has them.
OPTIONAL = {
    'nvv4l2h264enc': {'maxperf-enable': True, 'preset-level': 1, 'poc-type': 2},
}

# ROS image encoding -> (GStreamer raw format, bytes per pixel)
FORMATS = {
    'rgb8': ('RGB', 3),
    'bgr8': ('BGR', 3),
    'rgba8': ('RGBA', 4),
    'bgra8': ('BGRA', 4),
    'mono8': ('GRAY8', 1),
    'yuv422': ('UYVY', 2),
    'uyvy': ('UYVY', 2),
    'yuv422_yuy2': ('YUY2', 2),
    'yuyv': ('YUY2', 2),
}


def output_size(width: int, height: int, max_width: int) -> tuple:
    """Return the encoded size: at most max_width wide, same aspect ratio, even sides."""
    if 0 < max_width < width:
        height = height * max_width / width
        width = max_width
    return max(2, int(width) // 2 * 2), max(2, int(round(height)) // 2 * 2)


def stride(width: int, bytes_per_pixel: int) -> int:
    """Row size GStreamer expects for a packed raw format: padded to 4 bytes."""
    return (width * bytes_per_pixel + 3) // 4 * 4


def pack_rows(data, height: int, step: int, row: int, target: int) -> bytes:
    """Copy image rows of `step` bytes (the first `row` of them are pixels) to `target` bytes."""
    if step == target:
        return bytes(memoryview(data)[: height * step])
    rows = np.frombuffer(data, dtype=np.uint8, count=height * step).reshape(height, step)
    packed = np.zeros((height, target), dtype=np.uint8)
    packed[:, :row] = rows[:, :row]
    return packed.tobytes()


def raw_caps(encoding: str, width: int, height: int, fps: float) -> str:
    """Return the GStreamer caps of a ROS image."""
    rate = Fraction(fps).limit_denominator(1001)
    return (
        f'video/x-raw,format={FORMATS[encoding][0]},width={width},height={height},'
        f'framerate={rate.numerator}/{rate.denominator}'
    )


def encoder_chain(encoder: str, width: int, height: int, bitrate_kbps: int, keyframes: int):
    """Return the GStreamer elements from raw video to H.264 (baseline, no B-frames)."""
    size = f'width={width},height={height}'
    if encoder == 'nvv4l2h264enc':  # the hardware converts, scales and encodes
        return (
            f'videoconvert ! video/x-raw,format=BGRx ! nvvidconv ! '
            f'video/x-raw(memory:NVMM),format=NV12,{size} ! '
            f'nvv4l2h264enc name=enc bitrate={bitrate_kbps * 1000} control-rate=1 profile=0 '
            f'iframeinterval={keyframes} idrinterval={keyframes} insert-sps-pps=true'
        )
    raw = f'videoconvert ! videoscale ! video/x-raw,format=I420,{size}'
    if encoder == 'x264enc':
        return (
            f'{raw} ! x264enc name=enc tune=zerolatency speed-preset=ultrafast '
            f'bitrate={bitrate_kbps} key-int-max={keyframes} bframes=0 ! '
            'video/x-h264,profile=constrained-baseline'
        )
    if encoder == 'openh264enc':
        return (
            f'{raw} ! openh264enc name=enc bitrate={bitrate_kbps * 1000} gop-size={keyframes} '
            'rate-control=bitrate complexity=low'
        )
    raise ValueError(f'unknown encoder {encoder!r}, use auto or one of {", ".join(ENCODERS)}')


def pipeline_description(
    encoder: str, width: int, height: int, bitrate_kbps: int, keyframes: int, url: str
) -> str:
    """Camera frames (appsrc 'src') -> H.264 -> RTSP to MediaMTX; old frames are dropped."""
    return (
        'appsrc name=src ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 '
        f'max-size-time=0 ! {encoder_chain(encoder, width, height, bitrate_kbps, keyframes)} ! '
        f'h264parse config-interval=-1 ! rtspclientsink location="{url}" protocols=tcp latency=0'
    )


def probe_description(encoder: str) -> str:
    """Return a pipeline of a few test frames through one encoder (does it work here?)."""
    return (
        'videotestsrc num-buffers=3 ! video/x-raw,width=320,height=240,framerate=15/1 ! '
        f'{encoder_chain(encoder, 320, 240, 500, 15)} ! h264parse ! fakesink'
    )


def set_optional(element, encoder: str):
    """Set the OPTIONAL properties of an encoder that this version has."""
    if element is None:
        return
    for name, value in OPTIONAL.get(encoder, {}).items():
        if element.find_property(name) is not None:
            element.set_property(name, value)


def probe(encoder: str, timeout_sec: float = 5.0):
    """Encode test frames with one encoder: None when it works, else the reason it does not."""
    try:
        pipeline = Gst.parse_launch(probe_description(encoder))
    except GLib.Error as error:  # the element is not installed
        return error.message
    set_optional(pipeline.get_by_name('enc'), encoder)
    try:
        pipeline.set_state(Gst.State.PLAYING)
        message = pipeline.get_bus().timed_pop_filtered(
            int(timeout_sec * Gst.SECOND), Gst.MessageType.EOS | Gst.MessageType.ERROR
        )
    finally:
        pipeline.set_state(Gst.State.NULL)
    if message is None:
        return f'no frames within {timeout_sec:g} s'
    if message.type == Gst.MessageType.ERROR:
        return message.parse_error()[0].message
    return None


class VideoStreamNode(Node):
    """Encode the camera to H.264 and publish it to MediaMTX while the handheld watches."""

    def __init__(self):
        """Declare params, pick a working encoder and subscribe to the camera."""
        super().__init__('video_stream_node')
        defaults = {
            'image_topic': '/camera/camera/color/image_raw',  # raw sensor_msgs/Image
            'rtsp_url': 'rtsp://127.0.0.1:8654/camera',  # robot.launch.py passes its own
            'fps': 15.0,
            'max_width': 1280,
            'bitrate_kbps': 1500,
            'keyframe_sec': 1.0,  # a new viewer waits for a key frame: keep this short
            'encoder': 'auto',  # auto | nvv4l2h264enc | x264enc | openh264enc
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self._url = p('rtsp_url')
        self._topic = p('image_topic')
        self._fps = min(max(float(p('fps')), 1.0), 60.0)
        self._max_width = int(p('max_width'))
        self._bitrate = max(int(p('bitrate_kbps')), 100)
        self._keyframes = max(1, round(self._fps * float(p('keyframe_sec'))))
        self._encoder = self.choose_encoder(str(p('encoder')))
        self._pipeline = self._src = self._shape = None
        self._last = 0.0
        self._frames = 0
        self._warned_encoding = False
        self.running = True
        self.exit_code = 0

        self.create_subscription(Image, self._topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.2, self.poll_bus)
        self._waiting = self.create_timer(5.0, self.warn_no_frames)
        self.get_logger().info(f'{self._encoder} -> {self._url}, camera {self._topic}')

    def choose_encoder(self, wanted: str) -> str:
        """Return the first encoder that encodes test frames here."""
        candidates = ENCODERS if wanted == 'auto' else (wanted,)
        for encoder in candidates:
            reason = probe(encoder)
            if reason is None:
                return encoder
            self.get_logger().info(f'{encoder} does not work here: {reason}')
        raise RuntimeError(
            f'no working H.264 encoder ({", ".join(candidates)}): script/setup.sh install webrtc'
        )

    def warn_no_frames(self):
        """Say so while the camera sends nothing (the handheld waits on a black video)."""
        if self._frames:
            self._waiting.cancel()
            return
        self.get_logger().warn(f'no images on {self._topic} yet: is the camera running?')

    def on_image(self, msg: Image):
        """Push a frame into the pipeline, at most fps per second."""
        now = time.monotonic()
        if now - self._last < 0.75 / self._fps:  # the camera's own rate passes despite jitter
            return
        if msg.encoding not in FORMATS:
            if not self._warned_encoding:
                self._warned_encoding = True
                self.get_logger().error(
                    f'cannot encode {msg.encoding} images, use one of {", ".join(FORMATS)}'
                )
            return
        shape = (msg.encoding, msg.width, msg.height)
        if shape != self._shape and not self.start_pipeline(shape):
            return
        self._last = now
        self._frames += 1
        _, bytes_per_pixel = FORMATS[msg.encoding]
        data = pack_rows(
            msg.data,
            msg.height,
            msg.step,
            msg.width * bytes_per_pixel,
            stride(msg.width, bytes_per_pixel),
        )
        self._src.emit('push-buffer', Gst.Buffer.new_wrapped(data))

    def start_pipeline(self, shape: tuple) -> bool:
        """(Re)build the pipeline for this image encoding and size."""
        self.stop_pipeline()
        encoding, width, height = shape
        out_width, out_height = output_size(width, height, self._max_width)
        description = pipeline_description(
            self._encoder, out_width, out_height, self._bitrate, self._keyframes, self._url
        )
        try:
            pipeline = Gst.parse_launch(description)
        except GLib.Error as error:  # an element is not installed
            self.fail(f'video pipeline: {error.message} (script/setup.sh install webrtc)')
            return False
        src = pipeline.get_by_name('src')
        caps = raw_caps(encoding, width, height, self._fps)
        src.set_property('caps', Gst.Caps.from_string(caps))
        src.set_property('is-live', True)
        src.set_property('do-timestamp', True)
        src.set_property('format', Gst.Format.TIME)
        set_optional(pipeline.get_by_name('enc'), self._encoder)
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            self.fail('video pipeline does not start')
            return False
        self._pipeline, self._src, self._shape = pipeline, src, shape
        self.get_logger().info(
            f'{width}x{height} {encoding} -> {out_width}x{out_height} H.264 '
            f'{self._bitrate} kbit/s, {self._fps:g} fps, {self._encoder}'
        )
        return True

    def stop_pipeline(self):
        """Stop and drop the pipeline (closes the RTSP session)."""
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
        self._pipeline = self._src = self._shape = None

    def poll_bus(self):
        """Log pipeline warnings; stop the node on an error (MediaMTX starts it again)."""
        if self._pipeline is None:
            return
        bus = self._pipeline.get_bus()
        while self.running:
            message = bus.pop()  # every message: the bus would keep the others forever
            if message is None:
                return
            if message.type == Gst.MessageType.WARNING:
                warning, _ = message.parse_warning()
                self.get_logger().warn(f'video pipeline: {warning.message}')
            elif message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self.fail(f'video pipeline: {error.message} ({debug})')
            elif message.type == Gst.MessageType.EOS:
                self.fail('video pipeline ended')

    def fail(self, reason: str):
        """Log why and end the node with exit code 1."""
        self.get_logger().error(reason)
        self.running = False
        self.exit_code = 1


def main(args=None):
    """Spin the video node until MediaMTX stops it (SIGINT) or the pipeline fails."""
    rclpy.init(args=args)
    if Gst is None:
        print(
            'video_stream_node: no GStreamer for Python, run script/setup.sh install webrtc',
            file=sys.stderr,
        )
        rclpy.shutdown()
        sys.exit(1)
    Gst.init(None)
    try:
        node = VideoStreamNode()
    except RuntimeError as error:
        print(f'video_stream_node: {error}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_pipeline()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass
    sys.exit(node.exit_code)


if __name__ == '__main__':
    main()
