"""
video_stream_node: H.264 video of the camera for the handheld, published to MediaMTX (WebRTC).

MediaMTX starts this node when the handheld opens the WebRTC video (runOnDemand in the
configuration the bridge launch writes) and stops it with SIGINT once nobody watches. Frames go
through GStreamer: the Jetson hardware encoder when it works, else x264 or OpenH264, then over
RTSP to MediaMTX on the loopback interface.
"""

from __future__ import annotations

import contextlib
import sys
import time

import rclpy
from sensor_msgs.msg import Image

from dharma_utils.ros import SENSOR, DharmaNode, compat
from molly_bridge.core.config import VideoConfig
from molly_bridge.core.gstreamer import (
    ENCODERS,
    FORMATS,
    output_size,
    pack_rows,
    pipeline_description,
    raw_caps,
    stride,
)
from molly_bridge.utils.gst import GLib, Gst, probe, set_optional


class VideoStreamNode(DharmaNode):
    """Encode the camera to H.264 and publish it to MediaMTX while the handheld watches."""

    def __init__(self, **kwargs):
        super().__init__('video_stream_node', **kwargs)
        cfg = self.declare_config(VideoConfig).value
        self.url, self.topic = cfg.rtsp_url, cfg.image_topic
        self.fps = cfg.fps
        self.max_width = cfg.max_width
        self.bitrate = cfg.bitrate_kbps
        self.keyframes = max(1, round(self.fps * cfg.keyframe_sec))
        self.encoder = self.choose_encoder(cfg.encoder)
        self.pipeline = self.src = self.shape = None
        self.last = 0.0
        self.frames = 0
        self.warned_encoding = False
        self.running = True
        self.exit_code = 0
        self.create_subscription(Image, self.topic, self.on_image, SENSOR)
        self.every(0.2, self.poll_bus)
        self.waiting = self.every(5.0, self.warn_no_frames)
        self.log.info('video ready', encoder=self.encoder, url=self.url, camera=self.topic)

    def choose_encoder(self, wanted: str) -> str:
        """Return the first encoder that encodes test frames here."""
        candidates = ENCODERS if wanted == 'auto' else (wanted,)
        for encoder in candidates:
            reason = probe(encoder)
            if reason is None:
                return encoder
            self.log.info(f'{encoder} does not work here: {reason}')
        raise RuntimeError(
            f'no working H.264 encoder ({", ".join(candidates)}): scripts/setup.sh install webrtc'
        )

    def warn_no_frames(self):
        """Say so while the camera sends nothing (the handheld waits on a black video)."""
        if self.frames:
            self.waiting.cancel()
            return
        self.log.warning(f'no images on {self.topic} yet: is the camera running?')

    def on_image(self, msg: Image):
        """Push a frame into the pipeline, at most fps per second."""
        now = time.monotonic()
        if now - self.last < 0.75 / self.fps:  # the camera's own rate passes despite jitter
            return
        if msg.encoding not in FORMATS:
            if not self.warned_encoding:
                self.warned_encoding = True
                self.log.error(
                    f'cannot encode {msg.encoding} images, use one of {", ".join(FORMATS)}'
                )
            return
        shape = (msg.encoding, msg.width, msg.height)
        if shape != self.shape and not self.start_pipeline(shape):
            return
        self.last = now
        self.frames += 1
        _, bytes_per_pixel = FORMATS[msg.encoding]
        data = pack_rows(
            msg.data,
            msg.height,
            msg.step,
            msg.width * bytes_per_pixel,
            stride(msg.width, bytes_per_pixel),
        )
        self.src.emit('push-buffer', Gst.Buffer.new_wrapped(data))

    def start_pipeline(self, shape: tuple) -> bool:
        """(Re)build the pipeline for this image encoding and size."""
        self.stop_pipeline()
        encoding, width, height = shape
        out_width, out_height = output_size(width, height, self.max_width)
        description = pipeline_description(
            self.encoder, out_width, out_height, self.bitrate, self.keyframes, self.url
        )
        try:
            pipeline = Gst.parse_launch(description)
        except GLib.Error as error:  # an element is not installed
            self.fail(f'video pipeline: {error.message} (scripts/setup.sh install webrtc)')
            return False
        src = pipeline.get_by_name('src')
        caps = raw_caps(encoding, width, height, self.fps)
        src.set_property('caps', Gst.Caps.from_string(caps))
        src.set_property('is-live', True)
        src.set_property('do-timestamp', True)
        src.set_property('format', Gst.Format.TIME)
        set_optional(pipeline.get_by_name('enc'), self.encoder)
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            self.fail('video pipeline does not start')
            return False
        self.pipeline, self.src, self.shape = pipeline, src, shape
        self.log.info(
            f'{width}x{height} {encoding} -> {out_width}x{out_height} H.264 '
            f'{self.bitrate} kbit/s, {self.fps:g} fps, {self.encoder}'
        )
        return True

    def stop_pipeline(self):
        """Stop and drop the pipeline (closes the RTSP session)."""
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = self.src = self.shape = None

    def poll_bus(self):
        """Log pipeline warnings; stop the node on an error (MediaMTX starts it again)."""
        if self.pipeline is None:
            return
        bus = self.pipeline.get_bus()
        while self.running:
            message = bus.pop()  # every message: the bus would keep the others forever
            if message is None:
                return
            if message.type == Gst.MessageType.WARNING:
                warning, _ = message.parse_warning()
                self.log.warning(f'video pipeline: {warning.message}')
            elif message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self.fail(f'video pipeline: {error.message} ({debug})')
            elif message.type == Gst.MessageType.EOS:
                self.fail('video pipeline ended')

    def fail(self, reason: str):
        """Log why and end the node with exit code 1."""
        self.log.error(reason)
        self.running = False
        self.exit_code = 1


def main(argv=None):
    """Spin the video node until MediaMTX stops it (SIGINT) or the pipeline fails."""
    if Gst is None:
        sys.stderr.write('video_stream_node: no GStreamer for Python (setup.sh install webrtc)\n')
        sys.exit(1)
    Gst.init(None)
    compat.init(args=argv)
    try:
        node = VideoStreamNode()
    except RuntimeError as error:
        sys.stderr.write(f'video_stream_node: {error}\n')
        compat.try_shutdown()
        sys.exit(1)
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, compat.ExternalShutdownException):
        pass
    finally:
        node.stop_pipeline()
        node.destroy_node()
        with contextlib.suppress(Exception):
            compat.try_shutdown()
    sys.exit(node.exit_code)


if __name__ == '__main__':
    main()
