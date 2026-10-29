"""GStreamer pipelines for the handheld's H.264 video: raw ROS images in, RTSP out (no ROS)."""

from __future__ import annotations

from fractions import Fraction

import numpy as np

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
    return max(2, int(width) // 2 * 2), max(2, round(height) // 2 * 2)


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
