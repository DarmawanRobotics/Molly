"""Tests of video_stream_node: frame layout, pipelines, and the encoders of this computer."""
from __future__ import annotations

import numpy as np
import pytest

video = pytest.importorskip('molly_bridge.video_stream_node')

URL = 'rtsp://127.0.0.1:8654/camera'


def test_output_size_keeps_the_aspect_with_even_sides():
    """Wide cameras are scaled to max_width; H.264 4:2:0 needs even sides."""
    assert video.output_size(640, 480, 1280) == (640, 480)
    assert video.output_size(1920, 1080, 1280) == (1280, 720)
    assert video.output_size(1000, 333, 500) == (500, 166)
    assert video.output_size(1281, 721, 0) == (1280, 720)  # 0: any width


def test_rows_are_copied_to_the_gstreamer_stride():
    """Rows padded by the driver are re-padded to the 4-byte rows GStreamer expects."""
    width, height, step = 5, 2, 20  # RGB: 15 bytes of pixels in a 20-byte row
    assert video.stride(width, 3) == 16 and video.stride(4, 4) == 16 and video.stride(6, 1) == 8
    data = bytes(range(step * height))
    rows = np.frombuffer(video.pack_rows(data, height, step, 15, 16), np.uint8).reshape(2, 16)
    assert rows[1, :15].tolist() == list(range(20, 35)) and rows[1, 15] == 0
    assert video.pack_rows(data, height, step, 15, step) == data


def test_caps_and_pipelines():
    """ROS encodings map to raw caps; every encoder gets the same appsrc and RTSP ends."""
    assert video.raw_caps('rgb8', 640, 480, 15.0) == (
        'video/x-raw,format=RGB,width=640,height=480,framerate=15/1'
    )
    assert video.raw_caps('mono8', 2, 2, 7.5).endswith('framerate=15/2')
    for encoder in video.ENCODERS:
        text = video.pipeline_description(encoder, 640, 480, 1500, 15, URL)
        assert text.startswith('appsrc name=src ! queue leaky=downstream')
        assert 'name=enc' in text and text.endswith(f'location="{URL}" protocols=tcp latency=0')
    with pytest.raises(ValueError):
        video.encoder_chain('vp8enc', 640, 480, 1500, 15)


def gstreamer():
    """Return GStreamer, initialised, or skip the test."""
    if video.Gst is None:
        pytest.skip('no GStreamer for Python (python3-gi)')
    video.Gst.init(None)
    return video.Gst


def test_the_software_encoders_take_our_options():
    """x264 (and OpenH264 when installed) encode test frames with the node's exact options."""
    Gst = gstreamer()
    if Gst.ElementFactory.find('x264enc') is None:
        pytest.skip('x264enc not installed (gstreamer1.0-plugins-ugly)')
    assert video.probe('x264enc') is None
    if Gst.ElementFactory.find('openh264enc') is not None:
        assert video.probe('openh264enc') is None
    if Gst.ElementFactory.find('nvv4l2h264enc') is None:  # not a Jetson: a reason, no crash
        assert video.probe('nvv4l2h264enc')


def test_the_streaming_pipeline_builds():
    """The whole camera -> H.264 -> RTSP pipeline parses with this GStreamer."""
    Gst = gstreamer()
    for name in ('x264enc', 'h264parse', 'rtspclientsink'):
        if Gst.ElementFactory.find(name) is None:
            pytest.skip(f'{name} not installed')
    pipeline = Gst.parse_launch(video.pipeline_description('x264enc', 640, 480, 1500, 15, URL))
    assert pipeline.get_by_name('src') is not None and pipeline.get_by_name('enc') is not None
    pipeline.set_state(Gst.State.NULL)
