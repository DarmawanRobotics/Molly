"""The UDP codec: chunks, points, axes and JSON (no ROS)."""

from __future__ import annotations

import random

from molly_bridge.core import protocol
import numpy as np

from dharma_utils.math import default_rng


def test_tag_quads_are_image_fractions():
    """Tag corners in pixels become fractions of the image; no image size, no tags."""
    tags = [(7, [(0, 0), (320, 0), (320, 240), (0, 240)])]
    assert protocol.tag_quads(tags, 640, 480) == [[7, 0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.0, 0.5]]
    assert protocol.tag_quads(tags, 0, 0) == []


def test_chunks_reassemble_out_of_order():
    """Chunked payloads survive reordering."""
    payload = bytes(random.getrandbits(8) for _ in range(9000))
    datagrams = protocol.chunk(protocol.STREAM_CAMERA, 5, payload)
    assert all(len(d) <= protocol.HEADER.size + protocol.CHUNK_PAYLOAD for d in datagrams)
    random.shuffle(datagrams)
    r = protocol.Reassembler()
    done = [x for d in datagrams if (x := r.push(d))]
    assert done == [(protocol.STREAM_CAMERA, 5, payload)]


def test_incomplete_frame_is_dropped_for_a_newer_one():
    """A lost chunk only costs that frame."""
    r = protocol.Reassembler()
    for d in protocol.chunk(1, 1, bytes(5000))[:-1]:
        assert r.push(d) is None
    assert r.push(protocol.chunk(1, 2, b'ok')[0]) == (1, 2, b'ok')


def test_points_roundtrip_within_a_centimetre():
    """Packing to int16 cm keeps points within 5 mm."""
    xyz = default_rng(3).uniform(-20, 20, (500, 3)).astype(np.float32)
    assert np.abs(protocol.unpack_points(protocol.pack_points(xyz)) - xyz).max() <= 0.005 + 1e-6


def test_voxel_downsample_caps_points():
    """Downsampling keeps at most max_points."""
    xyz = default_rng(3).uniform(-5, 5, (20000, 3)).astype(np.float32)
    assert protocol.voxel_downsample(xyz, 0.1, 1000).shape[0] == 1000


def test_clamp_axis_rejects_garbage():
    """NaN, inf, strings and bools never turn into a speed; numbers are clamped."""
    assert protocol.clamp_axis(0.4, 1.0) == 0.4
    assert protocol.clamp_axis(5.0, 1.0) == 1.0
    assert protocol.clamp_axis(-5.0, 1.0) == -1.0
    assert protocol.clamp_axis('0.3', 1.0) == 0.3
    for bad in (float('nan'), float('inf'), float('-inf'), None, 'fast', [1], {}, True):
        assert protocol.clamp_axis(bad, 1.0) == 0.0


def test_decode_json_needs_an_object_with_a_type():
    """Only JSON objects with a 't' field are messages."""
    assert protocol.decode_json(b'{"t": "ping"}') == {'t': 'ping'}
    for bad in (b'', b'\xff\xfe', b'[1, 2]', b'{"x": 1}', b'not json'):
        assert protocol.decode_json(bad) is None
