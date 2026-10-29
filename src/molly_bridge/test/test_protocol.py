from __future__ import annotations

import random
import types

import numpy as np
import pytest

from molly_bridge import protocol


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
    xyz = np.random.uniform(-20, 20, (500, 3)).astype(np.float32)
    assert np.abs(protocol.unpack_points(protocol.pack_points(xyz)) - xyz).max() <= 0.005 + 1e-6


def test_voxel_downsample_caps_points():
    """Downsampling keeps at most max_points."""
    xyz = np.random.uniform(-5, 5, (20000, 3)).astype(np.float32)
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


def test_discovery_beacon_and_seq():
    """A discover probe gets a beacon without the token; states carry an increasing seq."""
    import socket

    node_module = pytest.importorskip('molly_bridge.teleop_udp_node')
    node = node_module.TeleopUdpNode.__new__(node_module.TeleopUdpNode)
    node._robot_name, node._token, node._client, node._client_seen = 'dog-01', 'secret', None, 0.0
    node._client_timeout, node._client_name, node._seq = 1.0, '', 0
    node._state = {'mode': 'remote', 'battery': {'pct': 80}}
    node._speed, node._people = None, None
    node.init_guide(1024, 3.0, [])
    node._site_name = 'hall'
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    node._sock = server
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    probe.settimeout(1)
    node.reply_beacon(probe.getsockname())
    beacon = protocol.decode_json(probe.recv(2048))
    expected = {
        't': 'beacon',
        'name': 'dog-01',
        'secured': True,
        'in_use_by': '',
        'mode': 'remote',
    }
    assert {k: beacon[k] for k in expected} == expected and beacon['battery'] == 80
    assert beacon['site'] == 'hall'
    sent = []
    node._client, node.pose, node.tags = ('127.0.0.1', 1), lambda: None, list
    node.send_json = sent.append
    node.send_state()
    node.send_state()
    assert [m['seq'] for m in sent] == [1, 2]


def test_camera_rate_from_the_handheld_is_bounded():
    """camera_fps from the client is capped by the parameter; NaN, text and bools are ignored."""
    node_module = pytest.importorskip('molly_bridge.teleop_udp_node')
    node = node_module.TeleopUdpNode.__new__(node_module.TeleopUdpNode)
    node.get_parameter = lambda name: types.SimpleNamespace(value=8.0)
    node._camera_period = 0.125
    node.set_camera_fps(2.0)
    assert node._camera_period == 0.5
    node.set_camera_fps(100)
    assert node._camera_period == 0.125
    for bad in (float('nan'), 'fast', True, None):
        node.set_camera_fps(bad)
        assert node._camera_period == 0.125
