"""Livox CustomMsg CDR parsing, checked against a hand-packed buffer and against rosbags."""

import struct

from molly_perception.livox_cdr import parse_custom_msg
import numpy as np
import pytest

POINTS = np.array(
    [[1.0, 2.0, 3.0], [0.0, 0.0, 0.0], [-4.5, 0.25, -0.5], [np.nan, 1.0, 1.0], [7.0, -8.0, 0.1]],
    dtype=np.float32,
)
VALID = POINTS[[0, 2, 4]]


def pad(buf: bytearray, size: int):
    """Pad buf with zeros to a multiple of size (offsets from the end of the 4-byte header)."""
    buf.extend(b'\x00' * (-(len(buf) - 4) % size))


def pack(points: np.ndarray, frame_id: str = 'livox_frame', sec: int = 12, nanosec: int = 34):
    """Serialize a CustomMsg by hand, the way rmw_fastrtps does (XCDR1, little endian)."""
    name = frame_id.encode() + b'\x00'
    buf = bytearray(b'\x00\x01\x00\x00')
    buf += struct.pack('<iII', sec, nanosec, len(name)) + name
    pad(buf, 8)
    buf += struct.pack('<Q', 42)
    buf += struct.pack('<IB3x', len(points), 1)
    buf += struct.pack('<I', len(points))
    for i, (x, y, z) in enumerate(points):
        pad(buf, 4)
        buf += struct.pack('<IfffBBB', i, x, y, z, 5, 0, 1)
    return bytes(buf)


def test_header_and_points():
    """Stamp, frame and points come back; points at the origin or with NaN are dropped."""
    sec, nanosec, frame_id, xyz = parse_custom_msg(pack(POINTS))
    assert (sec, nanosec, frame_id) == (12, 34, 'livox_frame')
    assert xyz.dtype == np.float32 and xyz.shape == (3, 3)
    np.testing.assert_array_equal(xyz, VALID)


@pytest.mark.parametrize('frame_id', ['a', 'abc', 'abcd', 'lidar_frame_x'])
def test_frame_name_length_moves_the_alignment(frame_id):
    """Every frame_id length lands the uint64 timebase and the points on their alignment."""
    _, _, parsed, xyz = parse_custom_msg(pack(POINTS, frame_id))
    assert parsed == frame_id
    np.testing.assert_array_equal(xyz, VALID)


def test_empty_cloud():
    """A scan without points parses to an empty Nx3 array."""
    _, _, _, xyz = parse_custom_msg(pack(POINTS[:0]))
    assert xyz.shape == (0, 3)


@pytest.mark.parametrize(
    'data',
    [
        b'',
        b'\x00\x01',
        b'\x01\x01\x00\x00' + pack(POINTS)[4:],
        pack(POINTS)[:-5],
        pack(POINTS)[:30],
    ],
)
def test_broken_buffers_are_refused(data):
    """Wrong encapsulation or a truncated buffer returns None instead of garbage."""
    assert parse_custom_msg(data) is None


def test_matches_rosbags_serializer():
    """Parity with an independent CDR serializer (skipped when rosbags is not installed)."""
    typesys = pytest.importorskip('rosbags.typesys')
    store = typesys.get_typestore(typesys.Stores.ROS2_HUMBLE)
    store.register(
        {
            **typesys.get_types_from_msg(
                'uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\n'
                'uint8 reflectivity\nuint8 tag\nuint8 line',
                'livox_ros_driver2/msg/CustomPoint',
            ),
            **typesys.get_types_from_msg(
                'std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\n'
                'uint8[3] rsvd\nlivox_ros_driver2/CustomPoint[] points',
                'livox_ros_driver2/msg/CustomMsg',
            ),
        }
    )
    t = store.types
    points = [
        t['livox_ros_driver2/msg/CustomPoint'](
            offset_time=i, x=float(x), y=float(y), z=float(z), reflectivity=5, tag=0, line=1
        )
        for i, (x, y, z) in enumerate(POINTS)
    ]
    header = t['std_msgs/msg/Header'](
        stamp=t['builtin_interfaces/msg/Time'](sec=1700000000, nanosec=999), frame_id='livox_frame'
    )
    msg = t['livox_ros_driver2/msg/CustomMsg'](
        header=header,
        timebase=42,
        point_num=len(points),
        lidar_id=1,
        rsvd=np.zeros(3, dtype=np.uint8),
        points=points,
    )
    data = bytes(store.serialize_cdr(msg, 'livox_ros_driver2/msg/CustomMsg'))
    assert data == pack(POINTS, sec=1700000000, nanosec=999)  # the hand packer above is right
    sec, nanosec, frame_id, xyz = parse_custom_msg(data)
    assert (sec, nanosec, frame_id) == (1700000000, 999, 'livox_frame')
    np.testing.assert_array_equal(xyz, VALID)
