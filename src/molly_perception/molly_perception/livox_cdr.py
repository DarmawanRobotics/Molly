"""
Read a serialized Livox CustomMsg straight from its CDR bytes (no ROS needed, unit-tested).

rclpy turns every CustomPoint of a deserialized CustomMsg into a Python object: ~20k objects per
Mid-360 scan, which held lidar_sector_node to ~6 Hz on a full Jetson core. The node subscribes with
raw=True instead and reads the points here with numpy, as a strided view of the buffer.

Layout (XCDR1 little endian, offsets aligned from the end of the 4-byte encapsulation header):
header (int32 sec, uint32 nanosec, string frame_id), uint64 timebase, uint32 point_num,
uint8 lidar_id, uint8[3] rsvd, sequence<CustomPoint> points; CustomPoint is uint32 offset_time,
float32 x y z, uint8 reflectivity tag line, padded to 20 bytes between elements.
"""

import struct

import numpy as np

CDR_LE = b'\x00\x01'
POINT_STRIDE = 20
POINT_SIZE = 19


def _align(offset: int, size: int) -> int:
    """Round offset up to a multiple of size."""
    return (offset + size - 1) // size * size


def parse_custom_msg(data: bytes):
    """
    Return (sec, nanosec, frame_id, xyz) from a serialized CustomMsg, or None if it does not fit.

    xyz is an Nx3 float32 array; points at the sensor origin (no return) or not finite are dropped.
    """
    buf = memoryview(data)
    if len(buf) < 4 or bytes(buf[:2]) != CDR_LE:
        return None
    body = buf[4:]
    try:
        sec, nanosec, name_len = struct.unpack_from('<iII', body, 0)
        offset = 12 + name_len
        frame_id = bytes(body[12:offset]).rstrip(b'\x00').decode()
        offset = _align(offset, 8) + 8  # timebase
        offset = _align(offset + 4 + 1 + 3, 4)  # point_num, lidar_id, rsvd
        (count,) = struct.unpack_from('<I', body, offset)
    except (struct.error, UnicodeDecodeError):
        return None
    offset += 4
    if name_len == 0 or (count and offset + (count - 1) * POINT_STRIDE + POINT_SIZE > len(body)):
        return None
    if count == 0:
        return sec, nanosec, frame_id, np.empty((0, 3), dtype=np.float32)
    xyz = np.ndarray(
        (count, 3), dtype='<f4', buffer=body, offset=offset + 4, strides=(POINT_STRIDE, 4)
    )
    keep = np.isfinite(xyz).all(axis=1) & xyz.any(axis=1)
    return sec, nanosec, frame_id, xyz[keep].astype(np.float32, copy=False)
