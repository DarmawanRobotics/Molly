"""
Point clouds as numpy: PointCloud2 bytes to xyz, voxel downsampling and rigid transforms.

Works on the message's fields and raw data only (no per-point Python objects), so a 20k-point
scan costs a few milliseconds. No ROS imports: any object with the PointCloud2 attributes works.
"""

import math

import numpy as np

FLOAT32 = 7  # sensor_msgs/PointField.FLOAT32
FLOAT64 = 8


def cloud_xyz(msg) -> np.ndarray:
    """Return the finite x, y, z of a PointCloud2 as an (N, 3) float64 array."""
    fields = {f.name: f for f in msg.fields}
    if not all(name in fields for name in 'xyz'):
        return np.empty((0, 3))
    step = int(msg.point_step)
    count = int(msg.width) * int(msg.height)
    if step <= 0 or count <= 0:
        return np.empty((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    count = min(count, raw.size // step)
    rows = raw[:count * step].reshape(count, step)
    order = '>' if msg.is_bigendian else '<'
    columns = []
    for name in 'xyz':
        field = fields[name]
        if field.datatype == FLOAT32:
            dtype, size = np.dtype(f'{order}f4'), 4
        elif field.datatype == FLOAT64:
            dtype, size = np.dtype(f'{order}f8'), 8
        else:
            return np.empty((0, 3))
        offset = int(field.offset)
        columns.append(
            np.ascontiguousarray(rows[:, offset:offset + size]).view(dtype).reshape(-1)
        )
    xyz = np.column_stack(columns).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """Keep one point per voxel (the first one), or all points when voxel <= 0."""
    if voxel <= 0.0 or points.shape[0] < 2:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the 3x3 rotation of a quaternion (normalized here)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose_matrix(translation, rotation) -> np.ndarray:
    """Return a 4x4 matrix from (x, y, z) and a quaternion (x, y, z, w)."""
    m = np.eye(4)
    m[:3, :3] = quaternion_matrix(*rotation)
    m[:3, 3] = translation
    return m


def transform_matrix(transform) -> np.ndarray:
    """Return the 4x4 matrix of a geometry_msgs/Transform(Stamped)."""
    t = getattr(transform, 'transform', transform)
    return pose_matrix(
        (t.translation.x, t.translation.y, t.translation.z),
        (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
    )


def apply(m: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Transform (N, 3) points by a 4x4 matrix."""
    return points @ m[:3, :3].T + m[:3, 3]


def yaw_of(m: np.ndarray) -> float:
    """Return the yaw (rad) of a 4x4 or 3x3 rotation."""
    return math.atan2(m[1, 0], m[0, 0])
