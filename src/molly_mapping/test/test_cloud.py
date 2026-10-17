"""PointCloud2 bytes to numpy, voxel downsampling and transforms."""

from types import SimpleNamespace

import numpy as np
import pytest

cloud = pytest.importorskip('molly_mapping.cloud')


def fake_cloud(points, extra=4, bigendian=False):
    """Build a PointCloud2-like object with x, y, z float32 and padding."""
    fields = [
        SimpleNamespace(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate('xyz')
    ]
    step = 12 + extra
    order = '>' if bigendian else '<'
    rows = np.zeros((len(points), step), dtype=np.uint8)
    rows[:, :12] = np.asarray(points, dtype=f'{order}f4').view(np.uint8).reshape(-1, 12)
    return SimpleNamespace(fields=fields, point_step=step, width=len(points), height=1,
                           data=rows.tobytes(), is_bigendian=bigendian)


def test_cloud_xyz_reads_points_and_drops_nan():
    """x, y, z come back in order; NaN points are dropped."""
    msg = fake_cloud([[1, 2, 3], [np.nan, 0, 0], [4, 5, 6]])
    assert cloud.cloud_xyz(msg).tolist() == [[1, 2, 3], [4, 5, 6]]
    assert cloud.cloud_xyz(fake_cloud([[1, 2, 3]], bigendian=True)).tolist() == [[1, 2, 3]]


def test_cloud_without_xyz_is_empty():
    """A cloud without x, y, z fields gives no points."""
    msg = fake_cloud([[1, 2, 3]])
    msg.fields = msg.fields[:2]
    assert cloud.cloud_xyz(msg).shape == (0, 3)


def test_voxel_downsample_keeps_one_point_per_voxel():
    """Points in the same voxel collapse to the first one."""
    points = np.array([[0.01, 0.01, 0.0], [0.02, 0.03, 0.0], [0.5, 0.0, 0.0]])
    assert cloud.voxel_downsample(points, 0.1).tolist() == [[0.01, 0.01, 0.0], [0.5, 0.0, 0.0]]
    assert cloud.voxel_downsample(points, 0.0).shape == (3, 3)


def test_pose_matrix_and_apply():
    """A 90 degree yaw plus a translation maps (1, 0, 0) to (tx, ty + 1, tz)."""
    q = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    m = cloud.pose_matrix((1.0, 2.0, 3.0), q)
    assert cloud.apply(m, np.array([[1.0, 0.0, 0.0]]))[0] == pytest.approx([1.0, 3.0, 3.0])
    assert cloud.yaw_of(m) == pytest.approx(np.pi / 2)
