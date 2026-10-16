import math

from molly_perception.sectors import bin_ranges, obstacle_points
import numpy as np

BOX = (-0.4, 0.4, -0.28, 0.28)


def test_obstacle_points_drop_floor_ceiling_and_body():
    """Only points between z_min and z_max and outside the robot box remain."""
    pts = np.array(
        [
            [2.0, 0.0, 0.0],   # obstacle
            [2.0, 0.0, -0.5],  # floor
            [2.0, 0.0, 2.0],   # ceiling
            [0.1, 0.1, 0.0],   # own leg
            [-1.0, 1.0, 0.5],  # obstacle behind-left
        ],
        dtype=np.float32,
    )
    xy = obstacle_points(pts, -0.25, 1.2, BOX)
    assert xy.tolist() == [[2.0, 0.0], [-1.0, 1.0]]
    assert obstacle_points(np.empty((0, 3)), -0.25, 1.2, BOX).shape == (0, 2)


def test_front_scan_keeps_nearest_per_sector():
    """Front fov: nearest range wins, points outside the fov are ignored."""
    xy = np.array([[2.0, 0.0], [1.5, 0.01], [-3.0, 0.0], [0.0, 2.0]], dtype=np.float32)
    ranges = bin_ranges(xy, -math.pi / 2, math.pi, 4, 0.1, 8.0)
    # sectors: [-90,-45) [-45,0) [0,45) [45,90); (0, 2) is exactly at +90 -> outside
    assert ranges[2] == np.float32(np.hypot(1.5, 0.01))
    assert ranges[0] == ranges[1] == ranges[3] == 8.0


def test_full_circle_wraps_behind():
    """The 360 deg scan sees obstacles behind the robot, at +-180 deg too."""
    xy = np.array([[-1.0, 0.001], [-1.2, -0.001], [0.0, -2.0]], dtype=np.float32)
    ranges = bin_ranges(xy, -math.pi, 2 * math.pi, 8, 0.1, 4.0)
    assert ranges[7] == np.float32(np.hypot(-1.0, 0.001))  # just below +180
    assert ranges[0] == np.float32(np.hypot(-1.2, 0.001))  # just above -180
    assert ranges[2] == 2.0  # -90 deg
    assert np.count_nonzero(ranges < 4.0) == 3


def test_ranges_outside_limits_are_ignored():
    """Points closer than range_min or beyond range_max do not count."""
    xy = np.array([[0.05, 0.0], [9.0, 0.0]], dtype=np.float32)
    assert (bin_ranges(xy, -math.pi, 2 * math.pi, 8, 0.1, 4.0) == 4.0).all()
