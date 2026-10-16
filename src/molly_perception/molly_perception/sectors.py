"""Pure helpers that turn LiDAR points into nearest-range sectors (unit-tested without ROS)."""

import math

import numpy as np


def obstacle_points(points: np.ndarray, z_min: float, z_max: float, self_box) -> np.ndarray:
    """
    Return the (x, y) of points that can block the robot, all in base_link.

    Keeps the points between z_min and z_max (obstacles, not floor or ceiling) and drops the
    points inside self_box = (x_min, x_max, y_min, y_max), the robot's own body and legs.
    """
    if points.shape[0] == 0:
        return np.empty((0, 2), dtype=np.float32)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    sx0, sx1, sy0, sy1 = self_box
    on_body = (x > sx0) & (x < sx1) & (y > sy0) & (y < sy1)
    keep = (z >= z_min) & (z <= z_max) & ~on_body
    return np.stack([x[keep], y[keep]], axis=1)


def bin_ranges(
    xy: np.ndarray,
    angle_min: float,
    fov: float,
    num_sectors: int,
    range_min: float,
    range_max: float,
) -> np.ndarray:
    """
    Return the nearest range per angular sector; range_max where a sector saw nothing.

    Sector i spans [angle_min + i * fov / n, angle_min + (i + 1) * fov / n) (REP-103, +left).
    """
    ranges = np.full(num_sectors, range_max, dtype=np.float32)
    if xy.shape[0] == 0:
        return ranges
    r = np.hypot(xy[:, 0], xy[:, 1])
    theta = np.arctan2(xy[:, 1], xy[:, 0])
    if fov < 2.0 * math.pi - 1e-6:
        inside = (theta >= angle_min) & (theta < angle_min + fov)
    else:  # full circle: wrap every angle into [angle_min, angle_min + 2 pi)
        theta = np.mod(theta - angle_min, 2.0 * math.pi) + angle_min
        inside = np.ones(theta.shape, dtype=bool)
    valid = inside & (r >= range_min) & (r <= range_max)
    r, theta = r[valid], theta[valid]
    if r.size:
        idx = np.clip(((theta - angle_min) / (fov / num_sectors)).astype(int), 0, num_sectors - 1)
        np.minimum.at(ranges, idx, r.astype(np.float32))
    return ranges
