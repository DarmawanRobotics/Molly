"""Pure helpers for the stillness gate and the odometry monitor (no ROS imports, unit-tested)."""

import math

import numpy as np


def stillness(gyro: np.ndarray, acc: np.ndarray) -> tuple[float, float, float]:
    """
    Return (max gyro norm, relative std of |acc|, tilt of mean acc from +z in degrees).

    gyro and acc are Nx3 arrays. The acceleration check is relative, so it works whether the IMU
    reports m/s^2 or g (the Livox Mid-360 reports g).
    """
    gyro_max = float(np.max(np.linalg.norm(gyro, axis=1)))
    acc_norm = np.linalg.norm(acc, axis=1)
    mean_norm = float(np.mean(acc_norm))
    if mean_norm <= 1e-6:
        return gyro_max, math.inf, 180.0
    acc_rel_std = float(np.std(acc_norm)) / mean_norm
    mean_acc = np.mean(acc, axis=0)
    cos_tilt = float(mean_acc[2]) / float(np.linalg.norm(mean_acc))
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))
    return gyro_max, acc_rel_std, tilt_deg


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the yaw (rad) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def looks_like_restart(prev_xy_dist: float, prev_yaw: float, xy_dist: float, yaw: float,
                       origin_radius: float, min_prev_offset: float) -> bool:
    """
    Return True when the pose snapped back to the origin, i.e. FAST-LIO restarted.

    A fresh FAST-LIO starts at identity; a robot that was away from the origin cannot get there in
    one frame.
    """
    at_origin = xy_dist < origin_radius and abs(yaw) < math.radians(1.0)
    was_away = prev_xy_dist > min_prev_offset or abs(prev_yaw) > math.radians(5.0)
    return at_origin and was_away
