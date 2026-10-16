import numpy as np


def pixel_ray(u: float, v: float, k: np.ndarray) -> np.ndarray:
    """Return the direction of a pixel in the optical frame (x right, y down, z forward)."""
    return np.array([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], 1.0])


def ground_point(ray: np.ndarray, t_base_cam: np.ndarray, floor_z: float, max_range: float):
    """Intersect a camera ray with the floor plane z = floor_z in the base frame, or None."""
    origin = t_base_cam[:3, 3]
    direction = t_base_cam[:3, :3] @ ray
    if direction[2] >= -1e-6:
        return None
    t = (floor_z - origin[2]) / direction[2]
    if t <= 0.0:
        return None
    point = origin + t * direction
    if np.hypot(point[0], point[1]) > max_range:
        return None
    return point


def feet_pixel(box) -> tuple:
    """Return the bottom-centre pixel of a box, where a standing person meets the floor."""
    x1, _, x2, y2 = box[:4]
    return (x1 + x2) / 2.0, y2
