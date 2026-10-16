import math

from molly_perception.person_projection import ground_point, pixel_ray
import numpy as np

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def camera(pitch_deg: float) -> np.ndarray:
    """Camera 0.25 m ahead and 0.1 m above base_link, pitched down."""
    a = math.radians(pitch_deg)
    rot_y = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    t = np.eye(4)
    t[:3, :3] = rot_y @ OPTICAL_TO_BODY
    t[:3, 3] = [0.25, 0.0, 0.1]
    return t


def test_feet_pixel_projects_back_to_the_floor_point():
    """Projecting a floor point to a pixel and back recovers it."""
    t = camera(10.0)
    floor = np.array([4.0, 0.5, -0.35])
    pc = np.linalg.inv(t) @ np.append(floor, 1.0)
    u, v = K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2]
    assert np.allclose(ground_point(pixel_ray(u, v, K), t, -0.35, 12.0), floor, atol=1e-6)


def test_pixels_above_the_horizon_do_not_hit_the_floor():
    """Rays pointing up return None."""
    assert ground_point(pixel_ray(320.0, 50.0, K), camera(0.0), -0.35, 12.0) is None


def test_far_points_are_rejected():
    """Points beyond max_range are dropped."""
    assert ground_point(pixel_ray(320.0, 245.0, K), camera(0.0), -0.35, 3.0) is None
