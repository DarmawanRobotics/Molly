import math
import random

from molly_navigation.utils.pose2d import Pose2D


def noisy_route(corners, spacing=0.2, noise=0.03, seed=0):
    """Recorded-route-like waypoints along the given corner polyline."""
    rng = random.Random(seed)
    route = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:]):
        n = int(math.hypot(bx - ax, by - ay) / spacing)
        for i in range(n):
            t = i / n
            route.append(
                (
                    ax + t * (bx - ax) + rng.gauss(0, noise),
                    ay + t * (by - ay) + rng.gauss(0, noise),
                    0.0,
                    0.0,
                )
            )
    route.append((corners[-1][0], corners[-1][1], 0.0, 0.0))
    return route


LOOP = [(0, 0), (10, 0), (10, 4), (0, 4), (0, 0)]
OUT_AND_BACK = [(0, 0), (10, 0), (10, 0.8), (0, 0.8)]


def step(pose, out, dt, max_yaw_rate=1.0):
    """Move a pose by a body-frame command (forward, sideways, yaw rate) for dt."""
    w = max(-max_yaw_rate, min(max_yaw_rate, out.angular))
    pose.yaw += w * dt
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    pose.x += (out.linear * c - out.lateral * s) * dt
    pose.y += (out.linear * s + out.lateral * c) * dt


def straight(x0, x1, y=0.0, spacing=0.1):
    """Points of a straight path along x."""
    n = int(abs(x1 - x0) / spacing)
    return [(x0 + (x1 - x0) * i / n, y) for i in range(n + 1)]


def drive(controller, path_xy, speed=0.4, dt=0.05, max_time=200.0, start=Pose2D(0.0, 0.1, 0.0)):
    """Kinematic simulation (unicycle unless the controller sets lateral); (finished, errors)."""
    pose = Pose2D(start.x, start.y, start.yaw)
    controller.set_path(path_xy)
    errors, t = [], 0.0
    while not controller.is_finished(pose) and t < max_time:
        out = controller.update(pose, dt, speed)
        step(pose, out, dt)
        i = controller.progress_index()
        near = path_xy[max(0, i - 30):i + 30]
        errors.append(min(math.hypot(px - pose.x, py - pose.y) for px, py in near))
        t += dt
    return controller.is_finished(pose), errors
