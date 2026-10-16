"""
2D world, robot kinematics and LiDAR ray casting for the simulator (pure numpy, unit-tested).

The world lives in the map frame: walls are line segments, pillars and people are circles.
Everything is infinitely tall for the LiDAR, which is enough for the obstacle sectors.
"""

from dataclasses import dataclass, field
import math

import numpy as np
import yaml


@dataclass
class World:
    """Static walls and circular obstacles in the map frame."""

    segments: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))  # x0, y0, x1, y1
    circles: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))   # x, y, radius
    start: tuple = (0.0, 0.0, 0.0)                                          # x, y, yaw (rad)

    @classmethod
    def from_yaml(cls, path: str) -> 'World':
        """
        Load a world file.

        walls: list of polylines [[x, y], ...] (closed: repeat the first point)
        pillars: list of [x, y, radius]
        start: [x, y, yaw_deg]
        """
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        segments = []
        for line in data.get('walls', []):
            for (x0, y0), (x1, y1) in zip(line, line[1:]):
                segments.append((x0, y0, x1, y1))
        circles = [tuple(c) for c in data.get('pillars', [])]
        x, y, yaw_deg = data.get('start', [0.0, 0.0, 0.0])
        return cls(
            np.array(segments, dtype=float).reshape(-1, 4),
            np.array(circles, dtype=float).reshape(-1, 3),
            (float(x), float(y), math.radians(float(yaw_deg))),
        )


def cast_rays(world: World, ox: float, oy: float, angles: np.ndarray, max_range: float,
              extra_circles: np.ndarray = None) -> np.ndarray:
    """Return the distance along each ray from (ox, oy) at the given map-frame angles."""
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    dist = np.full(angles.shape[0], max_range)
    if world.segments.shape[0]:
        x0, y0, x1, y1 = (world.segments[:, i][None, :] for i in range(4))
        ex, ey = x1 - x0, y1 - y0
        denom = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((x0 - ox) * ey - (y0 - oy) * ex) / denom   # along the ray
            u = ((x0 - ox) * dy - (y0 - oy) * dx) / denom   # along the segment
        hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        dist = np.minimum(dist, np.where(hit, t, np.inf).min(axis=1))
    circles = world.circles
    if extra_circles is not None and len(extra_circles):
        circles = np.vstack([circles, extra_circles])
    if circles.shape[0]:
        cx, cy, r = (circles[:, i][None, :] for i in range(3))
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        with np.errstate(invalid='ignore'):
            t = -b - np.sqrt(disc)
        hit = (disc >= 0.0) & (t > 0.0)
        dist = np.minimum(dist, np.where(hit, t, np.inf).min(axis=1))
    return dist


@dataclass
class Body:
    """Planar holonomic robot that follows velocity commands with acceleration limits."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    accel: float = 1.0       # m/s^2
    yaw_accel: float = 2.0   # rad/s^2

    def step(self, cmd: tuple, dt: float):
        """Move toward the body-frame command (vx, vy, wz) and integrate the pose."""
        def toward(value, target, rate):
            return value + max(-rate * dt, min(rate * dt, target - value))

        self.vx = toward(self.vx, cmd[0], self.accel)
        self.vy = toward(self.vy, cmd[1], self.accel)
        self.wz = toward(self.wz, cmd[2], self.yaw_accel)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * self.vx - s * self.vy) * dt
        self.y += (s * self.vx + c * self.vy) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * dt), math.cos(self.yaw + self.wz * dt))


def scan_points(world: World, body: Body, lidar_xy: tuple, rays: int, max_range: float,
                heights: list, noise: float, rng, extra_circles=None) -> np.ndarray:
    """Return LiDAR points (N x 3, base_link frame) at the given heights for the current pose."""
    lx, ly = lidar_xy
    c, s = math.cos(body.yaw), math.sin(body.yaw)
    ox, oy = body.x + c * lx - s * ly, body.y + s * lx + c * ly
    rel = np.linspace(-math.pi, math.pi, rays, endpoint=False)
    dist = cast_rays(world, ox, oy, rel + body.yaw, max_range, extra_circles)
    keep = dist < max_range
    rel, dist = rel[keep], dist[keep] + rng.normal(0.0, noise, keep.sum())
    px, py = lx + dist * np.cos(rel), ly + dist * np.sin(rel)
    return np.vstack(
        [np.stack([px, py, np.full_like(px, h)], axis=1) for h in heights]
    ).astype(np.float32)


def lio_pose(start: tuple, pose: tuple, imu_xy: tuple) -> tuple:
    """
    Return FAST-LIO's camera_init->body (x, y, yaw) for a base pose.

    camera_init is the IMU frame at the start pose and body is the IMU frame now; the IMU sits at
    imu_xy in base_link without rotation (as in the robot URDF).
    """
    (sx, sy, syaw), (x, y, yaw), (ix, iy) = start, pose, imu_xy

    def imu_in_map(px, py, pyaw):
        c, s = math.cos(pyaw), math.sin(pyaw)
        return px + c * ix - s * iy, py + s * ix + c * iy

    x0, y0 = imu_in_map(sx, sy, syaw)
    x1, y1 = imu_in_map(x, y, yaw)
    c, s = math.cos(syaw), math.sin(syaw)
    dx, dy = x1 - x0, y1 - y0
    dyaw = math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw))
    return c * dx + s * dy, -s * dx + c * dy, dyaw
