"""
Path-shift avoidance for holonomic robots: slide sideways around obstacles, keep the heading.

Pure numpy, unit-tested without ROS. Obstacle points (map frame) are expressed in path
coordinates, arc length along the route and signed distance to its left. A lateral offset `o`
is clear when no point lies in the robot's corridor (|d - o| < half_width + clearance) from its
rear edge to `look_ahead` beyond its front edge. Points beside the body block an offset outright
(the robot would slide into them), points ahead limit how far it can drive.

The offset nearest the route that is clear wins, with a cost on changing it, so the robot does not
flip sides and only returns to the route once the obstacle is beside or behind it and the route
corridor is free along its whole body. It never leaves the route by more than `max_offset`: with
no clear offset it slows down and stops short of the obstacle instead of wandering off.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class ShiftResult:
    """Chosen offset and the free distance ahead of the robot's front edge on it."""

    offset: float          # m, +left of the route
    free_distance: float   # m from the front edge to the first obstacle on that offset (inf)
    clear: bool            # False: no offset within max_offset is free, the robot must slow


@dataclass
class PathShiftConfig:
    """Footprint, search range and stopping model, in metres, seconds and m/s^2."""

    max_offset: float = 0.8       # furthest the robot leaves the route
    step: float = 0.1             # candidate spacing
    look_ahead: float = 2.0       # route ahead of the front edge that must be free
    front: float = 0.40           # base_link to the front edge
    rear: float = 0.40            # base_link to the rear edge
    half_width: float = 0.28      # base_link to the side edges
    clearance: float = 0.10       # gap kept to obstacles beside the corridor
    switch_cost: float = 0.5      # cost per metre of changing the offset (hysteresis)
    switch_margin: float = 0.05   # extra clearance a new offset needs, so it does not flicker
    decel: float = 0.5            # braking used to limit speed toward an obstacle
    stop_margin: float = 0.40     # stop this far before an obstacle (front edge)


class PathShift:
    """Choose the lateral offset from the route that passes obstacles (see module docstring)."""

    def __init__(self, cfg: PathShiftConfig = PathShiftConfig()):
        self.cfg = cfg
        n = int(round(cfg.max_offset / cfg.step))
        self.candidates = np.array([i * cfg.step for i in range(-n, n + 1)])

    def path_coordinates(self, path_xy, path_s, s_robot: float, points: np.ndarray):
        """
        Return (s, d) of every point that is near the route around the robot.

        path_xy / path_s: route points and their cumulative arc length. Only the route from the
        rear edge to look_ahead beyond the front edge is used.
        """
        cfg = self.cfg
        if points.size == 0 or len(path_xy) < 2:
            return np.empty(0), np.empty(0)
        s_lo = s_robot - cfg.rear - 0.5
        s_hi = s_robot + cfg.front + cfg.look_ahead + 0.5
        s = np.asarray(path_s)
        i0 = max(int(np.searchsorted(s, s_lo)) - 1, 0)
        i1 = min(int(np.searchsorted(s, s_hi)) + 1, len(s) - 1)
        if i1 <= i0:
            return np.empty(0), np.empty(0)
        xy = np.asarray(path_xy[i0:i1 + 1], dtype=float)
        a, b = xy[:-1], xy[1:]
        seg = b - a
        length = np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-9)
        rel = points[:, None, :] - a[None, :, :]                     # n x k x 2
        t_raw = (rel * seg[None]).sum(axis=2) / length**2
        t = np.clip(t_raw, 0.0, 1.0)
        closest = a[None] + t[..., None] * seg[None]
        dist = np.hypot(points[:, None, 0] - closest[..., 0], points[:, None, 1] - closest[..., 1])
        k = dist.argmin(axis=1)
        rows = np.arange(points.shape[0])
        cross = seg[k, 0] * rel[rows, k, 1] - seg[k, 1] * rel[rows, k, 0]
        d = cross / length[k]
        # before the window's first or after its last segment, measure along the extension, so
        # a wall behind the start of the route is behind the robot, not beside it
        along = t[rows, k]
        first, last = k == 0, k == seg.shape[0] - 1
        along = np.where(first & (t_raw[rows, k] < 0.0), t_raw[rows, k], along)
        along = np.where(last & (t_raw[rows, k] > 1.0), t_raw[rows, k], along)
        s_pts = s[i0:i1][k] + along * length[k]
        reach = cfg.max_offset + cfg.half_width + cfg.clearance + 0.2
        near = np.abs(d) < reach
        if i1 == len(s) - 1:
            # past the end of the route (the goal, or a loop that restarts): not in the way
            near &= ~(last & (t_raw[rows, k] > 1.0))
        return s_pts[near], d[near]

    def free_distances(self, s_pts, d_pts, s_robot: float, offsets, extra=0.0) -> np.ndarray:
        """Free distance ahead of the front edge for each offset; 0 if something is beside."""
        cfg = self.cfg
        offsets = np.atleast_1d(np.asarray(offsets, dtype=float))
        free = np.full(offsets.shape, math.inf)
        if s_pts.size == 0:
            return free
        width = cfg.half_width + cfg.clearance + np.broadcast_to(extra, offsets.shape)
        band = np.abs(d_pts[None, :] - offsets[:, None]) < width[:, None]
        ahead = s_pts - (s_robot + cfg.front)                         # from the front edge
        beside = (s_pts >= s_robot - cfg.rear) & (ahead <= 0.0)
        in_range = (ahead > 0.0) & (ahead <= cfg.look_ahead)
        blocked_beside = (band & beside[None, :]).any(axis=1)
        for i in range(offsets.size):
            hits = ahead[band[i] & in_range]
            if hits.size:
                free[i] = hits.min()
        free[blocked_beside] = 0.0
        return free

    def choose(self, s_pts, d_pts, s_robot: float, current: float) -> ShiftResult:
        """Pick the offset for this tick from obstacle path coordinates (path_coordinates())."""
        # the current offset keeps its plain clearance, any other needs switch_margin more
        extra = np.where(
            np.abs(self.candidates - current) < 1e-6, 0.0, self.cfg.switch_margin
        )
        free = self.free_distances(s_pts, d_pts, s_robot, self.candidates, extra)
        clear = np.isinf(free)
        if clear.any():
            cost = np.abs(self.candidates) + self.cfg.switch_cost * np.abs(
                self.candidates - current
            )
            cost[~clear] = math.inf
            i = int(cost.argmin())
            return ShiftResult(float(self.candidates[i]), math.inf, True)
        # nothing is clear: take the offset that gets furthest, preferring the current one
        best = free.max()
        ties = np.flatnonzero(free >= best - 1e-6)
        i = int(ties[np.abs(self.candidates[ties] - current).argmin()])
        return ShiftResult(float(self.candidates[i]), float(free[i]), False)

    def free_at(self, s_pts, d_pts, s_robot: float, offset: float) -> float:
        """Free distance ahead on one offset, e.g. where the robot is now."""
        return float(self.free_distances(s_pts, d_pts, s_robot, [offset])[0])

    def speed_limit(self, free_distance: float) -> float:
        """Highest speed that still stops stop_margin before an obstacle free_distance ahead."""
        if math.isinf(free_distance):
            return math.inf
        room = free_distance - self.cfg.stop_margin
        return math.sqrt(2.0 * self.cfg.decel * room) if room > 0.0 else 0.0
