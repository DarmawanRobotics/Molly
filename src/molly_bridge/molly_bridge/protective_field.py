"""
Speed and separation monitoring: limit a velocity command to what the robot can stop within.

Pure math, no ROS. The robot is a rectangle around base_link; for the direction it is about to
move in, the free distance from its edge to the nearest obstacle in its swept path (from a 360 deg
sector scan) bounds the speed so that, after a reaction time and braking at max_decel, it still
stops `margin` short of the obstacle:

    v * t_react + v^2 / (2 a) <= free - margin
"""

from dataclasses import dataclass
import math


@dataclass
class FieldConfig:
    """Robot footprint and stopping model, all in metres, seconds and m/s^2."""

    front: float = 0.40          # base_link to the front edge
    rear: float = 0.40           # base_link to the rear edge
    half_width: float = 0.28     # base_link to the side edges
    max_decel: float = 0.6       # braking the robot reliably achieves
    reaction_time: float = 0.25  # sensing + command latency
    margin: float = 0.15         # gap left between the robot and the obstacle
    lateral_margin: float = 0.10  # extra width of the swept path
    min_speed: float = 0.02      # m/s, slower commands are not limited (creeping, turning)


@dataclass
class FieldResult:
    """Outcome of one check."""

    scale: float      # 0..1 applied to the linear velocity
    clearance: float  # m from the robot edge to the nearest obstacle in the path, inf if none
    state: str        # clear | limited | stop


def footprint_extent(cfg: FieldConfig, ux: float, uy: float) -> tuple[float, float]:
    """Return (distance from base_link to the edge along u, half width across u) of the box."""
    length = cfg.front if ux >= 0.0 else cfg.rear
    along = abs(ux) * length + abs(uy) * cfg.half_width
    across = abs(uy) * length + abs(ux) * cfg.half_width
    return along, across


def path_clearance(cfg: FieldConfig, ux: float, uy: float, points) -> float:
    """
    Free distance from the robot edge to the nearest point in the swept path along (ux, uy).

    Ahead of the leading edge the path is the robot width plus lateral_margin on each side.
    Beside the robot only points inside the footprint itself count (negative clearance): the
    robot slides past anything next to it.
    """
    along_edge, half = footprint_extent(cfg, ux, uy)
    clearance = math.inf
    for x, y in points:
        along = x * ux + y * uy
        across = abs(-x * uy + y * ux)
        if along <= 0.0:
            continue
        if along >= along_edge:
            if across <= half + cfg.lateral_margin:
                clearance = min(clearance, along - along_edge)
        elif across <= half:
            clearance = min(clearance, along - along_edge)
    return clearance


def max_speed(cfg: FieldConfig, clearance: float) -> float:
    """Highest speed that still stops `margin` before an obstacle `clearance` away."""
    room = clearance - cfg.margin
    if room <= 0.0:
        return 0.0
    if math.isinf(room):
        return math.inf
    a, t = cfg.max_decel, cfg.reaction_time
    return a * (-t + math.sqrt(t * t + 2.0 * room / a))


def scan_points(angle_min: float, angle_increment: float, ranges, range_max: float):
    """Return the (x, y) sector centres with an obstacle (range below range_max)."""
    out = []
    for i, r in enumerate(ranges):
        if r < range_max - 1e-3:
            theta = angle_min + (i + 0.5) * angle_increment
            out.append((r * math.cos(theta), r * math.sin(theta)))
    return out


def limit(cfg: FieldConfig, vx: float, vy: float, points) -> FieldResult:
    """Return the scale for a planar velocity (vx, vy) given obstacle points in base_link."""
    speed = math.hypot(vx, vy)
    if speed < cfg.min_speed:
        return FieldResult(1.0, math.inf, 'clear')
    clearance = path_clearance(cfg, vx / speed, vy / speed, points)
    allowed = max_speed(cfg, clearance)
    if allowed >= speed:
        return FieldResult(1.0, clearance, 'clear')
    if allowed <= cfg.min_speed:
        return FieldResult(0.0, clearance, 'stop')
    return FieldResult(allowed / speed, clearance, 'limited')
