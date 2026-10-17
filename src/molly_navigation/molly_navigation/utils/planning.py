"""
Plan a walkable path on a costmap: from the robot's pose to a goal pose (pure, unit-tested).

A* on the inflated costmap, a start inside an inflated obstacle escapes to the nearest free cell,
a goal inside one snaps to the nearest free cell within the tolerance, line-of-sight shortcuts,
then waypoints every spacing metres, smoothed when that stays clear of obstacles. Waypoint yaw is
the direction of travel (degrees, as in route CSVs) and the goal heading on the last one.
"""

from dataclasses import dataclass, field
import math

from molly_navigation.utils.astar import astar, nearest_free, SearchTimeout, shortcut
from molly_navigation.utils.costmap import Costmap
from molly_navigation.utils.path_processing import headings, resample, smooth


@dataclass
class Plan:
    """Outcome of planning: waypoints (x, y, yaw_deg, dwell_sec) in the map frame."""

    ok: bool
    message: str
    waypoints: list = field(default_factory=list)
    length: float = 0.0
    goal: tuple = None          # (x, y, yaw) the path ends at (snapped when needed)


def polyline_length(points) -> float:
    """Return the length of a polyline of (x, y) points."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def collides(costmap: Costmap, points, skip: int = 0) -> bool:
    """Return True when a point (after the first skip) lies in a lethal cell."""
    return any(not math.isfinite(costmap.cost_at(x, y)) for x, y in points[skip:])


def plan_path(
    costmap: Costmap,
    start,
    goal,
    goal_tolerance: float = 0.5,
    start_escape: float = 1.0,
    spacing: float = 0.1,
    smooth_weights=(0.1, 0.3),
    heuristic_weight: float = 1.0,
    timeout_sec: float = 5.0,
) -> Plan:
    """Plan from start (x, y, yaw) to goal (x, y, yaw); yaws in radians."""
    sx, sy, _ = start
    gx, gy, gyaw = goal
    res = costmap.resolution
    start_cell = costmap.cell(sx, sy)
    if not costmap.inside(*start_cell):
        return Plan(False, f'the robot ({sx:.2f}, {sy:.2f}) is outside the map')
    goal_cell = costmap.cell(gx, gy)
    if not costmap.inside(*goal_cell):
        return Plan(False, f'the goal ({gx:.2f}, {gy:.2f}) is outside the map')
    free_start = nearest_free(costmap.cost, start_cell, int(math.ceil(start_escape / res)))
    if free_start is None:
        return Plan(False, 'the robot is boxed in: no free cell within '
                           f'{start_escape:.1f} m (is it localized on this map?)')
    snapped = False
    free_goal = goal_cell
    if costmap.lethal(*goal_cell):
        free_goal = nearest_free(costmap.cost, goal_cell, int(math.ceil(goal_tolerance / res)))
        if free_goal is None:
            return Plan(False, f'the goal ({gx:.2f}, {gy:.2f}) is inside an obstacle')
        snapped = True
        gx, gy = costmap.center(*free_goal)
    try:
        cells = astar(costmap.cost, free_start, free_goal, heuristic_weight,
                      timeout_sec=timeout_sec)
    except SearchTimeout as error:
        return Plan(False, f'no path found in time ({error})')
    if cells is None:
        return Plan(False, 'no path: the goal cannot be reached on this map')
    cells = shortcut(costmap.cost, cells)
    points = [(sx, sy)]
    if free_start != start_cell:
        points.append(costmap.center(*free_start))
    points += [costmap.center(*c) for c in cells[1:-1]]
    points.append((gx, gy))
    if len(points) == 2 and math.hypot(points[1][0] - sx, points[1][1] - sy) < 1e-6:
        points = [(sx, sy), (gx, gy)]
    dense = resample(points, spacing) if polyline_length(points) > spacing else points
    route = dense
    if len(dense) > 2:
        smoothed = smooth(dense, *smooth_weights)
        # the first points may sit in the inflation the robot is escaping from
        skip = 1 if free_start == start_cell else max(2, int(start_escape / spacing) + 1)
        if not collides(costmap, smoothed, skip):
            route = smoothed
    yaws = headings(route) if len(route) > 1 else [gyaw]
    waypoints = [(x, y, math.degrees(yaw), 0.0) for (x, y), yaw in zip(route, yaws)]
    x, y, _, _ = waypoints[-1]
    waypoints[-1] = (x, y, math.degrees(gyaw), 0.0)
    note = ' (goal moved out of an obstacle)' if snapped else ''
    return Plan(True, f'{polyline_length(route):.1f} m{note}', waypoints,
                polyline_length(route), (gx, gy, gyaw))


def remaining_length(path_xy, x: float, y: float) -> float:
    """Return the length still to walk from the path point nearest to (x, y)."""
    if not path_xy:
        return 0.0
    nearest = min(
        range(len(path_xy)), key=lambda i: math.hypot(path_xy[i][0] - x, path_xy[i][1] - y)
    )
    rest = path_xy[nearest:]
    return math.hypot(rest[0][0] - x, rest[0][1] - y) + polyline_length(rest)
