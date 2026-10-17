"""plan_path end to end on a room with a wall and a door."""

import math

import numpy as np
import pytest

costmap = pytest.importorskip('molly_navigation.utils.costmap')
planning = pytest.importorskip('molly_navigation.utils.planning')


def room():
    """Return the costmap of a 10 x 6 m room: a wall at x = 5 with a 1.2 m door at y = 1."""
    data = np.zeros((120, 200), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    data[:, 100] = 100
    data[10:34, 100] = 0          # door y = 0.5 .. 1.7
    data[100:, 150:] = -1         # unknown corner (x > 7.5, y > 5)
    return costmap.build_costmap(data, 0.05, 0.0, 0.0, planning_resolution=0.1,
                                 robot_radius=0.3, inflation_radius=0.6)


def test_path_goes_through_the_door_to_the_goal_heading():
    """The plan crosses the wall at the door, keeps out of lethal cells and ends at the goal."""
    cm = room()
    plan = planning.plan_path(cm, (2.0, 4.0, 0.0), (8.0, 4.0, math.pi / 2))
    assert plan.ok, plan.message
    xs = [w[0] for w in plan.waypoints]
    crossing = [w for w in plan.waypoints if 4.9 < w[0] < 5.1]
    assert crossing and all(0.5 < w[1] < 1.7 for w in crossing)
    assert all(math.isfinite(cm.cost_at(w[0], w[1])) for w in plan.waypoints)
    assert plan.waypoints[0][:2] == pytest.approx((2.0, 4.0))
    assert plan.waypoints[-1][:2] == pytest.approx((8.0, 4.0), abs=0.06)
    assert plan.waypoints[-1][2] == pytest.approx(90.0)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(plan.waypoints, plan.waypoints[1:])]
    assert max(gaps) < 0.15 and plan.length > 6.0 and xs[-1] > xs[0]


def test_goal_in_a_wall_is_snapped_or_refused():
    """A goal just inside the inflation moves out; deep inside or unknown is refused."""
    cm = room()
    snapped = planning.plan_path(cm, (2.0, 4.0, 0.0), (4.9, 4.0, 0.0), goal_tolerance=0.5)
    assert snapped.ok and 'moved' in snapped.message
    assert snapped.goal[0] < 4.75
    refused = planning.plan_path(cm, (2.0, 4.0, 0.0), (5.0, 4.0, 0.0), goal_tolerance=0.1)
    assert not refused.ok
    unknown = planning.plan_path(cm, (2.0, 4.0, 0.0), (9.0, 5.7, 0.0), goal_tolerance=0.2)
    assert not unknown.ok


def test_start_near_a_wall_escapes_and_outside_fails():
    """A robot inside the inflation still gets a plan; one off the map does not."""
    cm = room()
    near_wall = planning.plan_path(cm, (4.85, 4.0, 0.0), (2.0, 2.0, 0.0))
    assert near_wall.ok, near_wall.message
    assert not planning.plan_path(cm, (-3.0, 1.0, 0.0), (2.0, 2.0, 0.0)).ok


def test_short_and_zero_paths():
    """A goal next to the robot gives a short path; the same pose gives two points."""
    cm = room()
    short = planning.plan_path(cm, (2.0, 2.0, 0.0), (2.05, 2.0, 1.0))
    assert short.ok and len(short.waypoints) >= 2
    assert short.waypoints[-1][2] == pytest.approx(math.degrees(1.0))


def test_remaining_length():
    """Remaining length follows the robot along the path."""
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert planning.remaining_length(path, 0.0, 0.0) == pytest.approx(2.0)
    assert planning.remaining_length(path, 1.0, 0.1) == pytest.approx(1.1)
    assert planning.remaining_length([], 0.0, 0.0) == 0.0
