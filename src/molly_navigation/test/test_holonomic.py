"""Holonomic path following and path-shift avoidance, on kinematic simulations."""

import math

from molly_navigation.utils.controllers.holonomic_controller import HolonomicController
from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.path_shift import PathShift, PathShiftConfig
from molly_navigation.utils.pose2d import Pose2D
import numpy as np
import pytest
from sim_utils import drive, LOOP, noisy_route, step, straight


def run(controller, pose, seconds, speed=0.4, dt=0.05):
    """Drive for a while; return the poses and commands."""
    poses, commands = [], []
    for _ in range(int(seconds / dt)):
        out = controller.update(pose, dt, speed)
        step(pose, out, dt)
        poses.append(Pose2D(pose.x, pose.y, pose.yaw))
        commands.append(out)
    return poses, commands


def test_closes_cross_track_sideways_without_turning():
    """0.5 m beside the route: slides back without overshoot, the heading stays on the route."""
    controller = HolonomicController()
    controller.set_path(straight(0.0, 20.0))
    poses, commands = run(controller, Pose2D(0.0, 0.5, 0.0), 12.0)
    ys = [p.y for p in poses]
    assert abs(ys[-1]) < 0.02
    assert min(ys) > -0.01                                   # no overshoot
    assert all(b <= a + 1e-9 for a, b in zip(ys, ys[1:]))     # monotonic
    assert max(abs(p.yaw) for p in poses) < 0.02             # no weaving
    assert any(c.lateral < -0.05 for c in commands)          # it strafed


def test_backs_up_and_turns_when_the_route_is_behind():
    """Facing away from the route direction: reverse slowly while turning around, then follow."""
    controller = HolonomicController(max_reverse_velocity=0.15)
    controller.set_path(straight(5.0, -5.0))
    first = controller.update(Pose2D(0.0, 0.0, 0.0), 0.05, 0.4)
    assert -0.15 - 1e-9 <= first.linear < 0.0
    assert abs(first.angular) > 0.5
    finished, errors = drive(controller, straight(5.0, -5.0), start=Pose2D(0.0, 0.0, 0.0))
    assert finished and max(errors) < 0.2


def test_sideways_departure_after_a_stop():
    """Facing 90 deg off the route (as after a stop): moves along the route sideways, turns."""
    controller = HolonomicController(max_lateral_velocity=0.25)
    controller.set_path(straight(0.0, 10.0))
    out = controller.update(Pose2D(0.0, 0.0, math.pi / 2), 0.05, 0.4)
    assert -0.25 - 1e-9 <= out.lateral < 0.0   # body right = +x
    assert out.angular < -0.5
    poses, commands = run(controller, Pose2D(0.0, 0.0, math.pi / 2), 8.0)
    assert abs(poses[-1].yaw) < 0.05 and abs(poses[-1].y) < 0.05
    assert all(abs(c.lateral) <= 0.25 + 1e-9 for c in commands)


def test_follows_a_parallel_line_with_an_offset():
    """set_offset() moves the robot onto a line beside the route, heading unchanged."""
    controller = HolonomicController()
    controller.set_path(straight(0.0, 20.0))
    controller.set_offset(0.5)
    poses, _ = run(controller, Pose2D(0.0, 0.0, 0.0), 12.0)
    assert abs(poses[-1].y - 0.5) < 0.02
    assert max(abs(p.yaw) for p in poses) < 0.02


def test_robot_that_cannot_strafe_still_completes_the_loop():
    """max_lateral_velocity 0: turn toward the carrot and drive forward, never sideways."""
    route = [(w[0], w[1]) for w in noisy_route(LOOP)]
    controller = HolonomicController(max_lateral_velocity=0.0)
    controller.set_path(route)
    finished, errors = drive(controller, route)
    assert finished and np.mean(errors) < 0.15


def shift(points, current=0.0, s_robot=0.0, **cfg):
    """Choose an offset on a straight route along x for obstacle points (map frame)."""
    progress = PathProgress()
    progress.set_path(straight(-5.0, 15.0))
    ps = PathShift(PathShiftConfig(**cfg))
    s_pts, d_pts = ps.path_coordinates(
        progress.path, progress.arc_lengths, s_robot + 5.0, np.asarray(points, dtype=float)
    )
    return ps, ps.choose(s_pts, d_pts, s_robot + 5.0, current)


def person(x, y, r=0.25, n=12):
    """Points around the outline of someone standing at x, y (a circle of radius r)."""
    return [(x + r * math.cos(a), y + r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, n)]


def test_clear_route_keeps_the_offset_at_zero():
    """Nothing near the route: offset 0, nothing limits the speed."""
    _, result = shift([(3.0, 2.5), (6.0, -2.0)])
    assert result.clear and result.offset == 0.0 and math.isinf(result.free_distance)


def test_person_on_the_route_is_passed_with_clearance():
    """A person on the route: smallest offset that keeps body + clearance off them."""
    _, result = shift(person(2.0, 0.0))
    assert result.clear
    assert 0.25 + 0.28 + 0.10 <= abs(result.offset) <= 0.8 + 1e-9


def test_keeps_its_side_and_returns_only_when_the_body_is_past():
    """No flip-flop between sides; back on the route once the obstacle is behind the body."""
    _, keep = shift(person(2.0, 0.0), current=0.7)
    assert keep.offset == pytest.approx(0.7)
    _, beside = shift(person(0.0, 0.0), current=0.7)        # obstacle next to the robot
    assert beside.offset == pytest.approx(0.7)
    _, behind = shift(person(-1.2, 0.0), current=0.7)       # past it
    assert behind.offset == 0.0


def test_no_gap_within_max_offset_slows_to_a_stop():
    """A wall of people across the corridor: no clear offset, speed limited to stop short."""
    wall = [p for y in np.arange(-1.4, 1.5, 0.4) for p in person(2.0, y)]
    ps, result = shift(wall)
    assert not result.clear
    assert 0.0 < ps.speed_limit(result.free_distance) < 1.2
    _, close = shift(wall, s_robot=1.2)                     # front edge 0.4 m from the wall
    assert ps.speed_limit(close.free_distance) == 0.0


def test_wall_behind_the_route_start_is_not_beside_the_robot():
    """At the start of a route, a wall behind it must not count as an obstacle beside the body."""
    progress = PathProgress()
    progress.set_path(straight(0.0, 10.0))
    ps = PathShift()
    wall = np.array([(-1.5, y) for y in np.arange(-1.5, 1.51, 0.1)])
    s_pts, d_pts = ps.path_coordinates(progress.path, progress.arc_lengths, 0.0, wall)
    result = ps.choose(s_pts, d_pts, 0.0, 0.0)
    assert result.clear and result.offset == 0.0


def test_obstacles_past_the_route_end_are_ignored():
    """Near the end of the route nothing beyond its last point is in the way."""
    progress = PathProgress()
    progress.set_path(straight(0.0, 10.0))
    ps = PathShift()
    beyond = np.array(person(11.0, 0.0))
    s_pts, d_pts = ps.path_coordinates(progress.path, progress.arc_lengths, 9.0, beyond)
    assert ps.choose(s_pts, d_pts, 9.0, 0.0).offset == 0.0


def test_offset_never_exceeds_max_offset():
    """A wide obstacle needing more than max_offset is not passed by wandering off."""
    box = [(2.0, y) for y in np.arange(-0.7, 0.71, 0.05)]
    _, result = shift(box, max_offset=0.8)
    assert not result.clear
