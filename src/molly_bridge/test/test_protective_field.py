import math

from molly_bridge.protective_field import (
    FieldConfig,
    footprint_extent,
    limit,
    max_speed,
    path_clearance,
    scan_points,
)

CFG = FieldConfig()


def wall(x0: float, y_from: float = -2.0, y_to: float = 2.0, step: float = 0.05):
    """Points of a wall across the robot at x = x0."""
    n = int((y_to - y_from) / step) + 1
    return [(x0, y_from + i * step) for i in range(n)]


def test_footprint_extent_is_the_box_support():
    """Forward: front edge; sideways: half width; diagonal: projected box."""
    assert footprint_extent(CFG, 1.0, 0.0) == (0.40, 0.28)
    assert footprint_extent(CFG, 0.0, 1.0) == (0.28, 0.40)
    along, _ = footprint_extent(CFG, math.sqrt(0.5), math.sqrt(0.5))
    assert abs(along - (0.40 + 0.28) * math.sqrt(0.5)) < 1e-9


def test_max_speed_inverts_the_stopping_distance():
    """At the returned speed, reaction + braking distance equals the free room."""
    for clearance in (0.3, 0.6, 1.5, 3.0):
        v = max_speed(CFG, clearance)
        room = clearance - CFG.margin
        stopping = v * CFG.reaction_time + v * v / (2.0 * CFG.max_decel)
        assert abs(stopping - room) < 1e-9
    assert max_speed(CFG, CFG.margin) == 0.0
    assert max_speed(CFG, math.inf) == math.inf


def test_free_path_is_clear():
    """Nothing in the way: full speed in every direction."""
    for vx, vy in ((0.6, 0.0), (0.0, 0.4), (-0.3, 0.0)):
        assert limit(CFG, vx, vy, []).state == 'clear'


def test_wall_ahead_limits_then_stops_forward_motion_only():
    """A wall 1.2 m ahead limits a fast forward command, 0.5 m stops it; sideways stays free."""
    far = limit(CFG, 1.0, 0.0, wall(1.2))
    assert far.state == 'limited' and 0.0 < far.scale < 1.0
    assert abs(far.clearance - 0.8) < 1e-9
    near = limit(CFG, 0.3, 0.0, wall(0.5))
    assert near.state == 'stop' and near.scale == 0.0
    assert limit(CFG, 0.0, 0.3, wall(0.6)).state == 'clear'
    assert limit(CFG, -0.3, 0.0, wall(0.5)).state == 'clear'  # backing away


def test_reverse_and_sideways_are_protected_too():
    """Obstacles behind or beside the robot limit motion toward them (holonomic approach)."""
    assert limit(CFG, -0.3, 0.0, [(-0.6, 0.0)]).state == 'limited'  # 0.2 m behind the edge
    assert limit(CFG, -0.3, 0.0, [(-0.54, 0.0)]).state == 'stop'  # inside the margin
    left = [(0.0, 0.42)]
    assert limit(CFG, 0.0, 0.3, left).state == 'stop'
    assert limit(CFG, 0.3, 0.0, left).state == 'clear'


def test_point_inside_the_footprint_blocks_motion_toward_it():
    """Something already touching the robot stops motion toward it."""
    assert path_clearance(CFG, 1.0, 0.0, [(0.3, 0.0)]) < 0.0
    assert limit(CFG, 0.2, 0.0, [(0.3, 0.0)]).scale == 0.0


def test_scan_points_skip_empty_sectors():
    """Only sectors that saw something become points, at the sector centre."""
    pts = scan_points(-math.pi, math.pi / 2, [4.0, 1.0, 4.0, 2.0], 4.0)
    assert len(pts) == 2
    x, y = pts[0]
    assert abs(math.atan2(y, x) - (-math.pi / 4)) < 1e-9 and abs(math.hypot(x, y) - 1.0) < 1e-9


def test_sliding_past_something_beside_the_robot_is_allowed():
    """An obstacle just in front of the nose does not block sideways motion."""
    nose = [(0.48, 0.02)]
    assert limit(CFG, 0.0, 0.3, nose).state == 'clear'
    assert limit(CFG, 0.3, 0.0, nose).state == 'stop'
