import math

from molly_navigation.utils.path_processing import (
    curvatures,
    parse_route_rows,
    process_route,
    resample,
    velocity_profile,
)
from sim_utils import LOOP, noisy_route


def test_resample_uniform_spacing():
    """Resampled points are evenly spaced and keep both endpoints."""
    pts = resample([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 0.1)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert pts[0] == (0.0, 0.0) and pts[-1] == (1.0, 1.0)
    assert max(gaps) <= 0.1 + 1e-9


def test_smoothing_keeps_stop_points_and_reduces_jitter():
    """Stop points stay exactly where they were recorded and the straight gets straighter."""
    raw = noisy_route([(0, 0), (12, 0)], noise=0.03)
    raw[30] = (raw[30][0], raw[30][1], 90.0, 5.0)
    route, deviation = process_route(raw, 0.1, 0.05, 0.4)
    assert raw[30] in route
    assert deviation < 0.15
    raw_k = max(curvatures([(w[0], w[1]) for w in raw], 1))
    smooth_k = max(curvatures([(w[0], w[1]) for w in route]))
    assert smooth_k < raw_k / 5.0


def test_velocity_profile_brakes_into_stops():
    """Speed is capped at stop points and ramps down before them."""
    route, _ = process_route(noisy_route(LOOP), 0.1, 0.05, 0.4)
    pts = [(w[0], w[1]) for w in route]
    v = velocity_profile(
        pts, [50], cruise=0.4, min_speed=0.1, max_lateral_accel=0.3, decel=0.3, approach_speed=0.15
    )
    assert v[50] <= 0.15 + 1e-9
    assert v[48] < v[45] <= 0.4
    assert v[-1] <= 0.15 + 1e-9
    assert max(v) <= 0.4 + 1e-9


def test_route_rows_drop_what_cannot_be_driven():
    """Comments and blanks are skipped; NaN, inf and text rows are dropped, negative dwell is 0."""
    rows = [
        ['# x', 'y', 'yaw_deg', 'dwell_sec'],
        [],
        ['0.0', '0.0'],
        ['1.0', '0.0', '90'],
        [' 2.0', '0.0', '0', '5.0'],
        ['nan', '0.0'],
        ['3.0', 'inf'],
        ['x', 'y'],
        ['4.0'],
        ['5.0', '0.0', '0', '-2'],
    ]
    waypoints, problems = parse_route_rows(rows)
    assert waypoints == [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, 90.0, 0.0),
        (2.0, 0.0, 0.0, 5.0),
        (5.0, 0.0, 0.0, 0.0),
    ]
    lines = [p.split(':')[0] for p in problems]
    assert lines == ['line 6', 'line 7', 'line 8', 'line 9', 'line 10']
