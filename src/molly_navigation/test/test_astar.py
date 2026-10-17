"""A*, escaping, snapping, line checks and shortcuts on small costmaps."""

import math

import numpy as np
import pytest

astar = pytest.importorskip('molly_navigation.utils.astar')


def grid(rows=20, cols=20):
    """Return an all-free cost grid."""
    return np.ones((rows, cols))


def test_open_grid_path_is_optimal_and_connected():
    """On free space the path is as long as the octile distance and every step is adjacent."""
    cost = grid()
    path = astar.astar(cost, (2, 3), (15, 10))
    assert path[0] == (2, 3) and path[-1] == (15, 10)
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
    assert all(s in (1.0, pytest.approx(math.sqrt(2))) for s in steps)
    assert sum(steps) == pytest.approx(7 * math.sqrt(2) + 6)


def test_wall_with_a_door_and_no_corner_cutting():
    """The path goes through the door, never through a wall or diagonally past a corner."""
    cost = grid()
    cost[:, 10] = np.inf
    cost[14, 10] = 1.0  # the door
    path = astar.astar(cost, (2, 2), (2, 18))
    assert (14, 10) in path
    for r, c in path:
        assert math.isfinite(cost[r, c])
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        if r0 != r1 and c0 != c1:
            assert math.isfinite(cost[r0, c1]) and math.isfinite(cost[r1, c0])


def test_blocked_and_lethal_ends():
    """No path through a closed wall; lethal start or goal gives None."""
    cost = grid()
    cost[:, 10] = np.inf
    assert astar.astar(cost, (2, 2), (2, 18)) is None
    assert astar.astar(cost, (2, 10), (2, 2)) is None


def test_search_limit_raises():
    """Running out of expansions raises SearchTimeout instead of hanging."""
    with pytest.raises(astar.SearchTimeout):
        astar.astar(grid(60, 60), (0, 0), (59, 59), max_expansions=10)


def test_nearest_free_escapes_and_snaps():
    """The nearest passable cell is found within the radius, None beyond."""
    cost = grid()
    cost[5:10, 5:10] = np.inf
    assert astar.nearest_free(cost, (3, 3), 2) == (3, 3)
    assert astar.nearest_free(cost, (7, 9), 3) == (7, 10)
    assert astar.nearest_free(cost, (7, 7), 2) is None


def test_shortcut_never_cuts_a_wall_or_hugs_it():
    """Shortcuts replace staircases by lines but never cross lethal or costlier cells."""
    cost = grid(30, 30)
    cost[0:20, 15] = np.inf          # a wall with the way around at the bottom rows
    cost[0:20, 14] = 4.0             # costly cells along it (inflation)
    path = astar.astar(cost, (2, 2), (2, 28))
    short = astar.shortcut(cost, path)
    assert short[0] == path[0] and short[-1] == path[-1]
    assert len(short) < len(path)
    for a, b in zip(short, short[1:]):
        rows, cols = astar.line_cells(a, b)
        assert np.all(np.isfinite(cost[rows, cols]))
    worst = max(cost[c] for c in path)
    for a, b in zip(short, short[1:]):
        rows, cols = astar.line_cells(a, b)
        assert cost[rows, cols].max() <= worst


def test_line_cells_cover_the_ends():
    """A line starts and ends in its end cells."""
    rows, cols = astar.line_cells((1, 1), (4, 9))
    assert (rows[0], cols[0]) == (1, 1) and (rows[-1], cols[-1]) == (4, 9)
