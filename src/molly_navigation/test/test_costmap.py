"""Costmap: obstacle masks, pooling, distances against brute force, inflation costs."""

import math

import numpy as np
import pytest

costmap = pytest.importorskip('molly_navigation.utils.costmap')


def test_mask_pool_and_stamp():
    """Unknown blocks only when asked; pooling keeps any obstacle; points stamp their cells."""
    data = np.array([[0, -1], [100, 0]], dtype=np.int8)
    assert costmap.obstacle_mask(data, False).tolist() == [[False, False], [True, False]]
    assert costmap.obstacle_mask(data, True).tolist() == [[False, True], [True, False]]
    big = np.zeros((5, 5), dtype=bool)
    big[4, 4] = True
    pooled = costmap.max_pool(big, 2)
    assert pooled.shape == (3, 3) and pooled[2, 2] and pooled.sum() == 1
    stamped = costmap.stamp_points(np.zeros((4, 4), bool), [[0.25, 0.35], [9, 9]], 0.1, 0.0, 0.0)
    assert stamped[3, 2] and stamped.sum() == 1


def test_distance_matches_brute_force():
    """The shifted-disk distance equals the brute-force nearest obstacle within the radius."""
    rng = np.random.default_rng(1)
    mask = rng.random((25, 30)) < 0.03
    dist = costmap.obstacle_distance(mask, 6)
    obstacles = np.argwhere(mask)
    for r in range(mask.shape[0]):
        for c in range(mask.shape[1]):
            d = np.min(np.hypot(obstacles[:, 0] - r, obstacles[:, 1] - c))
            expected = d if d <= 6 + 1e-9 else np.inf
            assert dist[r, c] == pytest.approx(expected)


def test_inflation_is_lethal_near_walls_and_decays():
    """Lethal within robot_radius, costlier near walls, 1 beyond the inflation radius."""
    data = np.zeros((40, 40), dtype=np.int8)
    data[:, 20] = 100  # a wall along x = 1.0 .. 1.05
    cm = costmap.build_costmap(data, 0.05, 0.0, 0.0, planning_resolution=0.05,
                               robot_radius=0.3, inflation_radius=0.6)
    assert math.isinf(cm.cost_at(1.02, 1.0))
    assert math.isinf(cm.cost_at(0.8, 1.0))          # 0.2 m from the wall
    near, far = cm.cost_at(0.62, 1.0), cm.cost_at(0.5, 1.0)
    assert near > far > 1.0                           # inside the inflation, decaying
    assert cm.cost_at(0.3, 1.0) == pytest.approx(1.0)
    assert math.isinf(cm.cost_at(-1.0, 1.0))          # outside the grid


def test_extra_obstacles_and_unknown():
    """Stamped people block their surroundings; unknown space blocks unless allowed."""
    data = np.zeros((30, 30), dtype=np.int8)
    data[:, 25:] = -1
    blocked = costmap.build_costmap(data, 0.1, 0.0, 0.0, robot_radius=0.3,
                                    extra_obstacles=[[1.5, 1.5]])
    assert math.isinf(blocked.cost_at(1.55, 1.55)) and math.isinf(blocked.cost_at(2.8, 1.0))
    open_ = costmap.build_costmap(data, 0.1, 0.0, 0.0, unknown_is_obstacle=False)
    assert math.isfinite(open_.cost_at(2.8, 1.0))
