"""Log-odds grid: hits occupy, rays carve free space, nothing behind empty bins, growth."""

import math

import numpy as np
import pytest

grid_module = pytest.importorskip('molly_mapping.grid')
map_io = pytest.importorskip('molly_mapping.map_io')


def wall_scan(x=3.0, half=2.0, spacing=0.02):
    """Return points of a wall x = const from y = -half to half."""
    ys = np.arange(-half, half + 1e-9, spacing)
    return np.column_stack([np.full_like(ys, x), ys])


def cell(grid, x, y):
    """Return the occupancy value at a map point."""
    col, row = grid.cell_index(np.array([x]), np.array([y]))
    return int(grid.occupancy()[row[0], col[0]])


def test_wall_is_occupied_and_space_before_it_free():
    """After a few scans the wall is occupied, the corridor before it free, behind unknown."""
    grid = grid_module.LogOddsGrid(resolution=0.05, size_m=10.0)
    for _ in range(3):
        grid.integrate((0.0, 0.0), wall_scan())
    assert cell(grid, 3.0, 0.0) == map_io.OCCUPIED
    assert cell(grid, 1.5, 0.0) == map_io.FREE
    assert cell(grid, 2.0, 1.0) == map_io.FREE
    assert cell(grid, 4.0, 0.0) == map_io.UNKNOWN
    assert cell(grid, -2.0, 0.0) == map_io.UNKNOWN  # no hit behind the robot: nothing carved
    assert grid.scans == 3
    assert grid.known_area() > 3.0


def test_a_person_walking_through_is_erased_again():
    """Free space carved through a cell that was hit once turns it back into free."""
    grid = grid_module.LogOddsGrid(resolution=0.05, size_m=10.0)
    grid.integrate((0.0, 0.0), np.array([[1.5, 0.0]]))
    assert cell(grid, 1.5, 0.0) == map_io.OCCUPIED
    for _ in range(4):
        grid.integrate((0.0, 0.0), wall_scan())
    assert cell(grid, 1.5, 0.0) == map_io.FREE


def test_range_limits_and_non_finite_points_are_ignored():
    """Points beyond max_range, inside min_range or not finite change nothing."""
    grid = grid_module.LogOddsGrid(resolution=0.1, size_m=4.0)
    used = grid.integrate(
        (0.0, 0.0), np.array([[30.0, 0.0], [0.05, 0.0], [np.nan, 1.0]]), max_range=15.0,
        min_range=0.2,
    )
    assert used == 0 and grid.scans == 0
    assert (grid.occupancy() == map_io.UNKNOWN).all()


def test_grid_grows_without_moving_cells():
    """Hits outside the grid grow it in whole cells; old cells keep their map position."""
    grid = grid_module.LogOddsGrid(resolution=0.1, size_m=2.0)
    grid.integrate((0.0, 0.0), np.array([[0.5, 0.5]]))
    width = grid.width
    grid.integrate((0.0, 0.0), np.array([[-7.0, 4.0]]), max_range=20.0)
    assert grid.width > width
    assert cell(grid, 0.55, 0.55) == map_io.OCCUPIED
    assert cell(grid, -6.95, 4.05) == map_io.OCCUPIED
    ox, oy = grid.origin_x / 0.1, grid.origin_y / 0.1
    assert ox == pytest.approx(round(ox)) and oy == pytest.approx(round(oy))


def test_prior_round_trip_and_rejections():
    """A saved map loads back as the prior; another resolution or a rotated origin is refused."""
    grid = grid_module.LogOddsGrid(resolution=0.05, size_m=10.0)
    for _ in range(3):
        grid.integrate((0.0, 0.0), wall_scan())
    saved = map_io.crop_known(grid.to_grid_map(), 0.5)
    other = grid_module.LogOddsGrid(resolution=0.05, size_m=4.0)
    other.load_prior(saved)
    assert cell(other, 3.0, 0.0) == map_io.OCCUPIED
    assert cell(other, 1.0, 0.0) == map_io.FREE
    with pytest.raises(ValueError):
        grid_module.LogOddsGrid(resolution=0.1).load_prior(saved)
    with pytest.raises(ValueError):
        other.load_prior(map_io.GridMap(saved.data, 0.05, 0.0, 0.0, math.pi / 2))


def test_ray_cells_stop_at_the_reach():
    """ray_cells covers the cells from the sensor up to the reach, not past it."""
    grid = grid_module.LogOddsGrid(resolution=0.1, size_m=4.0)
    cells = grid.ray_cells(0.05, 0.05, np.array([0.0]), np.array([0.95]))
    cols = sorted(int(c) % grid.width - int(round(-grid.origin_x / 0.1)) for c in cells)
    assert cols == list(range(0, 10))
