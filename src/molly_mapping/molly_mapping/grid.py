"""
Log-odds occupancy grid built from LiDAR hits in the map frame (pure numpy, unit-tested).

Every integrated scan marks the cells of its hits occupied and carves free space along a 2D ray
from the sensor to the nearest hit of each angular bin. Bins without a hit carve nothing: an
open doorway or glass gives no evidence that the space behind it is free. The grid grows in
whole cells when hits land outside it, so its cells never move.
"""

import math

from molly_mapping.map_io import FREE, GridMap, OCCUPIED, UNKNOWN
import numpy as np


def logit(p: float) -> float:
    """Return the log-odds of a probability."""
    p = min(max(p, 1e-6), 1.0 - 1e-6)
    return math.log(p / (1.0 - p))


class LogOddsGrid:
    """A growing log-odds grid, row 0 at origin_y, with integrate / occupancy / prior helpers."""

    def __init__(
        self,
        resolution: float = 0.05,
        size_m: float = 20.0,
        center=(0.0, 0.0),
        hit: float = 0.85,
        miss: float = -0.4,
        clamp=(-2.0, 3.5),
        occupied_threshold: float = 0.65,
        free_threshold: float = 0.35,
        grow_m: float = 5.0,
    ):
        """Start an all-unknown square grid of size_m around center."""
        if resolution <= 0.0:
            raise ValueError('resolution must be > 0')
        self.resolution = float(resolution)
        self.hit, self.miss = float(hit), float(miss)
        self.l_min, self.l_max = float(clamp[0]), float(clamp[1])
        self.l_occupied = logit(occupied_threshold)
        self.l_free = logit(free_threshold)
        self.grow_cells = max(1, int(math.ceil(grow_m / self.resolution)))
        cells = max(1, int(math.ceil(size_m / self.resolution)))
        self.origin_x = self.snap(center[0]) - cells // 2 * self.resolution
        self.origin_y = self.snap(center[1]) - cells // 2 * self.resolution
        self.logodds = np.zeros((cells, cells), dtype=np.float32)
        self.scans = 0

    def snap(self, value: float) -> float:
        """Round a coordinate to the cell grid."""
        return math.floor(value / self.resolution) * self.resolution

    @property
    def width(self) -> int:
        """Return the number of columns."""
        return int(self.logodds.shape[1])

    @property
    def height(self) -> int:
        """Return the number of rows."""
        return int(self.logodds.shape[0])

    def clear(self):
        """Forget everything (same extent)."""
        self.logodds[:] = 0.0
        self.scans = 0

    def ensure(self, xmin: float, ymin: float, xmax: float, ymax: float):
        """Grow the grid (by whole cells, keeping cell positions) to contain the box."""
        res = self.resolution
        col0 = math.floor((xmin - self.origin_x) / res)
        row0 = math.floor((ymin - self.origin_y) / res)
        col1 = math.floor((xmax - self.origin_x) / res)
        row1 = math.floor((ymax - self.origin_y) / res)
        grow = self.grow_cells
        left = -col0 + grow if col0 < 0 else 0
        bottom = -row0 + grow if row0 < 0 else 0
        right = col1 - self.width + 1 + grow if col1 >= self.width else 0
        top = row1 - self.height + 1 + grow if row1 >= self.height else 0
        if not (left or bottom or right or top):
            return
        grown = np.zeros((self.height + bottom + top, self.width + left + right), np.float32)
        grown[bottom:bottom + self.height, left:left + self.width] = self.logodds
        self.logodds = grown
        self.origin_x -= left * res
        self.origin_y -= bottom * res

    def cell_index(self, x: np.ndarray, y: np.ndarray):
        """Return (col, row) integer arrays of map points (inside the grid after ensure)."""
        col = np.floor((x - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((y - self.origin_y) / self.resolution).astype(np.int64)
        return col, row

    def integrate(
        self,
        sensor_xy,
        points_xy: np.ndarray,
        angular_bins: int = 720,
        max_range: float = 15.0,
        min_range: float = 0.0,
    ) -> int:
        """Add one scan: hits (N x 2, map frame) seen from sensor_xy; return hits used."""
        points_xy = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        sx, sy = float(sensor_xy[0]), float(sensor_xy[1])
        dx, dy = points_xy[:, 0] - sx, points_xy[:, 1] - sy
        dist = np.hypot(dx, dy)
        keep = np.isfinite(dist) & (dist <= max_range) & (dist > max(min_range, 1e-6))
        if not keep.any():
            return 0
        points_xy, dx, dy, dist = points_xy[keep], dx[keep], dy[keep], dist[keep]
        margin = self.resolution
        self.ensure(
            min(points_xy[:, 0].min(), sx) - margin,
            min(points_xy[:, 1].min(), sy) - margin,
            max(points_xy[:, 0].max(), sx) + margin,
            max(points_xy[:, 1].max(), sy) + margin,
        )
        width = self.width
        col, row = self.cell_index(points_xy[:, 0], points_xy[:, 1])
        hit_cells = np.unique(row * width + col)

        bins = max(1, int(angular_bins))
        angle = np.arctan2(dy, dx)
        b = np.floor((angle + math.pi) / (2.0 * math.pi) * bins).astype(np.int64) % bins
        order = np.lexsort((dist, b))
        first = np.ones(order.size, dtype=bool)
        first[1:] = b[order][1:] != b[order][:-1]
        nearest = order[first]
        reach = dist[nearest] - self.resolution          # stop one cell short of the hit
        free_cells = self.ray_cells(sx, sy, angle[nearest], reach)
        if free_cells.size:
            free_cells = np.setdiff1d(free_cells, hit_cells, assume_unique=True)

        flat = self.logodds.reshape(-1)
        flat[free_cells] = np.maximum(flat[free_cells] + self.miss, self.l_min)
        flat[hit_cells] = np.minimum(flat[hit_cells] + self.hit, self.l_max)
        self.scans += 1
        return int(points_xy.shape[0])

    def ray_cells(self, sx: float, sy: float, angles: np.ndarray, reach: np.ndarray):
        """Return the unique flat cells crossed by rays from (sx, sy) up to reach (vectorized)."""
        reach = np.asarray(reach, dtype=np.float64)
        ok = reach > 0.0
        if not ok.any():
            return np.empty(0, dtype=np.int64)
        angles, reach = np.asarray(angles)[ok], reach[ok]
        step = self.resolution * 0.5
        t = np.arange(int(math.ceil(reach.max() / step)) + 1, dtype=np.float64) * step
        inside = t[None, :] <= reach[:, None]
        xs = sx + np.cos(angles)[:, None] * t[None, :]
        ys = sy + np.sin(angles)[:, None] * t[None, :]
        col, row = self.cell_index(xs[inside], ys[inside])
        valid = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        return np.unique(row[valid] * self.width + col[valid])

    def occupancy(self) -> np.ndarray:
        """Return OccupancyGrid values (int8, row 0 at origin_y): trinary."""
        data = np.full(self.logodds.shape, UNKNOWN, dtype=np.int8)
        data[self.logodds > self.l_occupied] = OCCUPIED
        data[self.logodds < self.l_free] = FREE
        return data

    def known_area(self) -> float:
        """Return the free + occupied area in m^2."""
        known = (self.logodds > self.l_occupied) | (self.logodds < self.l_free)
        return float(np.count_nonzero(known)) * self.resolution ** 2

    def to_grid_map(self) -> GridMap:
        """Return the grid as a GridMap (trinary)."""
        return GridMap(self.occupancy(), self.resolution, self.origin_x, self.origin_y, 0.0)

    def load_prior(self, grid: GridMap, occupied: float = 1.5, free: float = -1.5):
        """Replace the grid with a saved map (same resolution grid), known cells as the prior."""
        if grid.origin_yaw != 0.0:
            raise ValueError('maps with a rotated origin cannot be continued')
        if not math.isclose(grid.resolution, self.resolution, rel_tol=1e-6):
            raise ValueError(
                f'map resolution {grid.resolution} differs from the mapper {self.resolution}'
            )
        self.origin_x, self.origin_y = grid.origin_x, grid.origin_y
        data = np.asarray(grid.data)
        self.logodds = np.zeros(data.shape, dtype=np.float32)
        self.logodds[data >= 50] = occupied
        self.logodds[(data >= 0) & (data < 50)] = free
        self.scans = 0
