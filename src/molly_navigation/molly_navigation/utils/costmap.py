"""
Costmap for path planning from an occupancy grid (pure numpy, unit-tested).

Obstacles (occupied cells, and unknown ones when unknown_is_obstacle) are max-pooled onto the
planning resolution and inflated: cells closer than robot_radius are lethal (inf), cells up to
inflation_radius cost more the closer they get, everything else costs 1 per cell length. Extra
obstacles (people seen by the LiDAR) can be stamped in before inflating, for replanning.
"""

from dataclasses import dataclass
import math

import numpy as np

INFLATION_WEIGHT = 5.0  # extra cost right outside robot_radius, decaying to 0 at inflation_radius


@dataclass
class Costmap:
    """Planning grid: cost[row, col] (inf = lethal), row 0 at origin_y."""

    cost: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def rows(self) -> int:
        """Return the number of rows."""
        return int(self.cost.shape[0])

    @property
    def cols(self) -> int:
        """Return the number of columns."""
        return int(self.cost.shape[1])

    def cell(self, x: float, y: float):
        """Return (row, col) of a map point (may be outside the grid)."""
        return (
            int(math.floor((y - self.origin_y) / self.resolution)),
            int(math.floor((x - self.origin_x) / self.resolution)),
        )

    def center(self, row: int, col: int):
        """Return the map point at the centre of a cell."""
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def inside(self, row: int, col: int) -> bool:
        """Return True when the cell is in the grid."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def lethal(self, row: int, col: int) -> bool:
        """Return True for cells outside the grid or inside an inflated obstacle."""
        return not self.inside(row, col) or not math.isfinite(self.cost[row, col])

    def cost_at(self, x: float, y: float) -> float:
        """Return the cost at a map point (inf outside the grid)."""
        row, col = self.cell(x, y)
        return float(self.cost[row, col]) if self.inside(row, col) else math.inf


def obstacle_mask(data: np.ndarray, unknown_is_obstacle: bool, occupied_threshold: int = 50):
    """Return True where OccupancyGrid values block the robot."""
    data = np.asarray(data, dtype=np.int16)
    mask = data >= occupied_threshold
    if unknown_is_obstacle:
        mask |= data < 0
    return mask


def max_pool(mask: np.ndarray, factor: int) -> np.ndarray:
    """Return a coarser mask: a coarse cell is set when any of its fine cells is."""
    if factor <= 1:
        return mask.copy()
    rows = -(-mask.shape[0] // factor)
    cols = -(-mask.shape[1] // factor)
    padded = np.zeros((rows * factor, cols * factor), dtype=bool)
    padded[:mask.shape[0], :mask.shape[1]] = mask
    return padded.reshape(rows, factor, cols, factor).any(axis=(1, 3))


def disk_offsets(radius_cells: int):
    """Return (drow, dcol, distance) within a radius, sorted by distance."""
    r = int(radius_cells)
    offsets = [
        (dr, dc, math.hypot(dr, dc))
        for dr in range(-r, r + 1)
        for dc in range(-r, r + 1)
        if math.hypot(dr, dc) <= r + 1e-9
    ]
    return sorted(offsets, key=lambda o: o[2])


def obstacle_distance(mask: np.ndarray, max_cells: int) -> np.ndarray:
    """Return the distance (cells) to the nearest obstacle, inf beyond max_cells."""
    dist = np.full(mask.shape, np.inf)
    if not mask.any():
        return dist
    rows, cols = mask.shape
    for dr, dc, d in disk_offsets(max_cells):
        # cells (r, c) whose neighbour (r + dr, c + dc) is an obstacle
        src = mask[max(dr, 0):rows + min(dr, 0), max(dc, 0):cols + min(dc, 0)]
        dst = dist[max(-dr, 0):rows + min(-dr, 0), max(-dc, 0):cols + min(-dc, 0)]
        np.minimum(dst, np.where(src, d, np.inf), out=dst)
    return dist


def stamp_points(mask: np.ndarray, points_xy, resolution, origin_x, origin_y) -> np.ndarray:
    """Return a copy of the mask with the cells of map points set."""
    out = mask.copy()
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if points.size == 0:
        return out
    cols = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    rows = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    ok = (rows >= 0) & (rows < out.shape[0]) & (cols >= 0) & (cols < out.shape[1])
    out[rows[ok], cols[ok]] = True
    return out


def build_costmap(
    data: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    planning_resolution: float = 0.1,
    robot_radius: float = 0.35,
    inflation_radius: float = 0.7,
    cost_scaling: float = 3.0,
    unknown_is_obstacle: bool = True,
    extra_obstacles=None,
    occupied_threshold: int = 50,
) -> Costmap:
    """Return the inflated planning costmap of an occupancy grid (row 0 at origin_y)."""
    factor = max(1, int(round(planning_resolution / resolution)))
    res = resolution * factor
    mask = max_pool(obstacle_mask(data, unknown_is_obstacle, occupied_threshold), factor)
    if extra_obstacles is not None:
        mask = stamp_points(mask, extra_obstacles, res, origin_x, origin_y)
    reach = max(inflation_radius, robot_radius)
    dist = obstacle_distance(mask, int(math.ceil(reach / res)) + 1) * res
    cost = np.ones(mask.shape)
    lethal = dist <= robot_radius + 1e-6
    span = max(reach - robot_radius, 1e-6)
    near = ~lethal & (dist < reach)
    edge = math.exp(-cost_scaling * span)
    decay = (np.exp(-cost_scaling * (dist[near] - robot_radius)) - edge) / max(1.0 - edge, 1e-9)
    cost[near] = 1.0 + INFLATION_WEIGHT * np.clip(decay, 0.0, 1.0)
    cost[lethal] = np.inf
    return Costmap(cost, res, origin_x, origin_y)
