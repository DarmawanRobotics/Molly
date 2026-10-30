"""The site map's walls and pillars: clusters on them are not visitors (numpy, no ROS)."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def inflate(occupied: np.ndarray, cells: int) -> np.ndarray:
    """Grow a boolean grid by a disc of `cells` cells (numpy only, no scipy)."""
    if cells <= 0 or not occupied.any():
        return occupied.copy()
    grown = occupied.copy()
    h, w = occupied.shape
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx * dx + dy * dy > cells * cells or (dx == 0 and dy == 0):
                continue
            src_y = slice(max(0, -dy), min(h, h - dy))
            dst_y = slice(max(0, dy), min(h, h + dy))
            src_x = slice(max(0, -dx), min(w, w - dx))
            dst_x = slice(max(0, dx), min(w, w + dx))
            grown[dst_y, dst_x] |= occupied[src_y, src_x]
    return grown


@dataclass
class StaticMap:
    """Occupied cells of the site map, grown by the margin, with its geometry."""

    mask: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    @classmethod
    def from_grid(
        cls,
        data,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        margin: float,
        occupied_at: int = 65,
    ):
        """Build from OccupancyGrid data (row-major, row 0 at origin y)."""
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        cells = math.ceil(max(margin, 0.0) / resolution) if resolution > 0 else 0
        return cls(inflate(grid >= occupied_at, cells), resolution, origin_x, origin_y, origin_yaw)

    def occupied(self, x: float, y: float) -> bool:
        """Return True when the map point (x, y) is on or near a static obstacle."""
        dx, dy = x - self.origin_x, y - self.origin_y
        if self.origin_yaw:
            c, s = math.cos(-self.origin_yaw), math.sin(-self.origin_yaw)
            dx, dy = c * dx - s * dy, s * dx + c * dy
        col, row = math.floor(dx / self.resolution), math.floor(dy / self.resolution)
        h, w = self.mask.shape
        return 0 <= row < h and 0 <= col < w and bool(self.mask[row, col])


def to_frame(x: float, y: float, pose) -> tuple:
    """Apply a planar pose (tx, ty, yaw) to a point."""
    tx, ty, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def from_frame(x: float, y: float, pose) -> tuple:
    """Apply the inverse of a planar pose (tx, ty, yaw) to a point."""
    tx, ty, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - tx, y - ty
    return c * dx + s * dy, -s * dx + c * dy


def drop_static(clusters: list, static_map, map_base) -> list:
    """Drop clusters on the static map; keep everything without a map or a robot pose."""
    if static_map is None or map_base is None:
        return list(clusters)
    return [c for c in clusters if not static_map.occupied(*to_frame(c.x, c.y, map_base))]
