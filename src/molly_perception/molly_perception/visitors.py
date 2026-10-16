"""
Pure helpers that find people around the robot in a LiDAR scan (unit-tested without ROS).

Points in base_link are cut to a leg / torso height band, clustered on a grid, kept when a
cluster is person-sized, dropped when it sits on the static map (walls, pillars), and confirmed
over a few scans before they count as visitors.
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class Cluster:
    """One candidate person: centre on the floor in base_link, size and point count."""

    x: float
    y: float
    size: float
    points: int
    confidence: float = 0.5


def band_points(points: np.ndarray, z_min: float, z_max: float, self_box, range_min: float,
                range_max: float) -> np.ndarray:
    """Return the (x, y) of base_link points in the height band, off the body and in range."""
    if points.shape[0] == 0:
        return np.empty((0, 2), dtype=np.float32)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    sx0, sx1, sy0, sy1 = self_box
    on_body = (x > sx0) & (x < sx1) & (y > sy0) & (y < sy1)
    r = np.hypot(x, y)
    keep = (z >= z_min) & (z <= z_max) & ~on_body & (r >= range_min) & (r <= range_max)
    return np.stack([x[keep], y[keep]], axis=1).astype(np.float32, copy=False)


def cluster_labels(xy: np.ndarray, tolerance: float) -> np.ndarray:
    """
    Label points by connected grid cells (cell size = tolerance, 8-neighbours).

    Returns one label per point, 0..k-1. Points in touching cells share a label, so two points
    closer than the tolerance always do.
    """
    n = xy.shape[0]
    if n == 0:
        return np.empty(0, dtype=np.int64)
    cells = np.floor(xy / max(tolerance, 1e-3)).astype(np.int64)
    unique, inverse = np.unique(cells, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    index = {(int(cx), int(cy)): i for i, (cx, cy) in enumerate(unique)}
    parent = list(range(len(unique)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, (cx, cy) in enumerate(unique):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                j = index.get((int(cx) + dx, int(cy) + dy))
                if j is not None and j != i:
                    ri, rj = find(i), find(j)
                    if ri != rj:
                        parent[max(ri, rj)] = min(ri, rj)
    roots = np.array([find(i) for i in range(len(unique))])
    _, compact = np.unique(roots, return_inverse=True)
    return compact.reshape(-1)[inverse]


def major_extent(xy: np.ndarray) -> float:
    """Return the length of a point set along its principal axis (m)."""
    if xy.shape[0] < 2:
        return 0.0
    centred = xy - xy.mean(axis=0)
    cov = centred.T @ centred / xy.shape[0]
    _, vectors = np.linalg.eigh(cov)
    projection = centred @ vectors[:, -1]
    return float(projection.max() - projection.min())


def person_clusters(xy: np.ndarray, tolerance: float, min_points: int, min_width: float,
                    max_width: float) -> list:
    """Return the person-sized clusters of the band points."""
    labels = cluster_labels(xy, tolerance)
    if labels.size == 0:
        return []
    clusters = []
    for label in range(int(labels.max()) + 1):
        member = xy[labels == label]
        if member.shape[0] < min_points:
            continue
        size = major_extent(member)
        if not min_width <= size <= max_width:
            continue
        cx, cy = member.mean(axis=0)
        clusters.append(Cluster(float(cx), float(cy), size, int(member.shape[0])))
    return clusters


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
    def from_grid(cls, data, width: int, height: int, resolution: float, origin_x: float,
                  origin_y: float, origin_yaw: float, margin: float, occupied_at: int = 65):
        """Build from OccupancyGrid data (row-major, row 0 at origin y)."""
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        cells = int(math.ceil(max(margin, 0.0) / resolution)) if resolution > 0 else 0
        return cls(inflate(grid >= occupied_at, cells), resolution, origin_x, origin_y,
                   origin_yaw)

    def occupied(self, x: float, y: float) -> bool:
        """Return True when the map point (x, y) is on or near a static obstacle."""
        dx, dy = x - self.origin_x, y - self.origin_y
        if self.origin_yaw:
            c, s = math.cos(-self.origin_yaw), math.sin(-self.origin_yaw)
            dx, dy = c * dx - s * dy, s * dx + c * dy
        col, row = int(math.floor(dx / self.resolution)), int(math.floor(dy / self.resolution))
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


def fuse_camera(clusters: list, people_base: list, fuse_distance: float) -> list:
    """
    Merge camera people (x, y, score in base_link) into the LiDAR clusters.

    A camera person next to a cluster raises its confidence; one with no cluster nearby is added
    with the camera score.
    """
    fused = [Cluster(c.x, c.y, c.size, c.points, c.confidence) for c in clusters]
    for px, py, score in people_base:
        near = [c for c in fused if math.hypot(c.x - px, c.y - py) <= fuse_distance]
        if near:
            best = min(near, key=lambda c: math.hypot(c.x - px, c.y - py))
            best.confidence = min(1.0, max(best.confidence, 0.5) + 0.4 * score)
        else:
            fused.append(Cluster(px, py, 0.0, 0, max(0.0, min(1.0, score))))
    return fused


def lidar_confidence(size: float) -> float:
    """Return how much a LiDAR cluster of this size looks like one person (0.4-0.7)."""
    return 0.7 if 0.15 <= size <= 0.7 else 0.4


@dataclass
class Track:
    """A visitor followed across scans."""

    x: float
    y: float
    confidence: float
    hits: int = 1
    misses: int = 0


@dataclass
class VisitorTracker:
    """Confirms visitors over consecutive scans (nearest-neighbour association)."""

    confirm_scans: int = 2
    max_misses: int = 2
    assoc_distance: float = 0.6
    tracks: list = field(default_factory=list)

    def update(self, clusters: list) -> list:
        """Feed this scan's clusters; return the confirmed tracks."""
        free = list(range(len(clusters)))
        for track in self.tracks:
            if not free:
                track.misses += 1
                continue
            j = min(free, key=lambda k: math.hypot(clusters[k].x - track.x,
                                                   clusters[k].y - track.y))
            c = clusters[j]
            if math.hypot(c.x - track.x, c.y - track.y) <= self.assoc_distance:
                track.x, track.y, track.confidence = c.x, c.y, c.confidence
                track.hits += 1
                track.misses = 0
                free.remove(j)
            else:
                track.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]
        for k in free:
            c = clusters[k]
            self.tracks.append(Track(c.x, c.y, c.confidence))
        return [t for t in self.tracks if t.hits >= self.confirm_scans and t.misses == 0]


def in_sector(x: float, y: float, centre: float, width: float) -> bool:
    """Return True when the direction of (x, y) is within width / 2 of centre (rad)."""
    angle = math.atan2(y, x)
    diff = (angle - centre + math.pi) % (2.0 * math.pi) - math.pi
    return abs(diff) <= width / 2.0


def summarize(positions: list, front_width: float, behind_width: float) -> dict:
    """Return counts and nearest distances overall, in front and behind (-1: nobody)."""
    front, behind = [], []
    for x, y in positions:
        r = math.hypot(x, y)
        if in_sector(x, y, 0.0, front_width):
            front.append(r)
        elif in_sector(x, y, math.pi, behind_width):
            behind.append(r)
    everyone = [math.hypot(x, y) for x, y in positions]
    return {
        'count': len(positions),
        'front': len(front),
        'behind': len(behind),
        'nearest': min(everyone) if everyone else -1.0,
        'nearest_front': min(front) if front else -1.0,
        'nearest_behind': min(behind) if behind else -1.0,
    }
