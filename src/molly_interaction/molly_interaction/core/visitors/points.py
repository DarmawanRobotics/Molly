"""People-sized clusters in the LiDAR points around the robot (numpy, no ROS)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Cluster:
    """One candidate person: centre on the floor in base_link, size and point count."""

    x: float
    y: float
    size: float
    points: int
    confidence: float = 0.5


def band_points(
    points: np.ndarray, z_min: float, z_max: float, self_box, range_min: float, range_max: float
) -> np.ndarray:
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


def person_clusters(
    xy: np.ndarray, tolerance: float, min_points: int, min_width: float, max_width: float
) -> list:
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


def lidar_confidence(size: float) -> float:
    """Return how much a LiDAR cluster of this size looks like one person (0.4-0.7)."""
    return 0.7 if 0.15 <= size <= 0.7 else 0.4
