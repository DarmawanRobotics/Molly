"""Visitors confirmed over consecutive scans and counted in front and behind (no ROS)."""

from __future__ import annotations

from dataclasses import dataclass, field
import math


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
            j = min(
                free, key=lambda k: math.hypot(clusters[k].x - track.x, clusters[k].y - track.y)
            )
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
