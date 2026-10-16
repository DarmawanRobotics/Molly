from bisect import bisect_right
from dataclasses import dataclass
import math

from molly_navigation.utils.geometry import wrap_angle
from molly_navigation.utils.pose2d import Pose2D


@dataclass
class PathReference:
    """Robot pose expressed relative to the path at the current progress index."""

    lateral_error: float
    heading_error: float
    curvature: float
    path_yaw: float


class PathProgress:
    """Monotonic progress along a polyline path, robust to overlapping or looping routes."""

    def __init__(self, search_window: int = 200, heading_weight: float = 1.0):
        self._search_window = search_window
        self._heading_weight = heading_weight
        self._path: list[tuple[float, float]] = []
        self._index = 0
        self._initialized = False

    def set_path(self, path_xy: list[tuple[float, float]], start_index=None):
        """Replace the path; without start_index the next update searches the whole path."""
        self._path = list(path_xy)
        self._index = start_index or 0
        self._initialized = start_index is not None
        self._curvature = self.signed_curvatures()
        self._s = [0.0]  # cumulative arc length at every point
        for (ax, ay), (bx, by) in zip(self._path, self._path[1:]):
            self._s.append(self._s[-1] + math.hypot(bx - ax, by - ay))

    @property
    def arc_lengths(self) -> list[float]:
        """Cumulative arc length at every path point."""
        return self._s

    @property
    def length(self) -> float:
        """Total arc length of the path in metres."""
        return self._s[-1] if self._path else 0.0

    def projection(self, pose: Pose2D) -> float:
        """Arc length of the robot's projection onto the segment at the progress index."""
        if len(self._path) < 2:
            return 0.0
        i = min(self._index, len(self._path) - 2)
        fraction = max(0.0, min(1.0, self.segment_fraction(i, pose)))
        return self._s[i] + fraction * (self._s[i + 1] - self._s[i])

    def point_at(self, s: float) -> tuple[float, float]:
        """Point at arc length s (clamped to the path), linearly interpolated."""
        if not self._path:
            return 0.0, 0.0
        if len(self._path) == 1 or s <= 0.0:
            return self._path[0]
        if s >= self._s[-1]:
            return self._path[-1]
        i = bisect_right(self._s, s) - 1
        length = self._s[i + 1] - self._s[i]
        t = (s - self._s[i]) / length if length > 1e-12 else 0.0
        (ax, ay), (bx, by) = self._path[i], self._path[i + 1]
        return ax + t * (bx - ax), ay + t * (by - ay)

    def heading_between(self, s0: float, s1: float) -> float:
        """Heading of the chord from arc length s0 to s1 (a smoothed path tangent)."""
        ax, ay = self.point_at(s0)
        bx, by = self.point_at(s1)
        if math.hypot(bx - ax, by - ay) < 1e-6:
            i = min(max(bisect_right(self._s, s0) - 1, 0), max(len(self._path) - 2, 0))
            return self.segment_heading(i)
        return math.atan2(by - ay, bx - ax)

    @property
    def path(self) -> list[tuple[float, float]]:
        """Return the tracked path."""
        return self._path

    @property
    def index(self) -> int:
        """Return the current nearest path index."""
        return self._index

    def segment_heading(self, i: int) -> float:
        """Heading of the path segment starting at index i."""
        j = min(i + 1, len(self._path) - 1)
        k = max(j - 1, 0)
        return math.atan2(self._path[j][1] - self._path[k][1], self._path[j][0] - self._path[k][0])

    def score(self, i: int, pose: Pose2D, tracking: bool = False) -> float:
        """Distance to point i plus a penalty for driving against the segment direction."""
        d = math.hypot(self._path[i][0] - pose.x, self._path[i][1] - pose.y)
        weight = self._heading_weight * (0.3 if tracking else 1.0)
        return d + weight * abs(wrap_angle(self.segment_heading(i) - pose.yaw)) / math.pi

    def update(self, pose: Pose2D) -> int:
        """Advance the nearest index (global search on first call, forward window afterwards)."""
        if not self._path:
            return 0
        if not self._initialized:
            self._index = min(range(len(self._path)), key=lambda i: self.score(i, pose))
            self._initialized = True
            return self._index
        end = min(len(self._path), self._index + self._search_window)
        self._index = min(
            range(self._index, end), key=lambda i: self.score(i, pose, tracking=True)
        )
        while self._index < len(self._path) - 2 and self.segment_fraction(self._index, pose) > 1.0:
            self._index += 1
        return self._index

    def segment_fraction(self, i: int, pose: Pose2D) -> float:
        """Projection of the robot onto segment i -> i+1 (0 at start, 1 at end)."""
        (ax, ay), (bx, by) = self._path[i], self._path[i + 1]
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        return ((pose.x - ax) * dx + (pose.y - ay) * dy) / length_sq if length_sq > 1e-12 else 1.0

    def signed_curvatures(self, span: int = 3) -> list[float]:
        """Signed curvature (1/m, +left) at every point from heading change over +-span points."""
        n = len(self._path)
        out = [0.0] * n
        for i in range(span, n - span):
            (ax, ay), (bx, by), (cx, cy) = (
                self._path[i - span],
                self._path[i],
                self._path[i + span],
            )
            h1 = math.atan2(by - ay, bx - ax)
            h2 = math.atan2(cy - by, cx - bx)
            length = math.hypot(bx - ax, by - ay) + math.hypot(cx - bx, cy - by)
            if length > 1e-6:
                out[i] = 2.0 * wrap_angle(h2 - h1) / length
        return out

    def reference(self, pose: Pose2D) -> PathReference:
        """Return lateral error (+left), heading error, curvature and path yaw at the index."""
        i = min(self._index, len(self._path) - 2)
        (ax, ay), (bx, by) = self._path[i], self._path[i + 1]
        path_yaw = math.atan2(by - ay, bx - ax)
        dx, dy = pose.x - ax, pose.y - ay
        lateral = math.cos(path_yaw) * dy - math.sin(path_yaw) * dx
        return PathReference(
            lateral_error=lateral,
            heading_error=wrap_angle(pose.yaw - path_yaw),
            curvature=self._curvature[self._index] if self._curvature else 0.0,
            path_yaw=path_yaw,
        )

    def lookahead_point(self, pose: Pose2D, distance: float) -> tuple[float, float]:
        """First path point at least `distance` away, walking forward from the current index."""
        for i in range(self._index, len(self._path)):
            if math.hypot(self._path[i][0] - pose.x, self._path[i][1] - pose.y) >= distance:
                return self._path[i]
        return self._path[-1]

    def window(self, count: int) -> list[tuple[float, float]]:
        """Up to `count` path points starting at the current index."""
        return self._path[self._index:self._index + count]

    def is_finished(self, pose: Pose2D, tolerance: float, end_margin: int = 3) -> bool:
        """Return True once progress reached the end and the robot is within tolerance."""
        if not self._path:
            return True
        if self._index < len(self._path) - 1 - end_margin:
            return False
        gx, gy = self._path[-1]
        return math.hypot(gx - pose.x, gy - pose.y) < tolerance
