import math

from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.pose2d import Pose2D


class PurePursuit:
    """Tracks a path of x,y points and outputs steering curvature toward a lookahead point."""

    def __init__(self, lookahead_distance: float = 1.0, goal_tolerance: float = 0.3):
        self._lookahead_distance = lookahead_distance
        self._goal_tolerance = goal_tolerance
        self._progress = PathProgress()
        self._last_lookahead = (0.0, 0.0)

    def set_path(self, path_xy: list[tuple[float, float]], start_index=None):
        """Replace the tracked path, optionally pinning progress to start_index."""
        self._progress.set_path(path_xy, start_index)

    def set_lookahead_distance(self, distance: float):
        """Update the lookahead distance, floored to avoid a degenerate zero-length lookahead."""
        self._lookahead_distance = max(0.05, distance)

    def lookahead_distance(self) -> float:
        """Return the currently configured lookahead distance."""
        return self._lookahead_distance

    def last_lookahead_point(self) -> tuple[float, float]:
        """Return the lookahead point computed by the most recent update() call."""
        return self._last_lookahead

    def nearest_point(self) -> tuple[float, float]:
        """Return the path point at the current progress index."""
        path = self._progress.path
        return path[self._progress.index] if path else (0.0, 0.0)

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        return self._progress.index

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once progress reached the end and the robot is within goal_tolerance."""
        return self._progress.is_finished(pose, self._goal_tolerance)

    def update(self, pose: Pose2D) -> float:
        """Advance tracking to the current pose and return the target curvature (1/m, +left)."""
        if not self._progress.path:
            return 0.0

        self._progress.update(pose)
        gx, gy = self._progress.lookahead_point(pose, self._lookahead_distance)
        self._last_lookahead = (gx, gy)

        dx, dy = gx - pose.x, gy - pose.y
        local_x = math.cos(-pose.yaw) * dx - math.sin(-pose.yaw) * dy
        local_y = math.sin(-pose.yaw) * dx + math.cos(-pose.yaw) * dy

        lookahead_sq = local_x * local_x + local_y * local_y
        if lookahead_sq < 1e-6:
            return 0.0
        return 2.0 * local_y / lookahead_sq
