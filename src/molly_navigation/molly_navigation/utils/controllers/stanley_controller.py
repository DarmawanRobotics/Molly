import math

from molly_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.pose2d import Pose2D


class StanleyController(PathController):
    """Stanley steering law adapted to a unicycle, with curvature feedforward."""

    def __init__(
        self,
        goal_tolerance=0.3,
        k_cross_track=1.0,
        k_soft=0.2,
        k_heading=1.5,
        max_angular_velocity=1.0,
    ):
        self._goal_tolerance = goal_tolerance
        self._k_cross_track = k_cross_track
        self._k_soft = k_soft
        self._k_heading = k_heading
        self._max_angular_velocity = max_angular_velocity
        self._progress = PathProgress()

    def set_path(self, path_xy, start_index=None):
        """Replace the tracked path."""
        self._progress.set_path(path_xy, start_index)

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once progress reached the end and the robot is within goal_tolerance."""
        return self._progress.is_finished(pose, self._goal_tolerance)

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        return self._progress.index

    def update(self, pose: Pose2D, dt: float, target_speed: float) -> ControllerOutput:
        """Steer by -(heading error + atan(k * e / (v + k_soft))) on top of v * curvature."""
        if not self._progress.path:
            return ControllerOutput(0.0, 0.0)
        self._progress.update(pose)
        ref = self._progress.reference(pose)
        steer = -(
            ref.heading_error
            + math.atan2(self._k_cross_track * ref.lateral_error, target_speed + self._k_soft)
        )
        angular = target_speed * ref.curvature + self._k_heading * steer
        angular = max(-self._max_angular_velocity, min(self._max_angular_velocity, angular))
        return ControllerOutput(linear=target_speed, angular=angular)

    def debug_info(self) -> dict:
        """Expose the nearest path point for visualization."""
        path = self._progress.path
        return {'nearest_xy': path[self._progress.index]} if path else {}
