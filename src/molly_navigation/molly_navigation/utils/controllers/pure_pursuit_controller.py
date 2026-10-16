from molly_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from molly_navigation.utils.pose2d import Pose2D
from molly_navigation.utils.pure_pursuit import PurePursuit


class PurePursuitController(PathController):
    """Pure pursuit adapter: curvature-based steering at a constant cruise speed."""

    def __init__(self, lookahead_distance=1.0, goal_tolerance=0.3):
        self._pure_pursuit = PurePursuit(
            lookahead_distance=lookahead_distance, goal_tolerance=goal_tolerance
        )
        self._last_curvature = 0.0

    def set_path(self, path_xy, start_index=None):
        """Replace the tracked path, optionally pinning progress to start_index."""
        self._pure_pursuit.set_path(path_xy, start_index)

    def update(self, pose: Pose2D, dt: float, target_speed: float) -> ControllerOutput:
        """Compute the pure pursuit curvature and scale it by the requested speed."""
        curvature = self._pure_pursuit.update(pose)
        self._last_curvature = curvature
        return ControllerOutput(linear=target_speed, angular=target_speed * curvature)

    def is_finished(self, pose: Pose2D) -> bool:
        """Delegate to the underlying pure pursuit tracker."""
        return self._pure_pursuit.is_finished(pose)

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        return self._pure_pursuit.progress_index()

    def debug_info(self) -> dict:
        """Expose the lookahead point, nearest point, and curvature for visualization."""
        return {
            'lookahead_xy': self._pure_pursuit.last_lookahead_point(),
            'nearest_xy': self._pure_pursuit.nearest_point(),
            'lookahead_distance': self._pure_pursuit.lookahead_distance(),
            'curvature': self._last_curvature,
        }
