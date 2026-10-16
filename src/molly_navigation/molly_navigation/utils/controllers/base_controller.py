from dataclasses import dataclass

from molly_navigation.utils.pose2d import Pose2D


@dataclass
class ControllerOutput:
    """Nominal (pre-avoidance) body velocity command from a path controller."""

    linear: float = 0.0   # m/s forward
    angular: float = 0.0  # rad/s, +left
    lateral: float = 0.0  # m/s, +left; only holonomic controllers set it


class PathController:
    """Common interface every path-following controller implements."""

    def set_path(self, path_xy: list[tuple[float, float]], start_index=None):
        """Replace the tracked path; start_index pins progress instead of a global search."""
        raise NotImplementedError

    def update(self, pose: Pose2D, dt: float, target_speed: float) -> ControllerOutput:
        """Compute the nominal command for the current pose at the requested forward speed."""
        raise NotImplementedError

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once progress reached the end of the path."""
        raise NotImplementedError

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        raise NotImplementedError

    def debug_info(self) -> dict:
        """Return controller-specific debug values for visualization (all keys optional)."""
        return {}
