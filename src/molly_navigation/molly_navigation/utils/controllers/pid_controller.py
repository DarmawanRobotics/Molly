import math

from molly_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from molly_navigation.utils.geometry import angle_diff
from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.pose2d import Pose2D


class PidController(PathController):
    """Heading-error PID path follower that steers toward a lookahead point."""

    def __init__(self, lookahead_distance=1.0, goal_tolerance=0.3, kp=1.5, ki=0.0, kd=0.2):
        self._lookahead_distance = lookahead_distance
        self._goal_tolerance = goal_tolerance
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._progress = PathProgress()
        self._integral = 0.0
        self._last_error = 0.0
        self._last_lookahead = (0.0, 0.0)

    def set_path(self, path_xy, start_index=None):
        """Replace the tracked path and reset the integral/derivative state."""
        self._progress.set_path(path_xy, start_index)
        self._integral = 0.0
        self._last_error = 0.0

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once progress reached the end and the robot is within goal_tolerance."""
        return self._progress.is_finished(pose, self._goal_tolerance)

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        return self._progress.index

    def update(self, pose: Pose2D, dt: float, target_speed: float) -> ControllerOutput:
        """Run one PID step on the heading error toward the lookahead point."""
        if not self._progress.path:
            return ControllerOutput(0.0, 0.0)

        self._progress.update(pose)
        gx, gy = self._progress.lookahead_point(pose, self._lookahead_distance)
        self._last_lookahead = (gx, gy)

        error = angle_diff(math.atan2(gy - pose.y, gx - pose.x), pose.yaw)
        safe_dt = max(dt, 1e-3)
        self._integral += error * safe_dt
        derivative = (error - self._last_error) / safe_dt
        self._last_error = error

        angular = self._kp * error + self._ki * self._integral + self._kd * derivative
        return ControllerOutput(linear=target_speed, angular=angular)

    def debug_info(self) -> dict:
        """Expose the lookahead point for visualization."""
        return {
            'lookahead_xy': self._last_lookahead,
            'lookahead_distance': self._lookahead_distance,
        }
