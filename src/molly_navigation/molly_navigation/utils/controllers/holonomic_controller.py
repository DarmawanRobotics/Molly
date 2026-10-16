import math

from molly_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from molly_navigation.utils.geometry import clamp, to_local, wrap_angle
from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.pose2d import Pose2D


class HolonomicController(PathController):
    """
    Path following with x, y and yaw for legged and omnidirectional robots.

    Translation and heading are controlled separately:

    - The robot moves toward a carrot `lookahead_distance` further along the path (measured from
      its projection on the path), in any direction of its body. A cross-track error therefore
      closes sideways along a smooth first-order curve without changing the heading. When the
      path is behind the robot it backs up, and far from the path it heads straight back to it.
    - The heading turns toward the path direction a short `heading_lookahead` ahead, with a
      proportional, rate-limited yaw command. It does not depend on the cross-track error, so
      it cannot oscillate around the path the way steering-only trackers do.

    Sideways and backward speeds are capped (`max_lateral_velocity`, `max_reverse_velocity`)
    while keeping the direction of travel, and translation slows while the heading is far off,
    so a legged robot mostly walks forward and only creeps sideways or backwards.

    `set_offset()` makes the robot follow a parallel line `offset` metres to the left (+) or
    right (-) of the path, used to slide around obstacles without leaving the route heading.

    With `max_lateral_velocity` 0 (a robot that cannot strafe) it degrades to turn-toward-the-
    carrot and drive forward.
    """

    def __init__(
        self,
        lookahead_distance: float = 0.6,
        heading_lookahead: float = 0.6,
        goal_tolerance: float = 0.3,
        kp_yaw: float = 1.5,
        max_angular_velocity: float = 1.0,
        max_lateral_velocity: float = 0.25,
        max_reverse_velocity: float = 0.15,
        min_alignment_scale: float = 0.25,
    ):
        self._lookahead = max(0.1, lookahead_distance)
        self._heading_lookahead = max(0.1, heading_lookahead)
        self._goal_tolerance = goal_tolerance
        self._kp_yaw = kp_yaw
        self._max_wz = max_angular_velocity
        self._max_vy = max(0.0, max_lateral_velocity)
        self._max_reverse = max(0.0, max_reverse_velocity)
        self._min_alignment = max(0.0, min(1.0, min_alignment_scale))
        self._progress = PathProgress()
        self._offset = 0.0
        self._debug = {}

    def set_path(self, path_xy, start_index=None):
        """Replace the tracked path, optionally pinning progress to start_index."""
        self._progress.set_path(path_xy, start_index)
        self._debug = {}

    def set_offset(self, offset: float):
        """Follow a line parallel to the path, offset metres to the left (+) or right (-)."""
        self._offset = offset

    def offset(self) -> float:
        """Return the current lateral offset from the path."""
        return self._offset

    def path_progress(self) -> PathProgress:
        """Return the path tracker (route, arc lengths, projection) for path-shift avoidance."""
        return self._progress

    def shifted_point(self, s: float) -> tuple[float, float]:
        """Point at arc length s, moved sideways by the offset (left normal of the path)."""
        x, y = self._progress.point_at(s)
        if self._offset == 0.0:
            return x, y
        heading = self._progress.heading_between(s - 0.1, s + 0.1)
        return x - math.sin(heading) * self._offset, y + math.cos(heading) * self._offset

    def update(self, pose: Pose2D, dt: float, target_speed: float) -> ControllerOutput:
        """Command (vx, vy, wz) toward the carrot, with the heading along the path."""
        progress = self._progress
        if len(progress.path) < 2:
            return ControllerOutput()
        progress.update(pose)
        s = progress.projection(pose)
        cx, cy = self.shifted_point(s + self._lookahead)
        dx, dy = cx - pose.x, cy - pose.y
        distance = math.hypot(dx, dy)

        path_heading = progress.heading_between(s, s + self._heading_lookahead)
        if self._max_vy <= 0.0 and distance > 1e-3:
            path_heading = math.atan2(dy, dx)  # cannot strafe: face the carrot instead
        heading_error = wrap_angle(path_heading - pose.yaw)
        wz = clamp(self._kp_yaw * heading_error, self._max_wz)

        vx = vy = 0.0
        if distance > 1e-3 and target_speed > 0.0:
            # never faster than needed to reach the carrot within about a second
            speed = min(target_speed, distance)
            vx, vy = to_local(speed * dx / distance, speed * dy / distance, pose.yaw)
            vx, vy = self.limit(vx, vy, heading_error)

        self._debug = {
            'lookahead_xy': (cx, cy),
            'nearest_xy': self.shifted_point(s),
            'lookahead_distance': self._lookahead,
            'heading_error': heading_error,
        }
        return ControllerOutput(linear=vx, angular=wz, lateral=vy)

    def limit(self, vx: float, vy: float, heading_error: float) -> tuple[float, float]:
        """Slow down while misaligned, then cap reverse and sideways speed (keeping direction)."""
        alignment = math.cos(heading_error) if abs(heading_error) < math.pi / 2 else 0.0
        scale = max(self._min_alignment, alignment)
        if self._max_vy <= 0.0:  # cannot strafe: forward only, while turning toward the carrot
            return max(0.0, vx) * scale, 0.0
        if vx < 0.0:
            scale = min(scale, self._max_reverse / -vx) if self._max_reverse > 0.0 else 0.0
        if abs(vy) > 1e-9:
            scale = min(scale, self._max_vy / abs(vy))
        return vx * scale, vy * scale

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once progress reached the end of the path and the robot is near it."""
        return self._progress.is_finished(pose, self._goal_tolerance)

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        return self._progress.index

    def debug_info(self) -> dict:
        """Carrot, nearest point and lookahead for visualization."""
        return dict(self._debug)
