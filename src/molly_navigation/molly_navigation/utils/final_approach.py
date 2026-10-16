from dataclasses import dataclass
import math

from molly_navigation.utils.geometry import angle_diff, clamp, to_local
from molly_navigation.utils.pose2d import Pose2D


@dataclass
class ApproachCommand:
    """Velocity command and convergence state of one final-approach step."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    position_error: float = 0.0
    yaw_error: float = 0.0
    done: bool = False


class FinalApproach:
    """
    Brings the robot exactly onto a target x, y, yaw.

    Holonomic robots (legged, omni) close x/y/yaw simultaneously in the body frame; differential
    robots turn toward the point, drive to it, then rotate to the target yaw.
    """

    def __init__(
        self,
        holonomic: bool = True,
        kp_xy: float = 1.2,
        kp_yaw: float = 1.5,
        max_speed: float = 0.2,
        max_yaw_rate: float = 0.6,
        min_speed: float = 0.04,
        position_tolerance: float = 0.05,
        yaw_tolerance: float = math.radians(3.0),
    ):
        self._holonomic = holonomic
        self._kp_xy = kp_xy
        self._kp_yaw = kp_yaw
        self._max_speed = max_speed
        self._max_yaw_rate = max_yaw_rate
        self._min_speed = min_speed
        self._pos_tol = position_tolerance
        self._yaw_tol = yaw_tolerance
        self._position_reached = False

    def reset(self):
        """Forget convergence state before a new target."""
        self._position_reached = False

    def update(self, pose: Pose2D, x: float, y: float, yaw: float) -> ApproachCommand:
        """One approach step toward (x, y, yaw)."""
        ex, ey = to_local(x - pose.x, y - pose.y, pose.yaw)
        dist = math.hypot(ex, ey)
        eyaw = angle_diff(yaw, pose.yaw)
        if dist < self._pos_tol:
            self._position_reached = True
        elif dist > 2.0 * self._pos_tol:
            self._position_reached = False

        cmd = ApproachCommand(position_error=dist, yaw_error=eyaw)
        if self._position_reached and abs(eyaw) < self._yaw_tol:
            cmd.done = True
            return cmd

        if self._holonomic:
            self.holonomic_step(cmd, ex, ey, dist, eyaw)
        else:
            self.differential_step(cmd, ex, ey, dist, eyaw)
        return cmd

    def holonomic_step(self, cmd: ApproachCommand, ex: float, ey: float, dist: float, eyaw: float):
        """Close x, y and yaw together in the body frame."""
        if not self._position_reached:
            speed = min(self._max_speed, max(self._min_speed, self._kp_xy * dist))
            cmd.vx, cmd.vy = speed * ex / dist, speed * ey / dist
        cmd.wz = self.yaw_rate(eyaw)

    def differential_step(
        self, cmd: ApproachCommand, ex: float, ey: float, dist: float, eyaw: float
    ):
        """Turn toward the point, drive onto it, then rotate to the target yaw."""
        if self._position_reached:
            cmd.wz = self.yaw_rate(eyaw)
            return
        bearing = math.atan2(ey, ex)
        if abs(bearing) > math.pi / 2.0:
            bearing = angle_diff(bearing, math.pi)
            direction = -1.0
        else:
            direction = 1.0
        if abs(bearing) > math.radians(20.0):
            cmd.wz = self.yaw_rate(bearing)
            return
        speed = min(self._max_speed, max(self._min_speed, self._kp_xy * dist))
        cmd.vx = direction * speed * math.cos(bearing)
        cmd.wz = self.yaw_rate(bearing)

    def yaw_rate(self, error: float) -> float:
        """Proportional yaw rate with a small floor so the last degrees still converge."""
        if abs(error) < self._yaw_tol:
            return 0.0
        rate = self._kp_yaw * error
        floor = 0.1 if abs(rate) < 0.1 else 0.0
        return clamp(math.copysign(max(abs(rate), floor), error), self._max_yaw_rate)
