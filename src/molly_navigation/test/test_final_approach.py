import math

from molly_navigation.utils.final_approach import FinalApproach
from molly_navigation.utils.pose2d import Pose2D
import pytest


@pytest.mark.parametrize('holonomic', [True, False])
def test_final_approach_converges(holonomic):
    """The robot ends within 5 cm and 3 deg of the target from an offset start."""
    approach = FinalApproach(holonomic=holonomic)
    pose, dt, cmd = Pose2D(-0.4, 0.15, math.radians(20)), 0.05, None
    for _ in range(600):
        cmd = approach.update(pose, 0.0, 0.0, math.radians(90))
        if cmd.done:
            break
        pose.yaw += cmd.wz * dt
        pose.x += (cmd.vx * math.cos(pose.yaw) - cmd.vy * math.sin(pose.yaw)) * dt
        pose.y += (cmd.vx * math.sin(pose.yaw) + cmd.vy * math.cos(pose.yaw)) * dt
    assert cmd.done
    assert math.hypot(pose.x, pose.y) < 0.05
    assert abs(math.degrees(cmd.yaw_error)) < 3.0
