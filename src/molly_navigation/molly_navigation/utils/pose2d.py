from dataclasses import dataclass


@dataclass
class Pose2D:
    """A 2D pose: x, y position and yaw heading."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
