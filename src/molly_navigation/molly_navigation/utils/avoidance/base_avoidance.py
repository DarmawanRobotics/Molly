from dataclasses import dataclass
import math


@dataclass
class AvoidanceResult:
    """One avoidance update: steering bias, speed scale, emergency flag and sideways push."""

    steering_bias: float = 0.0
    velocity_scale: float = 1.0
    emergency: bool = False
    lateral_bias: float = 0.0


def clean_ranges(ranges: list[float], range_max: float) -> list[float]:
    """Replace NaN/inf/None sector ranges with range_max."""
    return [range_max if r is None or math.isnan(r) or math.isinf(r) else r for r in ranges]


def sector_angle(index: int, angle_min: float, angle_increment: float) -> float:
    """Centre angle of a sector."""
    return angle_min + (index + 0.5) * angle_increment


def smooth_result(
    previous: AvoidanceResult, raw: AvoidanceResult, alpha: float
) -> AvoidanceResult:
    """Exponential smoothing of the continuous outputs; the emergency flag is never delayed."""
    return AvoidanceResult(
        steering_bias=alpha * previous.steering_bias + (1 - alpha) * raw.steering_bias,
        velocity_scale=alpha * previous.velocity_scale + (1 - alpha) * raw.velocity_scale,
        emergency=raw.emergency,
        lateral_bias=alpha * previous.lateral_bias + (1 - alpha) * raw.lateral_bias,
    )


class AvoidanceAlgorithm:
    """Common interface every reactive obstacle-avoidance algorithm implements."""

    def update(
        self, ranges: list[float], angle_min: float, angle_increment: float, range_max: float
    ) -> AvoidanceResult:
        """Compute a new avoidance result from one sector scan."""
        raise NotImplementedError
