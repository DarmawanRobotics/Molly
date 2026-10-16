import math

from molly_navigation.utils.avoidance.base_avoidance import (
    AvoidanceAlgorithm,
    AvoidanceResult,
    clean_ranges,
    sector_angle,
    smooth_result,
)


class PotentialFieldAvoidance(AvoidanceAlgorithm):
    """
    Artificial potential field: every obstacle inside safe_distance pushes the robot away.

    The repulsion yields a steering bias, a sideways push for holonomic robots and a speed scale.
    """

    def __init__(
        self,
        safe_distance: float = 1.5,
        emergency_distance: float = 0.75,
        emergency_cone_deg: float = 60.0,
        repulsion_gain: float = 0.5,
        steering_gain: float = 0.8,
        lateral_gain: float = 0.3,
        max_lateral: float = 0.2,
        velocity_gain: float = 1.0,
        smoothing: float = 0.8,
    ):
        self._safe_distance = safe_distance
        self._emergency_distance = emergency_distance
        self._emergency_half_cone = math.radians(emergency_cone_deg) / 2.0
        self._repulsion_gain = repulsion_gain
        self._steering_gain = steering_gain
        self._lateral_gain = lateral_gain
        self._max_lateral = max_lateral
        self._velocity_gain = velocity_gain
        self._smoothing = smoothing
        self._filtered = AvoidanceResult()

    def update(self, ranges, angle_min, angle_increment, range_max) -> AvoidanceResult:
        """Sum the repulsive forces of all sectors and map them to the avoidance outputs."""
        if not ranges:
            return self._filtered
        fx = fy = 0.0
        emergency = False
        frontal_danger = 0.0
        for i, r in enumerate(clean_ranges(ranges, range_max)):
            theta = sector_angle(i, angle_min, angle_increment)
            in_cone = abs(theta) < self._emergency_half_cone
            if in_cone and r < self._emergency_distance:
                emergency = True
            if r >= self._safe_distance or r <= 1e-3:
                continue
            magnitude = self._repulsion_gain * (1.0 / r - 1.0 / self._safe_distance) / (r * r)
            fx -= magnitude * math.cos(theta)
            fy -= magnitude * math.sin(theta)
            if in_cone:
                frontal_danger = max(frontal_danger, 1.0 - r / self._safe_distance)

        steering = self._steering_gain * math.atan(fy)
        lateral = max(-self._max_lateral, min(self._max_lateral, self._lateral_gain * fy))
        velocity_scale = max(0.0, 1.0 - self._velocity_gain * frontal_danger)
        raw = AvoidanceResult(steering, velocity_scale, emergency, lateral)
        self._filtered = smooth_result(self._filtered, raw, self._smoothing)
        return self._filtered
