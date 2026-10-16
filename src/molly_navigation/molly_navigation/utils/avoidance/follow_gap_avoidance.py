import math

from molly_navigation.utils.avoidance.base_avoidance import (
    AvoidanceAlgorithm,
    AvoidanceResult,
    clean_ranges,
    sector_angle,
    smooth_result,
)


class FollowGapAvoidance(AvoidanceAlgorithm):
    """
    Follow-the-gap avoidance.

    Inflates the closest obstacle by a safety bubble and steers to the centre of the widest
    remaining gap, only while the frontal cone is blocked.
    """

    def __init__(
        self,
        safe_distance: float = 1.5,
        emergency_distance: float = 0.75,
        emergency_cone_deg: float = 60.0,
        bubble_radius: float = 0.45,
        steering_gain: float = 1.0,
        velocity_gain: float = 1.0,
        smoothing: float = 0.85,
    ):
        self._safe_distance = safe_distance
        self._emergency_distance = emergency_distance
        self._emergency_half_cone = math.radians(emergency_cone_deg) / 2.0
        self._bubble_radius = bubble_radius
        self._steering_gain = steering_gain
        self._velocity_gain = velocity_gain
        self._smoothing = smoothing
        self._filtered = AvoidanceResult()

    def update(self, ranges, angle_min, angle_increment, range_max) -> AvoidanceResult:
        """Bubble the nearest obstacle, find the widest free gap and steer to its centre."""
        if not ranges:
            return self._filtered
        r = clean_ranges(ranges, range_max)
        n = len(r)
        cone = [
            i
            for i in range(n)
            if abs(sector_angle(i, angle_min, angle_increment)) < self._emergency_half_cone
        ]
        emergency = any(r[i] < self._emergency_distance for i in cone)
        frontal_min = min((r[i] for i in cone), default=range_max)
        velocity_scale = max(
            0.0, 1.0 - self._velocity_gain * max(0.0, 1.0 - frontal_min / self._safe_distance)
        )

        steering = 0.0
        if frontal_min < self._safe_distance:
            nearest = min(range(n), key=lambda i: r[i])
            half = math.atan2(self._bubble_radius, max(r[nearest], 1e-3))
            free = [
                r[i] >= self._safe_distance
                and abs(
                    sector_angle(i, angle_min, angle_increment)
                    - sector_angle(nearest, angle_min, angle_increment)
                )
                > half
                for i in range(n)
            ]
            gap = self.widest_gap(free)
            if gap is None:
                velocity_scale = 0.0
            else:
                centre = sector_angle((gap[0] + gap[1]) / 2.0, angle_min, angle_increment)
                steering = self._steering_gain * centre

        raw = AvoidanceResult(steering, velocity_scale, emergency, 0.0)
        self._filtered = smooth_result(self._filtered, raw, self._smoothing)
        return self._filtered

    @staticmethod
    def widest_gap(free: list[bool]):
        """(start, end) of the longest run of free sectors, or None."""
        best, start = None, None
        for i, is_free in enumerate(free + [False]):
            if is_free and start is None:
                start = i
            elif not is_free and start is not None:
                if best is None or (i - 1 - start) > (best[1] - best[0]):
                    best = (start, i - 1)
                start = None
        return best
