import math

from molly_navigation.utils.avoidance.base_avoidance import AvoidanceAlgorithm, AvoidanceResult


def angle_diff(a: float, b: float) -> float:
    """Return the signed difference a-b wrapped into (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def tent_weight(theta: float, center: float, half_width: float) -> float:
    """Return a 1-at-center, 0-at-edge triangular weight for theta within a zone."""
    if half_width <= 0.0:
        return 0.0
    d = abs(angle_diff(theta, center))
    return max(0.0, 1.0 - d / half_width)


class BraitenbergAvoidance(AvoidanceAlgorithm):
    """Discrete-sector Braitenberg-style reactive avoidance: obstacles bias steering and speed."""

    def __init__(
        self,
        safe_distance: float = 3.0,
        emergency_distance: float = 0.4,
        steering_gain: float = 1.5,
        velocity_gain: float = 1.0,
        steering_zone_center_deg: float = 45.0,
        steering_zone_width_deg: float = 60.0,
        velocity_zone_center_deg: float = 60.0,
        velocity_zone_width_deg: float = 90.0,
        following_zone_width_deg: float = 30.0,
        emergency_cone_deg: float = 60.0,
        smoothing: float = 0.85,
    ):
        self._safe_distance = safe_distance
        self._emergency_distance = emergency_distance
        self._steering_gain = steering_gain
        self._velocity_gain = velocity_gain
        self._steering_zone_center = math.radians(steering_zone_center_deg)
        self._steering_zone_half = math.radians(steering_zone_width_deg) / 2.0
        self._velocity_zone_center = math.radians(velocity_zone_center_deg)
        self._velocity_zone_half = math.radians(velocity_zone_width_deg) / 2.0
        self._following_half_width = math.radians(following_zone_width_deg) / 2.0
        self._emergency_half_cone = math.radians(emergency_cone_deg) / 2.0
        self._smoothing = smoothing
        self._filtered = AvoidanceResult()

    def update(
        self, ranges: list[float], angle_min: float, angle_increment: float, range_max: float
    ) -> AvoidanceResult:
        """Compute a new avoidance result from one sector scan and blend it with the last one."""
        if len(ranges) == 0:
            return self._filtered

        num_l = den_l = num_r = den_r = 0.0
        num_vl = den_vl = num_vr = den_vr = 0.0
        num_f = den_f = 0.0
        emergency = False

        for i, r in enumerate(ranges):
            if r is None or math.isnan(r) or math.isinf(r):
                r = range_max
            theta = angle_min + (i + 0.5) * angle_increment
            danger = max(0.0, 1.0 - min(r, self._safe_distance) / self._safe_distance)

            if (
                r < self._emergency_distance
                and abs(angle_diff(theta, 0.0)) < self._emergency_half_cone
            ):
                emergency = True

            w_l = tent_weight(theta, self._steering_zone_center, self._steering_zone_half)
            w_r = tent_weight(theta, -self._steering_zone_center, self._steering_zone_half)
            w_vl = tent_weight(theta, self._velocity_zone_center, self._velocity_zone_half)
            w_vr = tent_weight(theta, -self._velocity_zone_center, self._velocity_zone_half)
            w_f = tent_weight(theta, 0.0, self._following_half_width)

            num_l += w_l * danger
            den_l += w_l
            num_r += w_r * danger
            den_r += w_r
            num_vl += w_vl * danger
            den_vl += w_vl
            num_vr += w_vr * danger
            den_vr += w_vr
            num_f += w_f * danger
            den_f += w_f

        danger_left = num_l / den_l if den_l > 1e-6 else 0.0
        danger_right = num_r / den_r if den_r > 1e-6 else 0.0
        danger_vel_left = num_vl / den_vl if den_vl > 1e-6 else 0.0
        danger_vel_right = num_vr / den_vr if den_vr > 1e-6 else 0.0
        danger_following = num_f / den_f if den_f > 1e-6 else 0.0

        steering_bias = self._steering_gain * (danger_right - danger_left)
        lateral_danger = max(danger_vel_left, danger_vel_right)
        velocity_scale = max(
            0.0, (1.0 - self._velocity_gain * lateral_danger) * (1.0 - danger_following)
        )

        raw = AvoidanceResult(steering_bias, velocity_scale, emergency)
        a = self._smoothing
        self._filtered = AvoidanceResult(
            steering_bias=a * self._filtered.steering_bias + (1 - a) * raw.steering_bias,
            velocity_scale=a * self._filtered.velocity_scale + (1 - a) * raw.velocity_scale,
            emergency=raw.emergency,
        )
        return self._filtered
