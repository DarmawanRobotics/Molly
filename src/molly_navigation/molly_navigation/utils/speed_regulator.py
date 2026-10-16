class SpeedRegulator:
    """Scale the linear velocity down on sharp turns, independent of the steering controller."""

    def __init__(self, kp=0.3, ki=0.0, kd=0.0, min_scale=0.3):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._min_scale = min_scale
        self._integral = 0.0
        self._last_error = 0.0

    def reset(self):
        """Clear the integral and derivative state, e.g. when a new path is loaded."""
        self._integral = 0.0
        self._last_error = 0.0

    def scale_for(self, angular: float, dt: float) -> float:
        """Return a 0..1 velocity multiplier that shrinks as |angular| grows."""
        error = abs(angular)
        safe_dt = max(dt, 1e-3)
        self._integral += error * safe_dt
        derivative = (error - self._last_error) / safe_dt
        self._last_error = error

        correction = self._kp * error + self._ki * self._integral + self._kd * derivative
        return max(self._min_scale, min(1.0, 1.0 - correction))
