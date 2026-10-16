class SlewLimiter:
    """Caps how fast a value can change per second."""

    def __init__(self, max_rate: float):
        self._max_rate = max_rate
        self._value = 0.0

    @property
    def value(self) -> float:
        """Return the current output value."""
        return self._value

    def step(self, target: float, dt: float) -> float:
        """Move toward target by at most max_rate * dt and return the new value."""
        max_delta = self._max_rate * dt
        self._value += max(-max_delta, min(max_delta, target - self._value))
        return self._value

    def reset(self, value: float = 0.0):
        """Jump straight to value, bypassing the rate limit."""
        self._value = value


class LowPassFilter:
    """First-order low-pass: alpha = 1 passes the input through, smaller values smooth more."""

    def __init__(self, alpha: float):
        self._alpha = max(0.0, min(1.0, alpha))
        self._value = 0.0

    @property
    def value(self) -> float:
        """Return the current filtered value."""
        return self._value

    def step(self, target: float) -> float:
        """Blend target into the filtered value and return it."""
        self._value += self._alpha * (target - self._value)
        return self._value

    def reset(self, value: float = 0.0):
        """Set the filter state directly."""
        self._value = value
