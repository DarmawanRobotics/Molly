class SafetySupervisor:
    """
    Debounces emergency stops and detects blocked and off-path conditions.

    All times are seconds from a monotonic clock supplied by the caller.
    """

    def __init__(
        self,
        estop_release_sec: float = 1.0,
        blocked_timeout_sec: float = 30.0,
        off_path_slow_distance: float = 0.6,
        off_path_stop_distance: float = 1.5,
    ):
        self._estop_release = estop_release_sec
        self._blocked_timeout = blocked_timeout_sec
        self._off_path_slow = off_path_slow_distance
        self._off_path_stop = off_path_stop_distance
        self.reset()

    def reset(self):
        """Clear all latches, e.g. when navigation is (re)started."""
        self._estop_latched = False
        self._estop_clear_since = None
        self._stalled_since = None
        self._off_path_latched = False

    def estop(self, emergency: bool, now: float) -> bool:
        """Latch on any emergency and release only after it stayed clear for estop_release_sec."""
        if emergency:
            self._estop_latched = True
            self._estop_clear_since = None
        elif self._estop_latched:
            if self._estop_clear_since is None:
                self._estop_clear_since = now
            elif now - self._estop_clear_since >= self._estop_release:
                self._estop_latched = False
        return self._estop_latched

    def blocked(self, stalled: bool, now: float) -> bool:
        """Return True once the robot could not make progress for blocked_timeout_sec."""
        if not stalled:
            self._stalled_since = None
            return False
        if self._stalled_since is None:
            self._stalled_since = now
        return now - self._stalled_since >= self._blocked_timeout

    def off_path(self, cross_track: float) -> tuple[bool, bool]:
        """(slow, stop) for the current cross-track error; stop latches until reset()."""
        if cross_track > self._off_path_stop:
            self._off_path_latched = True
        return cross_track > self._off_path_slow, self._off_path_latched
