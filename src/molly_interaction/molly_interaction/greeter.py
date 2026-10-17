"""When to greet a visitor (no ROS, unit-tested)."""

from dataclasses import dataclass
import math

# navigation/status states in which the robot walks on its own: no greeting, no gesture
MOVING_STATES = ('JOINING', 'FOLLOWING', 'APPROACHING', 'BLOCKED', 'OFF_PATH')
# /tour/state states that mean a tour is going on
TOUR_BUSY = (1, 2, 3)  # STARTING, RUNNING, PAUSED


@dataclass
class GreeterConfig:
    """Who counts as a visitor to greet, and how often."""

    approach_distance: float = 1.5  # m, somebody this close in front
    dwell_sec: float = 1.5  # ... for this long
    cooldown_sec: float = 30.0  # at most one greeting per this many seconds


class Greeter:
    """
    Decides when to greet: somebody stays close in front for dwell_sec.

    After a greeting the front must empty once (the same visitor is not greeted twice) and
    cooldown_sec must pass.
    """

    def __init__(self, cfg: GreeterConfig):
        """Start enabled, waiting for a visitor."""
        self.cfg = cfg
        self.enabled = True
        self.state = 'waiting'
        self._since = None
        self._last_greet = -math.inf
        self._cleared = True

    def update(self, now: float, nearest_front, busy: bool) -> bool:
        """Feed the nearest visitor in front (m; None or < 0: nobody); True means greet now."""
        present = nearest_front is not None and 0.0 <= nearest_front <= self.cfg.approach_distance
        if not present:
            self._since = None
            self._cleared = True
        if not self.enabled:
            self.state = 'off'
            return False
        if busy:
            self._since = None
            self.state = 'paused'
            return False
        cooling = now - self._last_greet < self.cfg.cooldown_sec
        if not present:
            self.state = 'cooldown' if cooling else 'waiting'
            return False
        if self._since is None:
            self._since = now
        if not self._cleared:
            self.state = 'greeted'
            return False
        if cooling:
            self.state = 'cooldown'
            return False
        if now - self._since < self.cfg.dwell_sec:
            self.state = 'watching'
            return False
        self.greeted(now)
        return True

    def greeted(self, now: float):
        """Note a greeting (automatic or asked for): the same visitor is not greeted again."""
        self._last_greet = now
        self._cleared = False
        self._since = None
        self.state = 'greeting'

    def last_greet_age(self, now: float):
        """Return seconds since the last greeting, or None before the first one."""
        return None if self._last_greet == -math.inf else round(now - self._last_greet, 1)


def next_text(texts: list, index: int):
    """Return (text, next index), going round the list; ('', 0) for an empty list."""
    if not texts:
        return '', 0
    return texts[index % len(texts)], (index + 1) % len(texts)
