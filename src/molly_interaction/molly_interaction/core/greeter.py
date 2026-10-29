"""When to greet a visitor (no ROS, unit-tested)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math

from dharma_utils.io import read_json, write_json_atomic

# navigation/status states in which the robot walks on its own: no greeting, no gesture
# (NavigationStatus JOINING, FOLLOWING, APPROACHING, BLOCKED, OFF_PATH)
MOVING_STATES = (1, 2, 3, 6, 7)
# tour/state states that mean a tour is going on (TourState STARTING, RUNNING, PAUSED)
TOUR_BUSY = (1, 2, 3)


class GreeterState(IntEnum):
    """GreeterStatus.state."""

    OFF = 0
    WAITING = 1
    WATCHING = 2
    GREETING = 3
    GREETED = 4
    COOLDOWN = 5
    PAUSED = 6
    UNAVAILABLE = 7


@dataclass(frozen=True)
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
        self.cfg = cfg
        self.enabled = True
        self.state = GreeterState.WAITING
        self._since: float | None = None
        self._last_greet = -math.inf
        self._cleared = True

    def update(self, now: float, nearest_front: float | None, busy: bool) -> bool:
        """Feed the nearest visitor in front (m; None or < 0: nobody); True means greet now."""
        present = nearest_front is not None and 0.0 <= nearest_front <= self.cfg.approach_distance
        if not present:
            self._since = None
            self._cleared = True
        if busy:
            self._since = None
        self.state = self._state(now, present, busy)
        if self.state is not GreeterState.GREETING:
            return False
        self.greeted(now)
        return True

    def _state(self, now: float, present: bool, busy: bool) -> GreeterState:
        cooling = now - self._last_greet < self.cfg.cooldown_sec
        if not self.enabled or busy:
            return GreeterState.PAUSED if self.enabled else GreeterState.OFF
        if not present:
            return GreeterState.COOLDOWN if cooling else GreeterState.WAITING
        if self._since is None:
            self._since = now
        if not self._cleared:
            return GreeterState.GREETED
        if cooling:
            return GreeterState.COOLDOWN
        if now - self._since < self.cfg.dwell_sec:
            return GreeterState.WATCHING
        return GreeterState.GREETING

    def greeted(self, now: float):
        """Note a greeting (automatic or asked for): the same visitor is not greeted again."""
        self._last_greet = now
        self._cleared = False
        self._since = None
        self.state = GreeterState.GREETING

    def last_greet_age(self, now: float) -> float | None:
        """Return seconds since the last greeting, or None before the first one."""
        return None if self._last_greet == -math.inf else round(now - self._last_greet, 1)


def next_text(texts: list[str], index: int) -> tuple[str, int]:
    """Return (text, next index), going round the list; ('', 0) for an empty list."""
    if not texts:
        return '', 0
    return texts[index % len(texts)], (index + 1) % len(texts)


def load_choice(path: str) -> tuple[bool | None, str]:
    """Return (the greeter on / off saved in path or None, a problem or '')."""
    if not path:
        return None, ''
    data, error = read_json(path)
    if error:
        return None, error
    if isinstance(data, dict) and isinstance(data.get('enabled'), bool):
        return data['enabled'], ''
    return None, '' if data is None else f'{path}: no "enabled" true / false'


def save_choice(path: str, enabled: bool):
    """Keep the greeter on / off for the next start (raises OSError)."""
    if path:
        write_json_atomic(path, {'enabled': bool(enabled)}, backup=False)
