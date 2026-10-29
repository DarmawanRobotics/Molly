"""Is anybody following the guide? Hold the walk when the visitors fall behind."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class WatchEvent(Enum):
    """What the watch asks the engine to do."""

    BEGIN_WAIT = 'begin_wait'  # nobody behind: hold the walk, ask them to follow
    VISITORS_BACK = 'visitors_back'  # somebody behind again: walk on
    WAIT_OVER = 'wait_over'  # waited long enough (or no data): walk on, no more waits this leg


@dataclass(frozen=True)
class WatchConfig:
    """Timings of the follower check (s)."""

    enabled: bool = True
    lost_sec: float = 6.0
    found_sec: float = 1.0
    wait_sec: float = 20.0
    timeout_sec: float = 2.0


class FollowerWatch:
    """Counts the visitors behind the robot and decides when the walk waits for them."""

    def __init__(self, config: WatchConfig):
        self.config = config
        self.behind, self.seen = 0, -math.inf
        self.reset()

    def reset(self):
        """Start the check again (a new leg, after a pause)."""
        self.waiting = False
        self.waiting_since: float | None = None
        self.lost_since: float | None = None
        self.found_since: float | None = None
        self.done = False  # waited once on this leg without them coming back
        self.last_wait = 0.0  # s the last wait lasted (not counted as walking time)

    def update(self, behind: int, now: float):
        """Visitors behind the robot (perception/visitors.behind)."""
        self.behind, self.seen = max(int(behind), 0), now

    def check(self, now: float, allowed: bool) -> WatchEvent | None:
        """Run the check while walking a leg; allowed is False in REMOTE."""
        cfg = self.config
        fresh = now - self.seen <= cfg.timeout_sec
        if not cfg.enabled or not fresh or not allowed:
            self.lost_since = None
            return self.end(now, found=False) if self.waiting else None
        if self.waiting:
            return self._while_waiting(now)
        if self.done:
            return None
        if self.behind > 0:
            self.lost_since = None
            return None
        self.lost_since = self.lost_since if self.lost_since is not None else now
        if now - self.lost_since < cfg.lost_sec:
            return None
        self.waiting, self.waiting_since, self.found_since = True, now, None
        return WatchEvent.BEGIN_WAIT

    def _while_waiting(self, now: float) -> WatchEvent | None:
        if self.behind > 0:
            self.found_since = self.found_since if self.found_since is not None else now
            if now - self.found_since >= self.config.found_sec:
                return self.end(now, found=True)
        else:
            self.found_since = None
        since = self.waiting_since if self.waiting_since is not None else now
        if now - since >= self.config.wait_sec:
            return self.end(now, found=False)
        return None

    def end(self, now: float, found: bool) -> WatchEvent:
        """Stop waiting; without the visitors back, no more waits on this leg."""
        self.last_wait = now - self.waiting_since if self.waiting_since is not None else 0.0
        self.waiting, self.waiting_since = False, None
        self.lost_since = now
        if not found:
            self.done = True
        return WatchEvent.VISITORS_BACK if found else WatchEvent.WAIT_OVER
