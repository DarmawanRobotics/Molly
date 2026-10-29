"""
The one handheld that drives: registration, token, busy and denied answers, acks (no ROS).

One driver at a time: a hello from another device (IP address) is answered with `busy` while
the current client is alive; the same device may come back from a new port (an app restart).
Commands are run once per (id, name) in a session; a new session forgets the old acks.
"""

from __future__ import annotations

import hmac

MAX_ACKS = 200
MAX_DENIED = 64
DENY_PERIOD_SEC = 1.0


class Session:
    """The connected client and its cached acknowledgements."""

    def __init__(self, token: str, timeout_sec: float):
        self.token = token
        self.timeout = timeout_sec
        self.client: tuple | None = None
        self.name = ''
        self.seen = 0.0
        self.number = 0
        self.acks: dict = {}
        self.denied: dict = {}
        self.streams = {'camera': False, 'lidar': False, 'video': False}

    def authorized(self, message: dict) -> bool:
        """Check the shared token (constant time) when one is configured."""
        if not self.token:
            return True
        return hmac.compare_digest(str(message.get('k', '')), self.token)

    def deny(self, addr, now: float) -> bool:
        """Return True when a `denied` may go to addr (at most once per second per address)."""
        if now - self.denied.get(addr, -1e9) < DENY_PERIOD_SEC:
            return False
        if len(self.denied) > MAX_DENIED:
            self.denied = {a: t for a, t in self.denied.items() if now - t < DENY_PERIOD_SEC}
        self.denied[addr] = now
        return True

    def alive(self, now: float) -> bool:
        """Return True while the client has been heard within the timeout."""
        return self.client is not None and now - self.seen <= self.timeout

    def busy_for(self, addr, now: float) -> bool:
        """Return True when another device drives and is alive (addr must wait)."""
        current = self.client
        return current is not None and addr[0] != current[0] and self.alive(now)

    def accept(self, addr, name: str, now: float) -> bool:
        """Register addr as the client; return True when it is a new session."""
        self.seen = now
        self.name = str(name)[:32]
        if addr == self.client:
            return False
        self.end()
        self.client = addr
        self.number += 1
        return True

    def end(self):
        """Forget the client and its acknowledgements."""
        self.client = None
        self.acks.clear()

    def expired(self, now: float) -> bool:
        """Return True when the client has been silent past the timeout."""
        return self.client is not None and now - self.seen > self.timeout

    def cache(self, key: tuple, reply: dict):
        """Keep an acknowledgement (bounded)."""
        self.acks[key] = reply
        if len(self.acks) > MAX_ACKS:
            self.acks.pop(next(iter(self.acks)))

    def set_streams(self, values: dict):
        """Switch the camera, lidar and video streams the client asked for."""
        self.streams.update({k: bool(v) for k, v in values.items() if k in self.streams})
