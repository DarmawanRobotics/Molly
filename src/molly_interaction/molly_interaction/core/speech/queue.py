"""The utterances waiting to be said (thread safe, no ROS)."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading


@dataclass
class Utterance:
    """One text to say."""

    uid: int
    text: str
    language: str


class SpeechQueue:
    """First-in-first-out utterances with ids; thread safe."""

    def __init__(self, max_queue: int = 10):
        """Start empty; ids count from 1."""
        self._cond = threading.Condition()
        self._items = deque()
        self._next_id = 1
        self._closed = False
        self.max_queue = max(1, int(max_queue))

    def add(self, text: str, language: str):
        """Queue a text; return its Utterance, or None when the queue is full."""
        with self._cond:
            if len(self._items) >= self.max_queue:
                return None
            utterance = Utterance(self._next_id, text, language)
            self._next_id += 1
            self._items.append(utterance)
            self._cond.notify()
            return utterance

    def take(self, timeout: float = 0.5):
        """Return the next utterance, or None after timeout or once closed."""
        with self._cond:
            if not self._items and not self._closed:
                self._cond.wait(timeout)
            if self._closed or not self._items:
                return None
            return self._items.popleft()

    def clear(self) -> list:
        """Drop every waiting utterance and return them."""
        with self._cond:
            dropped = list(self._items)
            self._items.clear()
            return dropped

    def pending(self) -> int:
        """Return how many utterances wait."""
        with self._cond:
            return len(self._items)

    def close(self):
        """Wake the worker and stop handing out utterances."""
        with self._cond:
            self._closed = True
            self._cond.notify_all()
