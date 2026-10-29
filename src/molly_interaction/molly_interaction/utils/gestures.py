"""The robot driver's gesture services (std_srvs/Trigger /<robot_ns>/<name>), one at a time."""

from __future__ import annotations

import re
import threading
import time

from std_srvs.srv import Trigger

NAME = re.compile(r'^[a-z][a-z0-9_]{0,31}$')
SERVICE_WAIT_SEC = 1.0


def gesture_names(names) -> list[str]:
    """Return the valid gesture names (a-z, 0-9, _), in order."""
    return [g for g in (str(x).strip() for x in names) if NAME.match(g)]


class GestureRunner:
    """Calls a gesture service and waits for its answer (the Lite3's return when done)."""

    def __init__(self, node, robot_ns: str, names, timeout_sec: float, group):
        self.node, self.ns, self.timeout = node, robot_ns.strip('/') or 'robot', timeout_sec
        self.names = gesture_names(names)
        self.group = group
        self.clients: dict = {}
        self.lock = threading.Lock()
        self.stopping = threading.Event()  # a gesture waiting for the driver gives up

    def client(self, name: str):
        """Return the (cached) Trigger client of the driver's service."""
        if name not in self.clients:
            self.clients[name] = self.node.create_client(
                Trigger, f'/{self.ns}/{name}', callback_group=self.group
            )
        return self.clients[name]

    def fire(self, name: str) -> bool:
        """Start a gesture without waiting (the greeting's); False when unavailable."""
        client = self.client(name)
        if not client.service_is_ready():
            return False
        client.call_async(Trigger.Request())
        return True

    def wait(self, event: threading.Event) -> bool:
        """Wait for event up to the timeout; False at once when the node is stopping."""
        end = time.monotonic() + self.timeout
        while not self.stopping.is_set():
            if event.wait(min(0.1, max(0.0, end - time.monotonic()))):
                return True
            if time.monotonic() >= end:
                return False
        return False

    def run(self, name: str, moving: bool) -> tuple[bool, str]:
        """Run a gesture and wait until it is done; return (success, message)."""
        if name not in self.names:
            return False, f'unknown gesture {name!r}, use one of {self.names}'
        if moving:
            return False, 'robot is walking, gesture refused'
        if not self.lock.acquire(blocking=False):
            return False, 'another gesture is running'
        try:
            client = self.client(name)
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
                return False, f'/{self.ns}/{name} not available'
            done = threading.Event()
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda _: done.set())
            if not self.wait(done) or future.result() is None:
                return False, f'/{self.ns}/{name} timed out'
            result = future.result()
            return result.success, result.message or name
        finally:
            self.lock.release()
