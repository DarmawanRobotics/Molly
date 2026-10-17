"""Shared helpers for the in-process node tests: a background executor and service calls."""

import threading
import time


class Spinner:
    """Spin an executor in a background thread; call services and wait from the test."""

    def __init__(self, executor):
        """Start spinning."""
        self.executor = executor
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        """Spin until the executor shuts down."""
        try:
            self.executor.spin()
        except Exception:  # noqa: B902  the context went away at the end of a test
            pass

    @staticmethod
    def wait(condition, seconds: float) -> bool:
        """Poll a condition for up to seconds."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.01)
        return condition()

    def call(self, client, request, seconds: float = 10.0):
        """Call a service and return its response (asserts it answered)."""
        assert self.wait(client.service_is_ready, 5.0), f'{client.srv_name} not available'
        future = client.call_async(request)
        assert self.wait(future.done, seconds), f'{client.srv_name} timed out'
        return future.result()

    def stop(self):
        """Stop spinning."""
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
