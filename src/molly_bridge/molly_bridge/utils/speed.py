"""The follower's walking speed (speed.cruise) and its cap, polled for the handheld."""

from __future__ import annotations

import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

POLL_SEC = 3.0
NAMES = ['speed.cruise', 'robot.limits.max_vx']


class SpeedPoll:
    """Asks path_follower_node for its cruise speed, one request at a time."""

    def __init__(self, node, navigation_node: str, low: float, group):
        self.client = node.create_client(
            GetParameters, f'{navigation_node}/get_parameters', callback_group=group
        )
        self.low = low
        self.value: dict | None = None  # {target, min, max}; None until the follower answered
        self.asked = 0.0
        self.future = None

    def due(self, now: float) -> bool:
        """Return True when it is time to ask again."""
        return now - self.asked > POLL_SEC

    def ask(self):
        """Ask the follower (no second request while one is out)."""
        self.asked = time.monotonic()
        if self.future is not None and not self.future.done():
            return
        if not self.client.service_is_ready():
            self.value = None
            return
        self.future = self.client.call_async(GetParameters.Request(names=NAMES))
        self.future.add_done_callback(self.answer)

    def answer(self, future):
        """Keep the cruise speed and its cap."""
        try:
            values = future.result().values
        except Exception:  # the node went away meanwhile: ask again later
            return
        double = ParameterType.PARAMETER_DOUBLE
        if len(values) == 2 and all(v.type == double for v in values):
            top = round(max(values[1].double_value, self.low), 2)
            self.value = {'target': round(values[0].double_value, 2), 'min': self.low, 'max': top}

    def range(self, default_max: float) -> tuple[float, float]:
        """Return the speeds the handheld may set (m/s)."""
        top = (self.value or {}).get('max') or default_max
        return self.low, float(top)
