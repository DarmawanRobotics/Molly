"""Helpers of the interaction node tests: a helper node, spinning, service calls."""

from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from dharma_utils.testing import Spinner


class Rig:
    """The nodes under test and a helper node on a multi-threaded executor."""

    def __init__(self, nodes, helper: str):
        self.nodes = nodes
        self.helper = rclpy.create_node(helper)
        executor = MultiThreadedExecutor(num_threads=4)
        for node in [*nodes, self.helper]:
            executor.add_node(node)
        self.spinner = Spinner(executor)

    def call(self, srv_type, service: str, timeout: float = 10.0, **fields):
        """Call a service from the helper node."""
        client = self.helper.create_client(srv_type, service)
        return self.spinner.call(client, srv_type.Request(**fields), timeout)

    def close(self):
        """Stop spinning and destroy every node."""
        self.spinner.stop()
        for node in [*self.nodes, self.helper]:
            node.destroy_node()
