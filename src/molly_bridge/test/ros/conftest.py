"""Fixtures of the in-process node tests (helpers: bridge_rig, a unique module name)."""

from __future__ import annotations

import pytest

from dharma_utils.ros import compat


@pytest.fixture
def ros():
    """Initialise rclpy for one test."""
    compat.init()
    yield
    compat.try_shutdown()
