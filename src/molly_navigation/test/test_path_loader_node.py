"""In-process test of path_loader_node's navigation/set_path."""

import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
qos = pytest.importorskip('rclpy.qos')
msgs = pytest.importorskip('molly_interfaces.msg')
SetPath = pytest.importorskip('molly_interfaces.srv').SetPath
PathLoaderNode = pytest.importorskip('molly_navigation.path_loader_node').PathLoaderNode


def test_set_path_publishes_new_paths_and_refuses_bad_ones(tmp_path):
    """A set path is published as is (or processed), every time with a new stamp."""
    state = tmp_path / 'last_route'
    rclpy.init(args=['--ros-args', '-p', f'route_state_file:={state}'])
    try:
        loader = PathLoaderNode()
        helper = rclpy.create_node('loader_test_helper')
        executor = SingleThreadedExecutor()
        executor.add_node(loader)
        executor.add_node(helper)
        paths = []
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        helper.create_subscription(msgs.WaypointPath, 'navigation/waypoints', paths.append,
                                   latched)
        client = helper.create_client(SetPath, 'navigation/set_path')

        def call(waypoints, process=False):
            request = SetPath.Request(process=process, source='test')
            request.waypoints = [msgs.Waypoint(x=x, y=y, yaw=yaw, dwell_sec=0.0)
                                 for x, y, yaw in waypoints]
            future = client.call_async(request)
            end = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)
            return future.result()

        line = [(0.1 * i, 0.0, 0.0) for i in range(11)]
        assert call(line).success
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)
        assert len(paths) == 1 and len(paths[0].waypoints) == 11
        assert call(line).success  # the same path again is a new delivery
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)
        assert len(paths) == 2 and paths[1].header.stamp != paths[0].header.stamp
        coarse = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 90.0)]
        assert call(coarse, process=True).success
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)
        assert len(paths[-1].waypoints) > 10  # resampled at resample_spacing
        assert paths[-1].waypoints[-1].yaw == pytest.approx(90.0)
        assert not call([(0.0, math.nan, 0.0)]).success
        assert not call([]).success
        assert not state.exists(), 'a set path must not become the remembered route'
        loader.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.shutdown()
