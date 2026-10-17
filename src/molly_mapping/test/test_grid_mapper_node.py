"""In-process test of grid_mapper_node: scans through TF into a grid, save, stop, load."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))

rclpy = pytest.importorskip('rclpy')
MultiThreadedExecutor = pytest.importorskip('rclpy.executors').MultiThreadedExecutor
qos = pytest.importorskip('rclpy.qos')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
nav_msgs = pytest.importorskip('nav_msgs.msg')
sensor_msgs = pytest.importorskip('sensor_msgs.msg')
std_srvs = pytest.importorskip('std_srvs.srv')
tf2_ros = pytest.importorskip('tf2_ros')
interfaces = pytest.importorskip('molly_interfaces')
MappingStatus = pytest.importorskip('molly_interfaces.msg').MappingStatus
LoadFile = pytest.importorskip('molly_interfaces.srv').LoadFile
GridMapperNode = pytest.importorskip('molly_mapping.grid_mapper_node').GridMapperNode
map_io = pytest.importorskip('molly_mapping.map_io')
Spinner = pytest.importorskip('ros_harness').Spinner

BODY_IN_BASE = (0.1, 0.0, 0.2)


def static(parent, child, xyz):
    """Return a static TransformStamped."""
    t = geometry_msgs.TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.w = 1.0
    return t


def scan(stamp):
    """Return a wall 3 m ahead (map x), the floor and the robot's legs, in the body frame."""
    ys = np.arange(-1.5, 1.5, 0.02)
    wall = [(3.0, y, z) for y in ys for z in (-0.1, 0.3)]
    floor = [(x, 0.0, -0.32) for x in np.arange(1.0, 2.5, 0.05)]
    legs = [(0.2, 0.15, -0.2), (-0.2, -0.15, -0.2)]
    points = np.array(wall + floor + legs, dtype=np.float32) - np.array(BODY_IN_BASE, np.float32)
    msg = sensor_msgs.PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp, 'body'
    msg.height, msg.width = 1, points.shape[0]
    msg.fields = [
        sensor_msgs.PointField(name=n, offset=4 * i, datatype=7, count=1)
        for i, n in enumerate('xyz')
    ]
    msg.point_step, msg.row_step, msg.is_dense = 12, 12 * points.shape[0], True
    msg.data = points.tobytes()
    return msg


def cell(grid, x, y):
    """Return the OccupancyGrid value at a map point (resolution is float32 on the wire)."""
    resolution = round(grid.info.resolution, 6)
    col = int((x - grid.info.origin.position.x) // resolution)
    row = int((y - grid.info.origin.position.y) // resolution)
    return grid.data[row * grid.info.width + col]


def test_scans_become_a_grid_that_saves_and_loads(tmp_path):
    """Walls are occupied, the floor and the legs are not, and the map round-trips a site dir."""
    rclpy.init(args=['--ros-args', '-p', 'rate_hz:=10.0', '-p', 'publish_hz:=5.0'])
    mapper = GridMapperNode()
    helper = rclpy.create_node('grid_mapper_test_helper')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(mapper)
    executor.add_node(helper)
    spinner = Spinner(executor)
    try:
        broadcaster = tf2_ros.StaticTransformBroadcaster(helper)
        broadcaster.sendTransform([
            static('map', 'base_link', (0.0, 0.0, 0.0)),
            static('base_link', 'body', BODY_IN_BASE),
        ])
        grids, statuses = [], []
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        helper.create_subscription(nav_msgs.OccupancyGrid, 'mapping/grid', grids.append, latched)
        helper.create_subscription(MappingStatus, 'mapping/status', statuses.append, latched)
        clouds = helper.create_publisher(sensor_msgs.PointCloud2, '/cloud_registered_body', 5)
        helper.create_timer(0.1, lambda: clouds.publish(scan(helper.get_clock().now().to_msg())))
        client = lambda t, n: helper.create_client(t, n)  # noqa: E731

        assert spinner.call(client(std_srvs.SetBool, 'grid_mapper/start'),
                            std_srvs.SetBool.Request(data=True)).success
        assert spinner.wait(lambda: statuses and statuses[-1].scans >= 4, 10.0), 'no scans'
        wall = lambda: grids and cell(grids[-1], 3.02, 0.02) == map_io.OCCUPIED  # noqa: E731
        assert spinner.wait(wall, 5.0), 'wall not in the published grid'
        grid = grids[-1]
        assert grid.header.frame_id == 'map'
        assert cell(grid, 3.02, 0.02) == map_io.OCCUPIED
        assert cell(grid, 1.52, 0.02) == map_io.FREE       # floor below the band: not a wall
        assert cell(grid, 0.22, 0.17) != map_io.OCCUPIED  # own legs filtered
        assert statuses[-1].active and statuses[-1].known_area_m2 > 2.0

        site = tmp_path / 'lobby'
        save = client(LoadFile, 'grid_mapper/save')
        saved = spinner.call(save, LoadFile.Request(path=str(site)))
        assert saved.success, saved.message
        assert (site / 'map.pgm').exists() and (site / 'map.yaml').exists()
        loaded_map = map_io.load_map(str(site))
        stopped = spinner.call(client(std_srvs.Trigger, 'grid_mapper/stop'),
                               std_srvs.Trigger.Request())
        assert stopped.success
        assert spinner.wait(lambda: not statuses[-1].active, 3.0)

        assert spinner.call(client(std_srvs.SetBool, 'grid_mapper/start'),
                            std_srvs.SetBool.Request(data=True)).success
        assert spinner.call(client(std_srvs.Trigger, 'grid_mapper/stop'),
                            std_srvs.Trigger.Request()).success
        load = client(LoadFile, 'grid_mapper/load')
        loaded = spinner.call(load, LoadFile.Request(path=str(site)))
        assert loaded.success, loaded.message
        resaved = spinner.call(save, LoadFile.Request(path=str(tmp_path / 'copy')))
        assert resaved.success
        assert np.array_equal(map_io.load_map(str(tmp_path / 'copy')).data, loaded_map.data)
        bad = spinner.call(load, LoadFile.Request(path=str(tmp_path / 'missing')))
        assert not bad.success
    finally:
        spinner.stop()
        mapper.destroy_node()
        helper.destroy_node()
        rclpy.shutdown()
