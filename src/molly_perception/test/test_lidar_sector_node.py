"""In-process test of lidar_sector_node with a fake PointCloud2 and a static TF."""

import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
sensor_msgs = pytest.importorskip('sensor_msgs.msg')
TransformStamped = pytest.importorskip('geometry_msgs.msg').TransformStamped
tf2_ros = pytest.importorskip('tf2_ros')
SectorScan = pytest.importorskip('molly_interfaces.msg').SectorScan
LidarSectorNode = pytest.importorskip('molly_perception.lidar_sector_node').LidarSectorNode


def cloud(points: np.ndarray) -> 'sensor_msgs.PointCloud2':
    """Pack Nx3 float32 points into a PointCloud2 in livox_frame."""
    msg = sensor_msgs.PointCloud2()
    msg.header.frame_id = 'livox_frame'
    msg.height, msg.width = 1, points.shape[0]
    msg.fields = [
        sensor_msgs.PointField(name=n, offset=4 * i, datatype=7, count=1)
        for i, n in enumerate('xyz')
    ]
    msg.point_step, msg.row_step = 12, 12 * points.shape[0]
    msg.data = points.astype(np.float32).tobytes()
    return msg


@pytest.fixture
def rig():
    """Run the node (PointCloud2 input) with a static base_link->livox_frame TF and recorders."""
    rclpy.init(args=['--ros-args', '-p', 'input_type:=pointcloud2', '-p',
                     'pointcloud_topic:=/test/cloud', '-p', 'publish_markers:=false'])
    node = LidarSectorNode()
    helper = rclpy.create_node('sector_test_helper')
    tf = tf2_ros.StaticTransformBroadcaster(helper)
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = 'base_link', 'livox_frame'
    t.transform.translation.z = 0.2
    t.transform.rotation.w = 1.0
    tf.sendTransform(t)
    scans = {'front': [], 'safety': []}
    helper.create_subscription(SectorScan, 'perception/sector_scan', scans['front'].append, 10)
    helper.create_subscription(SectorScan, 'perception/safety_scan', scans['safety'].append, 10)
    pub = helper.create_publisher(sensor_msgs.PointCloud2, '/test/cloud', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)

    def run(points, seconds=1.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pub.publish(cloud(points))
            for _ in range(10):
                executor.spin_once(timeout_sec=0.01)
        return scans

    yield run
    executor.shutdown()
    helper.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def test_front_and_safety_scans(rig):
    """A box 1 m ahead and 1.5 m behind: front scan sees one, the 360 deg scan sees both."""
    rng = np.random.default_rng(1)
    clutter = rng.uniform(-3.0, 3.0, (500, 3)) * [1, 1, 0] + [0, 0, 3.0]  # ceiling, filtered
    ahead = np.array([[1.0, y, 0.0] for y in np.linspace(-0.1, 0.1, 5)])
    behind = np.array([[-1.5, y, 0.0] for y in np.linspace(-0.1, 0.1, 5)])
    scans = rig(np.vstack([clutter, ahead, behind]).astype(np.float32))
    front, safety = scans['front'][-1], scans['safety'][-1]
    assert min(front.ranges) == pytest.approx(1.0, abs=0.01)
    assert safety.num_sectors == 72 and safety.angle_min == pytest.approx(-math.pi)
    hits = sorted(r for r in safety.ranges if r < safety.range_max)
    assert hits[0] == pytest.approx(1.0, abs=0.01) and hits[-1] == pytest.approx(1.5, abs=0.01)


def test_blind_lidar_blocks_every_safety_sector(rig):
    """Fewer than min_raw_points points: the safety scan reports everything blocked."""
    scans = rig(np.array([[2.0, 0.0, 0.0]] * 10, dtype=np.float32))
    assert set(scans['safety'][-1].ranges) == {0.0}
