"""In-process test of visitor_tracker_node with a fake cloud, map and TF."""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
sensor_msgs = pytest.importorskip('sensor_msgs.msg')
TransformStamped = pytest.importorskip('geometry_msgs.msg').TransformStamped
OccupancyGrid = pytest.importorskip('nav_msgs.msg').OccupancyGrid
QoS = pytest.importorskip('rclpy.qos')
tf2_ros = pytest.importorskip('tf2_ros')
Visitors = pytest.importorskip('molly_interfaces.msg').Visitors
VisitorTrackerNode = pytest.importorskip(
    'molly_perception.visitor_tracker_node'
).VisitorTrackerNode


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


def legs(x: float, y: float) -> np.ndarray:
    """Two legs at (x, y) in livox_frame (base_link at the same place, z band 0 m)."""
    rng = np.random.default_rng(3)
    return np.vstack([rng.normal([x, y + dy, 0.0], [0.03, 0.03, 0.1], (15, 3))
                      for dy in (-0.1, 0.1)])


def transform(parent: str, child: str, x: float = 0.0) -> 'TransformStamped':
    """Return an identity transform shifted by x."""
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    return t


@pytest.fixture
def rig():
    """Run the node on PointCloud2 with a map that has a pillar and the robot localized."""
    rclpy.init(args=['--ros-args', '-p', 'input_type:=pointcloud2', '-p',
                     'pointcloud_topic:=/test/cloud', '-p', 'rate_hz:=0.0', '-p',
                     'publish_markers:=false'])
    node = VisitorTrackerNode()
    helper = rclpy.create_node('visitor_test_helper')
    static = tf2_ros.StaticTransformBroadcaster(helper)
    static.sendTransform([transform('base_link', 'livox_frame'), transform('map', 'base_link')])
    latched = QoS.QoSProfile(depth=1, durability=QoS.QoSDurabilityPolicy.TRANSIENT_LOCAL)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution, grid.info.width, grid.info.height = 0.1, 100, 100
    grid.info.origin.position.x = grid.info.origin.position.y = -5.0
    grid.info.origin.orientation.w = 1.0
    data = np.zeros((100, 100), dtype=np.int8)
    data[50, 70] = 100  # pillar at map (2.05, 0.05)
    grid.data = data.ravel().tolist()
    map_pub = helper.create_publisher(OccupancyGrid, '/map', latched)
    map_pub.publish(grid)
    seen = []
    helper.create_subscription(Visitors, 'perception/visitors', seen.append, 10)
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
        return seen

    yield run
    executor.shutdown()
    helper.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def test_visitors_front_behind_and_pillar(rig):
    """A visitor ahead and one behind count, the pillar on the map does not."""
    points = np.vstack([legs(1.2, 0.0), legs(-1.5, 0.3), legs(2.05, 0.05)]).astype(np.float32)
    seen = rig(points)
    assert seen, 'no perception/visitors message'
    last = seen[-1]
    assert (last.count, last.front, last.behind) == (2, 1, 1)
    assert last.nearest_front == pytest.approx(1.2, abs=0.1)
    assert last.nearest_behind == pytest.approx(1.53, abs=0.1)
    assert len(last.positions) == len(last.confidences) == 2
