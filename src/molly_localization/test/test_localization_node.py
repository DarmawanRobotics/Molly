"""In-process tests of localization_node: tags file switching, origin, reset, pose and tags."""

import json
import math
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
MultiThreadedExecutor = pytest.importorskip('rclpy.executors').MultiThreadedExecutor
qos = pytest.importorskip('rclpy.qos')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
std_msgs = pytest.importorskip('std_msgs.msg')
Trigger = pytest.importorskip('std_srvs.srv').Trigger
tf2_ros = pytest.importorskip('tf2_ros')
LoadFile = pytest.importorskip('molly_interfaces.srv').LoadFile
LocalizationNode = pytest.importorskip(
    'molly_localization.localization_node'
).LocalizationNode

IMU_IN_BASE = (0.1, 0.0, 0.2)
CAMERA_IN_BASE = (0.25, 0.0, 0.1)


def transform(parent, child, xyz, yaw=0.0, stamp=None):
    """Return a TransformStamped."""
    t = geometry_msgs.TransformStamped()
    if stamp is not None:
        t.header.stamp = stamp
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


class Harness:
    """Localization node plus a helper that plays FAST-LIO, the URDF and a tag detector."""

    def __init__(self, tmp_path):
        """Start the node with record into tmp_path."""
        rclpy.init(args=['--ros-args', '-p', 'tag_record_duration_sec:=0.6',
                         '-p', 'tag_frames:=[base_map]', '-p', 'pose_hz:=20.0'])
        self.node = LocalizationNode()
        self.helper = rclpy.create_node('localization_test_helper')
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.statuses, self.poses = [], []
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.helper.create_subscription(
            std_msgs.String, 'localization/status', lambda m: self.statuses.append(m.data),
            latched,
        )
        self.helper.create_subscription(
            geometry_msgs.PoseStamped, 'localization/pose', self.poses.append, 10
        )
        self.static = tf2_ros.StaticTransformBroadcaster(self.helper)
        self.static.sendTransform([
            transform('base_link', 'livox_imu_frame', IMU_IN_BASE),
            transform('base_link', 'camera_color_optical_frame', CAMERA_IN_BASE),
        ])
        self.tf = tf2_ros.TransformBroadcaster(self.helper)
        self.robot_x = 1.0
        self.tag_visible = False
        self.helper.create_timer(0.05, self.publish_tf)
        self.start()

    def publish_tf(self):
        """FAST-LIO pose of the body and, when visible, the tag 2 m ahead of the camera."""
        stamp = self.helper.get_clock().now().to_msg()
        body = (self.robot_x + IMU_IN_BASE[0], 0.0, IMU_IN_BASE[2])
        out = [transform('camera_init', 'body', (body[0] - IMU_IN_BASE[0], 0.0, 0.0), 0.0, stamp)]
        if self.tag_visible:
            out.append(transform('camera_color_optical_frame', 'base_map', (2.0, 0.0, 0.0),
                                 0.0, stamp))
        self.tf.sendTransform(out)

    def start(self):
        """Spin in the background, as the node does in production."""
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        """Spin until shutdown."""
        try:
            self.executor.spin()
        except Exception:  # noqa: B902  the context went away at the end of a test
            pass

    def spin(self, seconds, until=None):
        """Wait for a while or until a condition holds."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if until is not None and until():
                return True
            time.sleep(0.01)
        return until() if until is not None else True

    def call(self, srv_type, name, request, timeout=5.0):
        """Call a service of the node and return the response."""
        client = self.helper.create_client(srv_type, name)
        assert self.spin(3.0, client.service_is_ready), f'{name} not available'
        future = client.call_async(request)
        assert self.spin(timeout, future.done), f'{name} timed out'
        return future.result()

    def close(self):
        """Tear everything down."""
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()
        self.helper.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def loc(tmp_path):
    """Provide a running harness."""
    harness = Harness(tmp_path)
    yield harness
    harness.close()


def test_origin_pose_reset_and_status_values(loc):
    """assume_origin localizes at identity, the pose is published, reset drops it again."""
    assert loc.spin(3.0, lambda: loc.statuses and loc.statuses[-1] == 'none')
    assert loc.spin(3.0, lambda: loc.node._m_base_imu is not None), 'frame bridge not up'
    result = loc.call(Trigger, 'localization/assume_origin', Trigger.Request())
    assert result.success, result.message
    assert loc.spin(2.0, lambda: loc.statuses[-1] == 'origin')
    assert loc.spin(2.0, lambda: len(loc.poses) > 3), 'no localization/pose'
    pose = loc.poses[-1]
    assert pose.header.frame_id == 'map'
    assert pose.pose.position.x == pytest.approx(loc.robot_x, abs=1e-3)
    result = loc.call(Trigger, 'localization/reset', Trigger.Request())
    assert result.success and 'origin' in result.message
    assert loc.spin(2.0, lambda: loc.statuses[-1] == 'none')
    count = len(loc.poses)
    loc.spin(0.5)
    assert len(loc.poses) == count, 'pose published while not localized'


def test_load_tags_creates_records_and_localizes(loc, tmp_path):
    """A new tags file is created empty, record_tags writes there, then a tag localizes."""
    path = tmp_path / 'site' / 'tags.json'
    missing = loc.call(LoadFile, 'localization/load_tags', LoadFile.Request(path=str(path)))
    assert missing.success and '0 tag' in missing.message
    assert not path.exists()
    created = loc.call(
        LoadFile, 'localization/load_tags', LoadFile.Request(path=str(path), create=True)
    )
    assert created.success, created.message
    assert json.loads(path.read_text()) == {}

    assert loc.spin(3.0, lambda: loc.node._m_base_imu is not None)
    assert loc.call(Trigger, 'localization/assume_origin', Trigger.Request()).success
    loc.tag_visible = True
    loc.spin(1.0)
    recorded = loc.call(Trigger, 'localization/record_tags', Trigger.Request(), timeout=6.0)
    assert recorded.success, recorded.message
    tags = json.loads(path.read_text())
    # robot at x=1, camera 0.25 ahead, tag 2 m further: x = 3.25 in the map
    assert tags['base_map']['x'] == pytest.approx(loc.robot_x + CAMERA_IN_BASE[0] + 2.0, abs=0.01)

    assert loc.call(Trigger, 'localization/reset', Trigger.Request()).success
    loc.robot_x = 0.0  # odometry restarted elsewhere; the tag gives the map pose back
    localized = loc.call(Trigger, 'localization/start', Trigger.Request())
    assert localized.success, localized.message
    assert loc.spin(2.0, lambda: loc.statuses[-1] == 'tag:base_map')
    assert loc.spin(2.0, lambda: abs(loc.poses[-1].pose.position.x - 1.0) < 0.02)
