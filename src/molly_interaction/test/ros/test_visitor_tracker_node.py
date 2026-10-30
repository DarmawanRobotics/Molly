"""visitor_tracker_node in-process: a person behind the robot in a PointCloud2."""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
from interaction_rig import Rig
from molly_interaction.visitor_tracker_node import VisitorTrackerNode
import numpy as np
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from dharma_utils.math import default_rng
from dharma_utils.ros.pointcloud import xyz_cloud
from dharma_utils.testing import Collector, wait_for
from molly_interfaces.msg import Visitors

PARAMS = [
    Parameter('input_type', value='pointcloud2'),
    Parameter('pointcloud_topic', value='test/cloud'),
    Parameter('use_map', value=False),
    Parameter('rate_hz', value=20.0),
    Parameter('robot.base_height', value=0.3),
]


def person(x: float, y: float) -> np.ndarray:
    """Points of two legs and a torso around (x, y), in the LiDAR frame (0.2 m above base)."""
    rng = default_rng(7)
    legs = [rng.normal([x, y + dy], 0.03, (15, 2)) for dy in (-0.1, 0.1)]
    xy = np.vstack(legs)
    z = rng.uniform(-0.1, 0.6, len(xy)).reshape(-1, 1)  # 0.4 .. 1.1 m above the floor
    return np.hstack([xy, z]).astype(np.float32)


def test_a_person_behind_is_a_follower(ros):
    node = VisitorTrackerNode(parameter_overrides=PARAMS)
    rig = Rig([node], 'visitor_helper')
    visitors = Collector(rig.helper, Visitors, 'perception/visitors', 10)
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = 'base_link', 'livox_frame'
    mount.transform.translation.z = 0.2
    mount.transform.rotation.w = 1.0
    static = StaticTransformBroadcaster(rig.helper)
    static.sendTransform([mount])
    pub = rig.helper.create_publisher(PointCloud2, 'test/cloud', 10)
    try:
        cloud = np.vstack([person(-1.5, 0.0), [[0.1, 0.0, 0.0]]])  # one point on the body

        def seen():
            header = Header(frame_id='livox_frame', stamp=rig.helper.get_clock().now().to_msg())
            pub.publish(xyz_cloud(header, cloud))
            last = visitors.last
            return last is not None and last.count == 1

        assert wait_for(seen, 10.0)
        last = visitors.last
        assert (last.front, last.behind) == (0, 1)
        assert math.isclose(last.nearest_behind, 1.5, abs_tol=0.1) and last.nearest_front == -1.0
    finally:
        rig.close()
