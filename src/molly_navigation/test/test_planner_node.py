"""In-process test of planner_node against a fake path follower on a room with a door."""

import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
MultiThreadedExecutor = pytest.importorskip('rclpy.executors').MultiThreadedExecutor
qos = pytest.importorskip('rclpy.qos')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
nav_msgs = pytest.importorskip('nav_msgs.msg')
std_srvs = pytest.importorskip('std_srvs.srv')
tf2_ros = pytest.importorskip('tf2_ros')
msgs = pytest.importorskip('molly_interfaces.msg')
srvs = pytest.importorskip('molly_interfaces.srv')
PlannerNode = pytest.importorskip('molly_navigation.planner_node').PlannerNode
Spinner = pytest.importorskip('ros_harness').Spinner


def room_map():
    """Return a 10 x 6 m OccupancyGrid with a wall at x = 5 and a door at y = 0.5 .. 1.7."""
    data = np.zeros((120, 200), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    data[:, 100] = 100
    data[10:34, 100] = 0
    msg = nav_msgs.OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = 0.05
    msg.info.width, msg.info.height = 200, 120
    msg.info.origin.orientation.w = 1.0
    msg.data = data.ravel().tolist()
    return msg


class FakeFollower:
    """navigation/set_path + start_nav and a navigation/status that the test can steer."""

    def __init__(self, node):
        """Offer the services and publish the status at 20 Hz."""
        self.node = node
        self.paths, self.starts = [], []
        self.active = False
        self.state = None  # forced status while active (None: FOLLOWING)
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoints = node.create_publisher(msgs.WaypointPath, 'navigation/waypoints', latched)
        self.status = node.create_publisher(msgs.NavigationStatus, 'navigation/status', 10)
        node.create_service(srvs.SetPath, 'navigation/set_path', self.set_path)
        node.create_service(std_srvs.SetBool, 'navigation/start_nav', self.start_nav)
        node.create_timer(0.05, self.tick)

    def set_path(self, request, response):
        """Record and republish the path like path_loader_node."""
        self.paths.append(list(request.waypoints))
        out = msgs.WaypointPath(waypoints=request.waypoints)
        out.header.stamp = self.node.get_clock().now().to_msg()
        self.waypoints.publish(out)
        response.success = True
        return response

    def start_nav(self, request, response):
        """Record starts and stops."""
        self.starts.append(request.data)
        self.active = request.data
        response.success = True
        return response

    def tick(self):
        """Publish the follower state."""
        msg = msgs.NavigationStatus()
        msg.state = (self.state or 'FOLLOWING') if self.active else 'NAV_INACTIVE'
        self.status.publish(msg)


class Robot:
    """map -> base_link at 20 Hz from a pose the test moves."""

    def __init__(self, node, pose):
        """Start broadcasting."""
        self.node = node
        self.pose = pose
        self.tf = tf2_ros.TransformBroadcaster(node)
        node.create_timer(0.05, self.tick)

    def tick(self):
        """Broadcast the pose."""
        x, y, yaw = self.pose
        t = geometry_msgs.TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(t)


@pytest.fixture
def world():
    """Run the planner next to the fake follower; yield (spinner, helper, follower, robot, rec)."""
    rclpy.init(args=['--ros-args', '-p', 'replan_after_sec:=0.5', '-p', 'status_hz:=10.0',
                     '-p', 'service_timeout_sec:=3.0'])
    helper = rclpy.create_node('planner_test_helper')
    follower = FakeFollower(helper)
    robot = Robot(helper, (2.0, 4.0, 0.0))
    latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
    helper.create_publisher(nav_msgs.OccupancyGrid, '/map', latched).publish(room_map())
    pois = msgs.PoiArray(site='test', pois=[msgs.Poi(id='exhibit', name='Exhibit', x=8.0,
                                                     y=4.0, yaw=math.pi / 2)])
    helper.create_publisher(msgs.PoiArray, 'site/pois', latched).publish(pois)
    recorded = {'status': [], 'path': []}
    helper.create_subscription(msgs.PlannerStatus, 'planner/status', recorded['status'].append,
                               latched)
    helper.create_subscription(nav_msgs.Path, 'planner/path', recorded['path'].append, latched)
    planner = PlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(helper)
    executor.add_node(planner)
    spinner = Spinner(executor)
    spinner.wait(lambda: planner._grid is not None and planner._pois, 5.0)
    spinner.wait(lambda: planner.robot_pose() is not None, 5.0)
    yield spinner, helper, follower, robot, recorded
    spinner.stop()
    planner.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def call(spinner, helper, srv_type, name, **fields):
    """Call a planner service."""
    request = srv_type.Request()
    for key, value in fields.items():
        setattr(request, key, value)
    return spinner.call(helper.create_client(srv_type, name), request, 15.0)


def last_state(recorded):
    """Return the latest planner state."""
    return recorded['status'][-1].state if recorded['status'] else ''


def test_go_to_a_poi_walks_through_the_door_and_arrives(world):
    """go_to plans through the door, starts the follower and reports the arrival."""
    spinner, helper, follower, robot, recorded = world
    result = call(spinner, helper, srvs.GoTo, 'planner/go_to', poi_id='exhibit')
    assert result.success, result.message
    assert follower.starts == [True]
    path = follower.paths[-1]
    assert (path[-1].x, path[-1].y) == pytest.approx((8.0, 4.0), abs=0.06)
    assert path[-1].yaw == pytest.approx(90.0)
    assert any(4.9 < w.x < 5.1 and 0.5 < w.y < 1.7 for w in path), 'not through the door'
    assert spinner.wait(lambda: last_state(recorded) == 'following', 3.0)
    assert recorded['status'][-1].path_length > 6.0 and recorded['path']

    follower.state = 'GOAL_REACHED'  # stale: the robot is not at the goal
    time.sleep(0.5)
    assert last_state(recorded) == 'following'
    robot.pose = (8.0, 4.0, math.pi / 2)
    assert spinner.wait(lambda: last_state(recorded) == 'arrived', 3.0)
    assert follower.starts[-1] is False  # stop_on_arrival
    assert recorded['status'][-1].goal_id == 'exhibit'


def test_blocked_replans_and_cancel_stops(world):
    """A blocked follower gets a new plan; cancel stops it; unknown POIs are refused."""
    spinner, helper, follower, robot, recorded = world
    assert call(spinner, helper, srvs.GoTo, 'planner/go_to', x=8.0, y=2.0, yaw=0.0).success
    assert len(follower.paths) == 1
    follower.state = 'BLOCKED'
    assert spinner.wait(lambda: len(follower.paths) == 2, 3.0), 'no replan'
    assert spinner.wait(lambda: recorded['status'][-1].replans == 1, 2.0)
    follower.state = None
    assert spinner.wait(lambda: last_state(recorded) == 'following', 2.0)
    stopped = call(spinner, helper, std_srvs.Trigger, 'planner/cancel')
    assert stopped.success and follower.starts[-1] is False
    assert spinner.wait(lambda: last_state(recorded) == 'cancelled', 2.0)

    missing = call(spinner, helper, srvs.GoTo, 'planner/go_to', poi_id='nope')
    assert not missing.success and 'nope' in missing.message
    nan = call(spinner, helper, srvs.GoTo, 'planner/go_to', x=math.nan)
    assert not nan.success


def test_plan_previews_without_moving_and_reports_unreachable(world):
    """planner/plan returns waypoints and never loads them; a goal off the map fails."""
    spinner, helper, follower, robot, recorded = world
    preview = call(spinner, helper, srvs.PlanPath, 'planner/plan', x=8.0, y=4.0, yaw=0.0)
    assert preview.success and preview.length > 6.0 and len(preview.waypoints) > 50
    assert follower.paths == [] and follower.starts == []
    off = call(spinner, helper, srvs.GoTo, 'planner/go_to', x=20.0, y=4.0, yaw=0.0)
    assert not off.success and 'outside the map' in off.message
    assert spinner.wait(lambda: last_state(recorded) == 'failed', 2.0)
