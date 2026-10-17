"""In-process tests of path_follower_node behaviour that needs a real rclpy node."""

import pytest

rclpy = pytest.importorskip('rclpy')
BatteryState = pytest.importorskip('sensor_msgs.msg').BatteryState
Trigger = pytest.importorskip('std_srvs.srv').Trigger
PathFollowerNode = pytest.importorskip(
    'molly_navigation.path_follower_node'
).PathFollowerNode


@pytest.fixture
def follower():
    """Provide a path follower node."""
    rclpy.init()
    node = PathFollowerNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_status_changes_between_info_and_warn_states(follower):
    """FOLLOWING -> BLOCKED -> FOLLOWING must not raise (rclpy severity per call site)."""
    stamp = follower.get_clock().now().to_msg()
    for state in ('FOLLOWING', 'BLOCKED', 'FOLLOWING', 'OFF_PATH', 'PAUSED', 'EMERGENCY_STOP'):
        follower.publish_status(stamp, state, 'test')


def test_low_battery_flag_clears_after_charging(follower):
    """The low-battery flag sets below the threshold and clears 5 % above it."""
    def battery(pct):
        msg = BatteryState()
        msg.percentage = pct
        follower.battery_callback(msg)
        return follower._battery_low

    assert not battery(0.5)
    assert battery(0.15)
    assert battery(0.22)  # within the hysteresis band
    assert battery(float('nan'))  # unknown readings change nothing
    assert not battery(0.3)


def test_stop_reports_inspection_when_every_inspector_is_down(follower):
    """With no inspection service up, 'inspected' still goes out (saying so) at the stop."""
    events = []
    follower.publish_stop_event = lambda event, message='', index=None: events.append(
        (event, message, index)
    )
    follower._inspection_clients = [follower.create_client(Trigger, 'no_such_inspector')]
    follower._next_stop_index = 0
    follower.start_inspection()
    assert len(events) == 1
    event, message, index = events[0]
    assert (event, index) == ('inspected', 0)
    assert 'no_such_inspector: not available' in message
    assert follower._inspection_pending == 0


def straight_route(follower, stop_at=30):
    """Load a 6 m straight route along x with one stop point."""
    follower._waypoints = [
        (0.1 * i, 0.0, 0.0, 5.0 if i == stop_at else 0.0) for i in range(60)
    ]
    follower.compute_speed_profile()
    follower.restart_route()


def test_start_finds_the_robot_again_after_the_route_end(follower):
    """A finished or far-away progress is replaced by the robot's place on the route at start."""
    from molly_navigation.utils.pose2d import Pose2D

    straight_route(follower)
    follower._controller.set_path([(w[0], w[1]) for w in follower._waypoints], 59)
    follower._follower_state = 'GOAL_HOLD'
    follower.current_pose = lambda: Pose2D(1.0, 0.05, 0.0)
    located, note = follower.find_robot_on_route()
    assert located and 'point' in note
    assert follower._follower_state == 'FOLLOWING'
    assert abs(follower._controller.progress_index() - 10) <= 1
    assert follower._next_stop_index == 30

    follower.current_pose = lambda: Pose2D(1.2, 0.0, 0.0)  # still on its progress: kept
    assert follower.find_robot_on_route() == (True, '')


def test_start_off_the_route_joins_it_or_is_refused(follower):
    """A robot beside the route walks onto it first; one too far away does not start."""
    from molly_navigation.utils.pose2d import Pose2D

    straight_route(follower)
    follower._controller.set_path([(w[0], w[1]) for w in follower._waypoints], 0)
    follower.current_pose = lambda: Pose2D(3.0, 2.0, 0.0)
    located, note = follower.find_robot_on_route()
    assert located and 'joining' in note
    assert follower._follower_state == 'JOINING'
    assert abs(follower._join_index - 30) <= 1

    follower.current_pose = lambda: Pose2D(3.0, 8.0, 0.0)
    located, note = follower.find_robot_on_route()
    assert not located and 'from the route' in note


def test_loading_the_same_route_again_starts_it_over(follower):
    """A redelivered route keeps the progress, the same file loaded again restarts it."""
    from molly_interfaces.msg import Waypoint, WaypointPath

    msg = WaypointPath()
    msg.header.stamp.sec = 10
    msg.waypoints = [Waypoint(x=0.1 * i, y=0.0, yaw=0.0, dwell_sec=0.0) for i in range(60)]
    follower.waypoints_callback(msg)
    follower._follower_state = 'GOAL_HOLD'
    follower.waypoints_callback(msg)
    assert follower._follower_state == 'GOAL_HOLD'
    msg.header.stamp.sec = 20
    follower.waypoints_callback(msg)
    assert follower._follower_state == 'FOLLOWING'


def test_a_new_delivery_of_the_same_path_starts_over(follower):
    """A planned path delivered again (new stamp) leaves the goal hold and starts over."""
    WaypointPath = pytest.importorskip('molly_interfaces.msg').WaypointPath
    Waypoint = pytest.importorskip('molly_interfaces.msg').Waypoint
    msg = WaypointPath()
    msg.waypoints = [Waypoint(x=0.1 * i, y=0.0, yaw=0.0, dwell_sec=0.0) for i in range(20)]
    msg.header.stamp.sec = 1
    follower.waypoints_callback(msg)
    follower._follower_state = 'GOAL_HOLD'
    follower.waypoints_callback(msg)  # redelivered: ignored
    assert follower._follower_state == 'GOAL_HOLD'
    msg.header.stamp.sec = 2
    follower.waypoints_callback(msg)
    assert follower._follower_state == 'FOLLOWING'
