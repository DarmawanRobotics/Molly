#!/usr/bin/env python3
"""
Path finder: plan on the site map to a POI or a pose and hand the path to the path follower.

planner/go_to plans with A* on the inflated /map, loads the path with navigation/set_path and
starts navigation/start_nav. The follower walks it (path shift around people) and converges on
the goal pose (GOAL_REACHED). When the follower reports BLOCKED for replan_after_sec, or OFF_PATH,
the planner plans again from where the robot is with the obstacles of the safety scan in the
costmap, up to max_replans times.
"""

import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from molly_interfaces.msg import (
    NavigationStatus,
    PlannerStatus,
    PoiArray,
    SectorScan,
    Waypoint,
    WaypointPath,
)
from molly_interfaces.srv import GoTo, PlanPath, SetPath
from molly_navigation.utils.costmap import build_costmap
from molly_navigation.utils.geometry import yaw_from_quaternion
from molly_navigation.utils.planning import plan_path, remaining_length
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULTS = {
    'robot_radius': 0.35,
    'inflation_radius': 0.7,
    'cost_scaling': 3.0,
    'unknown_is_obstacle': True,
    'occupied_threshold': 50,
    'planning_resolution': 0.1,
    'goal_tolerance': 0.5,
    'start_escape': 1.0,
    'waypoint_spacing': 0.1,
    'smooth_weight_data': 0.1,
    'smooth_weight_smooth': 0.3,
    'heuristic_weight': 1.0,
    'plan_timeout_sec': 5.0,
    'arrival_tolerance': 0.35,
    'stop_on_arrival': True,
    'replan_after_sec': 3.0,
    'max_replans': 5,
    'use_live_obstacles': True,
    'publish_costmap': False,
    'status_hz': 2.0,
    'service_timeout_sec': 5.0,
    'map_frame': 'map',
    'base_frame': 'base_link',
}
ACTIVE = ('following', 'replanning')


class PlannerNode(Node):
    """Plan to POIs and poses on /map, drive the path follower and replan around blockages."""

    def __init__(self):
        """Declare parameters and interfaces."""
        super().__init__('planner_node')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._grid = None            # (data, resolution, origin_x, origin_y) of /map
        self._grid_key = None
        self._costmap = None
        self._pois = {}
        self._scan = None            # (stamp, obstacle points in base_link)
        self._nav = None             # (state, message, receive time)
        self._waypoints_time = 0.0   # when navigation/waypoints last delivered a path
        self._state, self._message = 'idle', ''
        self._goal = None            # (goal_id, x, y, yaw)
        self._path_xy = []
        self._length = 0.0
        self._replans = 0
        self._set_time = 0.0
        self._blocked_since = None
        self._inactive_since = None
        self._last_status = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._work_group = MutuallyExclusiveCallbackGroup()  # planning and steering
        self._client_group = ReentrantCallbackGroup()
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoiArray, 'site/pois', self.on_pois, latched)
        self.create_subscription(NavigationStatus, 'navigation/status', self.on_nav, 10)
        self.create_subscription(SectorScan, 'perception/safety_scan', self.on_scan, 10)
        self.create_subscription(WaypointPath, 'navigation/waypoints', self.on_waypoints, latched)
        self._path_pub = self.create_publisher(Path, 'planner/path', latched)
        self._status_pub = self.create_publisher(PlannerStatus, 'planner/status', latched)
        self._costmap_pub = self.create_publisher(OccupancyGrid, 'planner/costmap', latched)
        self._set_path = self.create_client(
            SetPath, 'navigation/set_path', callback_group=self._client_group
        )
        self._start_nav = self.create_client(
            SetBool, 'navigation/start_nav', callback_group=self._client_group
        )
        self.create_service(PlanPath, 'planner/plan', self.plan_callback,
                            callback_group=self._work_group)
        self.create_service(GoTo, 'planner/go_to', self.go_to_callback,
                            callback_group=self._work_group)
        self.create_service(Trigger, 'planner/cancel', self.cancel_callback,
                            callback_group=self._work_group)
        self.create_timer(0.2, self.monitor, callback_group=self._work_group)
        self.publish_status()

    # ------------------------------------------------------------------ inputs

    def on_map(self, msg: OccupancyGrid):
        """Keep the latest map; the costmap is rebuilt on the next plan."""
        width, height = int(msg.info.width), int(msg.info.height)
        if width * height == 0 or len(msg.data) != width * height:
            self._grid = None
            return
        if abs(yaw_from_quaternion(msg.info.origin.orientation)) > 1e-6:
            self.get_logger().error('maps with a rotated origin are not supported')
            self._grid = None
            return
        data = np.frombuffer(msg.data, dtype=np.int8).reshape(height, width).copy()
        self._grid = (
            data,
            round(float(msg.info.resolution), 6),  # float32 on the wire
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
        )
        self._costmap = None

    def on_pois(self, msg: PoiArray):
        """Index the active site's POIs by id."""
        self._pois = {p.id: p for p in msg.pois}

    def on_nav(self, msg: NavigationStatus):
        """Remember the follower's state."""
        self._nav = (msg.state, msg.message, time.monotonic())

    def on_waypoints(self, msg: WaypointPath):
        """Note when the follower's path was (re)delivered."""
        self._waypoints_time = time.monotonic()

    def on_scan(self, msg: SectorScan):
        """Keep the obstacles of the 360 deg safety scan as points in base_link."""
        ranges = np.asarray(msg.ranges, dtype=float)
        start = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        hit = np.isfinite(ranges) & (ranges < msg.range_max - 1e-3) & (ranges > 0.05)
        r = np.repeat(ranges[hit], 3)
        a = (start[hit][:, None] + msg.angle_increment * np.array([0.0, 0.5, 1.0])).ravel()
        self._scan = (time.monotonic(), np.column_stack([r * np.cos(a), r * np.sin(a)]))

    # ------------------------------------------------------------------ helpers

    def param(self, name: str):
        """Return a parameter value."""
        return self.get_parameter(name).value

    def robot_pose(self):
        """Return (x, y, yaw) of base_link in the map, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)

    def live_obstacles(self, pose):
        """Return the fresh safety-scan obstacles in the map frame (or None)."""
        if not bool(self.param('use_live_obstacles')) or self._scan is None:
            return None
        stamp, local = self._scan
        if time.monotonic() - stamp > 1.0 or local.size == 0:
            return None
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack([
            x + c * local[:, 0] - s * local[:, 1],
            y + s * local[:, 0] + c * local[:, 1],
        ])

    def costmap(self, extra=None):
        """Return the costmap of the current map (cached unless extra obstacles are given)."""
        if self._grid is None:
            return None
        if extra is None and self._costmap is not None:
            return self._costmap
        data, resolution, ox, oy = self._grid
        p = self.param
        costmap = build_costmap(
            data, resolution, ox, oy,
            planning_resolution=float(p('planning_resolution')),
            robot_radius=float(p('robot_radius')),
            inflation_radius=float(p('inflation_radius')),
            cost_scaling=float(p('cost_scaling')),
            unknown_is_obstacle=bool(p('unknown_is_obstacle')),
            extra_obstacles=extra,
            occupied_threshold=int(p('occupied_threshold')),
        )
        if extra is None:
            self._costmap = costmap
        if bool(p('publish_costmap')):
            self.publish_costmap(costmap)
        return costmap

    def plan(self, goal, live: bool):
        """Plan from the robot to goal (x, y, yaw); return (Plan or None, message)."""
        pose = self.robot_pose()
        if pose is None:
            return None, f'robot pose unknown (no {self._map_frame} -> {self._base_frame})'
        extra = self.live_obstacles(pose) if live else None
        costmap = self.costmap(extra)
        if costmap is None:
            return None, 'no map: load a site with a map first'
        p = self.param
        result = plan_path(
            costmap, pose, goal,
            goal_tolerance=float(p('goal_tolerance')),
            start_escape=float(p('start_escape')),
            spacing=float(p('waypoint_spacing')),
            smooth_weights=(float(p('smooth_weight_data')), float(p('smooth_weight_smooth'))),
            heuristic_weight=float(p('heuristic_weight')),
            timeout_sec=float(p('plan_timeout_sec')),
        )
        return (result if result.ok else None), result.message

    def call(self, client, request):
        """Call a service and wait for it; (result, error)."""
        timeout = float(self.param('service_timeout_sec'))
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None, f'{client.srv_name} not available'
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            future.cancel()
            return None, f'{client.srv_name} timed out'
        try:
            return future.result(), ''
        except Exception as error:  # noqa: B902  a failed call is an answer too
            return None, f'{client.srv_name} failed: {error}'

    def send(self, plan):
        """Load the plan into the follower and start it; (ok, message)."""
        request = SetPath.Request(process=False, source='planner')
        request.waypoints = [
            Waypoint(x=float(x), y=float(y), yaw=float(yaw), dwell_sec=float(dwell))
            for x, y, yaw, dwell in plan.waypoints
        ]
        sent = time.monotonic()
        result, error = self.call(self._set_path, request)
        if result is None or not result.success:
            return False, f'path not loaded: {error or result.message}'
        end = time.monotonic() + 1.0
        while self._waypoints_time < sent and time.monotonic() < end:
            time.sleep(0.01)  # the follower gets the latched path at about the same time
        time.sleep(0.05)
        message = ''
        for _ in range(3):
            result, error = self.call(self._start_nav, SetBool.Request(data=True))
            if result is not None and result.success:
                self._set_time = time.monotonic()
                self._blocked_since = self._inactive_since = None
                self._path_xy = [(w[0], w[1]) for w in plan.waypoints]
                self._length = plan.length
                self.publish_path(plan.waypoints)
                return True, ''
            message = error or result.message
            time.sleep(0.3)
        return False, f'navigation did not start: {message}'

    def stop_navigation(self):
        """Stop the follower (best effort)."""
        result, error = self.call(self._start_nav, SetBool.Request(data=False))
        if result is None:
            self.get_logger().warn(f'could not stop navigation: {error}')

    def set_state(self, state: str, message: str = ''):
        """Change state, log it and publish the status."""
        if state != self._state or message != self._message:
            if state == 'failed':
                self.get_logger().error(f'[{state}] {message}')
            else:
                self.get_logger().info(f'[{state}] {message}')
        self._state, self._message = state, message
        self.publish_status()

    def goal_text(self) -> str:
        """Describe the goal for messages."""
        if self._goal is None:
            return 'nothing'
        goal_id, x, y, _ = self._goal
        return goal_id or f'({x:.2f}, {y:.2f})'

    # ------------------------------------------------------------------ publishing

    def publish_path(self, waypoints):
        """Publish the current plan as nav_msgs/Path (latched)."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        for x, y, yaw_deg, _ in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
            pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def publish_status(self):
        """Publish planner/status (latched)."""
        msg = PlannerStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state, msg.message = self._state, self._message
        if self._goal is not None:
            msg.goal_id = self._goal[0]
            msg.goal_x, msg.goal_y, msg.goal_yaw = (float(v) for v in self._goal[1:])
        msg.path_length = float(self._length)
        pose = self.robot_pose() if self._state in ACTIVE else None
        if pose is not None and self._path_xy:
            msg.remaining = float(remaining_length(self._path_xy, pose[0], pose[1]))
        elif self._state not in ('arrived', 'idle'):
            msg.remaining = float(self._length)
        msg.replans = int(self._replans)
        self._status_pub.publish(msg)
        self._last_status = time.monotonic()

    def publish_costmap(self, costmap):
        """Publish the costmap as an OccupancyGrid (debug): lethal 100, cost scaled 0-99."""
        cost = costmap.cost
        data = np.where(np.isfinite(cost), np.clip((cost - 1.0) * 15.0, 0, 99), 100)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = float(costmap.resolution)
        msg.info.width, msg.info.height = costmap.cols, costmap.rows
        msg.info.origin.position.x, msg.info.origin.position.y = (
            float(costmap.origin_x), float(costmap.origin_y),
        )
        msg.info.origin.orientation.w = 1.0
        msg.data = data.astype(np.int8).ravel().tolist()
        self._costmap_pub.publish(msg)

    # ------------------------------------------------------------------ services

    def resolve_goal(self, request):
        """Return ((goal_id, x, y, yaw), None) from a GoTo request, or (None, error)."""
        poi_id = request.poi_id.strip()
        if poi_id:
            poi = self._pois.get(poi_id)
            if poi is None:
                return None, f'no POI {poi_id!r} on the active site'
            return (poi_id, poi.x, poi.y, poi.yaw), None
        values = (request.x, request.y, request.yaw)
        if not all(math.isfinite(v) for v in values):
            return None, 'goal x, y and yaw must be finite'
        return ('', *values), None

    def plan_callback(self, request, response):
        """Service: plan to a pose without moving."""
        values = (request.x, request.y, request.yaw)
        if not all(math.isfinite(v) for v in values):
            response.success, response.message = False, 'goal x, y and yaw must be finite'
            return response
        plan, message = self.plan(values, live=False)
        response.success, response.message = plan is not None, message
        if plan is not None:
            response.length = float(plan.length)
            response.waypoints = [
                Waypoint(x=float(x), y=float(y), yaw=float(yaw), dwell_sec=0.0)
                for x, y, yaw, _ in plan.waypoints
            ]
        return response

    def go_to_callback(self, request, response):
        """Service: plan to a POI or pose, load the path and start walking."""
        goal, error = self.resolve_goal(request)
        if goal is None:
            response.success, response.message = False, error
            return response
        self._goal, self._replans = goal, 0
        self.set_state('planning', f'to {self.goal_text()}')
        plan, message = self.plan(goal[1:], live=False)
        if plan is None:
            self.set_state('failed', f'to {self.goal_text()}: {message}')
            response.success, response.message = False, self._message
            return response
        ok, error = self.send(plan)
        if not ok:
            self.set_state('failed', f'to {self.goal_text()}: {error}')
            response.success, response.message = False, self._message
            return response
        self.set_state('following', f'to {self.goal_text()}: {message}')
        response.success, response.message = True, self._message
        return response

    def cancel_callback(self, request, response):
        """Service: stop walking."""
        was = self._state
        self.stop_navigation()
        if was in ACTIVE:
            self.set_state('cancelled', f'to {self.goal_text()}: cancelled')
        response.success = True
        response.message = f'stopped ({was})'
        return response

    # ------------------------------------------------------------------ steering

    def replan(self, reason: str):
        """Plan again from where the robot is, with the obstacles it sees."""
        if self._replans >= int(self.param('max_replans')):
            self.stop_navigation()
            self.set_state('failed', f'to {self.goal_text()}: {reason}, gave up after '
                                     f'{self._replans} replans')
            return
        self._replans += 1
        self.set_state('replanning', f'to {self.goal_text()}: {reason}')
        plan, message = self.plan(self._goal[1:], live=True)
        if plan is None:
            self.stop_navigation()
            self.set_state('failed', f'to {self.goal_text()}: {reason}, {message}')
            return
        ok, error = self.send(plan)
        if not ok:
            self.set_state('failed', f'to {self.goal_text()}: {error}')
            return
        self.set_state('following', f'to {self.goal_text()}: replanned, {message}')

    def monitor(self):
        """Follow the follower: arrival, blockage, being stopped; publish the status."""
        if self._state not in ACTIVE or self._nav is None:
            if time.monotonic() - self._last_status > 5.0:
                self.publish_status()
            return
        state, message, received = self._nav
        now = time.monotonic()
        if received < self._set_time:
            return  # nothing from the follower since the path was loaded
        if state == 'GOAL_REACHED':
            pose = self.robot_pose()
            _, gx, gy, _ = self._goal
            near = pose is not None and math.hypot(pose[0] - gx, pose[1] - gy) <= float(
                self.param('arrival_tolerance')
            )
            if near:
                if bool(self.param('stop_on_arrival')):
                    self.stop_navigation()
                self.set_state('arrived', f'at {self.goal_text()}')
                return
        if state == 'BLOCKED':
            self._blocked_since = self._blocked_since or now
            if now - self._blocked_since >= float(self.param('replan_after_sec')):
                self._blocked_since = None
                self.replan('blocked')
                return
        else:
            self._blocked_since = None
        if state == 'OFF_PATH':
            self.replan('off the path')
            return
        if state == 'NAV_INACTIVE':
            self._inactive_since = self._inactive_since or now
            if now - self._inactive_since > 1.0:
                self.set_state('cancelled', f'to {self.goal_text()}: navigation was stopped')
                return
        else:
            self._inactive_since = None
        if now - self._last_status >= 1.0 / max(float(self.param('status_hz')), 0.1):
            self.publish_status()


def main(args=None):
    """Spin the planner; service calls to the follower need the extra threads."""
    rclpy.init(args=args)
    node = PlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
