#!/usr/bin/env python3
import math
import threading

from geometry_msgs.msg import Twist
from molly_interfaces.msg import (
    AvoidanceCommand,
    MissionStatus,
    NavigationStatus,
    SectorScan,
    StopPointEvent,
    WaypointPath,
)
from molly_navigation.utils import visualization
from molly_navigation.utils.command_filter import LowPassFilter, SlewLimiter
from molly_navigation.utils.controllers.holonomic_controller import HolonomicController
from molly_navigation.utils.controllers.registry import create_controller
from molly_navigation.utils.final_approach import FinalApproach
from molly_navigation.utils.geometry import clamp, yaw_from_quaternion
from molly_navigation.utils.path_processing import velocity_profile
from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.path_shift import PathShift, PathShiftConfig
from molly_navigation.utils.pose2d import Pose2D
from molly_navigation.utils.safety import SafetySupervisor
from molly_navigation.utils.speed_regulator import SpeedRegulator
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

DEFAULT_PARAMS = {
    'control_rate': 20.0,
    'target_linear_velocity': 0.4,
    'min_linear_velocity': 0.1,
    'max_lateral_accel': 0.3,
    'decel_limit': 0.3,
    'approach_speed': 0.15,
    'max_linear_velocity': 0.6,
    'max_angular_velocity': 1.0,
    'linear_accel_limit': 0.5,
    'angular_accel_limit': 1.5,
    'command_smoothing': 0.4,
    'arc_visualization_length': 2.0,
    'marker_lifetime_sec': 0.5,
    'avoidance_enabled': True,
    'avoidance_timeout_sec': 0.5,
    'estop_release_sec': 1.0,
    'blocked_timeout_sec': 30.0,
    'off_path_slow_distance': 0.6,
    'off_path_stop_distance': 1.5,
    'tf_timeout_sec': 0.5,
    'loop_route': True,
    'loop_close_distance': 1.0,
    'low_battery_percentage': 0.2,
    'holonomic': True,
    'avoidance_lateral_enabled': True,
    'approach_radius': 0.5,
    'approach_timeout_sec': 20.0,
    'join_max_distance': 3.0,
    'join_timeout_sec': 60.0,
    'approach.kp_xy': 1.2,
    'approach.kp_yaw': 1.5,
    'approach.max_speed': 0.2,
    'approach.max_yaw_rate': 0.6,
    'approach.min_speed': 0.04,
    'approach.position_tolerance': 0.05,
    'approach.yaw_tolerance_deg': 3.0,
    'stop_point_skip_margin': 5,
    'auto_mode_on_service': '',
    'auto_mode_off_service': '',
    'inspection_services': [''],
    'inspection_timeout_sec': 10.0,
    'require_localization': True,
    'controller': 'holonomic',
    'avoidance_mode': 'path_shift',
    'footprint.front': 0.40,
    'footprint.rear': 0.40,
    'footprint.half_width': 0.28,
    'path_shift.max_offset': 0.8,
    'path_shift.look_ahead': 2.0,
    'path_shift.clearance': 0.10,
    'path_shift.offset_rate': 0.3,
    'path_shift.switch_cost': 0.5,
    'path_shift.switch_margin': 0.05,
    'path_shift.return_delay_sec': 0.5,
    'path_shift.decel': 0.5,
    'path_shift.stop_margin': 0.40,
    'speed_regulator_enabled': False,
    'speed_regulator.kp': 0.3,
    'speed_regulator.ki': 0.0,
    'speed_regulator.kd': 0.0,
    'speed_regulator.min_scale': 0.3,
}

AVOIDANCE_MODES = ('path_shift', 'reactive')

CONTROLLER_PARAMS = {
    'holonomic': {
        'lookahead_distance': 0.6,
        'heading_lookahead': 0.6,
        'goal_tolerance': 0.3,
        'kp_yaw': 1.5,
        'max_lateral_velocity': 0.25,
        'max_reverse_velocity': 0.15,
        'min_alignment_scale': 0.25,
    },
    'pure_pursuit': {
        'lookahead_distance': 1.0,
        'goal_tolerance': 0.3,
    },
    'pid': {
        'lookahead_distance': 1.0,
        'goal_tolerance': 0.3,
        'kp': 1.5,
        'ki': 0.0,
        'kd': 0.2,
    },
    'mppi': {
        'goal_tolerance': 0.3,
        'horizon_steps': 15,
        'dt': 0.1,
        'num_samples': 200,
        'angular_std': 1.0,
        'temperature': 0.05,
        'window_points': 60,
    },
    'lqr': {
        'goal_tolerance': 0.3,
        'q_lateral': 1.0,
        'q_heading': 0.5,
        'r_angular': 0.5,
    },
    'stanley': {
        'goal_tolerance': 0.3,
        'k_cross_track': 1.0,
        'k_soft': 0.2,
        'k_heading': 1.5,
    },
}

STARTUP_ONLY_PARAMS = {
    'control_rate',
    'auto_mode_on_service',
    'auto_mode_off_service',
    'inspection_services',
}


class PathFollowerNode(Node):
    """
    Patrol a taught route with inspection stop points.

    Follows the route with a selectable controller and at each stop point converges onto x/y/yaw,
    triggers the inspection services and dwells. With the holonomic controller (legged robots) it
    walks forward, sideways or backwards as the route needs and, in path_shift avoidance, slides
    sideways around obstacles within path_shift.max_offset instead of steering away; other
    controllers blend the reactive avoidance node's steering.
    """

    _WARN_STATES = {
        'NO_PATH',
        'TF_UNAVAILABLE',
        'EMERGENCY_STOP',
        'AVOIDANCE_STALE',
        'BLOCKED',
        'OFF_PATH',
    }

    def __init__(self):
        """Declare params, set up TF, build the controller, and wire all interfaces."""
        super().__init__('path_follower_node')
        self.declare_params()
        p = self.get_parameter

        self._control_rate = float(p('control_rate').value)
        self._controller = None
        self._controller_name = ''
        self._approach = None
        self._linear_limiter = self._angular_limiter = self._lateral_limiter = None
        self._angular_filter = self._lateral_filter = None
        self._safety = None
        self._speed_regulator = None
        self._pending_params: set[str] = set()
        self._pending_lock = threading.Lock()

        self._avoidance = AvoidanceCommand(velocity_scale=1.0)
        self._avoidance_time = None
        self._scan_points = np.empty((0, 2))   # safety scan obstacles in base_link
        self._scan_time = None
        self._path_offset = 0.0                # applied lateral offset from the route (+left)
        self._shift_target = 0.0
        self._shift_free = math.inf
        self._return_since = None              # when a smaller offset was first proposed
        self._nav_active = False
        self._paused = False
        self._battery_low = False
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._speed_profile: list[float] = []
        self._next_stop_index = None
        self._follower_state = 'FOLLOWING'
        self._approach_start_time = None
        self._approach_index = 0
        self._approach_is_goal = False
        self._join_index = 0
        self._join_start_time = None
        self._waypoints_stamp = None
        self._dwell_duration = 0.0
        self._dwell_start_time = None
        self._inspection_pending = 0
        self._inspection_results: list[str] = []
        self._inspection_index = None
        self._inspection_round = 0
        self._lap = 0
        self._last_status = 'NAV_INACTIVE'

        self.load_tunables()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._srv_group = ReentrantCallbackGroup()
        self._loop_group = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            WaypointPath,
            'navigation/waypoints',
            self.waypoints_callback,
            latched,
            callback_group=self._loop_group,
        )
        self.create_subscription(
            AvoidanceCommand,
            'navigation/avoidance',
            self.avoidance_callback,
            10,
            callback_group=self._loop_group,
        )
        self.create_subscription(
            SectorScan,
            'perception/safety_scan',
            self.safety_scan_callback,
            10,
            callback_group=self._loop_group,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._markers_pub = self.create_publisher(MarkerArray, 'navigation/markers', 10)
        self._route_markers_pub = self.create_publisher(
            MarkerArray, 'navigation/route_markers', latched
        )
        self._route_markers_key = None
        self._status_pub = self.create_publisher(NavigationStatus, 'navigation/status', 10)
        self._stop_event_pub = self.create_publisher(
            StopPointEvent, 'navigation/stop_point_event', 10
        )

        self._auto_on_client = self.optional_client(p('auto_mode_on_service').value)
        self._auto_off_client = self.optional_client(p('auto_mode_off_service').value)
        self._inspection_clients = [
            self.create_client(Trigger, name, callback_group=self._srv_group)
            for name in p('inspection_services').value
            if name
        ]
        self.create_service(
            SetBool,
            'navigation/start_nav',
            self.start_nav_callback,
            callback_group=self._srv_group,
        )
        self.create_service(
            SetBool, 'navigation/pause', self.pause_callback, callback_group=self._srv_group
        )
        self.create_subscription(
            BatteryState, 'battery', self.battery_callback, 10, callback_group=self._loop_group
        )
        self._mission_pub = self.create_publisher(MissionStatus, 'navigation/mission_status', 10)
        self._localization_source = 'none'
        self.create_subscription(
            String,
            'localization/status',
            self.localization_status_callback,
            latched,
            callback_group=self._loop_group,
        )
        self.create_timer(1.0, self.publish_mission_status, callback_group=self._loop_group)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

        self._last_time = self.get_clock().now()
        self.create_timer(
            1.0 / self._control_rate, self.control_loop, callback_group=self._loop_group
        )

    def declare_params(self):
        """Declare the general parameters and every controller's namespaced parameters."""
        for name, value in DEFAULT_PARAMS.items():
            self.declare_parameter(name, value)
        for controller, params in CONTROLLER_PARAMS.items():
            for name, value in params.items():
                self.declare_parameter(f'{controller}.{name}', value)

    def optional_client(self, name: str):
        """Create a Trigger client, or None when the service name is empty."""
        return self.create_client(Trigger, name, callback_group=self._srv_group) if name else None

    def build_controller_params(self, name: str) -> dict:
        """Build constructor kwargs for a controller from its namespaced parameters."""
        if name not in CONTROLLER_PARAMS:
            raise ValueError(f'unknown controller {name!r}, options are {list(CONTROLLER_PARAMS)}')
        params = {
            key: type(default)(self.get_parameter(f'{name}.{key}').value)
            for key, default in CONTROLLER_PARAMS[name].items()
        }
        if name in ('holonomic', 'mppi', 'lqr', 'stanley'):
            params['max_angular_velocity'] = self._max_angular_velocity
        if name == 'holonomic' and not self._holonomic:
            params['max_lateral_velocity'] = 0.0  # the robot cannot strafe
        if name == 'lqr':
            params['dt'] = 1.0 / self._control_rate
        return params

    def load_tunables(self, changed: set | None = None):
        """(Re)read runtime-tunable parameters; rebuild only the parts whose parameters changed."""
        p = self.get_parameter
        everything = changed is None
        changed = changed or set()

        def touched(*prefixes):
            return everything or any(n.startswith(prefixes) for n in changed)

        self._target_linear_velocity = float(p('target_linear_velocity').value)
        self._min_linear_velocity = float(p('min_linear_velocity').value)
        self._max_lateral_accel = float(p('max_lateral_accel').value)
        self._decel_limit = float(p('decel_limit').value)
        self._approach_speed = float(p('approach_speed').value)
        self._max_linear_velocity = float(p('max_linear_velocity').value)
        self._max_angular_velocity = float(p('max_angular_velocity').value)
        self._arc_length = float(p('arc_visualization_length').value)
        self._marker_lifetime = Duration(seconds=float(p('marker_lifetime_sec').value)).to_msg()
        self._avoidance_enabled = bool(p('avoidance_enabled').value)
        self._avoidance_lateral = bool(p('avoidance_lateral_enabled').value)
        self._speed_regulator_enabled = bool(p('speed_regulator_enabled').value)
        self._avoidance_timeout = Duration(seconds=float(p('avoidance_timeout_sec').value))
        self._tf_timeout = Duration(seconds=float(p('tf_timeout_sec').value))
        self._loop_route = bool(p('loop_route').value)
        self._loop_close_distance = float(p('loop_close_distance').value)
        self._low_battery = float(p('low_battery_percentage').value)
        self._approach_radius = float(p('approach_radius').value)
        self._approach_timeout = float(p('approach_timeout_sec').value)
        self._approach_tolerance = float(p('approach.position_tolerance').value)
        self._holonomic = bool(p('holonomic').value)
        self._stop_point_skip_margin = int(p('stop_point_skip_margin').value)
        self._inspection_timeout = float(p('inspection_timeout_sec').value)
        self._require_localization = bool(p('require_localization').value)
        self._avoidance_mode = str(p('avoidance_mode').value)
        self._offset_rate = float(p('path_shift.offset_rate').value)
        self._return_delay = float(p('path_shift.return_delay_sec').value)

        if touched('approach.', 'holonomic'):
            self._approach = FinalApproach(
                holonomic=self._holonomic,
                kp_xy=float(p('approach.kp_xy').value),
                kp_yaw=float(p('approach.kp_yaw').value),
                max_speed=float(p('approach.max_speed').value),
                max_yaw_rate=float(p('approach.max_yaw_rate').value),
                min_speed=float(p('approach.min_speed').value),
                position_tolerance=float(p('approach.position_tolerance').value),
                yaw_tolerance=math.radians(float(p('approach.yaw_tolerance_deg').value)),
            )
        if touched('linear_accel_limit', 'angular_accel_limit'):
            self._linear_limiter = self.carry(
                self._linear_limiter, SlewLimiter(float(p('linear_accel_limit').value))
            )
            self._lateral_limiter = self.carry(
                self._lateral_limiter, SlewLimiter(float(p('linear_accel_limit').value))
            )
            self._angular_limiter = self.carry(
                self._angular_limiter, SlewLimiter(float(p('angular_accel_limit').value))
            )
        if touched('command_smoothing'):
            alpha = float(p('command_smoothing').value)
            self._angular_filter = self.carry(self._angular_filter, LowPassFilter(alpha))
            self._lateral_filter = self.carry(self._lateral_filter, LowPassFilter(alpha))
        if touched('estop_release_sec', 'blocked_timeout_sec', 'off_path_'):
            self._safety = SafetySupervisor(
                estop_release_sec=float(p('estop_release_sec').value),
                blocked_timeout_sec=float(p('blocked_timeout_sec').value),
                off_path_slow_distance=float(p('off_path_slow_distance').value),
                off_path_stop_distance=float(p('off_path_stop_distance').value),
            )
        if touched('path_shift.', 'footprint.'):
            self._path_shift = PathShift(
                PathShiftConfig(
                    max_offset=float(p('path_shift.max_offset').value),
                    look_ahead=float(p('path_shift.look_ahead').value),
                    front=float(p('footprint.front').value),
                    rear=float(p('footprint.rear').value),
                    half_width=float(p('footprint.half_width').value),
                    clearance=float(p('path_shift.clearance').value),
                    switch_cost=float(p('path_shift.switch_cost').value),
                    switch_margin=float(p('path_shift.switch_margin').value),
                    decel=float(p('path_shift.decel').value),
                    stop_margin=float(p('path_shift.stop_margin').value),
                )
            )
        if touched('speed_regulator.'):
            self._speed_regulator = SpeedRegulator(
                kp=float(p('speed_regulator.kp').value),
                ki=float(p('speed_regulator.ki').value),
                kd=float(p('speed_regulator.kd').value),
                min_scale=float(p('speed_regulator.min_scale').value),
            )
        name = p('controller').value
        if touched('controller', 'max_angular_velocity', 'holonomic', f'{name}.'):
            self.build_controller(name)
        if touched(
            'target_linear_velocity',
            'min_linear_velocity',
            'max_lateral_accel',
            'decel_limit',
            'approach_speed',
        ):
            self.compute_speed_profile()
        if not everything:
            self.get_logger().info(f'applied parameters: {", ".join(sorted(changed))}')

    @staticmethod
    def carry(old, new):
        """Return new, starting from the old filter/limiter output so commands do not jump."""
        if old is not None:
            new.reset(old.value)
        return new

    def build_controller(self, name: str):
        """Instantiate a controller, keeping the current route and progress."""
        progress = self._controller.progress_index() if self._controller else None
        self._controller = create_controller(name, self.build_controller_params(name))
        if self._waypoints:
            self._controller.set_path([(x, y) for x, y, _, _ in self._waypoints], progress)
        if name != self._controller_name and self._controller_name:
            self.get_logger().info(f'switched controller to {name}')
        self._controller_name = name
        self.reset_path_shift()

    def compute_speed_profile(self):
        """Recompute the per-point speed limits for the current route."""
        self._speed_profile = velocity_profile(
            [(w[0], w[1]) for w in self._waypoints],
            [i for i, w in enumerate(self._waypoints) if w[3] > 0.0],
            cruise=self._target_linear_velocity,
            min_speed=self._min_linear_velocity,
            max_lateral_accel=self._max_lateral_accel,
            decel=self._decel_limit,
            approach_speed=self._approach_speed,
        )

    def on_parameters_changed(self, params):
        """Validate a parameter change; it is applied on the next control tick."""
        for param in params:
            if param.name in STARTUP_ONLY_PARAMS:
                return SetParametersResult(
                    successful=False, reason=f'{param.name} is read at startup, restart the node'
                )
            if param.name == 'controller' and param.value not in CONTROLLER_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f'unknown controller {param.value!r}, use {list(CONTROLLER_PARAMS)}',
                )
            if param.name == 'avoidance_mode' and param.value not in AVOIDANCE_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=f'unknown avoidance_mode {param.value!r}, use {list(AVOIDANCE_MODES)}',
                )
        with self._pending_lock:
            self._pending_params.update(param.name for param in params)
        return SetParametersResult(successful=True)

    def find_next_stop_index(self, from_index: int):
        """Index of the next waypoint with dwell_sec > 0 at or after from_index, or None."""
        for i in range(from_index, len(self._waypoints)):
            if self._waypoints[i][3] > 0.0:
                return i
        return None

    def restart_route(self, from_start: bool = False):
        """Reset controller progress and the stop-point state machine to the start of the route."""
        self._controller.set_path(
            [(x, y) for x, y, _, _ in self._waypoints], 0 if from_start else None
        )
        self._speed_regulator.reset()
        self.reset_path_shift()
        self._follower_state = 'FOLLOWING'
        self._next_stop_index = self.find_next_stop_index(0)

    def find_robot_on_route(self) -> tuple[bool, str]:
        """
        On start: keep the progress if the robot is still near it, else search the whole route.

        Progress survives a stop so a paused patrol resumes where it was; after the route end or
        once the robot was moved elsewhere the old progress would report the route as finished or
        the robot as far off it, so the robot is located on the route again. A robot up to
        join_max_distance away first walks onto the route (JOINING); further away the start is
        refused. Returns (start allowed, note).
        """
        if not self._waypoints:
            return True, ''
        pose = self.current_pose()
        if pose is None:
            return True, ''
        held = self._follower_state == 'GOAL_HOLD'
        near = float(self.get_parameter('off_path_slow_distance').value)
        if not held and self.cross_track_error(pose) <= near:
            return True, ''
        path = [(x, y) for x, y, _, _ in self._waypoints]
        locator = PathProgress()
        locator.set_path(path)
        index = locator.update(pose)
        distance = math.hypot(path[index][0] - pose.x, path[index][1] - pose.y)
        join_max = float(self.get_parameter('join_max_distance').value)
        stop = float(self.get_parameter('off_path_stop_distance').value)
        if distance > max(join_max, stop):
            return False, (
                f'robot is {distance:.1f} m from the route (join_max_distance '
                f'{join_max:.1f} m): drive it closer or load the route it is on'
            )
        self.restart_route()
        self._controller.set_path(path, index)
        self._next_stop_index = self.find_next_stop_index(index)
        self.get_logger().info(f'robot on the route at point {index}, {distance:.2f} m away')
        if distance > near and join_max > 0.0:
            self.start_join(index)
            return True, f'joining the route at point {index}, {distance:.1f} m away'
        return True, f'progress found again on the route at point {index}'

    def waypoints_callback(self, msg: WaypointPath):
        """
        Load a new route; a redelivered one (same stamp) is ignored so progress is kept.

        Loading the same file again publishes it with a new stamp and starts it over.
        """
        waypoints = [(w.x, w.y, w.yaw, w.dwell_sec) for w in msg.waypoints]
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if waypoints == self._waypoints and stamp == self._waypoints_stamp:
            return
        self._waypoints = waypoints
        self._waypoints_stamp = stamp
        self.compute_speed_profile()
        self._lap = 0
        self.restart_route()
        stops = sum(1 for w in waypoints if w[3] > 0.0)
        self.get_logger().info(f'route loaded: {len(waypoints)} waypoints, {stops} stop points')

    def avoidance_callback(self, msg: AvoidanceCommand):
        """Cache the latest avoidance signal and its arrival time."""
        self._avoidance = msg
        self._avoidance_time = self.get_clock().now()

    def safety_scan_callback(self, msg: SectorScan):
        """
        Keep the obstacles of the 360 deg safety scan as points in base_link.

        A sector only says something lies within its angle at that range, so it becomes three
        points (centre and both edges): an obstacle is never thinner than its sectors.
        """
        ranges = np.asarray(msg.ranges, dtype=float)
        start = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        hit = np.isfinite(ranges) & (ranges < msg.range_max - 1e-3)
        r = np.repeat(ranges[hit], 3)
        a = (start[hit][:, None] + msg.angle_increment * np.array([0.0, 0.5, 1.0])).ravel()
        self._scan_points = np.column_stack([r * np.cos(a), r * np.sin(a)])
        self._scan_time = self.get_clock().now()

    def reset_path_shift(self):
        """Back onto the route itself (new route, lap, stop point or controller)."""
        self._path_offset = self._shift_target = 0.0
        self._shift_free = math.inf
        self._return_since = None
        if isinstance(self._controller, HolonomicController):
            self._controller.set_offset(0.0)

    def path_shifting(self) -> bool:
        """Return True when obstacles are passed by sliding sideways (holonomic controller)."""
        return (
            self._avoidance_enabled
            and self._avoidance_mode == 'path_shift'
            and isinstance(self._controller, HolonomicController)
        )

    def update_path_shift(self, pose: Pose2D, dt: float):
        """
        Slide toward the offset that passes obstacles; return the speed limit, or None if stale.

        The offset changes at most path_shift.offset_rate per second. The speed is limited so the
        robot can stop before the first obstacle both where it is and where it is going.
        """
        if (
            self._scan_time is None
            or self.get_clock().now() - self._scan_time > self._avoidance_timeout
        ):
            return None
        progress = self._controller.path_progress()
        s_robot = progress.projection(pose)
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        local = self._scan_points
        points = np.empty((0, 2))
        if local.size:
            points = np.column_stack(
                [
                    pose.x + c * local[:, 0] - s * local[:, 1],
                    pose.y + s * local[:, 0] + c * local[:, 1],
                ]
            )
        shift = self._path_shift
        s_pts, d_pts = shift.path_coordinates(progress.path, progress.arc_lengths, s_robot, points)
        self.set_shift_target(
            shift.choose(s_pts, d_pts, s_robot, self._shift_target).offset, pose
        )
        target = self._shift_target
        self._path_offset += clamp(target - self._path_offset, self._offset_rate * dt)
        self._controller.set_offset(self._path_offset)
        self._shift_free = min(
            shift.free_at(s_pts, d_pts, s_robot, target),
            shift.free_at(s_pts, d_pts, s_robot, self._path_offset),
        )
        return shift.speed_limit(self._shift_free)

    def call_trigger(self, client, timeout: float = 2.0):
        """Call a Trigger service and wait for it (requires the multi-threaded executor)."""
        if client is None:
            return True, 'skipped'
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'{client.srv_name} not available'
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout) or future.result() is None:
            return False, f'{client.srv_name} timed out'
        return future.result().success, future.result().message

    def start_nav_callback(self, request, response):
        """Enable or disable navigation, switching the robot's control mode along with it."""
        if request.data:
            if self._require_localization and self._localization_source == 'none':
                response.success = False
                response.message = (
                    "not localized: call 'localization/start' or set an RViz 2D Pose Estimate"
                )
                self.get_logger().error(response.message)
                return response
            located, note = self.find_robot_on_route()
            if not located:
                response.success = False
                response.message = note
                self.get_logger().error(note)
                return response
            ok, message = self.call_trigger(self._auto_on_client)
            if not ok:
                response.success = False
                response.message = f'failed to enable auto mode: {message}'
                self.get_logger().error(response.message)
                return response
            self._safety.reset()
            self._paused = False
            self._nav_active = True
            response.success = True
            response.message = 'navigation started' + (f' ({note})' if note else '')
            self.get_logger().info(response.message)
            return response

        self._nav_active = False
        self.publish_stop(hard=True)
        ok, message = self.call_trigger(self._auto_off_client)
        if not ok:
            self.get_logger().error(f'failed to switch back to manual mode: {message}')
        response.success = True
        response.message = 'navigation stopped'
        self.get_logger().info(response.message)
        return response

    def localization_status_callback(self, msg: String):
        """Track which source set map->odom ('none' until localized)."""
        if msg.data != self._localization_source:
            self.get_logger().info(f'localization source: {msg.data}')
        self._localization_source = msg.data
        lost = msg.data == 'none' and self._require_localization
        if lost and self._nav_active and not self._paused:
            # e.g. FAST-LIO restarted: the route is now in a different frame than the robot
            self._paused = True
            self.publish_stop(hard=True)
            self.get_logger().error('localization lost, navigation paused until relocalized')

    def pause_callback(self, request, response):
        """Pause (hold position, keep progress) or resume navigation."""
        self._paused = bool(request.data)
        if self._paused:
            self.publish_stop(hard=True)
        response.success = True
        response.message = 'paused' if self._paused else 'resumed'
        self.get_logger().info(f'navigation {response.message}')
        return response

    def battery_callback(self, msg: BatteryState):
        """
        Flag low battery so the patrol ends after this lap; clear it again once charged.

        Unknown readings (NaN, outside 0..1) are ignored. The flag clears only 5 % above the
        threshold so a sagging battery does not toggle it.
        """
        pct = msg.percentage
        if not 0.0 <= pct <= 1.0:
            return
        if pct < self._low_battery and not self._battery_low:
            self._battery_low = True
            self.get_logger().warn(f'battery {pct * 100:.0f}%, finishing this lap and stopping')
        elif self._battery_low and pct >= self._low_battery + 0.05:
            self._battery_low = False
            self.get_logger().info(f'battery {pct * 100:.0f}%, patrol may loop again')

    def publish_mission_status(self):
        """Publish a 1 Hz mission summary for dashboards."""
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = self._nav_active and not self._paused
        msg.root_status = self._last_status
        stops = sum(1 for w in self._waypoints if w[3] > 0.0)
        flags = ' | battery low' if self._battery_low else ''
        msg.active_behavior = (
            f'lap {self._lap} | next stop {self._next_stop_index} of {stops} | '
            f'{self._follower_state}{flags}'
        )
        try:
            self._mission_pub.publish(msg)
        except Exception:  # noqa: B902  shutdown took the context away
            if rclpy.ok():
                raise

    def current_pose(self):
        """Look up map->base_link; None if unavailable or older than tf_timeout_sec."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException as error:
            self.get_logger().warn(f'TF map->base_link failed: {error}', throttle_duration_sec=2.0)
            return None
        stamp = Time.from_msg(t.header.stamp)
        if stamp.nanoseconds > 0 and self.get_clock().now() - stamp > self._tf_timeout:
            self.get_logger().warn(
                'TF map->base_link is stale (odometry stopped?)', throttle_duration_sec=2.0
            )
            return None
        p = t.transform.translation
        return Pose2D(p.x, p.y, yaw_from_quaternion(t.transform.rotation))

    def avoidance_signal(self):
        """Return (steering, lateral, velocity_scale, emergency, stale) from avoidance."""
        if not self._avoidance_enabled:
            return 0.0, 0.0, 1.0, False, False
        stale = (
            self._avoidance_time is None
            or self.get_clock().now() - self._avoidance_time > self._avoidance_timeout
        )
        a = self._avoidance
        lateral = a.lateral_bias if self._holonomic and self._avoidance_lateral else 0.0
        return a.steering_bias, lateral, a.velocity_scale, a.emergency, stale

    def publish_status(self, stamp, status, message):
        """Publish NavigationStatus, logging once per state transition."""
        if status != self._last_status:
            # one call site per severity: rclpy raises if a call site changes its severity
            if status in self._WARN_STATES:
                self.get_logger().warn(f'[{status}] {message}')
            else:
                self.get_logger().info(f'[{status}] {message}')
        self._last_status = status
        msg = NavigationStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.state = status
        msg.message = message
        self._status_pub.publish(msg)

    def publish_markers(self, stamp, status, message, pose_x, pose_y, debug=None):
        """Publish the status and the per-tick debug drawing (no status text)."""
        self.publish_status(stamp, status, message)
        debug = debug or {}
        v = visualization
        ns, items = 'follower', []
        color = v.state_color(status)
        if status not in ('NAV_INACTIVE', 'NO_PATH', 'TF_UNAVAILABLE'):
            items.append(v.disc('map', stamp, ns, 0, pose_x, pose_y, 0.35, color, alpha=0.25))
            items.append(v.circle('map', stamp, ns, 1, pose_x, pose_y, 0.35, color, width=0.03))
        lookahead = debug.get('lookahead_xy')
        if lookahead is not None:
            lx, ly = lookahead
            items.append(
                v.line('map', stamp, ns, 2, [(pose_x, pose_y), (lx, ly)], (0.1, 0.9, 1.0))
            )
            items.append(v.sphere('map', stamp, ns, 3, lx, ly, 0.16, (0.1, 0.9, 1.0)))
            if debug.get('lookahead_distance'):
                items.append(
                    v.circle(
                        'map',
                        stamp,
                        ns,
                        4,
                        pose_x,
                        pose_y,
                        debug['lookahead_distance'],
                        (0.1, 0.9, 1.0),
                        width=0.01,
                        alpha=0.35,
                    )
                )
        nearest = debug.get('nearest_xy')
        if nearest is not None:
            nx, ny = nearest
            items.append(v.sphere('map', stamp, ns, 5, nx, ny, 0.1, (1.0, 0.6, 0.1)))
            items.append(
                v.line(
                    'map',
                    stamp,
                    ns,
                    6,
                    [(pose_x, pose_y), (nx, ny)],
                    visualization.ALARM,
                    width=0.015,
                    alpha=0.7,
                )
            )
        if debug.get('curvature') is not None:
            items.append(
                v.arc_from_curvature(
                    'base_link',
                    stamp,
                    ns,
                    7,
                    debug['curvature'],
                    self._arc_length,
                    (0.3, 0.6, 1.0),
                )
            )
        if debug.get('rollout_xy') is not None:
            items.append(
                v.line(
                    'map',
                    stamp,
                    ns,
                    8,
                    debug['rollout_xy'],
                    (1.0, 0.4, 0.9),
                    width=0.02,
                    alpha=0.8,
                )
            )
        target = debug.get('approach')
        if target is not None:
            tx, ty, tyaw, tolerance, progress = target
            items.append(
                v.disc(
                    'map', stamp, ns, 9, tx, ty, max(tolerance, 0.03), (1.0, 0.85, 0.1), alpha=0.5
                )
            )
            items.append(v.arrow('map', stamp, ns, 10, tx, ty, tyaw, 0.4, (1.0, 0.85, 0.1)))
            items.append(
                v.line(
                    'map',
                    stamp,
                    ns,
                    11,
                    [(pose_x, pose_y), (tx, ty)],
                    (1.0, 0.85, 0.1),
                    width=0.015,
                    alpha=0.6,
                )
            )
            if progress > 0.0:
                items.append(
                    v.circle(
                        'map',
                        stamp,
                        ns,
                        12,
                        tx,
                        ty,
                        0.3,
                        (0.2, 0.6, 1.0),
                        width=0.05,
                        start=math.pi / 2,
                        sweep=-2 * math.pi * min(progress, 1.0),
                    )
                )
        markers = MarkerArray()
        for m in items:
            m.lifetime = self._marker_lifetime
            markers.markers.append(m)
        self._markers_pub.publish(markers)

    def publish_route_markers(self):
        """Publish the latched route drawing: speed-coloured path, start/end and stop points."""
        key = (
            len(self._waypoints),
            self._next_stop_index,
            self._lap,
            self._target_linear_velocity,
        )
        if key == self._route_markers_key:
            return
        self._route_markers_key = key
        v, stamp, ns = visualization, self.get_clock().now().to_msg(), 'route'
        markers = MarkerArray()
        markers.markers.append(v.delete_all('map', stamp))
        if self._waypoints:
            path = v.base('map', stamp, ns, 0, v.Marker.LINE_STRIP)
            path.scale.x = 0.06
            vmax = max(self._speed_profile) if self._speed_profile else 1.0
            for i, w in enumerate(self._waypoints):
                path.points.append(v.pt(w[0], w[1], 0.01))
                speed = self._speed_profile[i] if i < len(self._speed_profile) else vmax
                path.colors.append(v.speed_color(speed, vmax))
            markers.markers.append(path)
            sx, sy = self._waypoints[0][:2]
            ex, ey = self._waypoints[-1][:2]
            markers.markers.append(v.sphere('map', stamp, ns, 1, sx, sy, 0.25, (0.2, 0.9, 0.3)))
            markers.markers.append(v.sphere('map', stamp, ns, 2, ex, ey, 0.2, (0.9, 0.2, 0.2)))
            stops = [i for i, w in enumerate(self._waypoints) if w[3] > 0.0]
            for n, i in enumerate(stops):
                x, y, yaw_deg, dwell = self._waypoints[i]
                nxt = self._next_stop_index
                rgb = (
                    (0.5, 0.5, 0.5)
                    if nxt is not None and i < nxt
                    else (1.0, 0.85, 0.1) if i == nxt else (0.3, 0.6, 1.0)
                )
                markers.markers.append(v.disc('map', stamp, ns, 10 + 3 * n, x, y, 0.25, rgb, 0.6))
                markers.markers.append(
                    v.arrow('map', stamp, ns, 11 + 3 * n, x, y, math.radians(yaw_deg), 0.45, rgb)
                )
                markers.markers.append(
                    v.label('map', stamp, ns, 12 + 3 * n, x, y, f'S{n + 1} {dwell:.0f}s', rgb)
                )
        self._route_markers_pub.publish(markers)

    def publish_stop_event(self, event: str, message: str = '', index=None):
        """Publish a StopPointEvent for the current (or the given) stop point."""
        index = self._next_stop_index if index is None else index
        if index is None or index >= len(self._waypoints):
            return
        x, y, yaw_deg, dwell = self._waypoints[index]
        msg = StopPointEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.index = index
        msg.x, msg.y, msg.yaw_deg, msg.dwell_sec = float(x), float(y), float(yaw_deg), float(dwell)
        msg.event = event
        msg.message = message
        self._stop_event_pub.publish(msg)

    def start_inspection(self):
        """Fire every inspection Trigger service asynchronously for the current stop."""
        self._inspection_round += 1
        self._inspection_index = self._next_stop_index
        self._inspection_results = []
        self._inspection_pending = 0
        for client in self._inspection_clients:
            if not client.service_is_ready():
                self._inspection_results.append(f'{client.srv_name}: not available')
                continue
            self._inspection_pending += 1
            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda f, name=client.srv_name, r=self._inspection_round: self.inspection_done(
                    name, f, r
                )
            )
        if self._inspection_results and not self._inspection_pending:
            # every inspector is down: report that now, the stop is not silently uninspected
            self.publish_stop_event(
                'inspected', ' | '.join(self._inspection_results), self._inspection_index
            )

    def inspection_done(self, name: str, future, round_id: int):
        """Collect one inspection result; answers for an earlier stop are dropped."""
        if round_id != self._inspection_round:
            self.get_logger().warn(f'{name} answered after the robot left the stop, ignored')
            return
        try:
            result = future.result()
        except Exception as error:  # noqa: B902  a failed service is a failed inspection
            result, error_text = None, str(error)
        else:
            error_text = ''
        verdict = 'ok' if result and result.success else 'fail'
        detail = result.message if result else error_text
        self._inspection_results.append(f'{name}: {verdict} {detail}'.strip())
        self._inspection_pending = max(0, self._inspection_pending - 1)
        if self._inspection_pending == 0:
            self.publish_stop_event(
                'inspected', ' | '.join(self._inspection_results), self._inspection_index
            )

    def run_approaching(self, pose: Pose2D, stamp):
        """Converge onto the target x, y, yaw, then dwell (stop point) or hold (route end)."""
        x, y, yaw_deg, _ = self._waypoints[self._approach_index]
        cmd = self._approach.update(pose, x, y, math.radians(yaw_deg))
        elapsed = (self.get_clock().now() - self._approach_start_time).nanoseconds / 1e9
        if cmd.done or elapsed > self._approach_timeout:
            self.publish_stop(hard=True)
            note = '' if cmd.done else f'approach timeout, error {cmd.position_error:.2f} m'
            if note:
                self.get_logger().warn(note)
            if self._approach_is_goal:
                self._follower_state = 'GOAL_HOLD'
                return
            self._follower_state = 'DWELLING'
            self._dwell_start_time = self.get_clock().now()
            self.publish_stop_event('arrived', note)
            self.start_inspection()
            self.publish_markers(
                stamp, 'DWELLING', f'stop {self._next_stop_index}, inspecting', pose.x, pose.y
            )
            return

        if self.stopped_for_obstacle(pose, stamp, 'final approach'):
            return

        target = 'goal' if self._approach_is_goal else f'stop {self._approach_index}'
        self.publish_approach(cmd)
        self.publish_markers(
            stamp,
            'APPROACHING',
            f'{target}: {cmd.position_error * 100:.1f} cm, '
            f'{math.degrees(cmd.yaw_error):.1f} deg',
            pose.x,
            pose.y,
            {'approach': (x, y, math.radians(yaw_deg), self._approach_tolerance, 0.0)},
        )

    def stopped_for_obstacle(self, pose: Pose2D, stamp, during: str) -> bool:
        """Stop on an emergency or a stale avoidance signal; True when stopped."""
        _, _, _, emergency, stale = self.avoidance_signal()
        if not (emergency or stale):
            return False
        self.publish_stop(hard=True)
        state, reason = (
            ('AVOIDANCE_STALE', f'no fresh navigation/avoidance during {during}')
            if stale
            else ('EMERGENCY_STOP', f'obstacle during {during}')
        )
        self.publish_markers(stamp, state, reason, pose.x, pose.y)
        return True

    def publish_approach(self, cmd):
        """Publish an approach command; the limiters start from it when following resumes."""
        out = Twist()
        out.linear.x = cmd.vx
        out.linear.y = cmd.vy
        out.angular.z = cmd.wz
        self._linear_limiter.reset(cmd.vx)
        self._lateral_limiter.reset(cmd.vy)
        self._angular_limiter.reset(cmd.wz)
        self._cmd_vel_pub.publish(out)

    def start_join(self, index: int):
        """Walk onto the route at waypoint index before following it."""
        self._follower_state = 'JOINING'
        self._join_index = index
        self._approach.reset()
        self._join_start_time = self.get_clock().now()

    def run_joining(self, pose: Pose2D, stamp):
        """Walk onto the route at the located point (slowly, facing along it), then follow."""
        x, y, yaw_deg, _ = self._waypoints[self._join_index]
        yaw = math.radians(yaw_deg)
        cmd = self._approach.update(pose, x, y, yaw)
        on_route = cmd.position_error < 0.15 and abs(cmd.yaw_error) < math.radians(20.0)
        elapsed = (self.get_clock().now() - self._join_start_time).nanoseconds / 1e9
        if cmd.done or on_route or elapsed > float(self.get_parameter('join_timeout_sec').value):
            if not (cmd.done or on_route):
                self.get_logger().warn(
                    f'join timeout, {cmd.position_error:.2f} m from the route: following anyway'
                )
            self.publish_stop(hard=True)
            self._follower_state = 'FOLLOWING'
            self.get_logger().info(f'on the route at point {self._join_index}')
            return
        if self.stopped_for_obstacle(pose, stamp, 'joining the route'):
            return
        self.publish_approach(cmd)
        self.publish_markers(
            stamp,
            'JOINING',
            f'route point {self._join_index}: {cmd.position_error * 100:.0f} cm, '
            f'{math.degrees(cmd.yaw_error):.0f} deg',
            pose.x,
            pose.y,
            {'approach': (x, y, yaw, self._approach_tolerance, 0.0)},
        )

    def run_dwelling(self, pose: Pose2D, stamp):
        """Hold until dwell time passed and inspections finished (or timed out), then move on."""
        self.publish_stop(hard=True)
        elapsed = (self.get_clock().now() - self._dwell_start_time).nanoseconds / 1e9
        waiting = (
            self._inspection_pending > 0
            and elapsed < self._dwell_duration + self._inspection_timeout
        )
        if elapsed < self._dwell_duration or waiting:
            remaining = max(0.0, self._dwell_duration - elapsed)
            note = (
                f', {self._inspection_pending} inspection(s) pending'
                if self._inspection_pending
                else ''
            )
            x, y, yaw_deg, _ = self._waypoints[self._next_stop_index]
            progress = elapsed / self._dwell_duration if self._dwell_duration > 0 else 1.0
            self.publish_markers(
                stamp,
                'DWELLING',
                f'resuming in {remaining:.1f}s{note}',
                pose.x,
                pose.y,
                {'approach': (x, y, math.radians(yaw_deg), self._approach_tolerance, progress)},
            )
            return

        if self._inspection_pending:
            self.publish_stop_event(
                'inspected', 'timeout: ' + ' | '.join(self._inspection_results)
            )
            self._inspection_pending = 0
            self._inspection_round += 1  # late answers belong to this stop, drop them
        self.publish_stop_event('departed')
        self._follower_state = 'FOLLOWING'
        self._next_stop_index = self.find_next_stop_index(self._next_stop_index + 1)

    def check_stop_point(self, pose: Pose2D, stamp) -> bool:
        """Enter APPROACHING near the next stop point, or skip it if progress already passed it."""
        if self._next_stop_index is None:
            return False
        sx, sy, _, sdwell = self._waypoints[self._next_stop_index]
        if math.hypot(sx - pose.x, sy - pose.y) < self._approach_radius:
            self._dwell_duration = sdwell
            self.start_approach(self._next_stop_index, is_goal=False)
            self.publish_markers(
                stamp, 'APPROACHING', f'stop point {self._next_stop_index}', pose.x, pose.y
            )
            return True
        if (
            self._controller.progress_index()
            > self._next_stop_index + self._stop_point_skip_margin
        ):
            self.get_logger().warn(f'stop point {self._next_stop_index} missed, skipping')
            self.publish_stop_event('skipped', 'passed without reaching tolerance')
            self._next_stop_index = self.find_next_stop_index(self._next_stop_index + 1)
        return False

    def set_shift_target(self, offset: float, pose: Pose2D = None):
        """
        Accept a new path-shift offset.

        Moving away from an obstacle happens at once; back toward the route only after that was
        proposed for return_delay_sec, so the robot does not hop back and forth.
        """
        target = self._shift_target
        if offset == target:
            self._return_since = None
            return
        if abs(offset) < abs(target) and offset * target >= 0.0:
            now = self.get_clock().now()
            if self._return_since is None:
                self._return_since = now
            if (now - self._return_since).nanoseconds / 1e9 < self._return_delay:
                return
        self._return_since = None
        where = f' at ({pose.x:.1f}, {pose.y:.1f})' if pose is not None else ''
        if offset == 0.0:
            self.get_logger().info(f'path shift: obstacle passed, back onto the route{where}')
        elif target == 0.0 or offset * target < 0.0:
            side = 'left' if offset > 0.0 else 'right'
            self.get_logger().info(
                f'path shift: passing an obstacle {abs(offset):.1f} m {side}{where}'
            )
        self._shift_target = offset

    def start_approach(self, index: int, is_goal: bool):
        """Switch to the final approach toward waypoint index."""
        self._follower_state = 'APPROACHING'
        self._approach_index = index
        self._approach_is_goal = is_goal
        self._approach.reset()
        self.reset_path_shift()
        self._approach_start_time = self.get_clock().now()

    def handle_route_end(self):
        """Start the next lap on a closed loop, else approach the last point and hold."""
        first_x, first_y = self._waypoints[0][0], self._waypoints[0][1]
        last_x, last_y = self._waypoints[-1][0], self._waypoints[-1][1]
        closed = math.hypot(first_x - last_x, first_y - last_y) < self._loop_close_distance
        if self._loop_route and closed and not self._battery_low:
            self._lap += 1
            self.get_logger().info(f'lap {self._lap} complete, restarting route')
            self.restart_route(from_start=True)
            return
        self.start_approach(len(self._waypoints) - 1, is_goal=True)

    def control_loop(self):
        """Run one control tick; a tick cut short by shutdown (context gone) ends quietly."""
        try:
            self.control_tick()
        except Exception:  # noqa: B902
            if rclpy.ok():
                raise

    def control_tick(self):
        """Run one control tick: safety checks, stop-point state machine, then path following."""
        with self._pending_lock:
            changed, self._pending_params = self._pending_params, set()
        if changed:
            self.load_tunables(changed)
        self.publish_route_markers()
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self._control_rate
        stamp = now.to_msg()

        if not self._nav_active:
            self.publish_markers(
                stamp, 'NAV_INACTIVE', "call 'navigation/start_nav' to begin", 0.0, 0.0
            )
            return
        if self._paused:
            self.publish_stop(hard=True)
            self.publish_markers(
                stamp, 'PAUSED', "call 'navigation/pause' false to resume", 0.0, 0.0
            )
            return
        if not self._waypoints:
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'NO_PATH', 'waiting for navigation/waypoints', 0.0, 0.0)
            return

        pose = self.current_pose()
        if pose is None:
            self.publish_stop(hard=True)
            self.publish_markers(
                stamp, 'TF_UNAVAILABLE', 'waiting for fresh map->base_link', 0.0, 0.0
            )
            return

        if self._follower_state == 'GOAL_HOLD':
            self.publish_stop(hard=True)
            self.publish_markers(
                stamp, 'GOAL_REACHED', 'holding position at the route end', pose.x, pose.y
            )
            return
        if self._follower_state == 'JOINING':
            self.run_joining(pose, stamp)
            return
        if self._follower_state == 'APPROACHING':
            self.run_approaching(pose, stamp)
            return
        if self._follower_state == 'DWELLING':
            self.run_dwelling(pose, stamp)
            return

        steering_bias, lateral_bias, velocity_scale, emergency, stale = self.avoidance_signal()
        if stale:
            self.publish_stop(hard=True)
            self.publish_markers(
                stamp, 'AVOIDANCE_STALE', 'no fresh navigation/avoidance, stopped', pose.x, pose.y
            )
            return

        if self.check_stop_point(pose, stamp):
            return
        if self._controller.is_finished(pose):
            self.handle_route_end()
            return

        now_sec = now.nanoseconds / 1e9
        shifting = self.path_shifting()
        # sliding around an obstacle is not being off the route
        cross_track = max(0.0, self.cross_track_error(pose) - abs(self._path_offset))
        slow, off_path = self._safety.off_path(cross_track)
        if off_path:
            self.publish_stop(hard=True)
            self.publish_markers(
                stamp,
                'OFF_PATH',
                f'{cross_track:.2f} m from the route, check the robot '
                "and call 'navigation/start_nav' again",
                pose.x,
                pose.y,
            )
            return

        index = min(self._controller.progress_index(), len(self._speed_profile) - 1)
        target_speed = (
            self._speed_profile[index] if self._speed_profile else self._target_linear_velocity
        )
        if slow:
            target_speed = min(target_speed, self._min_linear_velocity)
        if shifting:
            limit = self.update_path_shift(pose, dt)
            if limit is None:
                self.publish_stop(hard=True)
                self.publish_markers(
                    stamp, 'AVOIDANCE_STALE', 'no fresh perception/safety_scan, stopped',
                    pose.x, pose.y,
                )
                return
            velocity_scale = min(1.0, limit / target_speed) if target_speed > 1e-3 else 1.0
            target_speed = min(target_speed, limit)
        output = self._controller.update(pose, dt, target_speed)
        target_linear, target_lateral, target_angular = self.blend(
            output, steering_bias, lateral_bias, velocity_scale, shifting, dt
        )

        estop = self._safety.estop(emergency, now_sec)
        stalled = estop or (target_speed < 0.05 if shifting else velocity_scale < 0.05)
        blocked = self._safety.blocked(stalled, now_sec)
        if estop:
            self.publish_stop(hard=True)
        else:
            cmd = Twist()
            cmd.linear.x = self._linear_limiter.step(
                clamp(target_linear, self._max_linear_velocity), dt
            )
            lateral = self._lateral_filter.step(clamp(target_lateral, self._max_linear_velocity))
            angular = self._angular_filter.step(clamp(target_angular, self._max_angular_velocity))
            cmd.linear.y = self._lateral_limiter.step(lateral, dt)
            cmd.angular.z = self._angular_limiter.step(angular, dt)
            self._cmd_vel_pub.publish(cmd)

        status = 'BLOCKED' if blocked else 'EMERGENCY_STOP' if estop else 'FOLLOWING'
        shift_note = f'offset {self._path_offset:+.2f} m | ' if shifting else ''
        message = (
            f'lap {self._lap} | {self._controller_name} | {shift_note}v_scale {velocity_scale:.2f}'
            f' | cte {cross_track:.2f} m | next stop {self._next_stop_index}'
        )
        self.publish_markers(stamp, status, message, pose.x, pose.y, self._controller.debug_info())

    def blend(self, output, steering_bias, lateral_bias, velocity_scale, shifting, dt):
        """
        Combine the controller output with avoidance into (forward, sideways, yaw rate) targets.

        Path shift already moved the route the holonomic controller follows, so its output is
        used as is. With reactive avoidance a holonomic robot turns its direction of travel by the
        steering bias (keeping its heading), any other controller steers by it.
        """
        if shifting:
            return output.linear, output.lateral, output.angular
        if isinstance(self._controller, HolonomicController):
            c, s = math.cos(steering_bias), math.sin(steering_bias)
            vx = (c * output.linear - s * output.lateral) * velocity_scale
            vy = (s * output.linear + c * output.lateral) * velocity_scale
            return vx, vy + lateral_bias, output.angular
        target_angular = output.angular + steering_bias
        target_linear = output.linear * velocity_scale
        if self._speed_regulator_enabled:
            target_linear *= self._speed_regulator.scale_for(target_angular, dt)
        return target_linear, lateral_bias, target_angular

    def cross_track_error(self, pose: Pose2D) -> float:
        """Distance from the robot to the route around the current progress index."""
        i = self._controller.progress_index()
        window = self._waypoints[max(0, i - 20):i + 20]
        return min(math.hypot(w[0] - pose.x, w[1] - pose.y) for w in window) if window else 0.0

    def publish_stop(self, hard: bool = False):
        """Publish zero velocity; hard skips the slew ramp entirely."""
        if hard:
            for limiter in (
                self._linear_limiter,
                self._lateral_limiter,
                self._angular_limiter,
                self._angular_filter,
                self._lateral_filter,
            ):
                limiter.reset()
            self._cmd_vel_pub.publish(Twist())
            return
        dt = 1.0 / self._control_rate
        cmd = Twist()
        cmd.linear.x = self._linear_limiter.step(0.0, dt)
        cmd.linear.y = self._lateral_limiter.step(0.0, dt)
        cmd.angular.z = self._angular_limiter.step(0.0, dt)
        self._cmd_vel_pub.publish(cmd)


def main(args=None):
    """Spin the path follower node on a multi-threaded executor."""
    rclpy.init(args=args)
    node = PathFollowerNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
