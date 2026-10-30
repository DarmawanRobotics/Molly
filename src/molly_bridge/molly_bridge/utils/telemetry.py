"""The robot's state for the handheld: every status topic, summarised for the state message."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import BatteryState

from dharma_interfaces.msg import (
    ControlStatus,
    DockingStatus,
    HealthStatus,
    LocalizationStatus,
    MappingStatus,
    NavigationStatus,
    PlannerStatus,
    SiteStatus,
    WaypointPath,
)
from dharma_utils.ros import LATCHED, SENSOR
from molly_bridge.core import protocol
from molly_bridge.core import summaries as s
from molly_interfaces.msg import (
    GreeterStatus,
    PoiArray,
    SpeechStatus,
    TourArray,
    TourState,
    Visitors,
)

VISITORS_STALE_SEC = 3.0
ROUTE_POINTS = 600


def finite(value, digits: int = 2):
    """Round a float for JSON, or None when it is NaN / inf."""
    return round(float(value), digits) if math.isfinite(value) else None


def level_of(status) -> int:
    """Return a DiagnosticStatus level as int (rclpy gives the byte field as bytes)."""
    level = status.level
    return level[0] if isinstance(level, bytes) else int(level)


class Telemetry:
    """Subscribes to the stack and Molly; `changed(kind)` is called for route, map and site."""

    def __init__(self, node, changed):
        self.node, self.changed = node, changed
        self.state = {
            'mode': 'auto', 'estop': False, 'estop_reasons': [], 'field': None,
            'nav': 'INACTIVE', 'nav_msg': '', 'pause': [], 'localization': 'none',
            'recording': False, 'battery': None, 'driver': None, 'cmd': [0.0, 0.0, 0.0],
            'health': None,
        }  # fmt: skip
        self.guide = dict.fromkeys(('mapping', 'planner', 'tour', 'speech', 'greeter', 'docking'))
        self.visitors, self.visitors_time = None, 0.0
        self.system: dict = s.system_load([])
        self.site, self.site_rev = '', 0
        self.site_msg = None
        self.pois: list = []
        self.tours: list = []
        self.gestures: list | None = None
        self.route: list = []
        self.grid, self.map_rev = None, 0
        self.subscribe()

    def subscribe(self):
        """Every status the state message and the streams carry."""
        sub, keep = self.node.create_subscription, self.keep
        sub(ControlStatus, 'control/status', lambda m: self.state.update(s.control_summary(m)),
            LATCHED)  # fmt: skip
        sub(NavigationStatus, 'navigation/status',
            lambda m: self.state.update(s.navigation_summary(m)), LATCHED)  # fmt: skip
        sub(LocalizationStatus, 'localization/status', self.on_localization, LATCHED)
        sub(HealthStatus, 'health/status', self.on_health, LATCHED)
        sub(DiagnosticArray, '/diagnostics', self.on_system, 10)
        sub(BatteryState, 'battery', self.on_battery, SENSOR)
        sub(DiagnosticArray, 'diagnostics', self.on_driver, 10)
        sub(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        sub(WaypointPath, 'navigation/waypoints', self.on_route, LATCHED)
        sub(OccupancyGrid, '/map', self.on_grid, LATCHED)
        sub(SiteStatus, 'site/status', self.on_site, LATCHED)
        sub(PoiArray, 'site/pois', self.on_pois, LATCHED)
        sub(TourArray, 'site/tours', self.on_tours, LATCHED)
        sub(MappingStatus, 'mapping/status', keep('mapping', s.mapping_summary), LATCHED)
        sub(PlannerStatus, 'planner/status', keep('planner', self.planner), LATCHED)
        sub(TourState, 'tour/state', keep('tour', s.tour_summary), LATCHED)
        sub(SpeechStatus, 'interaction/speech_status', keep('speech', s.speech_summary), LATCHED)
        sub(GreeterStatus, 'interaction/greeter', self.on_greeter, LATCHED)
        sub(DockingStatus, 'docking/status', keep('docking', s.docking_summary), LATCHED)
        sub(Visitors, 'perception/visitors', self.on_visitors, 10)

    def keep(self, key: str, summary):
        """Return a callback keeping summary(msg) under key."""
        return lambda msg: self.guide.__setitem__(key, summary(msg))

    def planner(self, msg) -> dict:
        """Planner status with the POI of its goal."""
        return s.planner_summary(msg, self.pois)

    def on_localization(self, msg: LocalizationStatus):
        """Keep the localization source."""
        self.state['localization'] = s.localization_text(msg)

    def on_health(self, msg: HealthStatus):
        """Keep the health level, issues and the computer's load."""
        self.state['health'] = s.health_summary(msg, self.system)

    def on_system(self, msg: DiagnosticArray):
        """Keep CPU, GPU, RAM and temperature from the health monitor's diagnostics."""
        statuses = [(st.name, {kv.key: kv.value for kv in st.values}) for st in msg.status]
        load = s.system_load(statuses)
        if any(v is not None for v in load.values()):
            self.system = load

    def on_battery(self, msg: BatteryState):
        """Keep the battery readings."""
        self.state['battery'] = {
            'pct': finite(msg.percentage * 100.0, 0),
            'v': finite(msg.voltage, 1),
            'a': finite(msg.current, 1),
            'temp': finite(msg.temperature, 0),
        }

    def on_driver(self, msg: DiagnosticArray):
        """Keep the driver's status line."""
        if msg.status:
            status = msg.status[0]
            self.state['driver'] = {'level': level_of(status), 'msg': status.message}

    def on_cmd_vel(self, msg: Twist):
        """Keep the command the control mux sends to the robot."""
        self.state['cmd'] = [
            round(msg.linear.x, 2),
            round(msg.linear.y, 2),
            round(msg.angular.z, 2),
        ]

    def on_route(self, msg: WaypointPath):
        """Keep the loaded route and push it."""
        points = [[round(w.x, 2), round(w.y, 2), round(w.dwell_sec, 1)] for w in msg.waypoints]
        self.route = protocol.simplify_route(points, ROUTE_POINTS)
        self.changed('route')

    def on_grid(self, msg: OccupancyGrid):
        """Keep the map and push it."""
        self.grid = msg
        self.map_rev += 1
        self.changed('map')

    def on_site(self, msg: SiteStatus):
        """Keep the active site; a new site means a new site stream."""
        if msg.site != self.site:
            self.site = msg.site
            self.site_rev += 1
        self.site_msg = msg
        self.changed('site_status')

    def on_pois(self, msg: PoiArray):
        """Keep the POIs and push the site stream."""
        self.pois = list(msg.pois)
        self.site_rev += 1
        self.changed('site')

    def on_tours(self, msg: TourArray):
        """Keep the tours and push the site stream."""
        self.tours = list(msg.tours)
        self.site_rev += 1
        self.changed('site')

    def on_greeter(self, msg: GreeterStatus):
        """Keep the greeter state and the robot's gesture names."""
        self.guide['greeter'] = s.greeter_summary(msg)
        self.gestures = list(msg.gestures)[:20]

    def on_visitors(self, msg: Visitors):
        """Keep the visitors around the robot."""
        self.visitors = s.visitors_summary(msg)
        self.visitors_time = time.monotonic()

    def snapshot(self) -> dict:
        """Return the robot's part of the state message."""
        fresh = time.monotonic() - self.visitors_time <= VISITORS_STALE_SEC
        site = None
        if self.site_msg is not None:
            site = s.site_summary(self.site_msg, self.site_rev, len(self.pois), len(self.tours))
        return {
            **self.state,
            **self.guide,
            'site': site,
            'visitors': self.visitors if fresh else None,
            'map_rev': self.map_rev,
        }
