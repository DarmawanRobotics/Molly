"""
The handheld's link to Molly's guide: sites, mapping, POIs, tours, walking and interaction.

A mixin of teleop_udp_node: service clients for the Molly commands, the status subscriptions
behind the state message, and the map (4) and site (5) streams.
"""

import json
import time

from molly_bridge import guide_commands as args_of
from molly_bridge import guide_state, map_image, protocol
from molly_interfaces.msg import (
    MappingStatus,
    PlannerStatus,
    Poi,
    PoiArray,
    SiteStatus,
    SpeechStatus,
    Tour,
    TourArray,
    TourCommand,
    TourState,
    Visitors,
)
from molly_interfaces.srv import (
    Ask,
    DeleteById,
    Gesture,
    GoTo,
    ListSites,
    LoadSite,
    SavePoi,
    SaveTour,
    Say,
    StartMapping,
    TourControl,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

GESTURES = ['hello', 'nod', 'bow', 'look_left', 'look_right']
VISITORS_STALE_SEC = 3.0
TOUR_KINDS = {
    'tour_pause': TourCommand.KIND_PAUSE,
    'tour_resume': TourCommand.KIND_RESUME,
    'tour_stop': TourCommand.KIND_STOP,
    'tour_skip': TourCommand.KIND_SKIP,
}
# command name: service that tells whether the feature runs on this robot
FEATURES = {
    'mapping': 'map_start',
    'planner': 'go_to',
    'guide': 'tour_start',
    'speech': 'say',
    'greeter': 'greeter_set',
    'conversation': 'ask',
    'people': 'people_set',
}


class GuideLink:
    """Molly commands, guide state and the map and site streams of the teleop gateway."""

    def init_guide(self, map_max_px: int, map_push_sec: float, gestures: list):
        """Set up the guide state; call before make_guide_commands."""
        self._map_max_px = max(int(map_max_px), 64)
        self._map_push_sec = max(float(map_push_sec), 0.5)
        self._default_gestures = [g for g in gestures if g] or list(GESTURES)
        self._gestures = None  # from interaction/greeter, else the default
        self._grid = None  # latest OccupancyGrid
        self._map_rev = 0
        self._map_sent_rev = -1
        self._map_sent_at = 0.0
        self._map_payload = None  # (rev, payload)
        self._site_rev = 0
        self._site_name = ''
        self._pois: list = []
        self._tours: list = []
        self._guide = {
            'site': None,
            'mapping': None,
            'planner': None,
            'tour': None,
            'speech': None,
            'greeter': None,
        }
        self._visitors = None
        self._visitors_time = 0.0

    # ------------------------------------------------------------------ commands

    def make_guide_commands(self) -> dict:
        """Return the Molly commands as (service type, client, request factory)."""

        def client(kind, name):
            return self.create_client(kind, name)

        tour = client(TourControl, 'tour/control')
        go_to = client(GoTo, 'planner/go_to')
        save_poi = client(SavePoi, 'site/save_poi')

        def tour_command(kind, tour_id='', poi_ids=()):
            command = TourCommand(kind=kind, tour_id=tour_id, poi_ids=list(poi_ids))
            return TourControl.Request(command=command)

        def start(a):
            tour_id, stops = args_of.tour_start(a)
            return tour_command(TourCommand.KIND_START, tour_id, stops)

        commands = {
            'site_list': (
                ListSites, client(ListSites, 'site/list'), lambda a: ListSites.Request()
            ),
            'site_load': (
                LoadSite,
                client(LoadSite, 'site/load'),
                lambda a: LoadSite.Request(site=args_of.name(a.get('site'))),
            ),
            'map_start': (
                StartMapping,
                client(StartMapping, 'mapping/start'),
                lambda a: StartMapping.Request(
                    site=args_of.name(a.get('site')), reset=args_of.flag(a.get('reset'), True)
                ),
            ),
            'map_stop': (Trigger, client(Trigger, 'mapping/stop'), lambda a: Trigger.Request()),
            'map_save': (Trigger, client(Trigger, 'mapping/save'), lambda a: Trigger.Request()),
            'poi_add': (SavePoi, save_poi, self.poi_add_request),
            'poi_update': (SavePoi, save_poi, self.poi_update_request),
            'poi_delete': (
                DeleteById,
                client(DeleteById, 'site/delete_poi'),
                lambda a: DeleteById.Request(id=args_of.name(a.get('id'), 'POI id')),
            ),
            'tour_save': (
                SaveTour,
                client(SaveTour, 'site/save_tour'),
                lambda a: SaveTour.Request(tour=Tour(**args_of.tour_fields(a.get('tour')))),
            ),
            'tour_delete': (
                DeleteById,
                client(DeleteById, 'site/delete_tour'),
                lambda a: DeleteById.Request(id=args_of.name(a.get('id'), 'tour id')),
            ),
            'tour_start': (TourControl, tour, start),
            'go_to_poi': (
                GoTo,
                go_to,
                lambda a: GoTo.Request(poi_id=args_of.name(a.get('id'), 'POI id')),
            ),
            'go_to': (GoTo, go_to, self.go_to_request),
            'go_cancel': (Trigger, client(Trigger, 'planner/cancel'), lambda a: Trigger.Request()),
            'say': (
                Say,
                client(Say, 'interaction/say'),
                lambda a: Say.Request(
                    text=args_of.text(a.get('text'), 'text', required=True),
                    language=args_of.language(a.get('lang', '')),
                    interrupt=args_of.flag(a.get('interrupt'), False),
                ),
            ),
            'stop_speech': (
                Trigger,
                client(Trigger, 'interaction/stop_speech'),
                lambda a: Trigger.Request(),
            ),
            'greet': (Trigger, client(Trigger, 'interaction/greet'), lambda a: Trigger.Request()),
            'greeter_set': (
                SetBool,
                client(SetBool, 'interaction/set_greeter'),
                lambda a: SetBool.Request(data=self.enabled_arg(a)),
            ),
            'gesture': (
                Gesture,
                client(Gesture, 'interaction/gesture'),
                lambda a: Gesture.Request(name=args_of.gesture(a.get('name'), self.gestures())),
            ),
            'ask': (
                Ask,
                client(Ask, 'interaction/ask'),
                lambda a: Ask.Request(
                    question=args_of.text(a.get('question'), 'question', required=True),
                    language=args_of.language(a.get('lang', '')),
                    speak=args_of.flag(a.get('speak'), True),
                ),
            ),
        }
        for name, kind in TOUR_KINDS.items():
            commands[name] = (TourControl, tour, lambda a, k=kind: tour_command(k))
        return commands

    @staticmethod
    def enabled_arg(a: dict) -> bool:
        """Return the required `enabled` bool of a switch command."""
        if not isinstance(a.get('enabled'), bool):
            raise ValueError('enabled must be true or false')
        return a['enabled']

    def poi_add_request(self, a: dict) -> SavePoi.Request:
        """Return the request for a new POI at the robot's current pose."""
        fields = args_of.poi_fields(a)
        fields.pop('x', None), fields.pop('y', None), fields.pop('yaw', None)
        return SavePoi.Request(poi=Poi(**fields), at_robot_pose=True)

    def poi_update_request(self, a: dict) -> SavePoi.Request:
        """Change fields of a known POI; `here` moves it to the robot's pose."""
        raw = a.get('poi')
        if not isinstance(raw, dict):
            raise ValueError('poi must be an object with an id')
        poi_id = args_of.name(raw.get('id'), 'POI id')
        known = next((p for p in self._pois if p.id == poi_id), None)
        if known is None:
            raise ValueError(f'no POI {poi_id} in this site')
        fields = {
            'id': known.id,
            'name': known.name,
            'description': known.description,
            'x': known.x,
            'y': known.y,
            'yaw': known.yaw,
            'narration': known.narration,
            'language': known.language,
            'gesture': known.gesture,
            'dwell_sec': known.dwell_sec,
        }
        fields.update(args_of.poi_fields(raw, require_name=False))
        here = args_of.flag(raw.get('here'), False)
        return SavePoi.Request(poi=Poi(**fields), at_robot_pose=here)

    def go_to_request(self, a: dict) -> GoTo.Request:
        """Walk to a pose of the map."""
        x, y, yaw = args_of.pose(a)
        return GoTo.Request(x=x, y=y, yaw=yaw)

    @staticmethod
    def command_data(name: str, result):
        """Return the ack data of a finished Molly command, or None."""
        if name == 'site_list':
            return {'sites': list(result.sites), 'active': result.active}
        if name in ('poi_add', 'poi_update') and getattr(result, 'success', False):
            return guide_state.poi_entry(result.poi)
        if name == 'tour_save' and getattr(result, 'success', False):
            return guide_state.tour_entry(result.tour)
        if name == 'say' and getattr(result, 'success', False):
            return {'id': int(result.id)}
        if name == 'ask':
            return {'answer': result.answer}
        return None

    def gestures(self) -> list:
        """Gesture names the robot runs (interaction_node), else the configured default."""
        return list(self._gestures) if self._gestures is not None else self._default_gestures

    def features(self) -> list:
        """Molly features running now, by the services that answer."""
        return [
            feature
            for feature, command in FEATURES.items()
            if command in self._commands and self._commands[command][1].service_is_ready()
        ]

    # ------------------------------------------------------------------ status

    def make_guide_subscriptions(self):
        """Subscribe to the guide's status topics (all small, latched where they are)."""
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        def keep(key, summary):
            return lambda msg: self._guide.__setitem__(key, summary(msg))

        self.create_subscription(OccupancyGrid, '/map', self.on_grid, latched)
        self.create_subscription(SiteStatus, 'site/status', self.on_site_status, latched)
        self.create_subscription(PoiArray, 'site/pois', self.on_pois, latched)
        self.create_subscription(TourArray, 'site/tours', self.on_tours, latched)
        self.create_subscription(
            MappingStatus, 'mapping/status', keep('mapping', guide_state.mapping_summary), latched
        )
        self.create_subscription(
            PlannerStatus, 'planner/status', keep('planner', guide_state.planner_summary), latched
        )
        self.create_subscription(
            TourState, '/tour/state', keep('tour', guide_state.tour_summary), latched
        )
        self.create_subscription(
            SpeechStatus,
            'interaction/speech_status',
            keep('speech', guide_state.speech_summary),
            latched,
        )
        self.create_subscription(String, 'interaction/greeter', self.on_greeter, latched)
        self.create_subscription(Visitors, 'perception/visitors', self.on_visitors, 10)

    def on_site_status(self, msg: SiteStatus):
        """Keep the active site; a new site means a new site stream."""
        if msg.site != self._site_name:
            self._site_name = msg.site
            self._site_rev += 1
        self._guide['site'] = guide_state.site_summary(msg, self._site_rev)

    def on_pois(self, msg: PoiArray):
        """Keep the POIs and push the site stream."""
        self._pois = list(msg.pois)
        self.site_changed()

    def on_tours(self, msg: TourArray):
        """Keep the tours and push the site stream."""
        self._tours = list(msg.tours)
        self.site_changed()

    def site_changed(self):
        """Count a new site revision and send it."""
        self._site_rev += 1
        if self._guide['site'] is not None:
            self._guide['site']['rev'] = self._site_rev
        self.send_site()

    def on_greeter(self, msg: String):
        """Keep the greeter state and the robot's gesture names."""
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(status, dict):
            return
        self._guide['greeter'] = guide_state.greeter_summary(status)
        names = status.get('gestures')
        if isinstance(names, list) and all(isinstance(n, str) for n in names):
            self._gestures = names[:20]

    def on_visitors(self, msg: Visitors):
        """Keep the visitors around the robot."""
        self._visitors = guide_state.visitors_summary(msg)
        self._visitors_time = time.monotonic()

    def guide_state(self) -> dict:
        """Return the guide's fields of the state message."""
        fresh = time.monotonic() - self._visitors_time <= VISITORS_STALE_SEC
        return {
            **self._guide,
            'visitors': self._visitors if fresh else None,
            'map_rev': self._map_rev,
        }

    # ------------------------------------------------------------------ streams

    def on_grid(self, msg: OccupancyGrid):
        """Keep the map; push it now or, while it keeps changing (mapping), every few seconds."""
        self._grid = msg
        self._map_rev += 1
        self._map_payload = None
        self.send_map()

    def send_map(self, force: bool = False):
        """Send the map stream when the client lacks the latest map (rate limited)."""
        if self._client is None or self._grid is None:
            return
        if not force:
            if self._map_sent_rev == self._map_rev:
                return
            if time.monotonic() - self._map_sent_at < self._map_push_sec:
                return
        if self._map_payload is None or self._map_payload[0] != self._map_rev:
            header, png = map_image.map_frame(
                self._grid, self._map_max_px, self._map_rev, self._site_name
            )
            self._map_payload = (self._map_rev, protocol.map_payload(header, png))
        self._map_sent_rev, self._map_sent_at = self._map_rev, time.monotonic()
        self.send_stream(protocol.STREAM_MAP, self._map_payload[1])

    def send_site(self):
        """Send the POIs and tours of the active site."""
        if self._client is None:
            return
        payload = guide_state.site_payload(
            self._site_name, self._pois, self._tours, self._site_rev
        )
        self.send_stream(protocol.STREAM_SITE, protocol.encode_json(payload))
