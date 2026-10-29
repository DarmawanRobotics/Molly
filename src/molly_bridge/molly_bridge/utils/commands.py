"""The handheld's commands as ROS service calls: name -> (client, request from the args)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import SetBool, Trigger

from dharma_interfaces.srv import (
    GoTo,
    ListSites,
    LoadPath,
    LoadSite,
    MarkStopPoint,
    SetMode,
    StartMapping,
)
from molly_bridge.core import args as a
from molly_bridge.core import routes, summaries
from molly_interfaces.msg import Poi, Tour, TourCommand
from molly_interfaces.srv import (
    Ask,
    DeleteById,
    Gesture,
    GoToPoi,
    SavePoi,
    SaveTour,
    Say,
    TourControl,
)

TRIGGERS = {
    'estop': 'control/estop/engage',
    'estop_release': 'control/estop/release',
    'nav_start': 'navigation/start',
    'nav_stop': 'navigation/stop',
    'nav_pause': 'navigation/pause',
    'nav_resume': 'navigation/resume',
    'record_start': 'route/start',
    'record_stop': 'route/stop',
    'localize': 'localization/start',
    'record_tags': 'localization/record_tags',
    'map_stop': 'mapping/stop',
    'map_save': 'mapping/save',
    'go_cancel': 'planner/cancel',
    'stop_speech': 'interaction/stop_speech',
    'greet': 'interaction/greet',
    'dock': 'docking/dock',
    'undock': 'docking/undock',
    'dock_cancel': 'docking/cancel',
}
POI_FIELDS = (
    'id', 'name', 'description', 'x', 'y', 'yaw', 'narration', 'language', 'gesture', 'dwell_sec',
)  # fmt: skip
TOUR_KINDS = {
    'tour_pause': TourCommand.PAUSE,
    'tour_resume': TourCommand.RESUME,
    'tour_stop': TourCommand.STOP,
    'tour_skip': TourCommand.SKIP,
}
# feature -> the command whose service tells whether it runs on this robot
FEATURES = {
    'mapping': 'map_start',
    'planner': 'go_to',
    'guide': 'tour_start',
    'speech': 'say',
    'greeter': 'greeter_set',
    'conversation': 'ask',
    'docking': 'dock',
}


@dataclass
class Command:
    """One handheld command: a service client and how its request is made from the args."""

    client: object
    request: Callable[[dict], object]


def speed_request(value: float) -> SetParameters.Request:
    """Return the request that sets the follower's cruise speed."""
    param = ParameterMsg(
        name='speed.cruise',
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value),
    )
    return SetParameters.Request(parameters=[param])


def outcome(result) -> tuple[bool, str]:
    """Return (ok, message) of a service response: success / message or SetParameters results."""
    if result is None:
        return False, 'no response'
    results = getattr(result, 'results', None)
    if results is not None:
        refused = [r.reason or 'refused' for r in results if not r.successful]
        return not refused, '; '.join(refused)
    return bool(getattr(result, 'success', True)), getattr(result, 'message', '')


def ack_data(name: str, result):
    """Return the ack data of a finished command, or None."""
    ok = getattr(result, 'success', False)
    if name == 'site_list':
        return {'sites': list(result.sites), 'active': result.active}
    if name in ('poi_add', 'poi_update') and ok:
        return summaries.poi_entry(result.poi)
    if name == 'tour_save' and ok:
        return summaries.tour_entry(result.tour)
    if name == 'say' and ok:
        return {'id': int(result.id)}
    if name == 'ask':
        return {'answer': result.answer}
    return None


class CommandTable:
    """Builds every command of the gateway; `bridge` gives the state the requests need."""

    def __init__(self, node, bridge, group):
        self.node, self.bridge, self.group = node, bridge, group
        self.commands: dict[str, Command] = {}

    def client(self, kind, service: str):
        """Return a client in the gateway's callback group."""
        return self.node.create_client(kind, service, callback_group=self.group)

    def build(self, robot_prefix: str, robot_commands) -> dict[str, Command]:
        """Return name -> Command for the stack, Molly and the robot driver."""
        c = self.commands
        for name, service in TRIGGERS.items():
            c[name] = Command(self.client(Trigger, service), lambda _: Trigger.Request())
        self.stack_commands()
        self.guide_commands()
        self.interaction_commands()
        for name in robot_commands:
            if name:
                c[name] = Command(
                    self.client(Trigger, robot_prefix + name), lambda _: Trigger.Request()
                )
        return c

    def stack_commands(self):
        """Mode, mapping, sites, routes, walking (Dharma services)."""
        b, c = self.bridge, self.commands
        c['set_mode'] = Command(
            self.client(SetMode, 'control/set_mode'),
            lambda x: SetMode.Request(mode=a.mode(x.get('mode')), source=b.remote_source),
        )
        c['site_list'] = Command(
            self.client(ListSites, 'site/list'), lambda _: ListSites.Request()
        )
        c['site_load'] = Command(
            self.client(LoadSite, 'site/load'),
            lambda x: LoadSite.Request(site=a.name(x.get('site'))),
        )
        c['map_start'] = Command(
            self.client(StartMapping, 'mapping/start'),
            lambda x: StartMapping.Request(
                site=a.name(x.get('site')), reset=a.flag(x.get('reset'), True)
            ),
        )
        c['mark_stop'] = Command(
            self.client(MarkStopPoint, 'route/mark_stop_point'),
            lambda x: MarkStopPoint.Request(dwell_sec=a.dwell(x.get('dwell', 10.0))),
        )
        c['load_route'] = Command(
            self.client(LoadPath, 'navigation/load_path'),
            lambda x: LoadPath.Request(path=routes.route_path(b.sites_dir, x.get('file'))),
        )
        c['go_to'] = Command(
            self.client(GoTo, 'planner/go_to'),
            lambda x: GoTo.Request(**dict(zip(('x', 'y', 'yaw'), a.pose(x)))),
        )
        c['nav_speed'] = Command(
            self.client(SetParameters, f'{b.navigation_node}/set_parameters'),
            lambda x: speed_request(a.speed(x.get('speed'), *b.speed_range())),
        )

    def guide_commands(self):
        """POIs, tours and walking to a POI (Molly services)."""
        b, c = self.bridge, self.commands
        save_poi = self.client(SavePoi, 'site/save_poi')
        tour = self.client(TourControl, 'tour/control')
        c['poi_add'] = Command(save_poi, poi_add)
        c['poi_update'] = Command(save_poi, lambda x: poi_update(x, b.telemetry.pois))
        c['poi_delete'] = Command(
            self.client(DeleteById, 'site/delete_poi'),
            lambda x: DeleteById.Request(id=a.name(x.get('id'), 'POI id')),
        )
        c['tour_save'] = Command(
            self.client(SaveTour, 'site/save_tour'),
            lambda x: SaveTour.Request(tour=Tour(**a.tour_fields(x.get('tour')))),
        )
        c['tour_delete'] = Command(
            self.client(DeleteById, 'site/delete_tour'),
            lambda x: DeleteById.Request(id=a.name(x.get('id'), 'tour id')),
        )

        def start(x):
            tour_id, stops = a.tour_start(x)
            command = TourCommand(kind=TourCommand.START, tour_id=tour_id, poi_ids=list(stops))
            return TourControl.Request(command=command)

        c['tour_start'] = Command(tour, start)
        for name, kind in TOUR_KINDS.items():
            c[name] = Command(
                tour, lambda _, k=kind: TourControl.Request(command=TourCommand(kind=k))
            )
        c['go_to_poi'] = Command(
            self.client(GoToPoi, 'guide/go_to_poi'),
            lambda x: GoToPoi.Request(id=a.name(x.get('id'), 'POI id')),
        )

    def interaction_commands(self):
        """Speech, greeter, gestures and questions (Molly services)."""
        b, c = self.bridge, self.commands
        c['say'] = Command(
            self.client(Say, 'interaction/say'),
            lambda x: Say.Request(
                text=a.text(x.get('text'), 'text', required=True),
                language=a.language(x.get('lang', '')),
                interrupt=a.flag(x.get('interrupt'), False),
            ),
        )
        c['greeter_set'] = Command(
            self.client(SetBool, 'interaction/set_greeter'),
            lambda x: SetBool.Request(data=a.enabled(x)),
        )
        c['gesture'] = Command(
            self.client(Gesture, 'interaction/gesture'),
            lambda x: Gesture.Request(name=a.gesture(x.get('name'), b.gesture_names())),
        )
        c['ask'] = Command(
            self.client(Ask, 'interaction/ask'),
            lambda x: Ask.Request(
                question=a.text(x.get('question'), 'question', required=True),
                language=a.language(x.get('lang', '')),
                speak=a.flag(x.get('speak'), True),
            ),
        )

    def features(self) -> list[str]:
        """Return the features running now, by the services that answer."""
        return [
            feature
            for feature, command in FEATURES.items()
            if command in self.commands and self.commands[command].client.service_is_ready()
        ]


def poi_add(x: dict) -> SavePoi.Request:
    """Return the request for a new POI at the robot's current pose."""
    fields = a.poi_fields(x)
    for key in ('x', 'y', 'yaw'):
        fields.pop(key, None)
    return SavePoi.Request(poi=Poi(**fields), at_robot_pose=True)


def poi_update(x: dict, pois: list) -> SavePoi.Request:
    """Change fields of a known POI; `here` moves it to the robot's pose."""
    raw = x.get('poi')
    if not isinstance(raw, dict):
        raise ValueError('poi must be an object with an id')
    poi_id = a.name(raw.get('id'), 'POI id')
    known = next((p for p in pois if p.id == poi_id), None)
    if known is None:
        raise ValueError(f'no POI {poi_id} in this site')
    fields = {name: getattr(known, name) for name in POI_FIELDS}
    fields.update(a.poi_fields(raw, require_name=False))
    return SavePoi.Request(poi=Poi(**fields), at_robot_pose=a.flag(raw.get('here'), False))
