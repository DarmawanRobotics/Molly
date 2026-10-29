"""
The handheld's state message and site stream from the robot's typed status messages.

Pure: the messages are read by attribute (duck typed), the enum values are the constants of
dharma_interfaces / molly_interfaces, turned into the lower-case words of the protocol.
"""

from __future__ import annotations

import math

MODES = ('auto', 'remote')
FIELDS = ('off', 'clear', 'limited', 'stop', 'stale', 'blind')
NAVIGATION = (
    'INACTIVE', 'JOINING', 'FOLLOWING', 'APPROACHING', 'DWELLING', 'PAUSED', 'BLOCKED',
    'OFF_PATH', 'GOAL_REACHED', 'FAILED',
)  # fmt: skip
LOCALIZATION = ('none', 'localized', 'uncertain')
PLANNER = ('idle', 'planning', 'following', 'replanning', 'arrived', 'failed', 'cancelled')
TOUR = ('idle', 'starting', 'running', 'paused', 'stopping', 'error')
ACTIVITY = (
    '', 'navigating', 'waiting_visitors', 'performing_motion', 'narrating', 'dwelling',
    'returning',
)  # fmt: skip
SPEECH = ('idle', 'speaking', 'done', 'failed', 'cancelled')
GREETER = (
    'off', 'waiting', 'watching', 'greeting', 'greeted', 'cooldown', 'paused', 'unavailable',
)  # fmt: skip
DOCKING = (
    'idle', 'searching', 'centering', 'approaching', 'settling', 'backing_off', 'sitting',
    'charging', 'docked', 'undocking', 'failed',
)  # fmt: skip
HEALTH_OK = 0
POSE_MATCH_M = 0.05  # a planner goal this close to a POI is that POI


def word(names: tuple, value: int, default: str = '') -> str:
    """Return the protocol word of an enum value."""
    return names[value] if 0 <= int(value) < len(names) else default


def rounded(value, digits: int = 2):
    """Round a finite number for JSON, else None."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return round(value, digits) if math.isfinite(value) else None


def control_summary(msg) -> dict:
    """Mode, e-stop and protective field of ControlStatus."""
    return {
        'mode': word(MODES, msg.mode, 'auto'),
        'estop': bool(msg.estop),
        'estop_reasons': [str(r)[:80] for r in msg.estop_reasons][:5],
        'field': word(FIELDS, msg.field, 'off'),
    }


def navigation_summary(msg) -> dict:
    """State, message and pause reasons of NavigationStatus."""
    return {
        'nav': word(NAVIGATION, msg.state, 'INACTIVE'),
        'nav_msg': msg.message[:160],
        'pause': list(msg.pause_reasons),
    }


def localization_text(msg) -> str:
    """Return 'none', or the source of the last correction (tag:<frame>, pose_estimate, ...)."""
    if msg.state == 0:
        return 'none'
    return msg.source or word(LOCALIZATION, msg.state, 'localized')


def system_load(statuses: list) -> dict:
    """CPU, GPU and RAM use and the hottest temperature from the health diagnostics."""
    values = {}
    for name, items in statuses:
        key = name.rsplit(': ', 1)[-1]
        if key.startswith('system:'):
            values[key[len('system:') :]] = items

    def pick(item: str, key: str, digits: int = 0):
        value = rounded(values.get(item, {}).get(key), digits)
        return None if value is None else (round(value) if digits == 0 else value)

    return {
        'cpu': pick('cpu', 'value'),
        'gpu': pick('gpu', 'value'),
        'ram': pick('memory', 'value'),
        'ram_used': pick('memory', 'used_gb', 1),
        'ram_total': pick('memory', 'total_gb', 1),
        'temp': pick('temperature', 'value'),
    }


def health_summary(msg, system: dict) -> dict:
    """HealthStatus (level, issues, speed scale) with the computer's load."""
    return {
        'ok': msg.level == HEALTH_OK,
        'issues': list(msg.issues)[:5],
        'speed_scale': rounded(msg.speed_scale),
        **system,
    }


def goal_poi(x: float, y: float, pois: list) -> str:
    """Return the id of the POI at a planner goal, or ''."""
    for poi in pois:
        if math.hypot(poi.x - x, poi.y - y) <= POSE_MATCH_M:
            return poi.id
    return ''


def planner_summary(msg, pois: list) -> dict:
    """PlannerStatus; the goal named by its POI when it is one."""
    return {
        'state': word(PLANNER, msg.state, 'idle'),
        'goal': goal_poi(msg.goal_x, msg.goal_y, pois),
        'remaining': rounded(msg.remaining, 1),
        'msg': msg.message[:160],
    }


def tour_summary(msg) -> dict:
    """TourState."""
    state = word(TOUR, msg.state, 'idle')
    message = msg.error_reason if state == 'error' and msg.error_reason else msg.message
    return {
        'state': state,
        'tour_id': msg.tour_id,
        'name': msg.tour_name[:64],
        'index': int(msg.current_poi_index),
        'total': int(msg.total_pois),
        'poi_id': msg.current_poi_id,
        'poi_name': msg.current_poi_name[:64],
        'activity': word(ACTIVITY, msg.activity),
        'msg': message[:160],
    }


def speech_summary(msg) -> dict:
    """SpeechStatus."""
    return {'state': word(SPEECH, msg.state, 'idle'), 'text': msg.text[:80], 'queued': msg.queued}


def greeter_summary(msg) -> dict:
    """GreeterStatus."""
    return {'enabled': bool(msg.enabled), 'state': word(GREETER, msg.state, 'off')}


def docking_summary(msg) -> dict:
    """DockingStatus."""
    return {
        'state': word(DOCKING, msg.state, 'idle'),
        'tag': bool(msg.tag_visible),
        'charging': bool(msg.charging),
        'msg': msg.message[:160],
    }


def visitors_summary(msg) -> dict:
    """Visitors."""
    nearest = rounded(msg.nearest, 1)
    return {
        'count': int(msg.count),
        'front': int(msg.front),
        'behind': int(msg.behind),
        'nearest': nearest if nearest is not None and nearest >= 0.0 else None,
    }


def site_summary(msg, rev: int, pois: int, tours: int) -> dict:
    """SiteStatus with the POI and tour counts and the revision of the site stream."""
    return {
        'name': msg.site,
        'map': bool(msg.has_map),
        'pois': int(pois),
        'tours': int(tours),
        'tags': int(msg.tags),
        'routes': int(msg.routes),
        'rev': rev,
    }


def mapping_summary(msg) -> dict:
    """MappingStatus."""
    return {
        'active': bool(msg.active),
        'site': msg.site,
        'scans': int(msg.scans),
        'area': rounded(msg.known_area_m2, 0),
    }


def poi_entry(poi) -> dict:
    """Return a Poi for the site stream."""
    return {
        'id': poi.id,
        'name': poi.name,
        'description': poi.description,
        'x': rounded(poi.x, 3),
        'y': rounded(poi.y, 3),
        'yaw': rounded(poi.yaw, 3),
        'narration': poi.narration,
        'language': poi.language,
        'gesture': poi.gesture,
        'dwell': rounded(poi.dwell_sec, 1),
    }


def tour_entry(tour) -> dict:
    """Return a Tour for the site stream."""
    return {
        'id': tour.id,
        'name': tour.name,
        'description': tour.description,
        'poi_ids': list(tour.poi_ids),
        'return_poi_id': tour.return_poi_id,
    }


def site_payload(name: str, pois: list, tours: list, rev: int) -> dict:
    """Return the site stream message: POIs and tours of the active site."""
    return {
        't': 'site',
        'rev': rev,
        'name': name,
        'pois': [poi_entry(p) for p in pois],
        'tours': [tour_entry(t) for t in tours],
    }
