"""Arguments of the handheld commands: validated, trimmed, cut to their limits (no ROS)."""

from __future__ import annotations

import math
import re

NAME = re.compile(r'^[a-z0-9][a-z0-9_-]{0,47}$')  # sites, POI and tour ids
WORD = re.compile(r'^[a-z0-9_]{0,32}$')  # gestures
LANGUAGE = re.compile(r'^[a-z]{0,3}(-[a-zA-Z0-9]{1,8})*$')
TEXT_LIMITS = {
    'name': 64,
    'description': 1000,
    'narration': 2000,
    'text': 1000,
    'question': 500,
}
MAX_DWELL_SEC = 600.0
MAX_COORDINATE = 10000.0
MAX_TOUR_STOPS = 100


def name(value, what: str = 'site') -> str:
    """Return a valid site / POI / tour id."""
    if not isinstance(value, str) or not NAME.match(value):
        raise ValueError(f'{what} must be 1-48 of a-z, 0-9, _ and -')
    return value


def text(value, field: str, required: bool = False) -> str:
    """Return a trimmed text, cut to its limit; required texts may not be empty."""
    if value is None:
        value = ''
    if not isinstance(value, str):
        raise ValueError(f'{field} must be text')
    value = ' '.join(value.split()) if field in ('name', 'question') else value.strip()
    value = value[: TEXT_LIMITS.get(field, 1000)]
    if required and not value:
        raise ValueError(f'{field} is empty')
    return value


def flag(value, default: bool) -> bool:
    """Return a bool argument; anything that is not true/false keeps the default."""
    return value if isinstance(value, bool) else default


def finite(value, field: str, low: float = -MAX_COORDINATE, high: float = MAX_COORDINATE):
    """Return a finite number within [low, high]."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{field} must be a number')
    if not low <= float(value) <= high:
        raise ValueError(f'{field} must be between {low:g} and {high:g}')
    return float(value)


def language(value) -> str:
    """Return a language tag ("id", "en", "" for the default)."""
    value = text(value, 'language')
    if not LANGUAGE.match(value):
        raise ValueError('language must be a tag like id or en')
    return value


def gesture(value, allowed=None) -> str:
    """Return a gesture name ("" none); with allowed, it must be one of them."""
    value = text(value, 'gesture')
    if not WORD.match(value) or (allowed is not None and value not in allowed):
        listed = ', '.join(allowed) if allowed else 'a-z, 0-9 and _'
        raise ValueError(f'gesture must be one of {listed}')
    return value


def pose(args: dict) -> tuple:
    """Return (x, y, yaw) of a go_to, yaw wrapped to [-pi, pi]."""
    yaw = finite(args.get('yaw', 0.0), 'yaw', -1e6, 1e6)
    return (
        finite(args.get('x'), 'x'),
        finite(args.get('y'), 'y'),
        math.atan2(math.sin(yaw), math.cos(yaw)),
    )


def poi_fields(args: dict, require_name: bool = True) -> dict:
    """Return the Poi fields the handheld may set (only the ones it sent, except a new name)."""
    fields = {}
    if require_name or 'name' in args:
        fields['name'] = text(args.get('name'), 'name', required=True)
    for field in ('description', 'narration'):
        if field in args:
            fields[field] = text(args[field], field)
    if 'language' in args:
        fields['language'] = language(args['language'])
    if 'gesture' in args:
        fields['gesture'] = gesture(args['gesture'])
    if 'dwell' in args:
        fields['dwell_sec'] = finite(args['dwell'], 'dwell', 0.0, MAX_DWELL_SEC)
    for field in ('x', 'y'):
        if field in args:
            fields[field] = finite(args[field], field)
    if 'yaw' in args:
        yaw = finite(args['yaw'], 'yaw', -1e6, 1e6)
        fields['yaw'] = math.atan2(math.sin(yaw), math.cos(yaw))
    return fields


def tour_fields(raw) -> dict:
    """Return the Tour fields of a tour_save."""
    if not isinstance(raw, dict):
        raise ValueError('tour must be an object')
    stops = raw.get('poi_ids', [])
    if not isinstance(stops, list) or not 1 <= len(stops) <= MAX_TOUR_STOPS:
        raise ValueError(f'poi_ids: 1 to {MAX_TOUR_STOPS} POIs')
    tour_id = raw.get('id') or ''
    back = raw.get('return_poi_id') or ''
    return {
        'id': name(tour_id, 'tour id') if tour_id else '',
        'name': text(raw.get('name'), 'name', required=True),
        'description': text(raw.get('description'), 'description'),
        'poi_ids': [name(p, 'POI id') for p in stops],
        'return_poi_id': name(back, 'return POI id') if back else '',
    }


def tour_start(args: dict) -> tuple:
    """Return (tour_id, poi_ids) of a tour_start: a saved tour or a list of POIs."""
    tour_id = args.get('tour_id') or ''
    stops = args.get('poi_ids') or []
    if tour_id:
        return name(tour_id, 'tour id'), []
    if not isinstance(stops, list) or not 1 <= len(stops) <= MAX_TOUR_STOPS:
        raise ValueError('give tour_id or poi_ids')
    return '', [name(p, 'POI id') for p in stops]


def dwell(value) -> float:
    """Return a stop-point dwell time from the client (0 < dwell <= 1 h)."""
    seconds = finite(value, 'dwell', 0.0, 3600.0)
    if seconds <= 0.0:
        raise ValueError('dwell must be between 0 and 3600 s')
    return seconds


def speed(value, low: float, high: float) -> float:
    """Return a walking speed the handheld may set (m/s, within [low, high])."""
    return round(finite(value, 'speed', low, max(high, low)), 2)


def mode(value) -> str:
    """Return the control mode of a set_mode: auto or remote."""
    if value not in ('auto', 'remote'):
        raise ValueError('mode must be auto or remote')
    return value


def enabled(args: dict) -> bool:
    """Return the required `enabled` bool of a switch command."""
    if not isinstance(args.get('enabled'), bool):
        raise ValueError('enabled must be true or false')
    return args['enabled']
