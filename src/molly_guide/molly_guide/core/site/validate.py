"""Validation of POIs and tours (plain dicts with the fields of molly_interfaces/Poi and Tour)."""

from __future__ import annotations

import math
import re
import unicodedata

from dharma_utils.io import valid_name

LANGUAGE_RE = re.compile(r'^[a-z]{0,3}(-[a-zA-Z0-9]{1,8})*$')
GESTURE_RE = re.compile(r'^[a-z0-9_]{0,32}$')
TEXT_LIMITS = {'name': 64, 'description': 1000, 'narration': 2000}
MAX_DWELL_SEC = 600.0
MAX_COORDINATE = 10000.0
MAX_POIS = 500
MAX_TOURS = 100
MAX_TOUR_STOPS = 200
POI_FIELDS = (
    'id', 'name', 'description', 'x', 'y', 'yaw', 'narration', 'language', 'gesture', 'dwell_sec',
)  # fmt: skip
TOUR_FIELDS = ('id', 'name', 'description', 'poi_ids', 'return_poi_id')


class SiteError(ValueError):
    """A request about a POI or a tour is invalid."""


def check_name(name, what: str = 'site') -> str:
    """Return name if it is a valid site, POI or tour id, else raise SiteError."""
    if not isinstance(name, str) or not valid_name(name):
        raise SiteError(
            f'{what} {name!r} must be 1-48 of a-z, 0-9, _ and -, starting with a letter or digit'
        )
    return name


def slugify(text: str, taken, fallback: str = 'poi') -> str:
    """Return an id from text (ASCII, a-z0-9 and -) that is not in taken."""
    ascii_text = unicodedata.normalize('NFKD', str(text)).encode('ascii', 'ignore').decode()
    base = re.sub(r'[^a-z0-9]+', '-', ascii_text.lower()).strip('-')[:40].strip('-')
    base = base or fallback
    if base not in taken:
        return base
    n = 2
    while f'{base}-{n}' in taken:
        n += 1
    return f'{base}-{n}'


def clean_text(value, field: str) -> str:
    """Return a trimmed string within the limit of its field."""
    if value is None:
        return ''
    if not isinstance(value, str):
        raise SiteError(f'{field} must be text')
    text = value.strip()
    limit = TEXT_LIMITS.get(field)
    if limit is not None and len(text) > limit:
        raise SiteError(f'{field} is longer than {limit} characters')
    return text


def clean_number(value, field: str, low=-MAX_COORDINATE, high=MAX_COORDINATE) -> float:
    """Return a finite float within [low, high]."""
    if isinstance(value, bool):
        raise SiteError(f'{field} must be a number')
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise SiteError(f'{field} must be a number') from error
    if not math.isfinite(number) or not low <= number <= high:
        raise SiteError(f'{field} must be finite and within [{low:g}, {high:g}]')
    return number


def clean_poi(raw) -> dict:
    """Validate a POI and return it normalized (yaw wrapped to [-pi, pi])."""
    if not isinstance(raw, dict):
        raise SiteError('a POI must be an object')
    yaw = clean_number(raw.get('yaw', 0.0), 'yaw', -100.0, 100.0)
    poi = {
        'id': raw.get('id', '') or '',
        'name': clean_text(raw.get('name'), 'name'),
        'description': clean_text(raw.get('description'), 'description'),
        'x': clean_number(raw.get('x', 0.0), 'x'),
        'y': clean_number(raw.get('y', 0.0), 'y'),
        'yaw': math.remainder(yaw, math.tau),
        'narration': clean_text(raw.get('narration'), 'narration'),
        'language': clean_text(raw.get('language'), 'language'),
        'gesture': clean_text(raw.get('gesture'), 'gesture'),
        'dwell_sec': clean_number(raw.get('dwell_sec', 0.0), 'dwell_sec', 0.0, MAX_DWELL_SEC),
    }
    if poi['id']:
        check_name(poi['id'], 'POI id')
    if not poi['name']:
        raise SiteError('a POI needs a name')
    if not LANGUAGE_RE.match(poi['language']) or len(poi['language']) > 16:
        raise SiteError(f'language {poi["language"]!r} is not a language tag like "id" or "en"')
    if not GESTURE_RE.match(poi['gesture']):
        raise SiteError(f'gesture {poi["gesture"]!r} must be a-z, 0-9 and _')
    return poi


def clean_tour(raw, poi_ids) -> dict:
    """Validate a tour against the site's POI ids and return it normalized."""
    if not isinstance(raw, dict):
        raise SiteError('a tour must be an object')
    ids = raw.get('poi_ids') or []
    if not isinstance(ids, (list, tuple)) or not all(isinstance(i, str) for i in ids):
        raise SiteError('poi_ids must be a list of POI ids')
    tour = {
        'id': raw.get('id', '') or '',
        'name': clean_text(raw.get('name'), 'name'),
        'description': clean_text(raw.get('description'), 'description'),
        'poi_ids': [i.strip() for i in ids],
        'return_poi_id': clean_text(raw.get('return_poi_id'), 'return_poi_id'),
    }
    if tour['id']:
        check_name(tour['id'], 'tour id')
    if not tour['name']:
        raise SiteError('a tour needs a name')
    if not tour['poi_ids']:
        raise SiteError('a tour needs at least one POI')
    if len(tour['poi_ids']) > MAX_TOUR_STOPS:
        raise SiteError(f'a tour has at most {MAX_TOUR_STOPS} stops')
    known = set(poi_ids)
    unknown = [i for i in tour['poi_ids'] if i not in known]
    if tour['return_poi_id'] and tour['return_poi_id'] not in known:
        unknown.append(tour['return_poi_id'])
    if unknown:
        raise SiteError(f'unknown POI id(s): {", ".join(unknown)}')
    return tour
