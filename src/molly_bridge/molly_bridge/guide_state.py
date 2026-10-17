"""The guide's parts of the handheld state and site stream, from Molly's status messages."""

import math

TOUR_STATES = ('idle', 'starting', 'running', 'paused', 'stopping', 'error')


def rounded(value, digits: int = 2):
    """Round a finite number for JSON, else None."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return round(value, digits) if math.isfinite(value) else None


def tour_summary(msg) -> dict:
    """Compact TourState."""
    state = TOUR_STATES[msg.state] if 0 <= msg.state < len(TOUR_STATES) else 'idle'
    message = msg.error_reason if state == 'error' and msg.error_reason else msg.message
    return {
        'state': state,
        'tour_id': msg.tour_id,
        'name': msg.tour_name[:64],
        'index': int(msg.current_poi_index),
        'total': int(msg.total_pois),
        'poi_id': msg.current_poi_id,
        'poi_name': msg.current_poi_name[:64],
        'activity': msg.activity,
        'msg': message[:160],
    }


def planner_summary(msg) -> dict:
    """Compact PlannerStatus."""
    return {
        'state': msg.state,
        'goal': msg.goal_id,
        'remaining': rounded(msg.remaining, 1),
        'msg': msg.message[:160],
    }


def speech_summary(msg) -> dict:
    """Compact SpeechStatus."""
    return {'state': msg.state, 'text': msg.text[:80], 'queued': int(msg.queued)}


def visitors_summary(msg) -> dict:
    """Compact Visitors."""
    nearest = rounded(msg.nearest, 1)
    return {
        'count': int(msg.count),
        'front': int(msg.front),
        'behind': int(msg.behind),
        'nearest': nearest if nearest is not None and nearest >= 0.0 else None,
    }


def site_summary(msg, rev: int) -> dict:
    """Compact SiteStatus with the revision of the site stream."""
    return {
        'name': msg.site,
        'map': bool(msg.has_map),
        'pois': int(msg.pois),
        'tours': int(msg.tours),
        'tags': int(msg.tags),
        'rev': rev,
    }


def mapping_summary(msg) -> dict:
    """Compact MappingStatus."""
    return {
        'active': bool(msg.active),
        'site': msg.site,
        'scans': int(msg.scans),
        'area': rounded(msg.known_area_m2, 0),
    }


def greeter_summary(status: dict) -> dict:
    """Compact interaction/greeter JSON."""
    return {'enabled': status.get('enabled') is True, 'state': str(status.get('state', ''))[:16]}


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
