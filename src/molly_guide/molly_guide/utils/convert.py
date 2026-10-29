"""Messages <-> the plain values of molly_guide.core."""

from __future__ import annotations

from dharma_interfaces.msg import PlannerStatus
from molly_guide.core.site.validate import POI_FIELDS, TOUR_FIELDS
from molly_guide.core.tour import Snapshot, Stop, Walk
from molly_interfaces.msg import Poi, Tour, TourState

WALKS = {
    PlannerStatus.ARRIVED: Walk.ARRIVED,
    PlannerStatus.FAILED: Walk.FAILED,
    PlannerStatus.CANCELLED: Walk.CANCELLED,
}


def poi_msg(poi: dict) -> Poi:
    """Return a Poi message of a stored POI."""
    return Poi(
        id=poi['id'],
        name=poi['name'],
        description=poi['description'],
        x=float(poi['x']),
        y=float(poi['y']),
        yaw=float(poi['yaw']),
        narration=poi['narration'],
        language=poi['language'],
        gesture=poi['gesture'],
        dwell_sec=float(poi['dwell_sec']),
    )


def poi_dict(msg: Poi) -> dict:
    """Return the fields of a Poi message."""
    return {name: getattr(msg, name) for name in POI_FIELDS}


def tour_msg(tour: dict) -> Tour:
    """Return a Tour message of a stored tour."""
    return Tour(
        id=tour['id'],
        name=tour['name'],
        description=tour['description'],
        poi_ids=list(tour['poi_ids']),
        return_poi_id=tour['return_poi_id'],
    )


def tour_dict(msg: Tour) -> dict:
    """Return the fields of a Tour message."""
    out = {name: getattr(msg, name) for name in TOUR_FIELDS}
    out['poi_ids'] = list(out['poi_ids'])
    return out


def stop_of(poi: Poi) -> Stop:
    """Return the tour stop of a POI message."""
    return Stop(
        poi.id,
        poi.name or poi.id,
        float(poi.x),
        float(poi.y),
        float(poi.yaw),
        poi.narration,
        poi.language,
        poi.gesture,
        max(float(poi.dwell_sec), 0.0),
    )


def walk_of(status: PlannerStatus) -> Walk:
    """Return what a planner status says about the walk."""
    return WALKS.get(status.state, Walk.UNDER_WAY)


def tour_state(snapshot: Snapshot, stamp) -> TourState:
    """Return the TourState message of an engine snapshot."""
    msg = TourState(
        state=int(snapshot.state),
        activity=int(snapshot.activity),
        tour_id=snapshot.tour_id,
        tour_name=snapshot.tour_name,
        current_poi_index=int(snapshot.index),
        total_pois=int(snapshot.total),
        current_poi_id=snapshot.poi_id,
        current_poi_name=snapshot.poi_name,
        message=snapshot.message,
        error_reason=snapshot.error_reason,
    )
    msg.header.stamp = stamp
    return msg
