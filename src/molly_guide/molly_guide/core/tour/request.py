"""What a START needs: the stops of a saved tour or a POI list, and a robot ready to walk."""

from __future__ import annotations

from dataclasses import dataclass

from molly_guide.core.tour.model import Stop


@dataclass(frozen=True)
class TourPlan:
    """The stops of a tour to start."""

    stops: list[Stop]
    tour_id: str = ''
    name: str = ''
    return_stop: Stop | None = None


@dataclass(frozen=True)
class RobotState:
    """What the guide knows about the robot (None: not reported yet)."""

    localized: bool | None = None
    auto: bool | None = None
    estop: bool = False


def resolve(
    tour_id: str, poi_ids, stops: dict[str, Stop], tours: dict[str, dict], site: str
) -> tuple[TourPlan | None, str]:
    """Return (plan, '') of a START with a tour id or POI ids, or (None, why not)."""
    tour_id, name, back = tour_id.strip(), '', ''
    if tour_id:
        tour = tours.get(tour_id)
        if tour is None:
            return None, f'no tour {tour_id!r} in site {site!r}'
        ids = list(tour['poi_ids'])
        name, back = tour['name'] or tour_id, tour['return_poi_id']
    else:
        ids = [p.strip() for p in poi_ids if p.strip()]
    if not ids:
        return None, 'no POIs to visit'
    missing = [p for p in [*ids, *([back] if back else [])] if p not in stops]
    if missing:
        where = f'site {site!r}' if site else 'the site (none loaded)'
        return None, f'unknown POIs in {where}: {", ".join(missing)}'
    plan = TourPlan([stops[p] for p in ids], tour_id, name, stops[back] if back else None)
    return plan, ''


def not_ready(robot: RobotState, require_localization: bool, require_auto: bool) -> str:
    """Return why a tour (or a walk to a POI) cannot start now, or ''."""
    if require_localization and not robot.localized:
        return 'not localized: localize from an AprilTag first'
    if require_auto and robot.auto is not True:
        mode = {None: 'unknown', False: 'REMOTE'}[robot.auto]
        return f'control mode is {mode}: switch to AUTO'
    if robot.estop:
        return 'the e-stop is engaged'
    return ''
