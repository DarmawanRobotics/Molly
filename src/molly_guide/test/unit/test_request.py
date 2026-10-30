"""Resolving a START and the readiness checks (no ROS)."""

from __future__ import annotations

from molly_guide.core.tour import Stop, text_for
from molly_guide.core.tour.request import RobotState, not_ready, resolve

STOPS = {
    'lobby': Stop('lobby', 'Lobby', 0.0, 0.0, 0.0),
    'hall': Stop('hall', 'Hall', 1.0, 0.0, 0.0),
}
TOURS = {
    'main': {'id': 'main', 'name': 'Main', 'poi_ids': ['lobby', 'hall'], 'return_poi_id': 'lobby'},
    'broken': {'id': 'broken', 'name': '', 'poi_ids': ['gone'], 'return_poi_id': ''},
}


def test_saved_tour_with_return_stop():
    plan, error = resolve('main', [], STOPS, TOURS, 'museum')
    assert error == '' and [s.poi_id for s in plan.stops] == ['lobby', 'hall']
    assert (plan.tour_id, plan.name, plan.return_stop.poi_id) == ('main', 'Main', 'lobby')


def test_poi_list_and_refusals():
    plan, _ = resolve('', [' hall ', 'lobby'], STOPS, TOURS, 'museum')
    assert [s.poi_id for s in plan.stops] == ['hall', 'lobby'] and plan.tour_id == ''
    assert resolve('nope', [], STOPS, TOURS, 'museum') == (None, "no tour 'nope' in site 'museum'")
    assert resolve('', ['', ' '], STOPS, TOURS, 'museum') == (None, 'no POIs to visit')
    assert 'unknown POIs' in resolve('broken', [], STOPS, TOURS, 'museum')[1]
    assert 'none loaded' in resolve('', ['x'], {}, {}, '')[1]


def test_ready_needs_localization_auto_and_no_estop():
    assert 'localized' in not_ready(RobotState(), True, True)
    assert 'unknown' in not_ready(RobotState(True), True, True)
    assert 'REMOTE' in not_ready(RobotState(True, False), True, True)
    assert 'e-stop' in not_ready(RobotState(True, True, True), True, True)
    assert not_ready(RobotState(True, True), True, True) == ''
    assert not_ready(RobotState(False, False), False, False) == ''


def test_texts_by_language():
    texts = ['id:Halo', 'en:Hello', 'broken', 'fr:']
    assert text_for(texts, 'en') == 'Hello'
    assert text_for(texts, 'de') == 'Halo'
    assert text_for(['en:Hi'], 'id') == 'Hi'
    assert text_for([], 'id') == ''
