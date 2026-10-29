"""Waiting for the visitors who fall behind (engine and follower watch, no ROS)."""

from __future__ import annotations

from molly_guide.core.config import GuideConfig
from molly_guide.core.tour import Activity, Stop, TourEngine, TourTexts
from molly_guide.core.tour.followers import FollowerWatch, WatchConfig, WatchEvent

STOPS = [Stop('lobby', 'Lobby', 1.0, 2.0, 0.0), Stop('cafe', 'Cafe', 5.0, -1.0, 0.0)]


class Recorder:
    """Records what the engine asks for (only what these tests look at)."""

    def __init__(self):
        self.calls = []

    def go_to(self, token, stop):
        self.calls.append(('go_to', token, stop.poi_id))

    def pause_nav(self, pause):
        self.calls.append(('pause', pause))

    def say(self, token, text, language):
        self.calls.append(('say', token, text, language))

    def cancel_nav(self):
        self.calls.append(('cancel',))

    def stop_speech(self):
        self.calls.append(('stop_speech',))

    def gesture(self, token, name):
        self.calls.append(('gesture', token, name))

    def report(self, level, text):
        self.calls.append(('report', level, text))

    def last(self, kind):
        return next(c for c in reversed(self.calls) if c[0] == kind)


def engine(**cfg):
    base = {'arrival_timeout_sec': 60.0, 'wait_for_visitors': True}
    hooks = Recorder()
    texts = TourTexts(follow_me='Ikuti saya')
    return TourEngine(GuideConfig(**{**base, **cfg}), texts, hooks), hooks


def walking(e, hooks, now):
    e.start(STOPS, now)
    e.on_go_to(hooks.last('go_to')[1], True, 'walking', now)


def test_waits_for_visitors_who_fall_behind():
    e, hooks = engine(visitor_lost_sec=3.0, visitor_found_sec=1.0, visitor_wait_sec=10.0)
    walking(e, hooks, 0.5)
    for t in (1.0, 2.0, 3.0):
        e.on_visitors(0, t)
        e.tick(t)
    assert e.activity is Activity.NAVIGATING
    e.on_visitors(0, 4.0)
    e.tick(4.0)
    assert e.activity is Activity.WAITING_VISITORS
    assert hooks.calls[-2:] == [('pause', True), ('say', None, 'Ikuti saya', 'id')]
    e.on_visitors(2, 4.5)
    e.tick(4.5)
    assert e.activity is Activity.WAITING_VISITORS
    e.on_visitors(2, 5.6)
    e.tick(5.6)
    assert e.activity is Activity.NAVIGATING and hooks.calls[-1] == ('pause', False)


def test_the_wait_does_not_count_as_walking_time():
    e, hooks = engine(visitor_lost_sec=1.0, visitor_wait_sec=20.0, arrival_timeout_sec=10.0)
    walking(e, hooks, 0.0)
    t = 0.0
    while t <= 21.0:  # 1 s lost + 20 s waiting
        e.on_visitors(0, t)
        e.tick(t)
        t += 0.5
    assert e.activity is Activity.NAVIGATING and hooks.last('go_to')[2] == 'lobby'


def test_visitor_wait_times_out_once_per_leg():
    e, hooks = engine(visitor_lost_sec=1.0, visitor_wait_sec=5.0)
    walking(e, hooks, 0.0)
    t = 0.0
    while t < 20.0:
        e.on_visitors(0, t)
        e.tick(t)
        t += 0.5
    assert [c for c in hooks.calls if c[0] == 'pause'] == [('pause', True), ('pause', False)]
    assert e.activity is Activity.NAVIGATING


def test_no_waiting_without_visitor_data_or_in_remote():
    e, hooks = engine(visitor_lost_sec=1.0)
    walking(e, hooks, 0.0)
    e.on_visitors(0, 0.0)
    for t in (3.0, 6.0, 9.0):
        e.tick(t)  # data from t=0 is stale after 2 s
    e.allow_visitor_wait = False
    for t in (10.0, 12.0, 14.0):
        e.on_visitors(0, t)
        e.tick(t)
    assert ('pause', True) not in hooks.calls


def test_watch_alone():
    watch = FollowerWatch(WatchConfig(lost_sec=1.0, found_sec=0.5, wait_sec=3.0))
    watch.update(0, 0.0)
    assert watch.check(0.0, True) is None
    watch.update(0, 1.0)
    assert watch.check(1.0, True) is WatchEvent.BEGIN_WAIT
    watch.update(1, 1.2)
    assert watch.check(1.2, True) is None
    watch.update(1, 1.8)
    assert watch.check(1.8, True) is WatchEvent.VISITORS_BACK
    assert abs(watch.last_wait - 0.8) < 1e-9 and not watch.done
    watch.update(0, 2.0)
    assert watch.check(2.0, False) is None  # REMOTE: nothing
    assert Stop('a', 'A', 0.0, 0.0, 0.0).dwell_sec == 0.0
