"""The tour state machine with a fake clock and recorded hooks (no ROS)."""

from __future__ import annotations

import math

from molly_guide.core.config import GuideConfig
from molly_guide.core.tour import (
    Activity,
    Command,
    Hooks,
    Stop,
    TourEngine,
    TourStatus,
    TourTexts,
    Walk,
    can_transition,
)

S, C, A = TourStatus, Command, Activity


class Recorder(Hooks):
    """Records what the engine asks for."""

    def __init__(self):
        self.calls = []

    def go_to(self, token, stop):
        self.calls.append(('go_to', token, stop.poi_id))

    def cancel_nav(self):
        self.calls.append(('cancel',))

    def pause_nav(self, pause):
        self.calls.append(('pause', pause))

    def say(self, token, text, language):
        self.calls.append(('say', token, text, language))

    def stop_speech(self):
        self.calls.append(('stop_speech',))

    def gesture(self, token, name):
        self.calls.append(('gesture', token, name))

    def report(self, level, text):
        self.calls.append(('report', level, text))

    def last(self, kind):
        return next(c for c in reversed(self.calls) if c[0] == kind)

    def kinds(self):
        return [c[0] for c in self.calls]


LOBBY = Stop('lobby', 'Lobby', 1.0, 2.0, 0.0, 'Ini lobi.', 'id', 'nod', 2.0)
CAFE = Stop('cafe', 'Cafe', 5.0, -1.0, math.pi / 2, 'This is the cafe.', 'en', '', 0.0)
STOPS = [LOBBY, CAFE]
GOALS = {'lobby': (1.0, 2.0), 'cafe': (5.0, -1.0)}


def engine(texts=None, **cfg):
    """Return an engine with short timers and no guide texts unless given."""
    base = {'arrival_timeout_sec': 60.0, 'retry_delay_sec': 1.0, 'wait_for_visitors': False}
    hooks = Recorder()
    return TourEngine(GuideConfig(**{**base, **cfg}), texts or TourTexts(), hooks), hooks


def arrive(e, poi_id, stamp, walk=Walk.ARRIVED):
    e.on_planner(walk, GOALS[poi_id], stamp, stamp)


def walk(e, hooks, now, poi_id, arrived=True):
    """Answer the last go_to and report the arrival."""
    _, token, goal = hooks.last('go_to')
    assert goal == poi_id
    e.on_go_to(token, True, 'walking', now)
    if arrived:
        arrive(e, poi_id, now + 0.5)


def speak(e, hooks, now, speech_id):
    """Answer the last say and report it said."""
    _, token, _, _ = hooks.last('say')
    e.on_say(token, True, speech_id, 'queued', now)
    e.on_speech(speech_id, True, now + 0.1)


def test_transition_matrix():
    assert can_transition(S.IDLE, C.START) and not can_transition(S.IDLE, C.PAUSE)
    assert can_transition(S.STARTING, C.STOP) and not can_transition(S.STARTING, C.SKIP)
    assert all(can_transition(S.RUNNING, k) for k in (C.PAUSE, C.STOP, C.SKIP))
    assert not can_transition(S.RUNNING, C.START) and not can_transition(S.RUNNING, C.RESUME)
    assert can_transition(S.PAUSED, C.RESUME) and not can_transition(S.PAUSED, C.SKIP)
    assert not any(can_transition(S.STOPPING, k) for k in C)
    assert can_transition(S.ERROR, C.START) and can_transition(S.ERROR, C.STOP)


def test_two_stop_tour_runs_every_step():
    """Intro, walk, gesture, narration, dwell, walk, narration, outro, then IDLE."""
    e, hooks = engine(TourTexts('Halo', 'Dadah'))
    assert e.start(STOPS, 0.0, 'main', 'Main tour') == (True, 'tour of 2 stops started')
    assert e.state is S.STARTING and hooks.last('say')[2:] == ('Halo', 'id')
    speak(e, hooks, 1.0, 1)
    assert e.state is S.RUNNING and e.activity is A.NAVIGATING
    walk(e, hooks, 2.0, 'lobby')
    assert e.activity is A.PERFORMING_MOTION and hooks.last('gesture')[2] == 'nod'
    e.on_gesture(hooks.last('gesture')[1], True, 'done', 4.0)
    assert e.activity is A.NARRATING and hooks.last('say')[2:] == ('Ini lobi.', 'id')
    speak(e, hooks, 5.0, 2)
    assert e.activity is A.DWELLING
    e.tick(6.0)
    assert e.activity is A.DWELLING
    e.tick(7.2)
    assert e.activity is A.NAVIGATING and e.snapshot().poi_id == 'cafe'
    walk(e, hooks, 8.0, 'cafe')
    assert hooks.last('say')[2:] == ('This is the cafe.', 'en')  # no gesture at the cafe
    speak(e, hooks, 9.0, 3)
    assert hooks.last('say')[2] == 'Dadah'
    speak(e, hooks, 10.0, 4)
    assert e.state is S.IDLE and e.message == 'tour Main tour finished'
    assert e.snapshot().index == -1 and e.snapshot().poi_id == ''
    assert [c for c in hooks.kinds() if c in ('go_to', 'gesture')] == ['go_to', 'gesture', 'go_to']


def test_snapshot_while_running():
    e, hooks = engine()
    e.start(STOPS, 0.0, 'main', 'Main tour')
    walk(e, hooks, 1.0, 'lobby', arrived=False)
    s = e.snapshot()
    expected = (S.RUNNING, 'main', 'Main tour', 0, 2)
    assert (s.state, s.tour_id, s.tour_name, s.index, s.total) == expected
    assert (s.poi_id, s.poi_name, s.activity) == ('lobby', 'Lobby', A.NAVIGATING)


def test_arrival_must_be_for_the_goal_of_the_stop():
    """A planner status for another goal (an earlier walk, the operator) does not count."""
    e, hooks = engine()
    e.start(STOPS, 0.0)
    walk(e, hooks, 0.5, 'lobby', arrived=False)
    e.on_planner(Walk.ARRIVED, (1.0, 2.2), 1.0, 1.0)  # 20 cm off: another goal
    assert e.activity is A.NAVIGATING
    e.on_planner(Walk.ARRIVED, (1.01, 2.0), 1.0, 1.0)  # within goal_tolerance
    assert e.activity is A.PERFORMING_MOTION


def test_pause_holds_the_walk_and_freezes_the_dwell():
    e, hooks = engine()
    e.start(STOPS, 0.0)
    assert not e.command(C.PAUSE, 0.2)[0]  # STARTING only accepts STOP
    walk(e, hooks, 0.5, 'lobby', arrived=False)
    assert e.command(C.PAUSE, 1.0)[0] and e.state is S.PAUSED
    assert hooks.calls[-1] == ('pause', True)
    assert not e.command(C.PAUSE, 1.5)[0] and not e.command(C.SKIP, 1.5)[0]
    e.command(C.RESUME, 2.0)
    assert hooks.calls[-1] == ('pause', False) and e.state is S.RUNNING
    arrive(e, 'lobby', 3.0)
    e.on_gesture(hooks.last('gesture')[1], True, 'done', 4.0)
    speak(e, hooks, 5.0, 1)
    assert e.activity is A.DWELLING  # 2 s from 5.1
    e.command(C.PAUSE, 6.0)
    e.tick(20.0)
    assert e.activity is A.DWELLING
    e.command(C.RESUME, 20.0)
    e.tick(20.5)
    assert e.activity is A.DWELLING
    e.tick(21.2)
    assert e.activity is A.NAVIGATING


def test_answers_while_paused_are_kept_for_resume():
    e, hooks = engine()
    e.start(STOPS, 0.0)
    e.on_go_to(hooks.last('go_to')[1], True, 'walking', 0.5)
    e.command(C.PAUSE, 1.0)
    arrive(e, 'lobby', 1.5)
    assert e.activity is A.NAVIGATING and 'gesture' not in hooks.kinds()
    e.command(C.RESUME, 2.0)
    assert e.activity is A.PERFORMING_MOTION


def test_skip_cancels_and_moves_on():
    e, hooks = engine()
    e.start(STOPS, 0.0)
    walk(e, hooks, 1.0, 'lobby', arrived=False)
    assert e.command(C.SKIP, 2.0) == (True, 'skipped Lobby')
    assert ('cancel',) in hooks.calls and hooks.last('go_to')[2] == 'cafe'
    walk(e, hooks, 3.0, 'cafe')
    assert e.activity is A.NARRATING
    assert e.command(C.SKIP, 4.0)[0]
    assert hooks.calls[-1] == ('stop_speech',)
    assert e.state is S.IDLE


def test_failed_walk_retries_then_skips():
    e, hooks = engine(go_to_retries=1)
    e.start(STOPS, 0.0)
    _, token, _ = hooks.last('go_to')
    e.on_go_to(token, False, 'no path', 0.5)
    assert 'retrying' in e.message
    e.tick(1.6)
    _, token2, goal = hooks.last('go_to')
    assert token2 != token and goal == 'lobby'
    e.on_go_to(token2, True, 'walking', 2.0)
    arrive(e, 'lobby', 3.0, Walk.FAILED)
    assert hooks.last('go_to')[2] == 'cafe' and e.message.startswith('skipped Lobby')


def test_failed_walk_stops_the_tour_with_on_failure_stop():
    e, hooks = engine(go_to_retries=0, on_failure='stop')
    e.start(STOPS, 0.0)
    e.on_go_to(hooks.last('go_to')[1], False, 'no path', 0.5)
    assert e.state is S.ERROR and 'Lobby' in e.snapshot().error_reason
    assert e.command(C.STOP, 1.0) == (True, 'stopped') and e.state is S.IDLE


def test_arrival_timeout_and_stale_status():
    """An arrival stamped before the walk does not count; no arrival in time is a failure."""
    e, hooks = engine(go_to_retries=0, arrival_timeout_sec=30.0)
    e.start(STOPS, 10.0)
    e.on_go_to(hooks.last('go_to')[1], True, 'walking', 10.5)
    e.on_planner(Walk.ARRIVED, GOALS['lobby'], 5.0, 11.0)
    assert e.activity is A.NAVIGATING
    e.tick(41.0)
    assert hooks.last('go_to')[2] == 'cafe'


def test_early_arrival_before_the_go_to_answer():
    e, hooks = engine()
    e.start(STOPS, 0.0)
    _, token, _ = hooks.last('go_to')
    arrive(e, 'lobby', 0.2)
    e.on_go_to(token, True, 'walking', 0.3)
    assert e.activity is A.PERFORMING_MOTION


def test_stop_cancels_everything_then_idle():
    e, hooks = engine()
    e.start(STOPS, 0.0)
    walk(e, hooks, 1.0, 'lobby', arrived=False)
    assert e.command(C.STOP, 2.0) == (True, 'stopping') and e.state is S.STOPPING
    assert ('cancel',) in hooks.calls and ('stop_speech',) in hooks.calls
    e.tick(2.6)
    assert e.state is S.IDLE and e.message == 'stopped'
    calls = len(hooks.calls)
    e.on_go_to(99, True, 'late answer', 3.0)
    assert hooks.calls[calls:] == [('cancel',)]  # a walk started after STOP is cancelled


def test_estop_pauses_until_resume_after_the_release():
    e, hooks = engine()
    e.start(STOPS, 0.0)
    walk(e, hooks, 1.0, 'lobby', arrived=False)
    e.on_estop(True, 2.0)
    assert e.state is S.PAUSED and e.message == 'e-stop'
    assert e.command(C.RESUME, 2.5) == (False, 'release the e-stop first')
    e.on_estop(False, 3.0)
    assert e.state is S.PAUSED
    e.command(C.RESUME, 4.0)
    assert e.state is S.RUNNING and hooks.calls[-1] == ('pause', False)


def test_return_stop_after_the_last_poi():
    e, hooks = engine()
    e.start(STOPS[1:], 0.0, return_stop=LOBBY)
    walk(e, hooks, 1.0, 'cafe')
    speak(e, hooks, 2.0, 1)
    assert e.activity is A.RETURNING and hooks.last('go_to')[2] == 'lobby'
    assert e.snapshot().poi_id == 'lobby'
    walk(e, hooks, 3.0, 'lobby')
    assert e.state is S.IDLE


def test_speech_that_cannot_start_does_not_hold_the_tour():
    e, hooks = engine(TourTexts('Halo'))
    e.start(STOPS, 0.0)
    e.on_say(hooks.last('say')[1], False, 0, 'speech not running', 0.1)
    assert e.activity is A.NAVIGATING
    assert ('report', 'warn', 'could not speak: speech not running') in hooks.calls


def test_refusals():
    e, _ = engine()
    assert e.command(C.PAUSE, 0.0) == (False, 'PAUSE not possible while IDLE')
    assert e.start([], 0.0) == (False, 'the tour has no stops')
    e.start(STOPS, 0.0)
    assert e.start(STOPS, 1.0)[0] is False
