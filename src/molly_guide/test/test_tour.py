from molly_guide.tour import (
    can_transition,
    ERROR,
    GuideConfig,
    Hooks,
    IDLE,
    PAUSE,
    PAUSED,
    RESUME,
    RUNNING,
    SKIP,
    START,
    STARTING,
    STOP,
    Stop,
    STOPPING,
    TourEngine,
)


class Recorder(Hooks):
    """Records what the engine asks for."""

    def __init__(self):
        """Start with no calls."""
        self.calls = []

    def go_to(self, token, poi_id):
        """Record a walk."""
        self.calls.append(('go_to', token, poi_id))

    def cancel_nav(self):
        """Record a cancel."""
        self.calls.append(('cancel',))

    def pause_nav(self, pause):
        """Record a hold or continue."""
        self.calls.append(('pause', pause))

    def say(self, token, text, language):
        """Record a text to say."""
        self.calls.append(('say', token, text, language))

    def stop_speech(self):
        """Record a speech stop."""
        self.calls.append(('stop_speech',))

    def gesture(self, token, name):
        """Record a gesture."""
        self.calls.append(('gesture', token, name))

    def last(self, kind):
        """Return the last call of a kind."""
        return next(c for c in reversed(self.calls) if c[0] == kind)

    def kinds(self):
        """Return the kinds of every call."""
        return [c[0] for c in self.calls]


STOPS = [
    Stop('lobby', 'Lobby', 'Ini lobi.', 'id', 'nod', 2.0),
    Stop('cafe', 'Cafe', 'This is the cafe.', 'en', '', 0.0),
]


def engine(**cfg):
    """Return an engine with short timers and no guide texts unless given."""
    base = {'arrival_timeout_sec': 60.0, 'retry_delay_sec': 1.0, 'wait_for_visitors': False}
    hooks = Recorder()
    return TourEngine(GuideConfig(**{**base, **cfg}), hooks), hooks


def walk(e, hooks, now, poi_id, arrive=True):
    """Answer the last go_to and report arrival."""
    _, token, goal = hooks.last('go_to')
    assert goal == poi_id
    e.on_go_to(token, True, 'walking', now)
    if arrive:
        e.on_planner('arrived', poi_id, now + 0.5, now + 0.5)


def speak(e, hooks, now, speech_id):
    """Answer the last say and report it said."""
    _, token, _, _ = hooks.last('say')
    e.on_say(token, True, speech_id, 'queued', now)
    e.on_speech(speech_id, 'done', now + 0.1)


def test_transition_matrix():
    """The dashboard's matrix: what every state accepts."""
    assert can_transition(IDLE, START) and not can_transition(IDLE, PAUSE)
    assert can_transition(STARTING, STOP) and not can_transition(STARTING, SKIP)
    assert all(can_transition(RUNNING, k) for k in (PAUSE, STOP, SKIP))
    assert not can_transition(RUNNING, START) and not can_transition(RUNNING, RESUME)
    assert can_transition(PAUSED, RESUME) and not can_transition(PAUSED, SKIP)
    assert not any(can_transition(STOPPING, k) for k in range(5))
    assert can_transition(ERROR, START) and can_transition(ERROR, STOP)


def test_two_stop_tour_runs_every_step():
    """Intro, walk, gesture, narration, dwell, walk, narration, outro, then IDLE."""
    e, hooks = engine(intro_text='Halo', outro_text='Dadah')
    assert e.start(STOPS, 0.0, 'main', 'Main tour') == (True, 'tour of 2 stops started')
    assert e.state == STARTING and hooks.last('say')[2:] == ('Halo', 'id')
    speak(e, hooks, 1.0, 1)
    assert e.state == RUNNING and e.activity == 'navigating'
    walk(e, hooks, 2.0, 'lobby')
    assert e.activity == 'performing_motion' and hooks.last('gesture')[2] == 'nod'
    e.on_gesture(hooks.last('gesture')[1], True, 'done', 4.0)
    assert e.activity == 'narrating' and hooks.last('say')[2:] == ('Ini lobi.', 'id')
    speak(e, hooks, 5.0, 2)
    assert e.activity == 'dwelling'
    e.tick(6.0)
    assert e.activity == 'dwelling'
    e.tick(7.2)
    assert e.activity == 'navigating' and e.snapshot()['poi_id'] == 'cafe'
    walk(e, hooks, 8.0, 'cafe')
    assert hooks.last('say')[2:] == ('This is the cafe.', 'en')  # no gesture at the cafe
    speak(e, hooks, 9.0, 3)
    assert hooks.last('say')[2] == 'Dadah'
    speak(e, hooks, 10.0, 4)
    assert e.state == IDLE and e.message == 'tour Main tour finished'
    assert e.snapshot()['index'] == -1 and e.snapshot()['poi_id'] == ''
    assert [c for c in hooks.kinds() if c in ('go_to', 'gesture')] == ['go_to', 'gesture', 'go_to']


def test_snapshot_while_running():
    """The state names the tour, the stop and the activity."""
    e, hooks = engine()
    e.start(STOPS, 0.0, 'main', 'Main tour')
    walk(e, hooks, 1.0, 'lobby', arrive=False)
    s = e.snapshot()
    assert (s['state'], s['tour_id'], s['tour_name'], s['index'], s['total']) == (
        RUNNING, 'main', 'Main tour', 0, 2
    )
    assert (s['poi_id'], s['poi_name'], s['activity']) == ('lobby', 'Lobby', 'navigating')


def test_pause_holds_the_walk_and_freezes_the_dwell():
    """PAUSE holds navigation and the dwell timer; RESUME continues both."""
    e, hooks = engine()
    e.start(STOPS, 0.0)
    assert not e.command(PAUSE, 0.2)[0]  # STARTING only accepts STOP
    walk(e, hooks, 0.5, 'lobby', arrive=False)
    assert e.command(PAUSE, 1.0)[0] and e.state == PAUSED
    assert hooks.calls[-1] == ('pause', True)
    assert not e.command(PAUSE, 1.5)[0] and not e.command(SKIP, 1.5)[0]
    e.command(RESUME, 2.0)
    assert hooks.calls[-1] == ('pause', False) and e.state == RUNNING
    e.on_planner('arrived', 'lobby', 3.0, 3.0)
    e.on_gesture(hooks.last('gesture')[1], True, 'done', 4.0)
    speak(e, hooks, 5.0, 1)
    assert e.activity == 'dwelling'  # 2 s from 5.1
    e.command(PAUSE, 6.0)
    e.tick(20.0)
    assert e.activity == 'dwelling'
    e.command(RESUME, 20.0)
    e.tick(20.5)
    assert e.activity == 'dwelling'
    e.tick(21.2)
    assert e.activity == 'navigating'


def test_answers_while_paused_are_kept_for_resume():
    """An arrival during a pause starts the next step only after RESUME."""
    e, hooks = engine()
    e.start(STOPS, 0.0)
    _, token, _ = hooks.last('go_to')
    e.on_go_to(token, True, 'walking', 0.5)
    e.command(PAUSE, 1.0)
    e.on_planner('arrived', 'lobby', 1.5, 1.5)
    assert e.activity == 'navigating' and 'gesture' not in hooks.kinds()
    e.command(RESUME, 2.0)
    assert e.activity == 'performing_motion'


def test_skip_cancels_and_moves_on():
    """SKIP while walking cancels the walk and goes to the next stop."""
    e, hooks = engine()
    e.start(STOPS, 0.0)
    walk(e, hooks, 1.0, 'lobby', arrive=False)
    assert e.command(SKIP, 2.0) == (True, 'skipped Lobby')
    assert ('cancel',) in hooks.calls and hooks.last('go_to')[2] == 'cafe'
    walk(e, hooks, 3.0, 'cafe')
    assert e.activity == 'narrating'
    assert e.command(SKIP, 4.0)[0]
    assert hooks.calls[-1] == ('stop_speech',) or e.state == IDLE
    assert e.state == IDLE


def test_failed_walk_retries_then_skips():
    """A failed walk is tried again, then the stop is skipped (on_failure skip)."""
    e, hooks = engine(go_to_retries=1)
    e.start(STOPS, 0.0)
    _, token, _ = hooks.last('go_to')
    e.on_go_to(token, False, 'no path', 0.5)
    assert 'retrying' in e.message
    e.tick(1.6)
    _, token2, goal = hooks.last('go_to')
    assert token2 != token and goal == 'lobby'
    e.on_go_to(token2, True, 'walking', 2.0)
    e.on_planner('failed', 'lobby', 3.0, 3.0)
    assert hooks.last('go_to')[2] == 'cafe' and e.message.startswith('skipped Lobby')


def test_failed_walk_stops_the_tour_with_on_failure_stop():
    """on_failure stop: the tour ends in ERROR; START or STOP leave it."""
    e, hooks = engine(go_to_retries=0, on_failure='stop')
    e.start(STOPS, 0.0)
    e.on_go_to(hooks.last('go_to')[1], False, 'no path', 0.5)
    assert e.state == ERROR and 'Lobby' in e.snapshot()['error_reason']
    assert e.command(STOP, 1.0) == (True, 'stopped') and e.state == IDLE


def test_arrival_timeout_and_stale_status():
    """An 'arrived' from an earlier walk does not count; no arrival in time is a failure."""
    e, hooks = engine(go_to_retries=0, arrival_timeout_sec=30.0)
    e.start(STOPS, 10.0)
    e.on_go_to(hooks.last('go_to')[1], True, 'walking', 10.5)
    e.on_planner('arrived', 'lobby', 5.0, 11.0)  # stamped before the walk started
    assert e.activity == 'navigating'
    e.tick(41.0)
    assert hooks.last('go_to')[2] == 'cafe'


def test_early_arrival_before_the_go_to_answer():
    """The planner may report the arrival before its answer comes back."""
    e, hooks = engine()
    e.start(STOPS, 0.0)
    _, token, _ = hooks.last('go_to')
    e.on_planner('arrived', 'lobby', 0.2, 0.2)
    e.on_go_to(token, True, 'walking', 0.3)
    assert e.activity == 'performing_motion'


def test_stop_cancels_everything_then_idle():
    """STOP cancels the walk and the speech, shows STOPPING, then IDLE."""
    e, hooks = engine()
    e.start(STOPS, 0.0)
    walk(e, hooks, 1.0, 'lobby', arrive=False)
    assert e.command(STOP, 2.0) == (True, 'stopping') and e.state == STOPPING
    assert ('cancel',) in hooks.calls and ('stop_speech',) in hooks.calls
    e.tick(2.6)
    assert e.state == IDLE and e.message == 'stopped'
    calls = len(hooks.calls)
    e.on_go_to(99, True, 'late answer', 3.0)
    assert hooks.calls[calls:] == [('cancel',)]  # a walk started after STOP is cancelled


def test_estop_pauses_until_resume():
    """The e-stop pauses a running tour; RESUME continues it."""
    e, hooks = engine()
    e.start(STOPS, 0.0)
    walk(e, hooks, 1.0, 'lobby', arrive=False)
    e.on_estop(True, 2.0)
    assert e.state == PAUSED and e.message == 'e-stop'
    e.on_estop(False, 3.0)
    assert e.state == PAUSED
    e.command(RESUME, 4.0)
    assert e.state == RUNNING


def test_waits_for_visitors_who_fall_behind():
    """Nobody behind for visitor_lost_sec holds the walk and asks to follow; found: walk on."""
    e, hooks = engine(wait_for_visitors=True, visitor_lost_sec=3.0, visitor_found_sec=1.0,
                      visitor_wait_sec=10.0, follow_me_text='Ikuti saya')
    e.start(STOPS, 0.0)
    walk(e, hooks, 0.5, 'lobby', arrive=False)
    for t in (1.0, 2.0, 3.0):
        e.on_visitors(0, t)
        e.tick(t)
    assert e.activity == 'navigating'
    e.on_visitors(0, 4.0)
    e.tick(4.0)
    assert e.activity == 'waiting_visitors' and hooks.calls[-2:] == [
        ('pause', True), ('say', None, 'Ikuti saya', 'id')
    ]
    e.on_visitors(2, 4.5)
    e.tick(4.5)
    assert e.activity == 'waiting_visitors'
    e.on_visitors(2, 5.6)
    e.tick(5.6)
    assert e.activity == 'navigating' and hooks.calls[-1] == ('pause', False)


def test_visitor_wait_times_out_once_per_leg():
    """After visitor_wait_sec the walk goes on and does not wait again on this leg."""
    e, hooks = engine(wait_for_visitors=True, visitor_lost_sec=1.0, visitor_wait_sec=5.0)
    e.start(STOPS, 0.0)
    walk(e, hooks, 0.0, 'lobby', arrive=False)
    t = 0.0
    while t < 20.0:
        e.on_visitors(0, t)
        e.tick(t)
        t += 0.5
    assert [c for c in hooks.calls if c[0] == 'pause'] == [('pause', True), ('pause', False)]
    assert e.activity == 'navigating'


def test_no_waiting_without_visitor_data_or_in_remote():
    """Stale visitor data or REMOTE never hold the walk."""
    e, hooks = engine(wait_for_visitors=True, visitor_lost_sec=1.0)
    e.start(STOPS, 0.0)
    walk(e, hooks, 0.0, 'lobby', arrive=False)
    e.on_visitors(0, 0.0)
    for t in (3.0, 6.0, 9.0):
        e.tick(t)  # data from t=0 is stale after 2 s
    e.allow_visitor_wait = False
    for t in (10.0, 12.0, 14.0):
        e.on_visitors(0, t)
        e.tick(t)
    assert ('pause', True) not in hooks.calls


def test_return_stop_after_the_last_poi():
    """With a return stop the robot walks back before the outro."""
    e, hooks = engine()
    e.start(STOPS[1:], 0.0, return_stop=STOPS[0])
    walk(e, hooks, 1.0, 'cafe')
    speak(e, hooks, 2.0, 1)
    assert e.activity == 'returning' and hooks.last('go_to')[2] == 'lobby'
    assert e.snapshot()['poi_id'] == 'lobby'
    walk(e, hooks, 3.0, 'lobby')
    assert e.state == IDLE


def test_refusals():
    """Commands that do not fit the state are refused with a reason."""
    e, _ = engine()
    assert e.command(PAUSE, 0.0) == (False, 'PAUSE not possible while IDLE')
    assert e.start([], 0.0) == (False, 'the tour has no stops')
    assert e.command(9, 0.0)[0] is False
    e.start(STOPS, 0.0)
    assert e.start(STOPS, 1.0)[0] is False
