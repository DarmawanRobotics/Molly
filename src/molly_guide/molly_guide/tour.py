"""
The tour state machine and the steps of a tour (no ROS, unit-tested with a fake clock).

TourEngine never blocks: it asks its hooks to walk, talk or gesture (asynchronous, each request
carries a token), and the node feeds the answers and the robot's state back through the on_*
methods. tick() runs the timers. The robot is the authority of the tour state; the dashboard and
the handheld only mirror it.
"""

from dataclasses import dataclass
import math

IDLE, STARTING, RUNNING, PAUSED, STOPPING, ERROR = range(6)
STATE_NAMES = ('IDLE', 'STARTING', 'RUNNING', 'PAUSED', 'STOPPING', 'ERROR')
START, PAUSE, RESUME, STOP, SKIP = range(5)
KIND_NAMES = ('START', 'PAUSE', 'RESUME', 'STOP', 'SKIP')

# Which command each state accepts (the dashboard's transition matrix)
TRANSITIONS = {
    IDLE: (START,),
    STARTING: (STOP,),
    RUNNING: (PAUSE, STOP, SKIP),
    PAUSED: (RESUME, STOP),
    STOPPING: (),
    ERROR: (START, STOP),
}
FINISHED = ('done', 'failed', 'cancelled')  # speech states that end an utterance


def can_transition(state: int, kind: int) -> bool:
    """Return True when the state accepts the command."""
    return kind in TRANSITIONS.get(state, ())


@dataclass
class Stop:
    """One stop of a tour: a POI with what to do there."""

    poi_id: str
    name: str
    narration: str = ''
    language: str = ''
    gesture: str = ''
    dwell_sec: float = 0.0


@dataclass
class GuideConfig:
    """Behaviour of the guide, from the node parameters (texts already in the tour language)."""

    language: str = 'id'
    intro_text: str = ''
    outro_text: str = ''
    follow_me_text: str = ''
    gestures: bool = True
    narrate: bool = True
    arrival_timeout_sec: float = 300.0
    go_to_retries: int = 1
    retry_delay_sec: float = 3.0
    on_failure: str = 'skip'  # skip | stop
    gesture_timeout_sec: float = 25.0
    narration_timeout_base_sec: float = 15.0
    narration_chars_per_sec: float = 8.0
    wait_for_visitors: bool = True
    visitor_lost_sec: float = 6.0
    visitor_found_sec: float = 1.0
    visitor_wait_sec: float = 20.0
    visitors_timeout_sec: float = 2.0
    stop_settle_sec: float = 0.5


class Hooks:
    """What the engine asks for; the node implements it with ROS services."""

    def go_to(self, token: int, poi_id: str):
        """Plan and walk to a POI; the answer comes back through on_go_to."""

    def cancel_nav(self):
        """Stop walking."""

    def pause_nav(self, pause: bool):
        """Hold or continue the walk."""

    def say(self, token, text: str, language: str):
        """Speak a text; on_say brings the answer unless the token is None."""

    def stop_speech(self):
        """Stop talking."""

    def gesture(self, token: int, name: str):
        """Run a gesture; the answer comes back through on_gesture."""

    def log(self, level: str, text: str):
        """Log a line (info, warn, error)."""


class TourEngine:
    """Runs one tour at a time: walk, gesture, narrate, dwell at every stop."""

    def __init__(self, cfg: GuideConfig, hooks: Hooks):
        """Start idle."""
        self.cfg, self.hooks = cfg, hooks
        self.state = IDLE
        self.allow_visitor_wait = True  # the node clears it while the handheld drives
        self.estop = False
        self._token = 0
        self._finished_speech = []
        self.reset()

    # ---------------------------------------------------------------- state
    def reset(self):
        """Forget the tour (the engine stays in its state)."""
        self.stops, self.index = [], -1
        self.tour_id = self.tour_name = ''
        self.return_stop = None
        self.phase = None  # intro, leg, motion, narration, dwell, return, outro, stopping
        self.activity = self.message = self.error_reason = ''
        self._expect = None
        self._deadline = None
        self._remaining = None
        self._deferred = []
        self._retries = 0
        self._nav_started_at = -math.inf
        self._go_to_accepted = False
        self._arrived_early = False
        self._nav_paused = False
        self._paused_by_estop = False
        self.reset_visitors(-math.inf)
        self._behind, self._visitors_time = 0, -math.inf

    def reset_visitors(self, now: float):
        """Start the follower check again (new leg, after a pause)."""
        self._lost_since = None
        self._waiting = False
        self._waiting_since = None
        self._found_since = None
        self._wait_done = False

    def current(self):
        """Return the stop being handled (the return stop on the way back), or None."""
        if self.phase == 'return':
            return self.return_stop
        return self.stops[self.index] if 0 <= self.index < len(self.stops) else None

    def snapshot(self) -> dict:
        """Return what /tour/state carries."""
        stop = self.current() if self.state != IDLE else None
        return {
            'state': self.state,
            'tour_id': self.tour_id,
            'tour_name': self.tour_name,
            'index': self.index if self.state != IDLE else -1,
            'total': len(self.stops),
            'poi_id': stop.poi_id if stop else '',
            'poi_name': stop.name if stop else '',
            'activity': self.activity,
            'message': self.message,
            'error_reason': self.error_reason if self.state == ERROR else '',
        }

    def next_token(self) -> int:
        """Return a fresh request token (answers to older requests are ignored)."""
        self._token += 1
        return self._token

    # ---------------------------------------------------------------- commands
    def command(self, kind: int, now: float):
        """Apply PAUSE, RESUME, SKIP or STOP; return (ok, message). START has its own method."""
        if kind not in (PAUSE, RESUME, STOP, SKIP):
            return False, f'unknown command {kind}'
        if not can_transition(self.state, kind):
            return False, f'{KIND_NAMES[kind]} not possible while {STATE_NAMES[self.state]}'
        return {PAUSE: self.pause, RESUME: self.resume, STOP: self.stop, SKIP: self.skip}[kind](
            now
        )

    def start(self, stops: list, now: float, tour_id: str = '', tour_name: str = '',
              return_stop=None):
        """Start a tour of these stops; return (ok, message)."""
        if not can_transition(self.state, START):
            return False, f'START not possible while {STATE_NAMES[self.state]}'
        if not stops:
            return False, 'the tour has no stops'
        self.reset()
        self.stops = list(stops)
        self.tour_id, self.tour_name, self.return_stop = tour_id, tour_name, return_stop
        self.state = STARTING
        self.message = 'starting'
        if self.cfg.narrate and self.cfg.intro_text:
            self.index = 0
            self.speak('intro', self.cfg.intro_text, self.cfg.language, now)
        else:
            self.start_leg(0, now)
        return True, f'tour of {len(self.stops)} stops started'

    def pause(self, now: float, reason: str = 'paused'):
        """Hold the tour: the walk stops, timers freeze; what is said is finished."""
        self.state = PAUSED
        self.message = reason
        if self._deadline is not None:
            self._remaining = max(self._deadline - now, 0.0)
            self._deadline = None
        if self.phase in ('leg', 'return') and not self._nav_paused:
            self.hooks.pause_nav(True)
            self._nav_paused = True
        self._waiting = False
        if self.activity == 'waiting_visitors':
            self.activity = 'navigating'
        return True, reason

    def resume(self, now: float):
        """Continue a paused tour where it was."""
        self.state = RUNNING
        self.message = 'resumed'
        self._paused_by_estop = False
        if self._remaining is not None:
            self._deadline = now + self._remaining
            self._remaining = None
        if self._nav_paused:
            self.hooks.pause_nav(False)
            self._nav_paused = False
        self.reset_visitors(now)
        deferred, self._deferred = self._deferred, []
        for method, args in deferred:
            method(*args, now)
        return True, 'resumed'

    def skip(self, now: float):
        """Leave the current stop and go on with the next one."""
        stop = self.current()
        name = stop.name if stop else ''
        if self.phase in ('leg', 'return'):
            self.hooks.cancel_nav()
        if self.phase in ('intro', 'narration', 'outro'):
            self.hooks.stop_speech()
        if self.phase == 'intro':
            self.start_leg(0, now)
        elif self.phase == 'return':
            self.outro(now)
        elif self.phase == 'outro':
            self.finish(now)
        else:
            self.next_stop(now)
        return True, f'skipped {name}' if name else 'skipped'

    def stop(self, now: float):
        """End the tour: stop walking and talking."""
        if self.state == ERROR:
            self.state = IDLE
            self.reset()
            self.message = 'stopped'
            return True, 'stopped'
        if self.phase in ('leg', 'return'):
            self.hooks.cancel_nav()
        self.hooks.stop_speech()
        if self._nav_paused:
            self.hooks.pause_nav(False)
            self._nav_paused = False
        self.state = STOPPING
        self.phase = 'stopping'
        self.activity, self.message = '', 'stopping'
        self._expect = None
        self._deferred = []
        self._remaining = None
        self._deadline = now + self.cfg.stop_settle_sec
        return True, 'stopping'

    # ---------------------------------------------------------------- steps
    def start_leg(self, index: int, now: float):
        """Walk to stop `index`."""
        self.index = index
        self.phase = 'leg'
        self._retries = 0
        stop = self.stops[index]
        self.activity, self.message = 'navigating', f'walking to {stop.name}'
        self.reset_visitors(now)
        self.go_to(stop.poi_id, now)

    def go_to(self, poi_id: str, now: float):
        """Ask the planner to walk to a POI."""
        if self._nav_paused:
            self.hooks.pause_nav(False)
            self._nav_paused = False
        token = self.next_token()
        self._expect = ('go_to', token)
        self._go_to_accepted = False
        self._arrived_early = False
        self._nav_started_at = now
        self._deadline = now + self.cfg.arrival_timeout_sec
        self.hooks.go_to(token, poi_id)

    def arrived(self, now: float):
        """Go on after arriving at the stop (or back at the return stop)."""
        if self._nav_paused:
            self.hooks.pause_nav(False)
            self._nav_paused = False
        self._waiting = False
        if self.phase == 'return':
            self.outro(now)
        else:
            self.motion(now)

    def leg_failed(self, reason: str, now: float):
        """Handle a failed walk: retry, then skip the stop or stop the tour (on_failure)."""
        stop = self.current()
        name = stop.name if stop else '?'
        if self._retries < self.cfg.go_to_retries:
            self._retries += 1
            self.hooks.log('warn', f'walking to {name} failed ({reason}), retrying')
            self.message = f'retrying {name}: {reason}'
            self._expect = ('retry', self._retries)
            self._deadline = now + self.cfg.retry_delay_sec
            return
        self.hooks.cancel_nav()
        if self.phase == 'return':
            self.hooks.log('warn', f'could not walk back to {name}: {reason}')
            self.outro(now)
        elif self.cfg.on_failure == 'stop':
            self.fail(f'could not reach {name}: {reason}', now)
        else:
            self.hooks.log('warn', f'skipping {name}: {reason}')
            self.next_stop(now, note=f'skipped {name}: {reason}')

    def motion(self, now: float):
        """Play the stop's gesture, if any."""
        stop = self.current()
        if self.cfg.gestures and stop.gesture:
            self.phase = 'motion'
            self.activity, self.message = 'performing_motion', stop.gesture
            token = self.next_token()
            self._expect = ('gesture', token)
            self._deadline = now + self.cfg.gesture_timeout_sec
            self.hooks.gesture(token, stop.gesture)
        else:
            self.narration(now)

    def narration(self, now: float):
        """Say the stop's narration, if any."""
        stop = self.current()
        if self.cfg.narrate and stop.narration:
            self.speak('narration', stop.narration, stop.language or self.cfg.language, now)
        else:
            self.dwell(now)

    def speak(self, phase: str, text: str, language: str, now: float):
        """Say a text and wait until it is said (intro, narration, outro)."""
        self.phase = phase
        self.activity = 'narrating'
        self.message = phase if phase != 'narration' else self.current().name
        token = self.next_token()
        self._expect = ('say', token)
        self._deadline = now + self.cfg.narration_timeout_base_sec + len(text) / max(
            self.cfg.narration_chars_per_sec, 1.0
        )
        self.hooks.say(token, text, language)

    def spoken(self, now: float):
        """Go on once the text of the current phase is said (or could not be)."""
        if self.phase == 'intro':
            self.start_leg(0, now)
        elif self.phase == 'outro':
            self.finish(now)
        else:
            self.dwell(now)

    def dwell(self, now: float):
        """Wait at the stop for its dwell time."""
        stop = self.current()
        if stop.dwell_sec > 0.0:
            self.phase = 'dwell'
            self.activity, self.message = 'dwelling', f'{stop.dwell_sec:g} s at {stop.name}'
            self._expect = ('dwell', 0)
            self._deadline = now + stop.dwell_sec
        else:
            self.next_stop(now)

    def next_stop(self, now: float, note: str = ''):
        """Go on with the next stop, or end the tour."""
        if self.index + 1 < len(self.stops):
            self.start_leg(self.index + 1, now)
        elif self.return_stop is not None:
            self.phase = 'return'
            self._retries = 0
            self.activity = 'returning'
            self.message = f'returning to {self.return_stop.name}'
            self.reset_visitors(now)
            self.go_to(self.return_stop.poi_id, now)
        else:
            self.outro(now)
        if note:
            self.message = note

    def outro(self, now: float):
        """Say goodbye, then end."""
        if self.cfg.narrate and self.cfg.outro_text:
            self.speak('outro', self.cfg.outro_text, self.cfg.language, now)
        else:
            self.finish(now)

    def finish(self, now: float):
        """End the tour normally."""
        name = self.tour_name
        self.state = IDLE
        self.reset()
        self.message = f'tour {name} finished' if name else 'tour finished'

    def fail(self, reason: str, now: float):
        """Stop the tour in ERROR."""
        self.hooks.stop_speech()
        if self._nav_paused:
            self.hooks.pause_nav(False)
            self._nav_paused = False
        self.hooks.log('error', reason)
        self.state = ERROR
        self.phase = None
        self.activity, self.message, self.error_reason = '', reason, reason
        self._expect = None
        self._deadline = None

    # ---------------------------------------------------------------- answers and events
    def defer(self, method, *args) -> bool:
        """Keep an answer for resume() while the tour is paused; True when kept."""
        if self.state == PAUSED:
            self._deferred.append((method, args))
            return True
        return False

    def on_go_to(self, token: int, ok: bool, message: str, now: float):
        """Answer of the planner to a go_to request."""
        if self._expect != ('go_to', token):
            walking = self.phase in ('leg', 'return') and self._expect is not None
            if ok and not walking:
                self.hooks.cancel_nav()  # an answer after STOP: the walk must not start
            return
        if self.defer(self.on_go_to, token, ok, message):
            if ok and self._nav_paused:
                self.hooks.pause_nav(True)  # the walk started while paused: hold it
            return
        if not ok:
            self.leg_failed(message or 'no path', now)
            return
        self._go_to_accepted = True
        stop = self.current()
        self._expect = ('arrive', stop.poi_id)
        if self.state == STARTING:
            self.state = RUNNING
        if self._arrived_early:
            self.arrived(now)

    def on_planner(self, state: str, goal_id: str, stamp: float, now: float):
        """Planner status: arrived, failed or cancelled for the goal being walked to."""
        stop = self.current()
        if self.phase not in ('leg', 'return') or stop is None or goal_id != stop.poi_id:
            return
        if self._expect is None or self._expect[0] not in ('go_to', 'arrive'):
            return
        fresh = stamp >= self._nav_started_at - 0.05 if stamp > 0.0 else self._go_to_accepted
        if not fresh or state not in ('arrived', 'failed', 'cancelled'):
            return
        if self.defer(self.on_planner, state, goal_id, stamp):
            return
        if self._expect[0] == 'go_to':
            self._arrived_early = state == 'arrived'
            return
        if state == 'arrived':
            self.arrived(now)
        else:
            self.leg_failed('walk ' + state, now)

    def on_say(self, token, ok: bool, speech_id: int, message: str, now: float):
        """Answer of the speech node to a say request."""
        if token is None or self._expect != ('say', token):
            return
        if self.defer(self.on_say, token, ok, speech_id, message):
            return
        if self.state == STARTING:
            self.state = RUNNING
        if not ok or not speech_id:
            self.hooks.log('warn', f'could not speak: {message}')
            self.spoken(now)
            return
        self._expect = ('speech', int(speech_id))
        if int(speech_id) in self._finished_speech:
            self.spoken(now)

    def on_speech(self, speech_id: int, state: str, now: float):
        """Speech status: an utterance has been said, failed or was cancelled."""
        if state not in FINISHED:
            return
        self._finished_speech = (self._finished_speech + [int(speech_id)])[-20:]
        if self._expect != ('speech', int(speech_id)):
            return
        if self.defer(self.on_speech, speech_id, state):
            return
        self.spoken(now)

    def on_gesture(self, token: int, ok: bool, message: str, now: float):
        """Answer of a gesture request (when the gesture is over)."""
        if self._expect != ('gesture', token) or self.defer(self.on_gesture, token, ok, message):
            return
        if not ok:
            self.hooks.log('warn', f'gesture failed: {message}')
        self.narration(now)

    def on_visitors(self, behind: int, now: float):
        """Visitors behind the robot (following it)."""
        self._behind, self._visitors_time = int(behind), now

    def on_estop(self, engaged: bool, now: float):
        """Hold the tour while the software e-stop is engaged (until a RESUME)."""
        self.estop = bool(engaged)
        if self.estop and self.state in (STARTING, RUNNING):
            self.pause(now, 'e-stop')
            self._paused_by_estop = True

    # ---------------------------------------------------------------- timers
    def tick(self, now: float):
        """Run the timers: arrival, retries, gestures, narration, dwell, the follower check."""
        if self.state == STOPPING:
            if self._deadline is None or now >= self._deadline:
                self.state = IDLE
                self.reset()
                self.message = 'stopped'
            return
        if self.state not in (STARTING, RUNNING):
            return
        if self.phase in ('leg', 'return'):
            self.check_visitors(now)
        if self._deadline is None or now < self._deadline or self._waiting:
            return
        kind = self._expect[0] if self._expect else None
        self._deadline = None
        if kind == 'retry':
            stop = self.current()
            self.go_to(stop.poi_id, now)
        elif kind in ('go_to', 'arrive'):
            self.leg_failed(f'not there after {self.cfg.arrival_timeout_sec:g} s', now)
        elif kind == 'gesture':
            self.hooks.log('warn', 'gesture timed out')
            self.narration(now)
        elif kind in ('say', 'speech'):
            self.hooks.log('warn', f'{self.phase} not finished in time, going on')
            self.spoken(now)
        elif kind == 'dwell':
            self.next_stop(now)

    def check_visitors(self, now: float):
        """Hold the walk while nobody follows the robot, for up to visitor_wait_sec."""
        cfg = self.cfg
        if not cfg.wait_for_visitors or self.phase != 'leg' or not self._go_to_accepted:
            return
        fresh = now - self._visitors_time <= cfg.visitors_timeout_sec
        if not fresh or not self.allow_visitor_wait:
            if self._waiting:
                self.end_wait(now, found=False)
            self._lost_since = None
            return
        if self._waiting:
            if self._behind > 0:
                self._found_since = self._found_since if self._found_since is not None else now
                if now - self._found_since >= cfg.visitor_found_sec:
                    self.end_wait(now, found=True)
                    return
            else:
                self._found_since = None
            if now - self._waiting_since >= cfg.visitor_wait_sec:
                self.end_wait(now, found=False)
            return
        if self._wait_done:
            return
        if self._behind > 0:
            self._lost_since = None
            return
        self._lost_since = self._lost_since if self._lost_since is not None else now
        if now - self._lost_since >= cfg.visitor_lost_sec:
            self.begin_wait(now)

    def begin_wait(self, now: float):
        """Nobody follows: hold the walk and ask the visitors to come along."""
        self._waiting = True
        self._waiting_since = now
        self._found_since = None
        if not self._nav_paused:
            self.hooks.pause_nav(True)
            self._nav_paused = True
        self.activity, self.message = 'waiting_visitors', 'waiting for the visitors'
        if self.cfg.follow_me_text:
            self.hooks.say(None, self.cfg.follow_me_text, self.cfg.language)

    def end_wait(self, now: float, found: bool):
        """Walk on: the visitors are back, or the wait is over (then no more waits this leg)."""
        if self._deadline is not None and self._waiting_since is not None:
            self._deadline += now - self._waiting_since  # the wait does not count as walking
        self._waiting = False
        self._waiting_since = None
        if self._nav_paused:
            self.hooks.pause_nav(False)
            self._nav_paused = False
        stop = self.current()
        self.activity = 'navigating'
        self.message = f'walking to {stop.name}' if stop else ''
        self._lost_since = now
        if not found:
            self._wait_done = True
