"""
The tour state machine (no ROS, unit-tested with a fake clock).

TourEngine never blocks: it asks its hooks to walk, talk or gesture (asynchronous, each request
carries a token), and the node feeds the answers and the robot's state back through the on_*
methods. tick() runs the timers. The robot is the authority of the tour state; the dashboard and
the handheld only mirror it.
"""

from __future__ import annotations

import math

from molly_guide.core.tour.followers import WatchEvent
from molly_guide.core.tour.model import (
    SPEAKING,
    WALKING,
    Activity,
    Command,
    Expect,
    Phase,
    Stop,
    TourStatus,
    Walk,
    can_transition,
)
from molly_guide.core.tour.steps import TourSteps

S = TourStatus


class TourEngine(TourSteps):
    """Runs one tour at a time: walk, gesture, narrate, dwell at every stop."""

    # ---------------------------------------------------------------- commands
    def start(
        self,
        stops: list[Stop],
        now: float,
        tour_id: str = '',
        tour_name: str = '',
        return_stop: Stop | None = None,
    ) -> tuple[bool, str]:
        """Start a tour of these stops; return (ok, message)."""
        if not can_transition(self.state, Command.START):
            return False, f'START not possible while {self.state.name}'
        if not stops:
            return False, 'the tour has no stops'
        self.reset()
        self.stops = list(stops)
        self.tour_id, self.tour_name, self.return_stop = tour_id, tour_name, return_stop
        self.state = S.STARTING
        self.message = 'starting'
        if self.cfg.narrate and self.texts.intro:
            self.index = 0
            self.speak(Phase.INTRO, self.texts.intro, self.cfg.language, now)
        else:
            self.start_leg(0, now)
        return True, f'tour of {len(self.stops)} stops started'

    def command(self, kind: Command, now: float) -> tuple[bool, str]:
        """Apply PAUSE, RESUME, SKIP or STOP; return (ok, message). START has its own method."""
        handlers = {
            Command.PAUSE: self.pause,
            Command.RESUME: self.resume,
            Command.STOP: self.stop,
            Command.SKIP: self.skip,
        }
        if kind not in handlers:
            return False, f'unknown command {kind}'
        if not can_transition(self.state, kind):
            return False, f'{kind.name} not possible while {self.state.name}'
        return handlers[kind](now)

    def pause(self, now: float, reason: str = 'paused') -> tuple[bool, str]:
        """Hold the tour: the walk stops, timers freeze; what is said is finished."""
        self.state = S.PAUSED
        self.message = reason
        if self._deadline is not None:
            self._remaining = max(self._deadline - now, 0.0)
            self._deadline = None
        if self.phase in WALKING:
            self.hold_walk(True)
        if self.watch.waiting:
            self.watch.end(now, found=False)
        if self.activity is Activity.WAITING_VISITORS:
            self.activity = Activity.NAVIGATING
        return True, reason

    def resume(self, now: float) -> tuple[bool, str]:
        """Continue a paused tour where it was."""
        if self.estop:
            return False, 'release the e-stop first'
        self.state = S.RUNNING
        self.message = 'resumed'
        self._paused_by_estop = False
        if self._remaining is not None:
            self._deadline = now + self._remaining
            self._remaining = None
        self.hold_walk(False)
        self.watch.reset()
        deferred, self._deferred = self._deferred, []
        for method, args in deferred:
            method(*args, now)
        return True, 'resumed'

    def skip(self, now: float) -> tuple[bool, str]:
        """Leave the current stop and go on with the next one."""
        stop = self.current()
        name = stop.name if stop else ''
        if self.phase in WALKING:
            self.hooks.cancel_nav()
        if self.phase in SPEAKING:
            self.hooks.stop_speech()
        if self.phase is Phase.INTRO:
            self.start_leg(0, now)
        elif self.phase is Phase.RETURN:
            self.outro(now)
        elif self.phase is Phase.OUTRO:
            self.finish()
        else:
            self.next_stop(now)
        return True, f'skipped {name}' if name else 'skipped'

    def stop(self, now: float) -> tuple[bool, str]:
        """End the tour: stop walking and talking."""
        if self.state is S.ERROR:
            self.state = S.IDLE
            self.reset()
            self.message = 'stopped'
            return True, 'stopped'
        if self.phase in WALKING:
            self.hooks.cancel_nav()
        self.hooks.stop_speech()
        self.hold_walk(False)
        self.state = S.STOPPING
        self.phase = Phase.STOPPING
        self.activity, self.message = Activity.NONE, 'stopping'
        self._expect = None
        self._deferred = []
        self._remaining = None
        self._deadline = now + self.cfg.stop_settle_sec
        return True, 'stopping'

    # ---------------------------------------------------------------- answers and events
    def defer(self, method, *args) -> bool:
        """Keep an answer for resume() while the tour is paused; True when kept."""
        if self.state is S.PAUSED:
            self._deferred.append((method, args))
            return True
        return False

    def on_go_to(self, token: int, ok: bool, message: str, now: float):
        """Answer of the planner to a go_to request."""
        if self._expect != (Expect.GO_TO, token):
            if ok and not self.walking():
                self.hooks.cancel_nav()  # an answer after STOP: the walk must not start
            return
        if self.defer(self.on_go_to, token, ok, message):
            if ok:  # the walk started while paused: hold it (again)
                self.hooks.pause_nav(True)
                self._nav_paused = True
            return
        if not ok:
            self.leg_failed(message or 'no path', now)
            return
        self._go_to_accepted = True
        self._expect = (Expect.ARRIVE, token)
        if self.state is S.STARTING:
            self.state = S.RUNNING
        if self._arrived_early:
            self.arrived(now)

    def matches(self, goal: tuple[float, float]) -> bool:
        """Return True when a planner goal is the stop being walked to."""
        stop = self.current()
        if stop is None:
            return False
        return math.hypot(goal[0] - stop.x, goal[1] - stop.y) <= self.cfg.goal_tolerance

    def on_planner(self, walk: Walk, goal: tuple[float, float], stamp: float, now: float):
        """planner/status: arrived, failed or cancelled for the goal being walked to."""
        if self.phase not in WALKING or walk is Walk.UNDER_WAY or not self.matches(goal):
            return
        if self._expect is None or self._expect[0] not in (Expect.GO_TO, Expect.ARRIVE):
            return
        fresh = stamp >= self._nav_started_at - 0.05 if stamp > 0.0 else self._go_to_accepted
        if not fresh or self.defer(self.on_planner, walk, goal, stamp):
            return
        if self._expect[0] is Expect.GO_TO:
            self._arrived_early = walk is Walk.ARRIVED
            return
        if walk is Walk.ARRIVED:
            self.arrived(now)
        else:
            self.leg_failed(f'walk {walk.value}', now)

    def on_say(self, token: int | None, ok: bool, speech_id: int, message: str, now: float):
        """Answer of the speech node to a say request."""
        if token is None or self._expect != (Expect.SAY, token):
            return
        if self.defer(self.on_say, token, ok, speech_id, message):
            return
        if self.state is S.STARTING:
            self.state = S.RUNNING
        if not ok or not speech_id:
            self.hooks.report('warn', f'could not speak: {message}')
            self.spoken(now)
            return
        self._expect = (Expect.SPEECH, int(speech_id))
        if int(speech_id) in self._finished_speech:
            self.spoken(now)

    def on_speech(self, speech_id: int, finished: bool, now: float):
        """Speech status: an utterance has been said, failed or was cancelled."""
        if not finished:
            return
        self.note_finished_speech(speech_id)
        if self._expect != (Expect.SPEECH, int(speech_id)):
            return
        if self.defer(self.on_speech, speech_id, finished):
            return
        self.spoken(now)

    def on_gesture(self, token: int, ok: bool, message: str, now: float):
        """Answer of a gesture request (when the gesture is over)."""
        if self._expect != (Expect.GESTURE, token):
            return
        if self.defer(self.on_gesture, token, ok, message):
            return
        if not ok:
            self.hooks.report('warn', f'gesture failed: {message}')
        self.narration(now)

    def on_visitors(self, behind: int, now: float):
        """Visitors behind the robot (following it)."""
        self.watch.update(behind, now)

    def on_estop(self, engaged: bool, now: float):
        """Hold the tour while the software e-stop is engaged (until a RESUME)."""
        self.estop = bool(engaged)
        if self.estop and self.state in (S.STARTING, S.RUNNING):
            self.pause(now, 'e-stop')
            self._paused_by_estop = True

    # ---------------------------------------------------------------- timers
    def tick(self, now: float):
        """Run the timers: arrival, retries, gestures, narration, dwell, the follower check."""
        if self.state is S.STOPPING:
            if self._deadline is None or now >= self._deadline:
                self.state = S.IDLE
                self.reset()
                self.message = 'stopped'
            return
        if self.state not in (S.STARTING, S.RUNNING):
            return
        if self.phase is Phase.LEG and self._go_to_accepted:
            self.check_visitors(now)
        if self._deadline is None or now < self._deadline or self.watch.waiting:
            return
        kind = self._expect[0] if self._expect else None
        self._deadline = None
        if kind is Expect.RETRY:
            self.go_to(self.current(), now)
        elif kind in (Expect.GO_TO, Expect.ARRIVE):
            self.leg_failed(f'not there after {self.cfg.arrival_timeout_sec:g} s', now)
        elif kind is Expect.GESTURE:
            self.hooks.report('warn', 'gesture timed out')
            self.narration(now)
        elif kind in (Expect.SAY, Expect.SPEECH):
            self.hooks.report('warn', f'{self.phase.value} not finished in time, going on')
            self.spoken(now)
        elif kind is Expect.DWELL:
            self.next_stop(now)

    def check_visitors(self, now: float):
        """Hold the walk while nobody follows the robot, for up to visitor_wait_sec."""
        event = self.watch.check(now, self.allow_visitor_wait)
        if event is WatchEvent.BEGIN_WAIT:
            self.hold_walk(True)
            self.activity, self.message = Activity.WAITING_VISITORS, 'waiting for the visitors'
            if self.texts.follow_me:
                self.hooks.say(None, self.texts.follow_me, self.cfg.language)
        elif event is not None:
            if self._deadline is not None:
                self._deadline += self.watch.last_wait  # the wait does not count as walking
            self.hold_walk(False)
            stop = self.current()
            self.activity = Activity.NAVIGATING
            self.message = f'walking to {stop.name}' if stop else ''
