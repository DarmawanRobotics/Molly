"""The steps of a tour: walk, gesture, narrate, dwell, return, outro (no ROS)."""

from __future__ import annotations

import math

from molly_guide.core.config import GuideConfig
from molly_guide.core.tour.followers import FollowerWatch, WatchConfig
from molly_guide.core.tour.model import (
    WALKING,
    Activity,
    Expect,
    Hooks,
    Phase,
    Snapshot,
    Stop,
    TourStatus,
    TourTexts,
)

FINISHED_SPEECH_KEPT = 20


def watch_config(cfg: GuideConfig) -> WatchConfig:
    """Return the follower check timings of a guide configuration."""
    return WatchConfig(
        cfg.wait_for_visitors,
        cfg.visitor_lost_sec,
        cfg.visitor_found_sec,
        cfg.visitor_wait_sec,
        cfg.visitors_timeout_sec,
    )


class TourSteps:
    """The tour state and the steps from one stop to the next; TourEngine adds the commands."""

    def __init__(self, cfg: GuideConfig, texts: TourTexts, hooks: Hooks):
        self.cfg, self.texts, self.hooks = cfg, texts, hooks
        self.state = TourStatus.IDLE
        self.allow_visitor_wait = True  # False while an operator drives (REMOTE)
        self.estop = False
        self.watch = FollowerWatch(watch_config(cfg))
        self._token = 0
        self._finished_speech: list[int] = []
        self.reset()

    def reset(self):
        """Forget the tour (the engine stays in its state)."""
        self.stops: list[Stop] = []
        self.index = -1
        self.tour_id = self.tour_name = ''
        self.return_stop: Stop | None = None
        self.phase: Phase | None = None
        self.activity = Activity.NONE
        self.message = self.error_reason = ''
        self._expect: tuple[Expect, object] | None = None
        self._deadline: float | None = None
        self._remaining: float | None = None
        self._deferred: list = []
        self._retries = 0
        self._nav_started_at = -math.inf
        self._go_to_accepted = False
        self._arrived_early = False
        self._nav_paused = False
        self._paused_by_estop = False
        self.watch.reset()

    def current(self) -> Stop | None:
        """Return the stop being handled (the return stop on the way back), or None."""
        if self.phase is Phase.RETURN:
            return self.return_stop
        return self.stops[self.index] if 0 <= self.index < len(self.stops) else None

    def snapshot(self) -> Snapshot:
        """Return what tour/state carries."""
        idle = self.state is TourStatus.IDLE
        stop = None if idle else self.current()
        return Snapshot(
            state=self.state,
            activity=self.activity,
            tour_id=self.tour_id,
            tour_name=self.tour_name,
            index=-1 if idle else self.index,
            total=len(self.stops),
            poi_id=stop.poi_id if stop else '',
            poi_name=stop.name if stop else '',
            message=self.message,
            error_reason=self.error_reason if self.state is TourStatus.ERROR else '',
        )

    def next_token(self) -> int:
        """Return a fresh request token (answers to older requests are ignored)."""
        self._token += 1
        return self._token

    def hold_walk(self, hold: bool):
        """Pause or continue the walk, once each way."""
        if hold != self._nav_paused:
            self.hooks.pause_nav(hold)
            self._nav_paused = hold

    # ---------------------------------------------------------------- steps
    def start_leg(self, index: int, now: float):
        """Walk to stop `index`."""
        self.index = index
        self.phase = Phase.LEG
        self._retries = 0
        stop = self.stops[index]
        self.activity, self.message = Activity.NAVIGATING, f'walking to {stop.name}'
        self.watch.reset()
        self.go_to(stop, now)

    def go_to(self, stop: Stop, now: float):
        """Ask the planner to walk to a stop."""
        self.hold_walk(False)
        token = self.next_token()
        self._expect = (Expect.GO_TO, token)
        self._go_to_accepted = False
        self._arrived_early = False
        self._nav_started_at = now
        self._deadline = now + self.cfg.arrival_timeout_sec
        self.hooks.go_to(token, stop)

    def arrived(self, now: float):
        """Go on after arriving at the stop (or back at the return stop)."""
        self.hold_walk(False)
        self.watch.reset()
        if self.phase is Phase.RETURN:
            self.outro(now)
        else:
            self.motion(now)

    def leg_failed(self, reason: str, now: float):
        """Handle a failed walk: retry, then skip the stop or stop the tour (on_failure)."""
        stop = self.current()
        name = stop.name if stop else '?'
        if self._retries < self.cfg.go_to_retries:
            self._retries += 1
            self.hooks.report('warn', f'walking to {name} failed ({reason}), retrying')
            self.message = f'retrying {name}: {reason}'
            self._expect = (Expect.RETRY, self._retries)
            self._deadline = now + self.cfg.retry_delay_sec
            return
        self.hooks.cancel_nav()
        if self.phase is Phase.RETURN:
            self.hooks.report('warn', f'could not walk back to {name}: {reason}')
            self.outro(now)
        elif self.cfg.on_failure == 'stop':
            self.fail(f'could not reach {name}: {reason}', now)
        else:
            self.hooks.report('warn', f'skipping {name}: {reason}')
            self.next_stop(now, note=f'skipped {name}: {reason}')

    def motion(self, now: float):
        """Play the stop's gesture, if any."""
        stop = self.current()
        if not (self.cfg.gestures and stop.gesture):
            self.narration(now)
            return
        self.phase = Phase.MOTION
        self.activity, self.message = Activity.PERFORMING_MOTION, stop.gesture
        token = self.next_token()
        self._expect = (Expect.GESTURE, token)
        self._deadline = now + self.cfg.gesture_timeout_sec
        self.hooks.gesture(token, stop.gesture)

    def narration(self, now: float):
        """Say the stop's narration, if any."""
        stop = self.current()
        if self.cfg.narrate and stop.narration:
            self.speak(Phase.NARRATION, stop.narration, stop.language or self.cfg.language, now)
        else:
            self.dwell(now)

    def speak(self, phase: Phase, text: str, language: str, now: float):
        """Say a text and wait until it is said (intro, narration, outro)."""
        self.phase = phase
        self.activity = Activity.NARRATING
        self.message = self.current().name if phase is Phase.NARRATION else phase.value
        token = self.next_token()
        self._expect = (Expect.SAY, token)
        rate = max(self.cfg.narration_chars_per_sec, 1.0)
        self._deadline = now + self.cfg.narration_timeout_base_sec + len(text) / rate
        self.hooks.say(token, text, language)

    def spoken(self, now: float):
        """Go on once the text of the current phase is said (or could not be)."""
        if self.phase is Phase.INTRO:
            self.start_leg(0, now)
        elif self.phase is Phase.OUTRO:
            self.finish()
        else:
            self.dwell(now)

    def dwell(self, now: float):
        """Wait at the stop for its dwell time."""
        stop = self.current()
        if stop.dwell_sec <= 0.0:
            self.next_stop(now)
            return
        self.phase = Phase.DWELL
        self.activity, self.message = Activity.DWELLING, f'{stop.dwell_sec:g} s at {stop.name}'
        self._expect = (Expect.DWELL, 0)
        self._deadline = now + stop.dwell_sec

    def next_stop(self, now: float, note: str = ''):
        """Go on with the next stop, or end the tour."""
        if self.index + 1 < len(self.stops):
            self.start_leg(self.index + 1, now)
        elif self.return_stop is not None:
            self.phase = Phase.RETURN
            self._retries = 0
            self.activity = Activity.RETURNING
            self.message = f'returning to {self.return_stop.name}'
            self.watch.reset()
            self.go_to(self.return_stop, now)
        else:
            self.outro(now)
        if note:
            self.message = note

    def outro(self, now: float):
        """Say goodbye, then end."""
        if self.cfg.narrate and self.texts.outro:
            self.speak(Phase.OUTRO, self.texts.outro, self.cfg.language, now)
        else:
            self.finish()

    def finish(self):
        """End the tour normally."""
        name = self.tour_name
        self.state = TourStatus.IDLE
        self.reset()
        self.message = f'tour {name} finished' if name else 'tour finished'

    def fail(self, reason: str, now: float):
        """Stop the tour in ERROR."""
        self.hooks.stop_speech()
        self.hold_walk(False)
        self.hooks.report('error', reason)
        self.state = TourStatus.ERROR
        self.phase = None
        self.activity, self.message, self.error_reason = Activity.NONE, reason, reason
        self._expect = None
        self._deadline = None

    def walking(self) -> bool:
        """Return True while a walk was asked for and is not over."""
        return self.phase in WALKING and self._expect is not None

    def note_finished_speech(self, speech_id: int):
        """Remember an utterance that ended (its answer may still come)."""
        self._finished_speech = [*self._finished_speech, int(speech_id)][-FINISHED_SPEECH_KEPT:]
