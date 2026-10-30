"""States, commands, stops and hooks of the tour engine (values: molly_interfaces constants)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, IntEnum


class TourStatus(IntEnum):
    """TourState.state."""

    IDLE = 0
    STARTING = 1
    RUNNING = 2
    PAUSED = 3
    STOPPING = 4
    ERROR = 5


class Command(IntEnum):
    """TourCommand.kind."""

    START = 0
    PAUSE = 1
    RESUME = 2
    STOP = 3
    SKIP = 4


class Activity(IntEnum):
    """TourState.activity."""

    NONE = 0
    NAVIGATING = 1
    WAITING_VISITORS = 2
    PERFORMING_MOTION = 3
    NARRATING = 4
    DWELLING = 5
    RETURNING = 6


class Phase(Enum):
    """The step of the tour being run."""

    INTRO = 'intro'
    LEG = 'leg'
    MOTION = 'motion'
    NARRATION = 'narration'
    DWELL = 'dwell'
    RETURN = 'return'
    OUTRO = 'outro'
    STOPPING = 'stopping'


WALKING = (Phase.LEG, Phase.RETURN)
SPEAKING = (Phase.INTRO, Phase.NARRATION, Phase.OUTRO)


class Expect(Enum):
    """What the engine waits for."""

    GO_TO = 'go_to'  # the planner's answer to planner/go_to
    ARRIVE = 'arrive'  # planner/status ARRIVED for the goal
    RETRY = 'retry'  # the delay before another walk
    GESTURE = 'gesture'
    SAY = 'say'  # the answer to interaction/say
    SPEECH = 'speech'  # interaction/speech_status finished for the utterance
    DWELL = 'dwell'


class Walk(Enum):
    """What planner/status says about a walk."""

    UNDER_WAY = 'under_way'
    ARRIVED = 'arrived'
    FAILED = 'failed'
    CANCELLED = 'cancelled'


# Which command each state accepts (the dashboard's transition matrix)
TRANSITIONS = {
    TourStatus.IDLE: (Command.START,),
    TourStatus.STARTING: (Command.STOP,),
    TourStatus.RUNNING: (Command.PAUSE, Command.STOP, Command.SKIP),
    TourStatus.PAUSED: (Command.RESUME, Command.STOP),
    TourStatus.STOPPING: (),
    TourStatus.ERROR: (Command.START, Command.STOP),
}


def can_transition(state: TourStatus, kind: Command) -> bool:
    """Return True when the state accepts the command."""
    return kind in TRANSITIONS.get(state, ())


@dataclass(frozen=True)
class Stop:
    """One stop of a tour: a POI (map frame) and what to do there."""

    poi_id: str
    name: str
    x: float
    y: float
    yaw: float
    narration: str = ''
    language: str = ''
    gesture: str = ''
    dwell_sec: float = 0.0


@dataclass(frozen=True)
class TourTexts:
    """The guide's own texts, already in its language ('' : not said)."""

    intro: str = ''
    outro: str = ''
    follow_me: str = ''


@dataclass(frozen=True)
class Snapshot:
    """What tour/state carries."""

    state: TourStatus
    activity: Activity
    tour_id: str
    tour_name: str
    index: int
    total: int
    poi_id: str
    poi_name: str
    message: str
    error_reason: str


class Hooks(ABC):
    """What the engine asks for; the node implements it with ROS services (all asynchronous)."""

    @abstractmethod
    def go_to(self, token: int, stop: Stop):
        """Plan and walk to a stop; the answer comes back through on_go_to."""

    @abstractmethod
    def cancel_nav(self):
        """Stop walking."""

    @abstractmethod
    def pause_nav(self, pause: bool):
        """Hold (True) or continue (False) the walk."""

    @abstractmethod
    def say(self, token: int | None, text: str, language: str):
        """Speak a text; on_say brings the answer unless the token is None."""

    @abstractmethod
    def stop_speech(self):
        """Stop talking."""

    @abstractmethod
    def gesture(self, token: int, name: str):
        """Run a gesture; the answer comes back through on_gesture."""

    @abstractmethod
    def report(self, level: str, text: str):
        """Log a line (info, warn, error)."""


def text_for(items, language: str) -> str:
    """Pick the text of a language from ['id:...', 'en:...'] (Indonesian, then the first one)."""
    texts: dict[str, str] = {}
    for item in items or ():
        key, sep, value = str(item).partition(':')
        if sep and value.strip():
            texts.setdefault(key.strip().lower(), value.strip())
    return texts.get(language) or texts.get('id') or next(iter(texts.values()), '')
