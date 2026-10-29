"""
guide_node: Molly's tours.

Walks from POI to POI with the path finder (planner/go_to), plays the POI's gesture, says its
narration, dwells, and holds the walk when nobody follows.

The robot owns the tour state (tour/state); the dashboard and the handheld mirror it.
"""

from __future__ import annotations

import threading

from rclpy.callback_groups import ReentrantCallbackGroup

from dharma_interfaces.msg import ControlStatus, LocalizationStatus, PlannerStatus
from dharma_interfaces.srv import GoTo
from dharma_utils.ros import LATCHED, DharmaNode, run
from molly_guide.core.config import GuideConfig
from molly_guide.core.tour import Command, TourEngine, TourStatus, TourTexts, text_for
from molly_guide.core.tour.request import RobotState, not_ready, resolve
from molly_guide.core.tour.steps import watch_config
from molly_guide.utils import convert
from molly_guide.utils.hooks import GuideHooks
from molly_interfaces.msg import (
    PoiArray,
    SpeechStatus,
    TourArray,
    TourCommand,
    TourState,
    Visitors,
)
from molly_interfaces.srv import GetTourState, GoToPoi, TourControl

FINISHED_SPEECH = (SpeechStatus.DONE, SpeechStatus.FAILED, SpeechStatus.CANCELLED)
PLANNER_TIMEOUT_SEC = 30.0  # planner/go_to plans and starts the follower before it answers


def texts_of(cfg: GuideConfig) -> TourTexts:
    """Return the guide's texts in its language."""
    language = cfg.language.strip().lower() or 'id'
    return TourTexts(
        text_for(cfg.intro_texts, language),
        text_for(cfg.outro_texts, language),
        text_for(cfg.follow_me_texts, language),
    )


class GuideNode(DharmaNode):
    """Runs tours: walks from POI to POI, gestures, narrates and waits for the visitors."""

    def __init__(self, **kwargs):
        super().__init__('guide_node', **kwargs)
        self.config = self.declare_config(GuideConfig)
        self.lock = threading.RLock()
        self.clients_group = ReentrantCallbackGroup()
        self.hooks = GuideHooks(self, self.clients_group, self.lock)
        self.engine = TourEngine(self.config.value, texts_of(self.config.value), self.hooks)
        self.hooks.engine = self.engine
        self.config.on_change(self.on_config)
        self.site, self.stops, self.tours = '', {}, {}
        self.robot = RobotState()
        self.last_published, self.last_publish_time = None, 0.0
        self.state_pub = self.latched(TourState, 'tour/state')
        self.planner = self.create_client(GoTo, 'planner/go_to', callback_group=self.clients_group)
        sub = self.create_subscription
        sub(TourCommand, 'tour/cmd', self.locked(self.on_command_topic), 10)
        sub(PoiArray, 'site/pois', self.locked(self.on_pois), LATCHED)
        sub(TourArray, 'site/tours', self.locked(self.on_tours), LATCHED)
        sub(PlannerStatus, 'planner/status', self.locked(self.on_planner), LATCHED)
        sub(SpeechStatus, 'interaction/speech_status', self.locked(self.on_speech), LATCHED)
        sub(Visitors, 'perception/visitors', self.locked(self.on_visitors), 10)
        sub(ControlStatus, 'control/status', self.locked(self.on_control), LATCHED)
        sub(LocalizationStatus, 'localization/status', self.locked(self.on_localization), LATCHED)
        self.create_service(TourControl, 'tour/control', self.locked(self.on_control_service))
        self.create_service(GetTourState, 'tour/get_state', self.locked(self.on_get_state))
        self.create_service(
            GoToPoi, 'guide/go_to_poi', self.on_go_to_poi, callback_group=ReentrantCallbackGroup()
        )
        self.every(1.0 / self.config.tick_hz, self.locked(self.tick))
        self.publish_state(force=True)
        self.log.info('guide ready', language=self.config.language)

    def locked(self, fn):
        """Return fn running under the engine lock."""

        def run_locked(*args):
            with self.lock:
                return fn(*args)

        return run_locked

    def now(self) -> float:
        """Return the ROS time in seconds (the clock of the status stamps)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_config(self, old, new):
        """Apply live parameter changes to the engine."""
        with self.lock:
            self.engine.cfg = new
            self.engine.watch.config = watch_config(new)

    # ---------------------------------------------------------------- inputs
    def on_pois(self, msg: PoiArray):
        """Keep the active site's POIs as tour stops."""
        self.site = msg.site
        self.stops = {poi.id: convert.stop_of(poi) for poi in msg.pois}

    def on_tours(self, msg: TourArray):
        """Keep the active site's tours."""
        self.tours = {tour.id: convert.tour_dict(tour) for tour in msg.tours}

    def on_planner(self, msg: PlannerStatus):
        """Planner status: arrival or failure of the current walk."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        goal = (msg.goal_x, msg.goal_y)
        self.engine.on_planner(convert.walk_of(msg), goal, stamp, self.now())
        self.publish_state()

    def on_speech(self, msg: SpeechStatus):
        """Speech status: the narration is over."""
        self.engine.on_speech(msg.id, msg.state in FINISHED_SPEECH, self.now())
        self.publish_state()

    def on_visitors(self, msg: Visitors):
        """Visitors following the robot."""
        self.engine.on_visitors(msg.behind, self.now())

    def on_control(self, msg: ControlStatus):
        """Mode and e-stop of the control mux: the e-stop pauses the tour, REMOTE the waits."""
        auto = msg.mode == ControlStatus.MODE_AUTO
        self.robot = RobotState(self.robot.localized, auto, bool(msg.estop))
        self.engine.allow_visitor_wait = auto
        self.engine.on_estop(msg.estop, self.now())
        self.publish_state()

    def on_localization(self, msg: LocalizationStatus):
        """Localization: a tour needs the robot on the map."""
        localized = msg.state != LocalizationStatus.NONE
        self.robot = RobotState(localized, self.robot.auto, self.robot.estop)

    # ---------------------------------------------------------------- commands
    def apply_command(self, cmd: TourCommand) -> tuple[bool, str]:
        """Apply a tour command; return (ok, message). The state is republished either way."""
        try:
            kind = Command(int(cmd.kind))
        except ValueError:
            return False, f'unknown command {cmd.kind}'
        if kind is Command.START:
            ok, message = self.start(cmd)
        else:
            ok, message = self.engine.command(kind, self.now())
        level = self.log.info if ok else self.log.warning
        level(f'tour {kind.name}' + ('' if ok else ' refused'), detail=message)
        self.publish_state(force=True)
        return ok, message

    def start(self, cmd: TourCommand) -> tuple[bool, str]:
        """START a saved tour or a list of POIs, when the robot is ready."""
        if self.engine.state not in (TourStatus.IDLE, TourStatus.ERROR):
            return False, f'START not possible while {self.engine.state.name}'
        cfg = self.config.value
        plan, error = resolve(cmd.tour_id, cmd.poi_ids, self.stops, self.tours, self.site)
        error = error or not_ready(self.robot, cfg.require_localization, cfg.require_auto_mode)
        if plan is None or error:
            return False, error
        return self.engine.start(plan.stops, self.now(), plan.tour_id, plan.name, plan.return_stop)

    def on_command_topic(self, msg: TourCommand):
        """Apply a tour/cmd command (no answer)."""
        self.apply_command(msg)

    def on_control_service(self, request, response):
        """Apply a tour/control command and answer."""
        response.success, response.message = self.apply_command(request.command)
        return response

    def on_get_state(self, _request, response):
        """Answer tour/get_state (for a client that just connected)."""
        stamp = self.get_clock().now().to_msg()
        response.state = convert.tour_state(self.engine.snapshot(), stamp)
        return response

    def on_go_to_poi(self, request, response):
        """Service: walk to one POI with the path finder (not while a tour runs)."""
        with self.lock:
            stop = self.stops.get(request.id.strip())
            busy = self.engine.state not in (TourStatus.IDLE, TourStatus.ERROR)
            cfg = self.config.value
            error = not_ready(self.robot, cfg.require_localization, cfg.require_auto_mode)
        response.success = False
        if stop is None:
            response.message = f'no POI {request.id!r} in site {self.site!r}'
        elif busy:
            response.message = 'a tour is running: stop it first'
        elif error:
            response.message = error
        elif not self.planner.service_is_ready():
            response.message = 'planner not running'
        else:
            done = threading.Event()
            future = self.planner.call_async(GoTo.Request(x=stop.x, y=stop.y, yaw=stop.yaw))
            future.add_done_callback(lambda _: done.set())
            if done.wait(PLANNER_TIMEOUT_SEC) and future.result() is not None:
                result = future.result()
                response.success, response.message = result.success, result.message
                response.message = response.message or f'walking to {stop.name}'
            else:
                response.message = 'planner/go_to did not answer'
        return response

    # ---------------------------------------------------------------- output
    def tick(self):
        """Run the engine's timers and publish the state."""
        self.engine.tick(self.now())
        self.publish_state()

    def publish_state(self, force: bool = False):
        """Publish tour/state on a change and every publish_period_sec."""
        snapshot = self.engine.snapshot()
        now = self.now()
        unchanged = snapshot == self.last_published
        recent = now - self.last_publish_time < self.config.publish_period_sec
        if not force and unchanged and recent:
            return
        if not unchanged:
            self.log.info(
                f'{snapshot.state.name} {snapshot.activity.name}',
                poi=snapshot.poi_id or '-',
                detail=snapshot.message,
            )
        self.last_published, self.last_publish_time = snapshot, now
        self.state_pub.publish(convert.tour_state(snapshot, self.get_clock().now().to_msg()))


def main(args=None):
    """Run the guide (a walk to a POI waits for the planner on a second thread)."""
    run(GuideNode, threads=2, args=args)


if __name__ == '__main__':
    main()
