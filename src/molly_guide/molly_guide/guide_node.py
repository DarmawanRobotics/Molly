#!/usr/bin/env python3
from dharma_interfaces.msg import PlannerStatus, PoiArray, TourArray, Visitors
from dharma_interfaces.srv import GoTo
from molly_guide.tour import (
    can_transition,
    GuideConfig,
    Hooks,
    KIND_NAMES,
    RESUME,
    START,
    STATE_NAMES,
    Stop,
    TourEngine,
)
from molly_interfaces.msg import SpeechStatus, TourCommand, TourState
from molly_interfaces.srv import Gesture, GetTourState, Say, TourControl
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


def text_for(items, language: str) -> str:
    """Pick the text of a language from ['id:...', 'en:...'] (Indonesian, then the first one)."""
    texts = {}
    for item in items or []:
        key, sep, value = str(item).partition(':')
        if sep and value.strip():
            texts.setdefault(key.strip().lower(), value.strip())
    return texts.get(language) or texts.get('id') or next(iter(texts.values()), '')


def stamp_seconds(stamp) -> float:
    """Return a builtin_interfaces/Time in seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class GuideNode(Node, Hooks):
    """Runs tours: walks from POI to POI, gestures, narrates and waits for the visitors."""

    def __init__(self):
        """Declare params and wire the tour commands, the state and the robot services."""
        super().__init__('guide_node')
        defaults = {
            'language': 'id',
            'intro_texts': [
                'id:Halo semuanya, saya Molly. Mari ikuti saya, kita mulai turnya.',
                "en:Hello everyone, I am Molly. Please follow me, let's start the tour.",
            ],
            'outro_texts': [
                'id:Turnya sudah selesai. Terima kasih sudah berkeliling bersama saya!',
                'en:That is the end of the tour. Thank you for walking with me!',
            ],
            'follow_me_texts': [
                'id:Ayo, ikuti saya ya.',
                'en:Please follow me.',
            ],
            'gestures': True,
            'narrate': True,
            'arrival_timeout_sec': 300.0,
            'go_to_retries': 1,
            'retry_delay_sec': 3.0,
            'on_failure': 'skip',
            'gesture_timeout_sec': 25.0,
            'narration_timeout_base_sec': 15.0,
            'narration_chars_per_sec': 8.0,
            'wait_for_visitors': True,
            'visitor_lost_sec': 6.0,
            'visitor_found_sec': 1.0,
            'visitor_wait_sec': 20.0,
            'visitors_timeout_sec': 2.0,
            'require_localization': True,
            'require_auto_mode': True,
            'tick_hz': 10.0,
            'publish_period_sec': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        language = str(p('language')).strip().lower() or 'id'
        on_failure = str(p('on_failure')).strip().lower()
        if on_failure not in ('skip', 'stop'):
            self.get_logger().error(f'on_failure {on_failure!r} unknown, using skip')
            on_failure = 'skip'
        cfg = GuideConfig(
            language=language,
            intro_text=text_for(p('intro_texts'), language),
            outro_text=text_for(p('outro_texts'), language),
            follow_me_text=text_for(p('follow_me_texts'), language),
            gestures=bool(p('gestures')),
            narrate=bool(p('narrate')),
            arrival_timeout_sec=float(p('arrival_timeout_sec')),
            go_to_retries=max(int(p('go_to_retries')), 0),
            retry_delay_sec=float(p('retry_delay_sec')),
            on_failure=on_failure,
            gesture_timeout_sec=float(p('gesture_timeout_sec')),
            narration_timeout_base_sec=float(p('narration_timeout_base_sec')),
            narration_chars_per_sec=float(p('narration_chars_per_sec')),
            wait_for_visitors=bool(p('wait_for_visitors')),
            visitor_lost_sec=float(p('visitor_lost_sec')),
            visitor_found_sec=float(p('visitor_found_sec')),
            visitor_wait_sec=float(p('visitor_wait_sec')),
            visitors_timeout_sec=float(p('visitors_timeout_sec')),
        )
        self._require_localization = bool(p('require_localization'))
        self._require_auto = bool(p('require_auto_mode'))
        self._publish_period = float(p('publish_period_sec'))
        self._engine = TourEngine(cfg, self)
        self._site, self._pois, self._tours = '', {}, {}
        self._localization = None
        self._mode = None
        self._last_published = None
        self._last_publish_time = 0.0

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(TourState, '/tour/state', latched)
        self._go_to = self.create_client(GoTo, 'planner/go_to')
        self._cancel = self.create_client(Trigger, 'planner/cancel')
        self._pause = self.create_client(SetBool, 'navigation/pause')
        self._say = self.create_client(Say, 'interaction/say')
        self._stop_speech = self.create_client(Trigger, 'interaction/stop_speech')
        self._gesture = self.create_client(Gesture, 'interaction/gesture')
        self.create_subscription(TourCommand, '/tour/cmd', self.on_command_topic, 10)
        self.create_subscription(PoiArray, 'site/pois', self.on_pois, latched)
        self.create_subscription(TourArray, 'site/tours', self.on_tours, latched)
        self.create_subscription(PlannerStatus, 'planner/status', self.on_planner, latched)
        self.create_subscription(
            SpeechStatus, 'interaction/speech_status', self.on_speech, latched
        )
        self.create_subscription(Visitors, 'perception/visitors', self.on_visitors, 10)
        self.create_subscription(Bool, 'bridge/estop_engaged', self.on_estop, latched)
        self.create_subscription(String, 'bridge/mode', self.on_mode, latched)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        self.create_service(TourControl, '/tour/control', self.on_control)
        self.create_service(GetTourState, '/tour/get_state', self.on_get_state)
        tick_hz = max(float(p('tick_hz')), 1.0)
        self.create_timer(1.0 / tick_hz, self.tick)
        self.publish_state(force=True)

    def now(self) -> float:
        """Return the ROS time in seconds (the clock of the status stamps)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------------------------------------- hooks (TourEngine)
    def go_to(self, token: int, poi_id: str):
        """Plan and walk to a POI through planner/go_to."""
        if not self._go_to.service_is_ready():
            self._engine.on_go_to(token, False, 'planner not running', self.now())
            return
        future = self._go_to.call_async(GoTo.Request(poi_id=poi_id))
        future.add_done_callback(lambda f: self.go_to_done(token, f))

    def go_to_done(self, token: int, future):
        """Hand the planner's answer to the engine."""
        result = future.result()
        ok, message = (result.success, result.message) if result else (False, 'no answer')
        self._engine.on_go_to(token, ok, message, self.now())
        self.publish_state()

    def cancel_nav(self):
        """Stop walking (planner/cancel)."""
        if self._cancel.service_is_ready():
            self._cancel.call_async(Trigger.Request())

    def pause_nav(self, pause: bool):
        """Hold or continue the walk (navigation/pause)."""
        if self._pause.service_is_ready():
            self._pause.call_async(SetBool.Request(data=pause))
        else:
            self.get_logger().warn('navigation/pause not available', throttle_duration_sec=5.0)

    def say(self, token, text: str, language: str):
        """Speak through interaction/say."""
        if not self._say.service_is_ready():
            if token is not None:
                self._engine.on_say(token, False, 0, 'speech not running', self.now())
            return
        future = self._say.call_async(Say.Request(text=text, language=language))
        if token is not None:
            future.add_done_callback(lambda f: self.say_done(token, f))

    def say_done(self, token: int, future):
        """Hand the speech node's answer to the engine."""
        result = future.result()
        if result is None:
            self._engine.on_say(token, False, 0, 'no answer', self.now())
        else:
            self._engine.on_say(token, result.success, result.id, result.message, self.now())
        self.publish_state()

    def stop_speech(self):
        """Stop talking (interaction/stop_speech)."""
        if self._stop_speech.service_is_ready():
            self._stop_speech.call_async(Trigger.Request())

    def gesture(self, token: int, name: str):
        """Run a gesture through interaction/gesture."""
        if not self._gesture.service_is_ready():
            self._engine.on_gesture(token, False, 'interaction node not running', self.now())
            return
        future = self._gesture.call_async(Gesture.Request(name=name))
        future.add_done_callback(lambda f: self.gesture_done(token, f))

    def gesture_done(self, token: int, future):
        """Hand the gesture's answer to the engine."""
        result = future.result()
        ok, message = (result.success, result.message) if result else (False, 'no answer')
        self._engine.on_gesture(token, ok, message, self.now())
        self.publish_state()

    def log(self, level: str, text: str):
        """Log a line of the engine."""
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    # ---------------------------------------------------------------- inputs
    def on_pois(self, msg: PoiArray):
        """Keep the active site's POIs."""
        self._site = msg.site
        self._pois = {poi.id: poi for poi in msg.pois}

    def on_tours(self, msg: TourArray):
        """Keep the active site's tours."""
        self._tours = {tour.id: tour for tour in msg.tours}

    def on_planner(self, msg: PlannerStatus):
        """Planner status: arrival or failure of the current walk."""
        stamp = stamp_seconds(msg.header.stamp)
        self._engine.on_planner(msg.state, msg.goal_id, stamp, self.now())
        self.publish_state()

    def on_speech(self, msg: SpeechStatus):
        """Speech status: the narration is over."""
        self._engine.on_speech(msg.id, msg.state, self.now())
        self.publish_state()

    def on_visitors(self, msg: Visitors):
        """Visitors following the robot."""
        self._engine.on_visitors(msg.behind, self.now())

    def on_estop(self, msg: Bool):
        """Pause the tour when the software e-stop engages."""
        self._engine.on_estop(msg.data, self.now())
        self.publish_state()

    def on_mode(self, msg: String):
        """AUTO or REMOTE: no waiting for visitors while the handheld drives."""
        self._mode = msg.data
        self._engine.allow_visitor_wait = msg.data == 'auto'

    def on_localization(self, msg: String):
        """Localization source; a tour needs one."""
        self._localization = msg.data

    # ---------------------------------------------------------------- commands
    def stop_of(self, poi) -> Stop:
        """Return the tour stop of a POI message."""
        return Stop(poi.id, poi.name or poi.id, poi.narration, poi.language, poi.gesture,
                    max(float(poi.dwell_sec), 0.0))

    def resolve(self, cmd: TourCommand):
        """Return (stops, tour id, tour name, return stop, error) of a START command."""
        tour_id, name, return_id = cmd.tour_id.strip(), '', ''
        if tour_id:
            tour = self._tours.get(tour_id)
            if tour is None:
                return None, '', '', None, f'no tour {tour_id!r} in site {self._site!r}'
            poi_ids, name, return_id = list(tour.poi_ids), tour.name or tour_id, tour.return_poi_id
        else:
            poi_ids = [p.strip() for p in cmd.poi_ids if p.strip()]
        if not poi_ids:
            return None, '', '', None, 'no POIs to visit'
        missing = [p for p in poi_ids + ([return_id] if return_id else []) if p not in self._pois]
        if missing:
            where = f'site {self._site!r}' if self._site else 'the site (none loaded)'
            return None, '', '', None, f'unknown POIs in {where}: {", ".join(missing)}'
        stops = [self.stop_of(self._pois[p]) for p in poi_ids]
        ret = self.stop_of(self._pois[return_id]) if return_id else None
        return stops, tour_id, name, ret, ''

    def ready(self) -> str:
        """Return why a tour cannot start now, or ''."""
        if self._require_localization and self._localization in (None, '', 'none'):
            return 'not localized: localize from an AprilTag first'
        if self._require_auto and self._mode != 'auto':
            return f'control mode is {self._mode or "unknown"}: switch to AUTO'
        if self._engine.estop:
            return 'the e-stop is engaged'
        return ''

    def apply_command(self, cmd: TourCommand):
        """Apply a tour command; return (ok, message). The state is republished either way."""
        now = self.now()
        kind = int(cmd.kind)
        if kind == START:
            if not can_transition(self._engine.state, START):
                ok, message = False, f'START not possible while {STATE_NAMES[self._engine.state]}'
            else:
                stops, tour_id, name, ret, error = self.resolve(cmd)
                error = error or self.ready()
                if error:
                    ok, message = False, error
                else:
                    ok, message = self._engine.start(stops, now, tour_id, name, ret)
        elif kind == RESUME and self._engine.estop:
            ok, message = False, 'release the e-stop first'
        else:
            ok, message = self._engine.command(kind, now)
        label = KIND_NAMES[kind] if 0 <= kind < len(KIND_NAMES) else str(kind)
        if ok:
            self.get_logger().info(f'tour {label}: {message}')
        else:
            self.get_logger().warn(f'tour {label} refused: {message}')
        self.publish_state(force=True)
        return ok, message

    def on_command_topic(self, msg: TourCommand):
        """Apply a /tour/cmd command (no answer)."""
        self.apply_command(msg)

    def on_control(self, request, response):
        """Apply a /tour/control command and answer."""
        response.success, response.message = self.apply_command(request.command)
        return response

    def on_get_state(self, _request, response):
        """Answer /tour/get_state with the current state (for a client that just connected)."""
        response.state = self.state_msg()
        return response

    # ---------------------------------------------------------------- output
    def tick(self):
        """Run the engine's timers and publish the state."""
        self._engine.tick(self.now())
        self.publish_state()

    def state_msg(self) -> TourState:
        """Return the engine snapshot as a TourState."""
        s = self._engine.snapshot()
        msg = TourState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = s['state']
        msg.tour_id, msg.tour_name = s['tour_id'], s['tour_name']
        msg.current_poi_index, msg.total_pois = s['index'], s['total']
        msg.current_poi_id, msg.current_poi_name = s['poi_id'], s['poi_name']
        msg.activity, msg.message = s['activity'], s['message']
        msg.error_reason = s['error_reason']
        return msg

    def publish_state(self, force: bool = False):
        """Publish /tour/state on a change and every publish_period_sec."""
        snapshot = self._engine.snapshot()
        now = self.now()
        if (
            not force
            and snapshot == self._last_published
            and now - self._last_publish_time < self._publish_period
        ):
            return
        if snapshot != self._last_published:
            self.get_logger().info(
                f'{STATE_NAMES[snapshot["state"]]} {snapshot["activity"] or "-"} '
                f'{snapshot["poi_id"] or "-"}: {snapshot["message"]}'
            )
        self._last_published, self._last_publish_time = snapshot, now
        self._state_pub.publish(self.state_msg())


def main(args=None):
    """Spin the guide node."""
    rclpy.init(args=args)
    node = GuideNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
