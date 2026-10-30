"""
interaction_node: robot gestures on request, and a greeter for visitors who stop in front.

Gestures call the driver's std_srvs/Trigger /<robot_ns>/<name> and answer when the driver does.
The greeter's on / off is a live parameter (`enabled`), kept across restarts in `state_file`.
"""

from __future__ import annotations

import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool, Trigger

from dharma_interfaces.msg import NavigationStatus
from dharma_utils.io import expand
from dharma_utils.ros import LATCHED, DharmaNode, run
from molly_interaction.core.config import InteractionConfig
from molly_interaction.core.greeter import (
    MOVING_STATES,
    TOUR_BUSY,
    Greeter,
    GreeterConfig,
    GreeterState,
    load_choice,
    next_text,
    save_choice,
)
from molly_interaction.utils.gestures import GestureRunner
from molly_interfaces.msg import GreeterStatus, SpeechStatus, TourState, Visitors
from molly_interfaces.srv import Gesture, Say

STATUS_PERIOD_SEC = 2.0


class InteractionNode(DharmaNode):
    """interaction/gesture, interaction/greet, interaction/set_greeter and the greeter status."""

    def __init__(self, **kwargs):
        super().__init__('interaction_node', **kwargs)
        self.config = self.declare_config(InteractionConfig)
        self.declare_robot()  # the shared robot.* (frames and limits are not used here)
        cfg = self.config.value
        self.available = cfg.greeter_available
        self.language = cfg.language.strip().lower() or 'id'
        self.texts = {
            'id': [t for t in cfg.greeting_texts_id if t.strip()],
            'en': [t for t in cfg.greeting_texts_en if t.strip()],
        }
        self.text_index = 0
        self.state_file = expand(cfg.state_file)
        self.greeter = Greeter(
            GreeterConfig(cfg.approach_distance, cfg.dwell_sec, cfg.cooldown_sec)
        )
        self.nearest_front: float | None = None
        self.visitors_time = -1e9
        self.tour_state, self.nav_state, self.speaking = 0, 0, False
        self.last_status, self.status_time = None, -1e9
        services = ReentrantCallbackGroup()
        loop = MutuallyExclusiveCallbackGroup()
        self.gestures = GestureRunner(
            self, cfg.robot_ns, cfg.gestures, cfg.gesture_timeout_sec, services
        )
        self.status_pub = self.latched(GreeterStatus, 'interaction/greeter')
        self.say_client = self.create_client(Say, 'interaction/say', callback_group=services)
        sub = self.create_subscription
        sub(Visitors, 'perception/visitors', self.on_visitors, 10)
        sub(TourState, 'tour/state', lambda m: setattr(self, 'tour_state', m.state), LATCHED)
        sub(NavigationStatus, 'navigation/status', self.on_nav, LATCHED)
        sub(SpeechStatus, 'interaction/speech_status', self.on_speech, LATCHED)
        srv = self.create_service
        srv(Gesture, 'interaction/gesture', self.on_gesture, callback_group=services)
        self.create_service(Trigger, 'interaction/greet', self.on_greet)
        self.create_service(SetBool, 'interaction/set_greeter', self.on_set_greeter)
        self.config.on_change(self.on_config)
        self.restore_choice()
        self.greeter.enabled = self.available and self.config.enabled
        self.every(0.2, self.tick, group=loop)
        self.every(STATUS_PERIOD_SEC, self.publish_status, group=loop)
        self.publish_status(force=True)
        self.log.info(
            'interaction ready',
            gestures=','.join(self.gestures.names),
            greeter='unavailable' if not self.available else self.greeter.enabled,
        )

    # ---------------------------------------------------------------- inputs
    def on_visitors(self, msg: Visitors):
        """Keep the nearest visitor in front."""
        self.nearest_front = float(msg.nearest_front) if msg.front > 0 else None
        self.visitors_time = time.monotonic()

    def on_nav(self, msg: NavigationStatus):
        """Keep the follower state: no greeting or gesture while walking."""
        self.nav_state = msg.state

    def on_speech(self, msg: SpeechStatus):
        """Keep whether the robot is talking."""
        self.speaking = msg.state == SpeechStatus.SPEAKING

    def moving(self) -> bool:
        """Return True while navigation moves the robot."""
        return self.nav_state in MOVING_STATES

    def busy(self) -> bool:
        """Return True while a tour runs, the robot walks or talks."""
        return self.tour_state in TOUR_BUSY or self.moving() or self.speaking

    # ---------------------------------------------------------------- greeter
    def tick(self):
        """Run the greeter on the latest visitors."""
        now = time.monotonic()
        fresh = now - self.visitors_time <= self.config.visitors_timeout_sec
        if self.greeter.update(now, self.nearest_front if fresh else None, self.busy()):
            self.greet()
        self.publish_status()

    def greet(self) -> str:
        """Say the next greeting (and play the greeting gesture); return the text."""
        texts = self.texts.get(self.language) or self.texts.get('id') or []
        text, self.text_index = next_text(texts, self.text_index)
        if text and self.say_client.service_is_ready():
            self.say_client.call_async(Say.Request(text=text, language=self.language))
        elif text:
            self.log.warning('interaction/say is not available, greeting not spoken')
        gesture = self.config.greeting_gesture.strip()
        if gesture and not self.moving():
            self.gestures.fire(gesture)
        self.log.info('greeting', text=text)
        return text

    def on_greet(self, _request, response):
        """Service: greet the visitor in front now."""
        self.greeter.greeted(time.monotonic())
        text = self.greet()
        response.success = bool(text)
        response.message = text or 'no greeting text configured'
        self.publish_status(force=True)
        return response

    def on_set_greeter(self, request, response):
        """Service: automatic greeting on or off (kept across restarts)."""
        if not self.available:
            response.success, response.message = False, 'greeter not available on this robot'
            return response
        result = self.set_parameters([Parameter('enabled', value=bool(request.data))])[0]
        response.success = result.successful
        on_off = 'on' if request.data else 'off'
        response.message = f'greeter {on_off}' if result.successful else result.reason
        return response

    def on_config(self, old, new):
        """Apply a changed live on / off and keep it."""
        self.greeter.enabled = self.available and new.enabled
        if new.enabled != old.enabled:
            try:
                save_choice(self.state_file, new.enabled)
            except OSError as error:
                self.log.warning('cannot keep the greeter choice', error=str(error))
        self.publish_status(force=True)

    def restore_choice(self):
        """Apply the greeter on / off chosen before a restart (state_file)."""
        saved, problem = load_choice(self.state_file)
        if problem:
            self.log.warning('greeter choice ignored', reason=problem)
        if saved is not None and saved != self.config.enabled:
            self.set_parameters([Parameter('enabled', value=saved)])

    def publish_status(self, force: bool = False):
        """Publish interaction/greeter on a change and every few seconds."""
        now = time.monotonic()
        state = self.greeter.state if self.available else GreeterState.UNAVAILABLE
        key = (self.greeter.enabled, state)
        if not force and key == self.last_status and now - self.status_time < STATUS_PERIOD_SEC:
            return
        self.last_status, self.status_time = key, now
        age = self.greeter.last_greet_age(now)
        msg = GreeterStatus(
            enabled=key[0],
            state=int(state),
            last_greet_age=-1.0 if age is None else float(age),
            gestures=self.gestures.names,
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)

    # ---------------------------------------------------------------- gestures
    def on_gesture(self, request, response):
        """Service: run a gesture of the robot driver and wait until it is done."""
        response.success, response.message = self.gestures.run(request.name.strip(), self.moving())
        return response

    def destroy_node(self):
        """Release a gesture that waits for the driver, then destroy the node."""
        self.gestures.stopping.set()
        super().destroy_node()


def main(args=None):
    """Run the node on three threads (gestures wait for the driver)."""
    run(InteractionNode, threads=3, args=args)


if __name__ == '__main__':
    main()
