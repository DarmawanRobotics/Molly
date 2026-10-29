"""What the tour engine asks for, as asynchronous ROS service calls."""

from __future__ import annotations

from std_srvs.srv import Trigger

from dharma_interfaces.srv import GoTo
from molly_guide.core.tour import Hooks, Stop
from molly_interfaces.srv import Gesture, Say


class GuideHooks(Hooks):
    """The planner, navigation, speech and gesture services of the robot."""

    def __init__(self, node, group, engine_lock):
        self.node = node
        self.lock = engine_lock
        self.engine = None  # set by the node once the engine exists

        def client(kind, name):
            return node.create_client(kind, name, callback_group=group)

        self.go_to_client = client(GoTo, 'planner/go_to')
        self.cancel_client = client(Trigger, 'planner/cancel')
        self.pause_client = client(Trigger, 'navigation/pause')
        self.resume_client = client(Trigger, 'navigation/resume')
        self.say_client = client(Say, 'interaction/say')
        self.stop_speech_client = client(Trigger, 'interaction/stop_speech')
        self.gesture_client = client(Gesture, 'interaction/gesture')

    def now(self) -> float:
        """Return the ROS time (the clock of the status stamps)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def answer(self, future, handle):
        """Call handle(result or None) under the engine lock once the future is done."""

        def done(f):
            try:
                result = f.result()
            except Exception as error:
                self.node.log.warning('service call failed', error=str(error))
                result = None
            with self.lock:
                handle(result)
                self.node.publish_state()

        future.add_done_callback(done)

    def go_to(self, token: int, stop: Stop):
        """Plan and walk to a stop through planner/go_to."""
        if not self.go_to_client.service_is_ready():
            self.engine.on_go_to(token, False, 'planner not running', self.now())
            return
        request = GoTo.Request(x=stop.x, y=stop.y, yaw=stop.yaw)

        def handle(result):
            ok, message = (result.success, result.message) if result else (False, 'no answer')
            self.engine.on_go_to(token, ok, message, self.now())

        self.answer(self.go_to_client.call_async(request), handle)

    def trigger(self, client):
        """Call a Trigger service without waiting; False when it is not available."""
        if not client.service_is_ready():
            self.node.log.warning(f'{client.srv_name} not available', throttle=5.0)
            return False
        client.call_async(Trigger.Request())
        return True

    def cancel_nav(self):
        """Stop walking (planner/cancel)."""
        self.trigger(self.cancel_client)

    def pause_nav(self, pause: bool):
        """Hold or continue the walk (navigation/pause, navigation/resume)."""
        self.trigger(self.pause_client if pause else self.resume_client)

    def say(self, token: int | None, text: str, language: str):
        """Speak through interaction/say."""
        if not self.say_client.service_is_ready():
            if token is not None:
                self.engine.on_say(token, False, 0, 'speech not running', self.now())
            return
        future = self.say_client.call_async(Say.Request(text=text, language=language))
        if token is None:
            return

        def handle(result):
            if result is None:
                self.engine.on_say(token, False, 0, 'no answer', self.now())
            else:
                self.engine.on_say(token, result.success, result.id, result.message, self.now())

        self.answer(future, handle)

    def stop_speech(self):
        """Stop talking (interaction/stop_speech)."""
        if self.stop_speech_client.service_is_ready():
            self.stop_speech_client.call_async(Trigger.Request())

    def gesture(self, token: int, name: str):
        """Run a gesture through interaction/gesture."""
        if not self.gesture_client.service_is_ready():
            self.engine.on_gesture(token, False, 'interaction node not running', self.now())
            return

        def handle(result):
            ok, message = (result.success, result.message) if result else (False, 'no answer')
            self.engine.on_gesture(token, ok, message, self.now())

        self.answer(self.gesture_client.call_async(Gesture.Request(name=name)), handle)

    def report(self, level: str, text: str):
        """Log a line of the engine."""
        if level == 'error':
            self.node.log.error(text)
        elif level == 'warn':
            self.node.log.warning(text)
        else:
            self.node.log.info(text)
