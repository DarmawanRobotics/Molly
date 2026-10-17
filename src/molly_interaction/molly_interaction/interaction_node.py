#!/usr/bin/env python3
import json
import os
import re
import threading
import time

from molly_interaction.greeter import Greeter, GreeterConfig, MOVING_STATES, next_text, TOUR_BUSY
from molly_interfaces.msg import NavigationStatus, SpeechStatus, TourState, Visitors
from molly_interfaces.srv import Gesture, Say
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

NAME = re.compile(r'^[a-z][a-z0-9_]{0,31}$')
STATUS_PERIOD_SEC = 2.0


class InteractionNode(Node):
    """Robot gestures on request and a greeter for visitors who stop in front of the robot."""

    def __init__(self):
        """Declare params, restore the greeter choice and wire topics and services."""
        super().__init__('interaction_node')
        defaults = {
            'robot_ns': 'robot',
            'gestures': ['hello', 'nod', 'bow', 'look_left', 'look_right'],
            'gesture_timeout_sec': 20.0,
            'greeter_available': True,
            'enabled': True,
            'state_file': '~/.molly/greeter.json',
            'approach_distance': 1.5,
            'dwell_sec': 1.5,
            'cooldown_sec': 30.0,
            'visitors_timeout_sec': 1.0,
            'language': 'id',
            'greeting_texts_id': [
                'Halo, selamat datang! Saya Molly, robot pemandu di sini.',
                'Hai, senang bertemu dengan Anda! Ada yang bisa saya bantu?',
            ],
            'greeting_texts_en': [
                'Hello and welcome! I am Molly, the guide robot here.',
                'Hi, nice to meet you! Can I help you?',
            ],
            'greeting_gesture': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._ns = str(p('robot_ns')).strip('/') or 'robot'
        self._gestures = [g for g in (str(x).strip() for x in p('gestures')) if NAME.match(g)]
        self._gesture_timeout = float(p('gesture_timeout_sec'))
        self._available = bool(p('greeter_available'))
        self._visitors_timeout = float(p('visitors_timeout_sec'))
        self._language = str(p('language')).strip().lower() or 'id'
        self._texts = {
            'id': [str(t) for t in p('greeting_texts_id') if str(t).strip()],
            'en': [str(t) for t in p('greeting_texts_en') if str(t).strip()],
        }
        self._text_index = 0
        self._greeting_gesture = str(p('greeting_gesture')).strip()
        state_file = str(p('state_file')).strip()
        self._state_file = os.path.expanduser(state_file) if state_file else ''
        self._greeter = Greeter(
            GreeterConfig(
                float(p('approach_distance')), float(p('dwell_sec')), float(p('cooldown_sec'))
            )
        )
        self.restore_choice()
        self._greeter.enabled = self._available and bool(self.get_parameter('enabled').value)

        self._nearest_front = None
        self._visitors_time = -1e9
        self._tour_state = 0
        self._nav_state = ''
        self._speaking = False
        self._gesture_lock = threading.Lock()
        self._driver_clients = {}
        self._last_status = None
        self._status_time = -1e9

        self._srv_group = ReentrantCallbackGroup()
        self._loop_group = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(String, 'interaction/greeter', latched)
        self._say = self.create_client(Say, 'interaction/say', callback_group=self._srv_group)
        self.create_subscription(Visitors, 'perception/visitors', self.on_visitors, 10)
        self.create_subscription(TourState, '/tour/state', self.on_tour, latched)
        self.create_subscription(NavigationStatus, 'navigation/status', self.on_nav, 10)
        self.create_subscription(
            SpeechStatus, 'interaction/speech_status', self.on_speech, latched
        )
        self.create_service(
            Gesture, 'interaction/gesture', self.on_gesture, callback_group=self._srv_group
        )
        self.create_service(Trigger, 'interaction/greet', self.on_greet)
        self.create_service(SetBool, 'interaction/set_greeter', self.on_set_greeter)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(0.2, self.tick, callback_group=self._loop_group)
        self.create_timer(STATUS_PERIOD_SEC, self.publish_status, callback_group=self._loop_group)
        self.publish_status(force=True)
        self.get_logger().info(
            f'gestures {self._gestures} via /{self._ns}/<name>, greeter '
            f'{"unavailable" if not self._available else "on" if self._greeter.enabled else "off"}'
        )

    # ---------------------------------------------------------------- inputs
    def on_visitors(self, msg: Visitors):
        """Keep the nearest visitor in front."""
        self._nearest_front = float(msg.nearest_front) if msg.front > 0 else None
        self._visitors_time = time.monotonic()

    def on_tour(self, msg: TourState):
        """Keep the tour state: no greeting while a tour runs."""
        self._tour_state = int(msg.state)

    def on_nav(self, msg: NavigationStatus):
        """Keep the follower state: no greeting or gesture while walking."""
        self._nav_state = msg.state

    def on_speech(self, msg: SpeechStatus):
        """Keep whether the robot is talking."""
        self._speaking = msg.state == 'speaking'

    def busy(self) -> bool:
        """Return True while a tour runs, the robot walks or talks."""
        return (
            self._tour_state in TOUR_BUSY or self._nav_state in MOVING_STATES or self._speaking
        )

    # ---------------------------------------------------------------- greeter
    def tick(self):
        """Run the greeter on the latest visitors."""
        now = time.monotonic()
        fresh = now - self._visitors_time <= self._visitors_timeout
        nearest = self._nearest_front if fresh else None
        if self._greeter.update(now, nearest, self.busy()):
            self.greet()
        self.publish_status()

    def greet(self) -> str:
        """Say the next greeting (and play the greeting gesture); return the text."""
        texts = self._texts.get(self._language) or self._texts.get('id') or []
        text, self._text_index = next_text(texts, self._text_index)
        if text and self._say.service_is_ready():
            self._say.call_async(Say.Request(text=text, language=self._language))
        elif text:
            self.get_logger().warn('interaction/say is not available, greeting not spoken')
        if self._greeting_gesture and self._nav_state not in MOVING_STATES:
            client = self.driver_client(self._greeting_gesture)
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        self.get_logger().info(f'greeting: {text!r}')
        return text

    def on_greet(self, _request, response):
        """Service: greet the visitor in front now."""
        self._greeter.greeted(time.monotonic())
        text = self.greet()
        response.success = bool(text)
        response.message = text or 'no greeting text configured'
        self.publish_status(force=True)
        return response

    def on_set_greeter(self, request, response):
        """Service: automatic greeting on or off (kept across restarts)."""
        if not self._available:
            response.success = False
            response.message = 'greeter not available (greeter:=false)'
            return response
        result = self.set_parameters([Parameter('enabled', value=bool(request.data))])[0]
        response.success = result.successful
        response.message = f'greeter {"on" if request.data else "off"}'
        return response

    def on_parameters(self, params):
        """Accept the live on / off of the greeter; the rest needs a restart."""
        for param in params:
            if param.name != 'enabled':
                return SetParametersResult(
                    successful=False, reason=f'{param.name} is read at start, restart the node'
                )
        for param in params:
            self._greeter.enabled = self._available and bool(param.value)
        self.save_choice()
        self.publish_status(force=True)
        return SetParametersResult(successful=True)

    def restore_choice(self):
        """Apply the greeter on / off chosen before a restart (state_file)."""
        if not self._state_file:
            return
        try:
            with open(self._state_file) as f:
                saved = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as error:
            self.get_logger().warn(f'ignoring {self._state_file}: {error}')
            return
        if isinstance(saved, dict) and isinstance(saved.get('enabled'), bool):
            self.set_parameters([Parameter('enabled', value=saved['enabled'])])

    def save_choice(self):
        """Keep the greeter on / off for the next start (state_file)."""
        if not self._state_file:
            return
        try:
            os.makedirs(os.path.dirname(self._state_file) or '.', exist_ok=True)
            partial = self._state_file + '.tmp'
            with open(partial, 'w') as f:
                json.dump({'enabled': self._greeter.enabled}, f)
            os.replace(partial, self._state_file)
        except OSError as error:
            self.get_logger().warn(f'cannot keep the greeter choice: {error}')

    def publish_status(self, force: bool = False):
        """Publish interaction/greeter (latched JSON) on a change and every few seconds."""
        now = time.monotonic()
        key = (self._greeter.enabled, self._greeter.state if self._available else 'unavailable')
        if not force and key == self._last_status and now - self._status_time < STATUS_PERIOD_SEC:
            return
        self._last_status, self._status_time = key, now
        status = {
            'enabled': key[0],
            'state': key[1],
            'last_greet_age': self._greeter.last_greet_age(now),
            'gestures': self._gestures,
        }
        self._status_pub.publish(String(data=json.dumps(status)))

    # ---------------------------------------------------------------- gestures
    def driver_client(self, name: str):
        """Return the (cached) Trigger client of the driver's gesture service."""
        if name not in self._driver_clients:
            self._driver_clients[name] = self.create_client(
                Trigger, f'/{self._ns}/{name}', callback_group=self._srv_group
            )
        return self._driver_clients[name]

    def on_gesture(self, request, response):
        """Service: run a gesture of the robot driver and wait until it is done."""
        name = request.name.strip()
        response.success = False
        if name not in self._gestures:
            response.message = f'unknown gesture {name!r}, use one of {self._gestures}'
            return response
        if self._nav_state in MOVING_STATES:
            response.message = f'robot is walking ({self._nav_state}), gesture refused'
            return response
        if not self._gesture_lock.acquire(blocking=False):
            response.message = 'another gesture is running'
            return response
        try:
            client = self.driver_client(name)
            if not client.wait_for_service(timeout_sec=1.0):
                response.message = f'/{self._ns}/{name} not available'
                return response
            done = threading.Event()
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda _: done.set())
            if not done.wait(self._gesture_timeout) or future.result() is None:
                response.message = f'/{self._ns}/{name} timed out'
                return response
            response.success = future.result().success
            response.message = future.result().message or name
            return response
        finally:
            self._gesture_lock.release()


def main(args=None):
    """Spin the interaction node on a multi-threaded executor (gestures wait for the driver)."""
    rclpy.init(args=args)
    node = InteractionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
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
