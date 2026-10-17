#!/usr/bin/env python3
import threading

from molly_interaction.speech import (
    choose_engine,
    ENGINES,
    parse_pairs,
    Player,
    sanitize,
    speech_timeout,
    SpeechConfig,
    SpeechQueue,
)
from molly_interfaces.msg import SpeechStatus
from molly_interfaces.srv import Say
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SpeechNode(Node):
    """Speaks queued texts through Piper or eSpeak NG and the robot's speaker."""

    def __init__(self):
        """Declare params, start the speaking thread and serve interaction/say."""
        super().__init__('speech_node')
        defaults = {
            'engine': 'auto',
            'default_language': 'id',
            'piper_bin': '/opt/molly/piper/piper',
            'voices_dir': '/opt/molly/piper/voices',
            'voices': ['id:id_ID-news_tts-medium', 'en:en_US-lessac-medium'],
            'piper_length_scale': 1.0,
            'espeak_bin': 'espeak-ng',
            'espeak_voices': ['id:id', 'en:en-us'],
            'espeak_wpm': 150,
            'aplay_bin': 'aplay',
            'audio_device': '',
            'max_queue': 10,
            'max_chars': 1000,
            'none_chars_per_sec': 15.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._engine = str(p('engine')).strip().lower()
        if self._engine not in ENGINES:
            self.get_logger().error(f'engine {self._engine!r} unknown, using auto')
            self._engine = 'auto'
        self._language = str(p('default_language')).strip().lower() or 'id'
        self._max_chars = max(int(p('max_chars')), 1)
        self._cfg = SpeechConfig(
            piper_bin=str(p('piper_bin')),
            voices_dir=str(p('voices_dir')),
            voices=parse_pairs(p('voices')),
            length_scale=float(p('piper_length_scale')),
            espeak_bin=str(p('espeak_bin')),
            espeak_voices=parse_pairs(p('espeak_voices')),
            espeak_wpm=int(p('espeak_wpm')),
            aplay_bin=str(p('aplay_bin')),
            audio_device=str(p('audio_device')),
        )
        self._queue = SpeechQueue(int(p('max_queue')))
        self._player = Player(float(p('none_chars_per_sec')))
        self._lock = threading.Lock()
        self._current = None
        self._cancelled = set()
        self._notes = set()
        self._closing = False

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(
            SpeechStatus, 'interaction/speech_status', latched
        )
        self.create_service(Say, 'interaction/say', self.on_say)
        self.create_service(Trigger, 'interaction/stop_speech', self.on_stop)
        self.create_subscription(String, 'interaction/say_text', self.on_say_text, 10)
        engine, note = choose_engine(self._engine, self._language, self._cfg)
        self.get_logger().info(
            f'speech engine {engine.name} for {self._language!r}' + (f' ({note})' if note else '')
        )
        self.publish_status(0, 'idle', '', self._language, engine.name, note)
        self._worker = threading.Thread(target=self.run, name='speech', daemon=True)
        self._worker.start()

    # ---------------------------------------------------------------- requests
    def queue_text(self, text, language: str, interrupt: bool):
        """Queue a text; return (utterance or None, message)."""
        clean = sanitize(text, self._max_chars)
        if not clean:
            return None, 'nothing to say'
        if interrupt:
            self.stop_all()
        utterance = self._queue.add(clean, (language or self._language).strip().lower())
        if utterance is None:
            return None, f'speech queue full ({self._queue.max_queue})'
        return utterance, f'queued as {utterance.uid}'

    def on_say(self, request, response):
        """Service: speak a text; the answer carries its id for interaction/speech_status."""
        utterance, response.message = self.queue_text(
            request.text, request.language, request.interrupt
        )
        response.success = utterance is not None
        response.id = utterance.uid if utterance is not None else 0
        return response

    def on_say_text(self, msg: String):
        """Topic: speak a text, fire and forget."""
        utterance, message = self.queue_text(msg.data, '', False)
        if utterance is None:
            self.get_logger().warn(f'interaction/say_text: {message}')

    def on_stop(self, _request, response):
        """Service: stop talking and drop the queue."""
        count = self.stop_all()
        response.success = True
        response.message = f'stopped, {count} dropped' if count else 'stopped'
        return response

    def stop_all(self) -> int:
        """Cancel the utterance being spoken and every waiting one; return how many waited."""
        dropped = self._queue.clear()
        with self._lock:
            if self._current is not None:
                self._cancelled.add(self._current.uid)
        self._player.stop()
        for utterance in dropped:
            self.publish_status(
                utterance.uid, 'cancelled', utterance.text, utterance.language, '', 'dropped'
            )
        return len(dropped)

    # ---------------------------------------------------------------- speaking thread
    def run(self):
        """Speak the queue, one utterance after the other, until the node stops."""
        while rclpy.ok() and not self._closing:
            utterance = self._queue.take(0.5)
            if utterance is None:
                continue
            with self._lock:
                self._current = utterance
                self._player.reset()
                cancelled = utterance.uid in self._cancelled
            engine, note = choose_engine(self._engine, utterance.language, self._cfg)
            if note and note not in self._notes:
                self._notes.add(note)
                self.get_logger().warn(note)
            if cancelled:
                state, message = 'cancelled', 'stopped'
            else:
                self.publish_status(
                    utterance.uid, 'speaking', utterance.text, utterance.language, engine.name, ''
                )
                state, message = self._player.play(
                    engine, utterance.text, speech_timeout(utterance.text, 4.0, 30.0)
                )
            with self._lock:
                self._current = None
                self._cancelled.discard(utterance.uid)
            if state == 'failed':
                self.get_logger().error(f'speech {utterance.uid} failed: {message}')
            self.publish_status(
                utterance.uid, state, utterance.text, utterance.language, engine.name, message
            )

    def publish_status(self, uid: int, state: str, text: str, language: str, engine: str,
                       message: str):
        """Publish interaction/speech_status (latched)."""
        msg = SpeechStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = int(uid)
        msg.state = state
        msg.text = text
        msg.language = language
        msg.queued = self._queue.pending()
        msg.engine = engine
        msg.message = message
        try:
            self._status_pub.publish(msg)
        except Exception:  # noqa: B902  the context is shutting down
            pass

    def destroy_node(self):
        """Stop the speaking thread and any sound being played."""
        self._closing = True
        self._queue.close()
        self._player.stop()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """Spin the speech node."""
    rclpy.init(args=args)
    node = SpeechNode()
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
