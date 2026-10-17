#!/usr/bin/env python3
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout
import json
import threading
import time

from molly_interaction.conversation import (
    ask_ollama,
    chat_request,
    clean_answer,
    site_knowledge,
    system_prompt,
)
from molly_interaction.speech import sanitize
from molly_interfaces.msg import PoiArray
from molly_interfaces.srv import Ask, Say
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

MAX_QUESTION_CHARS = 500


class ConversationNode(Node):
    """Answers visitors' questions about the site with a local language model (Ollama)."""

    def __init__(self):
        """Declare params, follow the site's POIs and serve interaction/ask."""
        super().__init__('conversation_node')
        defaults = {
            'url': 'http://127.0.0.1:11434',
            'model': 'qwen2.5:3b',
            'timeout_sec': 30.0,
            'max_answer_chars': 600,
            'max_tokens': 200,
            'language': 'id',
            'robot_name': 'Molly',
            'history_turns': 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._url, self._model = str(p('url')), str(p('model'))
        self._timeout = float(p('timeout_sec'))
        self._max_chars = int(p('max_answer_chars'))
        self._max_tokens = int(p('max_tokens'))
        self._language = str(p('language')).strip().lower() or 'id'
        self._robot_name = str(p('robot_name'))
        self._history_turns = max(int(p('history_turns')), 0)
        self._history = []
        self._site, self._pois = '', []
        self._lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ollama')

        group = ReentrantCallbackGroup()
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._log_pub = self.create_publisher(String, 'interaction/conversation', 10)
        self._say = self.create_client(Say, 'interaction/say', callback_group=group)
        self.create_subscription(PoiArray, 'site/pois', self.on_pois, latched)
        self.create_service(Ask, 'interaction/ask', self.on_ask, callback_group=group)
        self.get_logger().info(f'answering with {self._model} at {self._url}')

    def on_pois(self, msg: PoiArray):
        """Keep the site's POIs as the model's knowledge; a new site starts a new talk."""
        if msg.site != self._site:
            self._history = []
        self._site = msg.site
        self._pois = [
            {'name': p.name, 'description': p.description, 'narration': p.narration}
            for p in msg.pois
        ]

    def answer(self, question: str, language: str) -> str:
        """Ask the model (worker thread); return the cleaned answer."""
        system = system_prompt(
            language, self._robot_name, self._site, site_knowledge(self._pois)
        )
        history = self._history[-self._history_turns:] if self._history_turns else []
        body = chat_request(self._model, system, history, question, self._max_tokens)
        return clean_answer(ask_ollama(self._url, body, self._timeout), self._max_chars)

    def on_ask(self, request, response):
        """Service: answer one question (one at a time), optionally out loud."""
        question = sanitize(request.question, MAX_QUESTION_CHARS)
        language = request.language.strip().lower() or self._language
        response.success, response.answer = False, ''
        if not question:
            response.message = 'empty question'
            return response
        if not self._lock.acquire(timeout=self._timeout):
            response.message = 'still answering the previous question'
            return response
        start = time.monotonic()
        try:
            future = self._pool.submit(self.answer, question, language)
            answer = future.result(timeout=self._timeout + 2.0)
        except FutureTimeout:
            response.message = f'no answer within {self._timeout:.0f} s'
        except (OSError, ValueError) as error:
            response.message = f'model not reachable: {error}'
        else:
            response.success, response.answer = bool(answer), answer
            response.message = 'ok' if answer else 'the model answered nothing'
            if answer:
                self._history.append((question, answer))
                self._history = self._history[-max(self._history_turns, 1):]
        finally:
            self._lock.release()
        ms = int((time.monotonic() - start) * 1000)
        self._log_pub.publish(String(data=json.dumps(
            {'question': question, 'answer': response.answer, 'ms': ms, 'ok': response.success}
        )))
        if not response.success:
            self.get_logger().warn(f'ask: {response.message}')
        elif request.speak:
            if self._say.service_is_ready():
                self._say.call_async(Say.Request(text=response.answer, language=language))
            else:
                response.message = 'answered, but interaction/say is not available'
        return response

    def destroy_node(self):
        """Stop the worker thread."""
        self._pool.shutdown(wait=False)
        super().destroy_node()


def main(args=None):
    """Spin the conversation node on a multi-threaded executor."""
    rclpy.init(args=args)
    node = ConversationNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
        executor.shutdown(timeout_sec=2.0)  # running callbacks end before the node goes
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
