"""conversation_node: answers visitors' questions about the site with a local model (Ollama)."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeout
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup

from dharma_utils.ros import LATCHED, DharmaNode, run
from molly_interaction.core.config import ConversationConfig
from molly_interaction.core.conversation import (
    ask_ollama,
    chat_request,
    clean_answer,
    site_knowledge,
    system_prompt,
)
from molly_interaction.core.speech import sanitize
from molly_interfaces.msg import Conversation, PoiArray
from molly_interfaces.srv import Ask, Say

MAX_QUESTION_CHARS = 500


class ConversationNode(DharmaNode):
    """interaction/ask: one question at a time, the site's POIs as the model's knowledge."""

    def __init__(self, **kwargs):
        super().__init__('conversation_node', **kwargs)
        self.cfg = self.declare_config(ConversationConfig).value
        self.declare_robot()  # the shared robot.* (frames and limits are not used here)
        self.language = self.cfg.language.strip().lower() or 'id'
        self.history: list[tuple[str, str]] = []
        self.site, self.pois = '', []
        self.lock = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ollama')
        group = ReentrantCallbackGroup()
        self.log_pub = self.create_publisher(Conversation, 'interaction/conversation', 10)
        self.say_client = self.create_client(Say, 'interaction/say', callback_group=group)
        self.create_subscription(PoiArray, 'site/pois', self.on_pois, LATCHED)
        self.create_service(Ask, 'interaction/ask', self.on_ask, callback_group=group)
        self.log.info('conversation ready', model=self.cfg.model, url=self.cfg.url)

    def on_pois(self, msg: PoiArray):
        """Keep the site's POIs as the model's knowledge; a new site starts a new talk."""
        if msg.site != self.site:
            self.history = []
        self.site = msg.site
        self.pois = [
            {'name': p.name, 'description': p.description, 'narration': p.narration}
            for p in msg.pois
        ]

    def answer(self, question: str, language: str) -> str:
        """Ask the model (worker thread); return the cleaned answer."""
        system = system_prompt(language, self.cfg.robot_name, self.site, site_knowledge(self.pois))
        turns = self.cfg.history_turns
        history = self.history[-turns:] if turns else []
        body = chat_request(self.cfg.model, system, history, question, self.cfg.max_tokens)
        raw = ask_ollama(self.cfg.url, body, self.cfg.timeout_sec)
        return clean_answer(raw, self.cfg.max_answer_chars)

    def ask(self, question: str, language: str) -> tuple[bool, str, str]:
        """Return (success, answer, message) of one question."""
        if not self.lock.acquire(timeout=self.cfg.timeout_sec):
            return False, '', 'still answering the previous question'
        try:
            future = self.pool.submit(self.answer, question, language)
            answer = future.result(timeout=self.cfg.timeout_sec + 2.0)
        except FutureTimeout:
            return False, '', f'no answer within {self.cfg.timeout_sec:.0f} s'
        except (OSError, ValueError) as error:
            return False, '', f'model not reachable: {error}'
        finally:
            self.lock.release()
        if not answer:
            return False, '', 'the model answered nothing'
        self.history = [*self.history, (question, answer)][-max(self.cfg.history_turns, 1) :]
        return True, answer, 'ok'

    def on_ask(self, request, response):
        """Service: answer one question, optionally out loud."""
        question = sanitize(request.question, MAX_QUESTION_CHARS)
        language = request.language.strip().lower() or self.language
        start = time.monotonic()
        if not question:
            response.success, response.answer, response.message = False, '', 'empty question'
        else:
            response.success, response.answer, response.message = self.ask(question, language)
        record = Conversation(
            question=question,
            answer=response.answer,
            latency_ms=int((time.monotonic() - start) * 1000),
            success=response.success,
            message=response.message,
        )
        record.header.stamp = self.get_clock().now().to_msg()
        self.log_pub.publish(record)
        if not response.success:
            self.log.warning('question not answered', reason=response.message)
        elif request.speak:
            if self.say_client.service_is_ready():
                self.say_client.call_async(Say.Request(text=response.answer, language=language))
            else:
                response.message = 'answered, but interaction/say is not available'
        return response

    def destroy_node(self):
        """Stop the worker thread."""
        self.pool.shutdown(wait=False)
        super().destroy_node()


def main(args=None):
    """Run the node on two threads (the model's answer takes seconds)."""
    run(ConversationNode, threads=2, args=args)


if __name__ == '__main__':
    main()
