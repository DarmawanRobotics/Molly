"""speech_node: speaks queued texts through Piper or eSpeak NG on the robot's speaker."""

from __future__ import annotations

import threading

from std_msgs.msg import String
from std_srvs.srv import Trigger

from dharma_utils.ros import DharmaNode, run
from molly_interaction.core.config import SpeechConfig
from molly_interaction.core.speech import (
    EnginePaths,
    Player,
    SpeechQueue,
    SpeechState,
    Utterance,
    choose_engine,
    parse_pairs,
    sanitize,
    speech_timeout,
)
from molly_interfaces.msg import SpeechStatus
from molly_interfaces.srv import Say


def engine_paths(cfg: SpeechConfig) -> EnginePaths:
    """Return where the engines and voices are."""
    return EnginePaths(
        piper_bin=cfg.piper_bin,
        voices_dir=cfg.voices_dir,
        voices=parse_pairs(cfg.voices),
        length_scale=cfg.piper_length_scale,
        espeak_bin=cfg.espeak_bin,
        espeak_voices=parse_pairs(cfg.espeak_voices),
        espeak_wpm=cfg.espeak_wpm,
        aplay_bin=cfg.aplay_bin,
        audio_device=cfg.audio_device,
    )


class SpeechNode(DharmaNode):
    """interaction/say, interaction/stop_speech and interaction/speech_status."""

    def __init__(self, **kwargs):
        super().__init__('speech_node', **kwargs)
        cfg = self.declare_config(SpeechConfig).value
        self.declare_robot()  # the shared robot.* (frames and limits are not used here)
        self.cfg = cfg
        self.language = cfg.default_language.strip().lower() or 'id'
        self.paths = engine_paths(cfg)
        self.queue = SpeechQueue(cfg.max_queue)
        self.player = Player(cfg.none_chars_per_sec)
        self.lock = threading.Lock()
        self.current: Utterance | None = None
        self.cancelled: set[int] = set()
        self.notes: set[str] = set()
        self.closing = False
        self.status_pub = self.latched(SpeechStatus, 'interaction/speech_status')
        self.create_service(Say, 'interaction/say', self.on_say)
        self.create_service(Trigger, 'interaction/stop_speech', self.on_stop)
        self.create_subscription(String, 'interaction/say_text', self.on_say_text, 10)
        engine, note = choose_engine(cfg.engine, self.language, self.paths)
        self.log.info('speech ready', engine=engine.name, language=self.language, note=note)
        self.publish_status(Utterance(0, '', self.language), SpeechState.IDLE, engine.name, note)
        self.worker = threading.Thread(target=self.speak_queue, name='speech', daemon=True)
        self.worker.start()

    # ---------------------------------------------------------------- requests
    def queue_text(self, text, language: str, interrupt: bool) -> tuple[Utterance | None, str]:
        """Queue a text; return (utterance or None, message)."""
        clean = sanitize(text, self.cfg.max_chars)
        if not clean:
            return None, 'nothing to say'
        if interrupt:
            self.stop_all()
        utterance = self.queue.add(clean, (language or self.language).strip().lower())
        if utterance is None:
            return None, f'speech queue full ({self.queue.max_queue})'
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
            self.log.warning('interaction/say_text refused', reason=message)

    def on_stop(self, _request, response):
        """Service: stop talking and drop the queue."""
        count = self.stop_all()
        response.success = True
        response.message = f'stopped, {count} dropped' if count else 'stopped'
        return response

    def stop_all(self) -> int:
        """Cancel the utterance being spoken and every waiting one; return how many waited."""
        dropped = self.queue.clear()
        with self.lock:
            if self.current is not None:
                self.cancelled.add(self.current.uid)
        self.player.stop()
        for utterance in dropped:
            self.publish_status(utterance, SpeechState.CANCELLED, '', 'dropped')
        return len(dropped)

    # ---------------------------------------------------------------- speaking thread
    def speak_queue(self):
        """Speak the queue, one utterance after the other, until the node stops."""
        while not self.closing:
            utterance = self.queue.take(0.5)
            if utterance is not None:
                self.speak(utterance)

    def speak(self, utterance: Utterance):
        """Speak one utterance and report how it ended."""
        with self.lock:
            self.current = utterance
            self.player.reset()
            cancelled = utterance.uid in self.cancelled
        engine, note = choose_engine(self.cfg.engine, utterance.language, self.paths)
        if note and note not in self.notes:
            self.notes.add(note)
            self.log.warning(note)
        if cancelled:
            state, message = SpeechState.CANCELLED, 'stopped'
        else:
            self.publish_status(utterance, SpeechState.SPEAKING, engine.name, '')
            limit = speech_timeout(utterance.text, 4.0, 30.0)
            state, message = self.player.play(engine, utterance.text, limit)
        with self.lock:
            self.current = None
            self.cancelled.discard(utterance.uid)
        if state is SpeechState.FAILED:
            self.log.error('speech failed', id=utterance.uid, reason=message)
        self.publish_status(utterance, state, engine.name, message)

    def publish_status(self, utterance: Utterance, state: SpeechState, engine: str, message: str):
        """Publish interaction/speech_status (latched)."""
        msg = SpeechStatus(
            id=int(utterance.uid),
            state=int(state),
            text=utterance.text,
            language=utterance.language,
            queued=self.queue.pending(),
            engine=engine,
            message=message,
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            self.status_pub.publish(msg)
        except Exception:  # the context is shutting down
            self.log.debug('speech status not published (shutting down)')

    def destroy_node(self):
        """Stop the speaking thread and any sound being played."""
        self.closing = True
        self.queue.close()
        self.player.stop()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """Run the node."""
    run(SpeechNode, args=args)


if __name__ == '__main__':
    main()
