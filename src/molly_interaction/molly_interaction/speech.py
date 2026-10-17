"""
Pure helpers of the speech node: text clean-up, engine choice, command lines, queue and player.

No ROS here. Text never goes through a shell: it reaches the synthesizer on stdin, and every
command is an argument list.
"""

from collections import deque
from dataclasses import dataclass, field
import json
import os
import re
import shutil
import subprocess
import threading
import time

CONTROL = re.compile(r'[\x00-\x1f\x7f]')
ENGINES = ('auto', 'piper', 'espeak', 'none')
DEFAULT_RATE = 22050  # Piper voices are mostly 22.05 kHz; the voice's .onnx.json says


def sanitize(text, limit: int = 1000) -> str:
    """Return the text on one line, without control characters, at most `limit` characters."""
    clean = ' '.join(CONTROL.sub(' ', str(text)).split())
    if len(clean) <= limit:
        return clean
    cut = clean[:limit]
    space = cut.rfind(' ')
    return (cut[:space] if space > limit // 2 else cut).rstrip()


def parse_pairs(items) -> dict:
    """Parse ['id:value', 'en:value'] parameters into {'id': 'value', 'en': 'value'}."""
    pairs = {}
    for item in items or []:
        key, sep, value = str(item).partition(':')
        if sep and key.strip() and value.strip():
            pairs[key.strip().lower()] = value.strip()
    return pairs


def find_binary(path: str) -> str:
    """Return an executable for `path` (as given, else its name on PATH), or ''."""
    if not path:
        return ''
    if os.path.isfile(path) and os.access(path, os.X_OK):
        return path
    return shutil.which(os.path.basename(path)) or ''


def voice_rate(model: str) -> int:
    """Return the sample rate of a Piper voice from its .onnx.json, or the usual 22050 Hz."""
    try:
        with open(model + '.json') as f:
            rate = int(json.load(f)['audio']['sample_rate'])
    except (OSError, ValueError, KeyError, TypeError):
        return DEFAULT_RATE
    return rate if 8000 <= rate <= 48000 else DEFAULT_RATE


@dataclass
class SpeechConfig:
    """Where the engines and voices are, from the node parameters."""

    piper_bin: str = '/opt/molly/piper/piper'
    voices_dir: str = '/opt/molly/piper/voices'
    voices: dict = field(default_factory=dict)
    length_scale: float = 1.0
    espeak_bin: str = 'espeak-ng'
    espeak_voices: dict = field(default_factory=dict)
    espeak_wpm: int = 150
    aplay_bin: str = 'aplay'
    audio_device: str = ''


@dataclass
class Engine:
    """How to speak one utterance: synthesizer (text on stdin, audio out) and player."""

    name: str
    synth: list = field(default_factory=list)
    play: list = field(default_factory=list)


def player_command(aplay: str, device: str, raw_rate: int = 0) -> list:
    """Return the aplay command line for raw 16-bit mono audio at raw_rate, or for WAV input."""
    command = [aplay, '-q']
    if device:
        command += ['-D', device]
    if raw_rate:
        command += ['-r', str(raw_rate), '-f', 'S16_LE', '-t', 'raw', '-c', '1']
    return command + ['-']


def choose_engine(engine: str, language: str, cfg: SpeechConfig):
    """
    Return (Engine, note) for one utterance in this language.

    auto: Piper when its binary and the language's voice exist, else eSpeak NG, else none. An
    engine that was asked for but cannot run falls back to none, with the reason in the note.
    """
    language = (language or '').lower()
    aplay = find_binary(cfg.aplay_bin)
    if engine in ('auto', 'piper'):
        piper = find_binary(cfg.piper_bin)
        voice = cfg.voices.get(language, '')
        model = os.path.join(cfg.voices_dir, voice + '.onnx') if voice else ''
        if piper and model and os.path.isfile(model) and aplay:
            synth = [piper, '--model', model, '--output_raw']
            if cfg.length_scale and cfg.length_scale != 1.0:
                synth += ['--length_scale', f'{cfg.length_scale:g}']
            return Engine('piper', synth, player_command(aplay, cfg.audio_device,
                                                         voice_rate(model))), ''
        if engine == 'piper':
            missing = 'piper' if not piper else 'aplay' if not aplay else f'voice for {language!r}'
            return Engine('none'), f'piper cannot speak: {missing} missing'
    if engine in ('auto', 'espeak'):
        espeak = find_binary(cfg.espeak_bin)
        if espeak and aplay:
            voice = cfg.espeak_voices.get(language, language or 'en')
            synth = [espeak, '--stdin', '-v', voice, '-s', str(int(cfg.espeak_wpm)), '--stdout']
            return Engine('espeak', synth, player_command(aplay, cfg.audio_device)), ''
        if engine == 'espeak':
            missing = 'espeak-ng' if not espeak else 'aplay'
            return Engine('none'), f'espeak cannot speak: {missing} missing'
    if engine == 'auto':
        return Engine('none'), 'no speech engine found (piper or espeak-ng with aplay)'
    return Engine('none'), ''


@dataclass
class Utterance:
    """One text to say."""

    uid: int
    text: str
    language: str


class SpeechQueue:
    """First-in-first-out utterances with ids; thread safe."""

    def __init__(self, max_queue: int = 10):
        """Start empty; ids count from 1."""
        self._cond = threading.Condition()
        self._items = deque()
        self._next_id = 1
        self._closed = False
        self.max_queue = max(1, int(max_queue))

    def add(self, text: str, language: str):
        """Queue a text; return its Utterance, or None when the queue is full."""
        with self._cond:
            if len(self._items) >= self.max_queue:
                return None
            utterance = Utterance(self._next_id, text, language)
            self._next_id += 1
            self._items.append(utterance)
            self._cond.notify()
            return utterance

    def take(self, timeout: float = 0.5):
        """Return the next utterance, or None after timeout or once closed."""
        with self._cond:
            if not self._items and not self._closed:
                self._cond.wait(timeout)
            if self._closed or not self._items:
                return None
            return self._items.popleft()

    def clear(self) -> list:
        """Drop every waiting utterance and return them."""
        with self._cond:
            dropped = list(self._items)
            self._items.clear()
            return dropped

    def pending(self) -> int:
        """Return how many utterances wait."""
        with self._cond:
            return len(self._items)

    def close(self):
        """Wake the worker and stop handing out utterances."""
        with self._cond:
            self._closed = True
            self._cond.notify_all()


class Player:
    """Runs synthesizer | player for one utterance at a time; stop() interrupts it."""

    def __init__(self, none_chars_per_sec: float = 15.0):
        """none_chars_per_sec: how fast the silent engine 'speaks' (sim and tests)."""
        self._lock = threading.Lock()
        self._processes = []
        self._stop = threading.Event()
        self._none_cps = max(float(none_chars_per_sec), 1.0)

    def reset(self):
        """Forget an earlier stop() before the next utterance is taken."""
        self._stop.clear()

    def play(self, engine: Engine, text: str, timeout: float = 120.0):
        """Speak the text; return (state, message), state done, failed or cancelled."""
        if self._stop.is_set():
            return 'cancelled', 'stopped'
        if engine.name == 'none':
            if self._stop.wait(min(len(text) / self._none_cps, timeout)):
                return 'cancelled', 'stopped'
            return 'done', ''
        try:
            synth = subprocess.Popen(
                engine.synth, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
            player = subprocess.Popen(
                engine.play, stdin=synth.stdout, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except OSError as error:
            return 'failed', f'{engine.name}: {error}'
        synth.stdout.close()  # the player owns the pipe now
        with self._lock:
            self._processes = [synth, player]
        if self._stop.is_set():
            self.kill()
        try:
            synth.stdin.write(text.encode('utf-8') + b'\n')
            synth.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        deadline = time.monotonic() + timeout
        try:
            player_rc = player.wait(max(deadline - time.monotonic(), 0.1))
            synth_rc = synth.wait(max(deadline - time.monotonic(), 0.1))
        except subprocess.TimeoutExpired:
            self.kill()
            return 'failed', f'{engine.name}: still speaking after {timeout:.0f} s, stopped'
        finally:
            error = player.stderr.read().decode(errors='replace').strip() if player.stderr else ''
            if player.stderr:
                player.stderr.close()
            with self._lock:
                self._processes = []
        if self._stop.is_set():
            return 'cancelled', 'stopped'
        if player_rc != 0 or synth_rc != 0:
            detail = error.splitlines()[-1] if error else f'exit {synth_rc}/{player_rc}'
            return 'failed', f'{engine.name}: {detail}'
        return 'done', ''

    def stop(self):
        """Interrupt the utterance being spoken."""
        self._stop.set()
        self.kill()

    def kill(self):
        """Kill the running processes, if any."""
        with self._lock:
            processes = list(self._processes)
        for process in processes:
            try:
                process.kill()
            except OSError:
                pass


def speech_timeout(text: str, chars_per_sec: float = 8.0, base: float = 10.0) -> float:
    """Return a generous time limit to say a text (s)."""
    return base + len(text) / max(chars_per_sec, 1.0)
