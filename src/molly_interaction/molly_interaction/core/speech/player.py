"""Plays one utterance at a time: synthesizer | aplay, interruptible (no ROS)."""

from __future__ import annotations

import contextlib
from enum import IntEnum
import subprocess
import threading
import time

from molly_interaction.core.speech.engines import Engine


class SpeechState(IntEnum):
    """SpeechStatus.state."""

    IDLE = 0
    SPEAKING = 1
    DONE = 2
    FAILED = 3
    CANCELLED = 4


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

    def play(self, engine: Engine, text: str, timeout: float = 120.0) -> tuple[SpeechState, str]:
        """Speak the text; return (SpeechState DONE, FAILED or CANCELLED, message)."""
        if self._stop.is_set():
            return SpeechState.CANCELLED, 'stopped'
        if engine.name == 'none':
            stopped = self._stop.wait(min(len(text) / self._none_cps, timeout))
            return (SpeechState.CANCELLED, 'stopped') if stopped else (SpeechState.DONE, '')
        try:
            synth = subprocess.Popen(
                engine.synth, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )  # fmt: skip
            player = subprocess.Popen(
                engine.play, stdin=synth.stdout, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
            )
        except OSError as error:
            return SpeechState.FAILED, f'{engine.name}: {error}'
        synth.stdout.close()  # the player owns the pipe now
        with self._lock:
            self._processes = [synth, player]
        if self._stop.is_set():
            self.kill()
        with contextlib.suppress(OSError):
            synth.stdin.write(text.encode('utf-8') + b'\n')
            synth.stdin.close()
        state, message = self._wait(engine.name, synth, player, timeout)
        if self._stop.is_set():
            return SpeechState.CANCELLED, 'stopped'
        return state, message

    def _wait(self, name: str, synth, player, timeout: float) -> tuple[SpeechState, str]:
        """Wait for both processes; FAILED on a timeout or an exit code."""
        deadline = time.monotonic() + timeout
        try:
            player_rc = player.wait(max(deadline - time.monotonic(), 0.1))
            synth_rc = synth.wait(max(deadline - time.monotonic(), 0.1))
        except subprocess.TimeoutExpired:
            self.kill()
            return SpeechState.FAILED, f'{name}: still speaking after {timeout:.0f} s, stopped'
        finally:
            error = player.stderr.read().decode(errors='replace').strip() if player.stderr else ''
            if player.stderr:
                player.stderr.close()
            with self._lock:
                self._processes = []
        if player_rc != 0 or synth_rc != 0:
            detail = error.splitlines()[-1] if error else f'exit {synth_rc}/{player_rc}'
            return SpeechState.FAILED, f'{name}: {detail}'
        return SpeechState.DONE, ''

    def stop(self):
        """Interrupt the utterance being spoken."""
        self._stop.set()
        self.kill()

    def kill(self):
        """Kill the running processes, if any."""
        with self._lock:
            processes = list(self._processes)
        for process in processes:
            with contextlib.suppress(OSError):
                process.kill()
