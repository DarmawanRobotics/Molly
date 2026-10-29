"""Text to speech: engines (Piper, eSpeak NG, silent), the queue and the player."""

from __future__ import annotations

from molly_interaction.core.speech.engines import ENGINES, Engine, EnginePaths, choose_engine
from molly_interaction.core.speech.player import Player, SpeechState
from molly_interaction.core.speech.queue import SpeechQueue, Utterance
from molly_interaction.core.speech.text import parse_pairs, sanitize, speech_timeout

__all__ = [
    'ENGINES',
    'Engine',
    'EnginePaths',
    'Player',
    'SpeechQueue',
    'SpeechState',
    'Utterance',
    'choose_engine',
    'parse_pairs',
    'sanitize',
    'speech_timeout',
]
