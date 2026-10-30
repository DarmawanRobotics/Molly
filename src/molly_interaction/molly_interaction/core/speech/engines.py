"""
Which engine speaks an utterance and its command lines (no ROS).

Text never goes through a shell: it reaches the synthesizer on stdin, and every command is an
argument list.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
import shutil

ENGINES = ('auto', 'piper', 'espeak', 'none')
DEFAULT_RATE = 22050  # Piper voices are mostly 22.05 kHz; the voice's .onnx.json says


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
class EnginePaths:
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
    return [*command, '-']


def choose_engine(engine: str, language: str, cfg: EnginePaths):
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
            return Engine(
                'piper', synth, player_command(aplay, cfg.audio_device, voice_rate(model))
            ), ''
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
