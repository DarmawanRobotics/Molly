from __future__ import annotations

import dataclasses
import json
import os
import stat
import sys
import threading
import time

from molly_interaction.core.speech import (
    Engine,
    EnginePaths,
    Player,
    SpeechQueue,
    SpeechState,
    choose_engine,
    parse_pairs,
    sanitize,
    speech_timeout,
)
from molly_interaction.core.speech.engines import find_binary, player_command, voice_rate


def executable(path, body='#!/bin/sh\nexit 0\n'):
    """Write an executable script and return its path."""
    path.write_text(body)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)
    return str(path)


def test_sanitize_one_line_no_control_characters():
    """Newlines, tabs and control bytes become single spaces; long text is cut at a word."""
    assert sanitize('  Halo\n\tdunia\x07!  ') == 'Halo dunia !'
    long = 'kata ' * 300
    cut = sanitize(long, 100)
    assert len(cut) <= 100 and not cut.endswith(' ') and cut.endswith('kata')
    assert sanitize(None) == 'None' and sanitize('') == ''


def test_parse_pairs_keeps_colons_in_values():
    """language:value pairs, keys lower case, broken items skipped."""
    assert parse_pairs(['id:id_ID-news', 'EN: en_US', 'broken', ':x', 'x:']) == {
        'id': 'id_ID-news',
        'en': 'en_US',
    }
    assert parse_pairs(['fr:a:b']) == {'fr': 'a:b'}


def test_find_binary_by_path_or_name(tmp_path):
    """An executable path is used as is; otherwise its name is looked up on PATH."""
    tool = executable(tmp_path / 'tool')
    assert find_binary(tool) == tool
    assert find_binary(str(tmp_path / 'missing' / 'sh')).endswith('/sh')
    assert find_binary('') == '' and find_binary('/no/such/thing-xyz') == ''


def test_voice_rate_from_the_voice_json(tmp_path):
    """The sample rate comes from <voice>.onnx.json, 22050 otherwise."""
    model = tmp_path / 'v.onnx'
    (tmp_path / 'v.onnx.json').write_text(json.dumps({'audio': {'sample_rate': 16000}}))
    assert voice_rate(str(model)) == 16000
    assert voice_rate(str(tmp_path / 'other.onnx')) == 22050


def test_player_command_raw_and_wav():
    """Raw Piper audio names its format; eSpeak's WAV does not; a device is passed on."""
    assert player_command('aplay', '', 22050) == [
        'aplay',
        '-q',
        '-r',
        '22050',
        '-f',
        'S16_LE',
        '-t',
        'raw',
        '-c',
        '1',
        '-',
    ]
    assert player_command('aplay', 'plughw:1', 0) == ['aplay', '-q', '-D', 'plughw:1', '-']


def test_choose_engine_prefers_piper_then_espeak(tmp_path):
    """Pick Piper with a voice, else eSpeak; a forced engine that cannot run gives none."""
    piper = executable(tmp_path / 'piper')
    espeak = executable(tmp_path / 'espeak-ng')
    aplay = executable(tmp_path / 'aplay')
    voices = tmp_path / 'voices'
    voices.mkdir()
    (voices / 'id_voice.onnx').write_bytes(b'')
    cfg = EnginePaths(
        piper,
        str(voices),
        {'id': 'id_voice', 'en': 'en_voice'},
        1.2,
        espeak,
        {'id': 'id'},
        140,
        aplay,
        '',
    )
    engine, note = choose_engine('auto', 'id', cfg)
    assert engine.name == 'piper' and note == ''
    assert engine.synth == [
        piper,
        '--model',
        str(voices / 'id_voice.onnx'),
        '--output_raw',
        '--length_scale',
        '1.2',
    ]
    engine, _ = choose_engine('auto', 'en', cfg)  # no English voice file: eSpeak
    assert engine.name == 'espeak' and engine.synth[:2] == [espeak, '--stdin']
    assert engine.synth[engine.synth.index('-v') + 1] == 'en'
    engine, note = choose_engine('piper', 'en', cfg)
    assert engine.name == 'none' and 'voice' in note
    cfg = dataclasses.replace(cfg, aplay_bin=str(tmp_path / 'no_aplay'))
    engine, note = choose_engine('auto', 'id', cfg)
    assert engine.name == 'none' and note
    assert choose_engine('none', 'id', cfg) == (Engine('none'), '')


def test_queue_ids_limit_clear_and_close():
    """Ids count up, a full queue refuses, clear drops, close wakes the taker."""
    queue = SpeechQueue(max_queue=2)
    a, b = queue.add('a', 'id'), queue.add('b', 'en')
    assert (a.uid, b.uid) == (1, 2) and queue.add('c', 'id') is None
    assert queue.take(0.1) == a and queue.pending() == 1
    assert [u.uid for u in queue.clear()] == [2] and queue.take(0.01) is None
    got = []
    worker = threading.Thread(target=lambda: got.append(queue.take(5.0)))
    worker.start()
    time.sleep(0.05)
    queue.close()
    worker.join(2.0)
    assert got == [None]


def python_engine(play_code: str) -> Engine:
    """Return an engine that pipes the text through Python (no audio needed)."""
    return Engine(
        'test',
        [sys.executable, '-c', 'import sys; sys.stdout.write(sys.stdin.read())'],
        [sys.executable, '-c', play_code],
    )


def test_player_pipes_text_and_reports_done(tmp_path):
    """Pipe the text through the synthesizer's stdout to the player."""
    out = tmp_path / 'heard.txt'
    code = f'import sys; open({str(out)!r}, "w").write(sys.stdin.read())'
    assert Player().play(python_engine(code), 'halo dunia') == (SpeechState.DONE, '')
    assert out.read_text().strip() == 'halo dunia'


def test_player_failure_and_stop():
    """A failing player is reported; stop() interrupts a long utterance."""
    state, message = Player().play(python_engine('import sys; sys.exit(3)'), 'x')
    assert state is SpeechState.FAILED and 'test' in message
    player = Player()
    slow = python_engine('import sys, time; sys.stdin.read(); time.sleep(10)')
    threading.Timer(0.3, player.stop).start()
    start = time.monotonic()
    assert player.play(slow, 'x')[0] is SpeechState.CANCELLED
    assert time.monotonic() - start < 5.0
    assert player.play(slow, 'x')[0] is SpeechState.CANCELLED  # still stopped until reset()
    player.reset()
    missing = Engine('test', [os.path.join('/no', 'synth')], ['aplay'])
    assert player.play(missing, 'x')[0] is SpeechState.FAILED


def test_silent_engine_takes_time_and_stops():
    """Engine none 'speaks' len / none_chars_per_sec seconds, interruptible."""
    player = Player(none_chars_per_sec=100.0)
    start = time.monotonic()
    assert player.play(Engine('none'), 'x' * 20) == (SpeechState.DONE, '')
    assert 0.15 <= time.monotonic() - start < 1.0
    threading.Timer(0.1, player.stop).start()
    assert player.play(Engine('none'), 'x' * 1000)[0] is SpeechState.CANCELLED


def test_speech_timeout_grows_with_text():
    """Longer texts get more time."""
    assert speech_timeout('') == 10.0
    assert speech_timeout('x' * 80, 8.0, 10.0) == 20.0
