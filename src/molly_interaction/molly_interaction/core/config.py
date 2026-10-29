"""Configuration of the interaction nodes (typed parameters)."""

from __future__ import annotations

from dataclasses import dataclass

from dharma_utils.config import param
from molly_interaction.core.speech.engines import ENGINES

GESTURES = ('hello', 'nod', 'bow', 'look_left', 'look_right')
GREETINGS_ID = (
    'Halo, selamat datang! Saya Molly, robot pemandu di sini.',
    'Hai, senang bertemu dengan Anda! Ada yang bisa saya bantu?',
)
GREETINGS_EN = (
    'Hello and welcome! I am Molly, the guide robot here.',
    'Hi, nice to meet you! Can I help you?',
)


@dataclass(frozen=True)
class SpeechConfig:
    """speech_node: engines, voices, the sound card and the queue (read at start)."""

    engine: str = param('auto', choices=ENGINES, live=False, doc='auto: piper, espeak, none')
    default_language: str = param('id', live=False, doc='language of a text without one')
    piper_bin: str = param('/opt/molly/piper/piper', live=False, doc='else piper on PATH')
    voices_dir: str = param('/opt/molly/piper/voices', live=False, doc='<voice>.onnx(.json)')
    voices: tuple[str, ...] = param(
        ('id:id_ID-news_tts-medium', 'en:en_US-lessac-medium'),
        live=False,
        doc='Piper voice per language, language:voice',
    )
    piper_length_scale: float = param(1.0, min=0.3, max=3.0, live=False, doc='> 1 slower')
    espeak_bin: str = param('espeak-ng', live=False, doc='eSpeak NG binary')
    espeak_voices: tuple[str, ...] = param(
        ('id:id', 'en:en-us'), live=False, doc='eSpeak NG voice per language'
    )
    espeak_wpm: int = param(150, min=50, max=400, live=False, doc='words per minute')
    aplay_bin: str = param('aplay', live=False, doc='player (alsa-utils)')
    audio_device: str = param('', live=False, doc='aplay -D device; empty: the default card')
    max_queue: int = param(10, min=1, max=100, live=False, doc='texts waiting; more refused')
    max_chars: int = param(1000, min=1, max=10000, live=False, doc='longer texts are cut')
    none_chars_per_sec: float = param(
        15.0, min=1.0, max=10000.0, live=False, doc='engine none: how long speaking takes'
    )


@dataclass(frozen=True)
class InteractionConfig:
    """interaction_node: the driver's gestures and the greeter."""

    robot_ns: str = param('robot', live=False, doc='gestures call /<robot_ns>/<name> (Trigger)')
    gestures: tuple[str, ...] = param(GESTURES, live=False, doc="the driver's gesture services")
    gesture_timeout_sec: float = param(
        20.0, min=1.0, max=300.0, unit='s', live=False, doc='a gesture taking longer fails'
    )
    greeter_available: bool = param(True, live=False, doc='false: no greeter on this robot')
    enabled: bool = param(True, doc='greet approaching visitors (kept in state_file)')
    state_file: str = param(
        '~/.molly/greeter.json', live=False, doc='keeps the on / off; empty: not kept'
    )
    approach_distance: float = param(
        1.5, min=0.2, max=5.0, unit='m', live=False, doc='somebody this close in front'
    )
    dwell_sec: float = param(1.5, min=0.0, max=30.0, unit='s', live=False, doc='... this long')
    cooldown_sec: float = param(
        30.0, min=0.0, max=3600.0, unit='s', live=False, doc='at most one greeting per'
    )
    visitors_timeout_sec: float = param(
        1.0, min=0.1, max=10.0, unit='s', live=False, doc='older perception/visitors: nobody'
    )
    language: str = param('id', live=False, doc='greeting_texts_<language> are said')
    greeting_texts_id: tuple[str, ...] = param(GREETINGS_ID, live=False, doc='round robin')
    greeting_texts_en: tuple[str, ...] = param(GREETINGS_EN, live=False, doc='round robin')
    greeting_gesture: str = param(
        '', live=False, doc='a gesture with the greeting (robot standing still); empty: none'
    )


@dataclass(frozen=True)
class ConversationConfig:
    """conversation_node: the local language model (Ollama)."""

    url: str = param('http://127.0.0.1:11434', live=False, doc='Ollama')
    model: str = param('qwen2.5:3b', live=False, doc='a chat model pulled into Ollama')
    timeout_sec: float = param(
        30.0, min=1.0, max=600.0, unit='s', live=False, doc='the first question loads the model'
    )
    max_answer_chars: int = param(600, min=50, max=5000, live=False, doc='cut after a sentence')
    max_tokens: int = param(200, min=10, max=4000, live=False, doc='num_predict')
    language: str = param('id', live=False, doc='answer language when the question names none')
    robot_name: str = param('Molly', live=False, doc='the name in the system prompt')
    history_turns: int = param(2, min=0, max=20, live=False, doc='earlier questions kept')


@dataclass(frozen=True)
class VisitorConfig:
    """visitor_tracker_node: people in the LiDAR around the robot."""

    input_type: str = param(
        'livox', choices=('livox', 'pointcloud2'), live=False, doc='Livox CustomMsg or a cloud'
    )
    livox_topic: str = param('/livox/lidar', live=False, doc='Livox CustomMsg (raw)')
    pointcloud_topic: str = param(
        '/cloud_registered_body', live=False, doc='PointCloud2 in a frame fixed to the robot'
    )
    lidar_frame: str = param('livox_frame', live=False, doc='used when a cloud has no frame_id')
    height_min: float = param(
        0.15, min=0.0, max=2.0, unit='m', live=False, doc='legs and torso above the floor'
    )
    height_max: float = param(1.3, min=0.1, max=3.0, unit='m', live=False, doc='band top')
    self_margin: float = param(
        0.05, min=0.0, max=1.0, unit='m', live=False, doc='points this close to the body: robot'
    )
    range_min: float = param(0.3, min=0.0, max=5.0, unit='m', live=False, doc='from base_link')
    max_range: float = param(
        5.0, min=0.5, max=30.0, unit='m', live=False, doc='visitors further away do not count'
    )
    cluster_tolerance: float = param(
        0.25, min=0.02, max=2.0, unit='m', live=False, doc='closer points join a cluster'
    )
    min_points: int = param(4, min=1, max=1000, live=False, doc='a person has this many points')
    min_width: float = param(0.1, min=0.0, max=2.0, unit='m', live=False, doc='person size')
    max_width: float = param(1.0, min=0.1, max=5.0, unit='m', live=False, doc='person size')
    use_map: bool = param(True, live=False, doc='clusters on the map walls are not visitors')
    map_margin: float = param(0.25, min=0.0, max=2.0, unit='m', live=False, doc='walls grown')
    front_angle_deg: float = param(
        90.0, min=10.0, max=360.0, unit='deg', live=False, doc='front sector (the greeter)'
    )
    behind_angle_deg: float = param(
        150.0, min=10.0, max=360.0, unit='deg', live=False, doc='rear sector (followers)'
    )
    confirm_scans: int = param(2, min=1, max=20, live=False, doc='scans in a row to count')
    max_misses: int = param(2, min=0, max=20, live=False, doc='scans kept after it went')
    assoc_distance: float = param(
        0.6, min=0.05, max=5.0, unit='m', live=False, doc='movement between two scans'
    )
    rate_hz: float = param(5.0, min=0.5, max=50.0, unit='Hz', live=False, doc='scans processed')
    publish_markers: bool = param(True, live=False, doc='perception/visitor_markers for RViz')
