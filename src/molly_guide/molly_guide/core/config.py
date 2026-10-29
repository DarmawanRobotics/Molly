"""Configuration of guide_node and site_content_node (typed parameters)."""

from __future__ import annotations

from dataclasses import dataclass

from dharma_utils.config import param

ON_FAILURE = ('skip', 'stop')
INTRO_TEXTS = (
    'id:Halo semuanya, saya Molly. Mari ikuti saya, kita mulai turnya.',
    "en:Hello everyone, I am Molly. Please follow me, let's start the tour.",
)
OUTRO_TEXTS = (
    'id:Turnya sudah selesai. Terima kasih sudah berkeliling bersama saya!',
    'en:That is the end of the tour. Thank you for walking with me!',
)
FOLLOW_ME_TEXTS = ('id:Ayo, ikuti saya ya.', 'en:Please follow me.')


@dataclass(frozen=True)
class GuideConfig:
    """How a tour runs: texts, timeouts, retries and waiting for the visitors."""

    language: str = param('id', live=False, doc="language of the guide's own texts")
    intro_texts: tuple[str, ...] = param(
        INTRO_TEXTS, live=False, doc='said at the start of a tour, language:text'
    )
    outro_texts: tuple[str, ...] = param(
        OUTRO_TEXTS, live=False, doc='said at the end of a tour, language:text'
    )
    follow_me_texts: tuple[str, ...] = param(
        FOLLOW_ME_TEXTS, live=False, doc='said when nobody follows, language:text'
    )
    gestures: bool = param(True, doc='play the POI gestures (interaction/gesture)')
    narrate: bool = param(True, doc="say the POI narrations and the guide's texts")
    arrival_timeout_sec: float = param(
        300.0, min=5.0, max=3600.0, unit='s', doc='walking time to a POI (waits not counted)'
    )
    go_to_retries: int = param(1, min=0, max=10, doc='tries after a failed walk')
    retry_delay_sec: float = param(3.0, min=0.0, max=120.0, unit='s', doc='wait before a retry')
    on_failure: str = param(
        'skip', choices=ON_FAILURE, doc='unreachable POI: skip it, or stop the tour in ERROR'
    )
    gesture_timeout_sec: float = param(
        25.0, min=1.0, max=300.0, unit='s', doc='a gesture that takes longer is given up'
    )
    narration_timeout_base_sec: float = param(
        15.0, min=1.0, max=600.0, unit='s', doc='narration time limit: base + length / rate'
    )
    narration_chars_per_sec: float = param(
        8.0, min=1.0, max=100.0, unit='1/s', doc='narration time limit: characters per second'
    )
    wait_for_visitors: bool = param(
        True, doc='hold the walk and ask the visitors to follow when nobody is behind'
    )
    visitor_lost_sec: float = param(
        6.0, min=0.5, max=120.0, unit='s', doc='nobody behind this long: wait for them'
    )
    visitor_found_sec: float = param(
        1.0, min=0.0, max=60.0, unit='s', doc='somebody behind this long: walk on'
    )
    visitor_wait_sec: float = param(
        20.0, min=1.0, max=600.0, unit='s', doc='longest wait, then not again on this leg'
    )
    visitors_timeout_sec: float = param(
        2.0, min=0.1, max=30.0, unit='s', doc='older perception/visitors: no waiting'
    )
    require_localization: bool = param(True, doc='START needs localization/status localized')
    require_auto_mode: bool = param(True, doc='START needs the control mux in AUTO')
    stop_settle_sec: float = param(
        0.5, min=0.0, max=10.0, unit='s', doc='STOPPING lasts this long before IDLE'
    )
    tick_hz: float = param(10.0, min=1.0, max=50.0, unit='Hz', live=False, doc='timer rate')
    publish_period_sec: float = param(
        1.0, min=0.1, max=10.0, unit='s', doc='tour/state also without a change'
    )
    goal_tolerance: float = param(
        0.05, min=0.001, max=1.0, unit='m', doc='planner/status goal counts as the POI within'
    )


@dataclass(frozen=True)
class SiteContentConfig:
    """Where the POIs and tours of the sites live."""

    sites_dir: str = param(
        '~/dev/Molly/map', live=False, doc='one directory per site (same as dharma_mapping)'
    )
    refresh_sec: float = param(
        2.0, min=0.2, max=60.0, unit='s', doc='files re-read when changed on disk'
    )
    archive_with_map: bool = param(
        True, doc='move POIs and tours into the backup a mapping reset makes of the site'
    )
