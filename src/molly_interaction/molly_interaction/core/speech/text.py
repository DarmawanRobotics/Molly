"""Text clean-up for the speech engines (no ROS)."""

from __future__ import annotations

import re

CONTROL = re.compile(r'[\x00-\x1f\x7f]')


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


def speech_timeout(text: str, chars_per_sec: float = 8.0, base: float = 10.0) -> float:
    """Return a generous time limit to say a text (s)."""
    return base + len(text) / max(chars_per_sec, 1.0)
