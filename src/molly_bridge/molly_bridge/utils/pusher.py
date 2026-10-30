"""The route, map and site streams: sent on a change, the map rate limited while it grows."""

from __future__ import annotations

import time

from molly_bridge.core import map_image, protocol, summaries


class Pusher:
    """Builds the stream payloads of the telemetry's route, map and site."""

    def __init__(self, telemetry, send_stream, map_max_px: int, map_push_sec: float):
        self.telemetry, self.send_stream = telemetry, send_stream
        self.map_max_px, self.map_push_sec = map_max_px, map_push_sec
        self.map_sent = (-1, 0.0)  # (revision, monotonic time)
        self.map_payload: tuple | None = None

    def push(self, kind: str, force: bool = False):
        """Send the route, map or site stream."""
        t = self.telemetry
        if kind == 'route':
            route = protocol.encode_json({'t': 'route', 'points': t.route})
            self.send_stream(protocol.STREAM_ROUTE, route)
        elif kind == 'site':
            payload = summaries.site_payload(t.site, t.pois, t.tours, t.site_rev)
            self.send_stream(protocol.STREAM_SITE, protocol.encode_json(payload))
        elif kind == 'map':
            self.push_map(force)

    def push_map(self, force: bool):
        """Send the map when the client lacks the latest one (not faster than map_push_sec)."""
        t = self.telemetry
        if t.grid is None:
            return
        rev, sent_at = self.map_sent
        recent = time.monotonic() - sent_at < self.map_push_sec
        if not force and (rev == t.map_rev or recent):
            return
        if self.map_payload is None or self.map_payload[0] != t.map_rev:
            header, png = map_image.map_frame(t.grid, self.map_max_px, t.map_rev, t.site)
            self.map_payload = (t.map_rev, protocol.map_payload(header, png))
        self.map_sent = (t.map_rev, time.monotonic())
        self.send_stream(protocol.STREAM_MAP, self.map_payload[1])
