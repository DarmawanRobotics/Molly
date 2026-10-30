"""GStreamer for the video node: probing an encoder, its optional properties."""

from __future__ import annotations

from molly_bridge.core.gstreamer import OPTIONAL, probe_description

try:
    import gi

    gi.require_version('Gst', '1.0')
    from gi.repository import GLib, Gst
except (ImportError, ValueError):  # python3-gi or gir1.2-gstreamer-1.0 missing
    GLib = Gst = None


def set_optional(element, encoder: str):
    """Set the OPTIONAL properties of an encoder that this version has."""
    if element is None:
        return
    for name, value in OPTIONAL.get(encoder, {}).items():
        if element.find_property(name) is not None:
            element.set_property(name, value)


def probe(encoder: str, timeout_sec: float = 5.0):
    """Encode test frames with one encoder: None when it works, else the reason it does not."""
    try:
        pipeline = Gst.parse_launch(probe_description(encoder))
    except GLib.Error as error:  # the element is not installed
        return error.message
    set_optional(pipeline.get_by_name('enc'), encoder)
    try:
        pipeline.set_state(Gst.State.PLAYING)
        message = pipeline.get_bus().timed_pop_filtered(
            int(timeout_sec * Gst.SECOND), Gst.MessageType.EOS | Gst.MessageType.ERROR
        )
    finally:
        pipeline.set_state(Gst.State.NULL)
    if message is None:
        return f'no frames within {timeout_sec:g} s'
    if message.type == Gst.MessageType.ERROR:
        return message.parse_error()[0].message
    return None
