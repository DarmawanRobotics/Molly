from molly_bridge.mux import ControlMux, envelope
import pytest


def test_auto_forwards_navigation_only():
    """Auto mode passes navigation and drops teleop."""
    mux = ControlMux('auto')
    assert mux.on_nav('nav') == 'nav'
    assert mux.on_remote('joy', 0.0) is None


def test_remote_forwards_teleop_only_and_stops_when_silent():
    """Remote mode passes teleop, drops navigation and zeroes once after the timeout."""
    mux = ControlMux('auto', remote_timeout=0.3)
    assert mux.set_mode('remote')
    assert mux.watchdog(0.0)  # zero right after the switch
    assert mux.on_nav('nav') is None
    assert mux.on_remote('joy', 1.0) == 'joy'
    assert not mux.watchdog(1.2)
    assert mux.watchdog(1.4)
    assert not mux.watchdog(1.5)


def test_unknown_mode_rejected():
    """Only auto and remote exist."""
    with pytest.raises(ValueError):
        ControlMux('manual')


def test_estop_blocks_both_sources_until_released():
    """Nothing but zero passes while the e-stop is latched; release restores forwarding."""
    mux = ControlMux('auto')
    assert mux.engage_estop('test')
    assert not mux.engage_estop('again')  # already engaged
    assert mux.watchdog(0.0)  # one zero right away
    assert mux.on_nav('nav', 0.1) is None
    mux.set_mode('remote')
    assert mux.on_remote('joy', 0.2) is None
    assert mux.estop and mux.estop_reason == 'again'
    assert mux.release_estop()
    assert not mux.release_estop()
    assert mux.on_remote('joy', 0.3) == 'joy'


def test_navigation_silence_zeroes_once():
    """In auto mode a silent navigation source gets exactly one zero command."""
    mux = ControlMux('auto', nav_timeout=0.5)
    mux.watchdog(0.0)  # settle the initial zero
    assert mux.on_nav('nav', 1.0) == 'nav'
    assert not mux.watchdog(1.3)
    assert mux.watchdog(1.6)
    assert not mux.watchdog(2.0)


def test_envelope_keeps_direction_and_rejects_non_finite():
    """Over-limit commands are scaled down without turning; NaN / inf are rejected."""
    assert envelope(0.4, 0.1, 0.5, 1.0, 0.5, 1.5) == (0.4, 0.1, 0.5)
    vx, vy, wz = envelope(2.0, 1.0, -3.0, 1.0, 0.5, 1.5)
    assert (vx, vy, wz) == (1.0, 0.5, -1.5)  # both halved: same heading
    vx, vy, _ = envelope(0.5, -1.0, 0.0, 1.0, 0.5, 1.5)
    assert (vx, vy) == (0.25, -0.5)
    for bad in (float('nan'), float('inf'), -float('inf')):
        assert envelope(bad, 0.0, 0.0, 1.0, 0.5, 1.5) is None
        assert envelope(0.0, 0.0, bad, 1.0, 0.5, 1.5) is None
