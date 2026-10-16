import math

MODES = ('auto', 'remote')


def envelope(vx: float, vy: float, wz: float, max_vx: float, max_vy: float, max_wz: float):
    """
    Return (vx, vy, wz) inside the robot's speed envelope, or None if a value is not finite.

    Linear x and y are scaled together, so the direction of travel is kept; the yaw rate is
    clamped on its own.
    """
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        return None
    scale = 1.0
    if abs(vx) > max_vx:
        scale = min(scale, max_vx / abs(vx))
    if abs(vy) > max_vy:
        scale = min(scale, max_vy / abs(vy))
    return vx * scale, vy * scale, max(-max_wz, min(max_wz, wz))


class ControlMux:
    """
    Chooses which velocity command reaches the robot: navigation (auto) or teleop (remote).

    A latched software e-stop overrides both: while it is engaged nothing but zero reaches the
    robot, and only an explicit release clears it. The watchdog zeroes the robot once when the
    active source goes silent (teleop after remote_timeout, navigation after nav_timeout).
    """

    def __init__(self, mode: str = 'auto', remote_timeout: float = 0.3, nav_timeout: float = 0.5):
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.mode = mode
        self._remote_timeout = remote_timeout
        self._nav_timeout = nav_timeout
        self._stamp = None  # last forwarded command time of the active source
        self._stale_sent = True
        self.estop = False
        self.estop_reason = ''

    def set_mode(self, mode: str) -> bool:
        """Switch mode; returns True when it changed."""
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        changed = mode != self.mode
        self.mode = mode
        self._stamp = None
        self._stale_sent = False
        return changed

    def engage_estop(self, reason: str) -> bool:
        """Latch the e-stop; returns True when it was not engaged yet."""
        changed = not self.estop
        self.estop = True
        self.estop_reason = reason or 'unspecified'
        self._stale_sent = False
        return changed

    def release_estop(self) -> bool:
        """Clear the e-stop; returns True when it was engaged."""
        changed = self.estop
        self.estop = False
        self.estop_reason = ''
        self._stamp = None
        return changed

    def on_nav(self, cmd, now: float = 0.0):
        """Return the command to forward for a navigation command, or None to drop it."""
        if self.estop or self.mode != 'auto':
            return None
        self._stamp = now
        self._stale_sent = False
        return cmd

    def on_remote(self, cmd, now: float):
        """Return the command to forward for a teleop command, or None to drop it."""
        if self.estop or self.mode != 'remote':
            return None
        self._stamp = now
        self._stale_sent = False
        return cmd

    def watchdog(self, now: float) -> bool:
        """Return True once when the active source went silent, after a switch or an e-stop."""
        if self._stale_sent:
            return False
        if not self.estop and self._stamp is not None:
            timeout = self._remote_timeout if self.mode == 'remote' else self._nav_timeout
            if now - self._stamp < timeout:
                return False
        self._stale_sent = True
        return True
