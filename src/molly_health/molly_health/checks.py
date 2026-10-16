from collections import deque
import glob
import os
import shutil


class RateTracker:
    """Message rate over a sliding window; drops to zero when the topic goes silent."""

    def __init__(self, window_sec: float = 5.0):
        self.window = window_sec
        self._stamps: deque = deque()

    def tick(self, now: float) -> None:
        """Record one message at time now (s)."""
        self._stamps.append(now)
        while self._stamps and now - self._stamps[0] > self.window:
            self._stamps.popleft()

    def rate(self, now: float) -> float:
        """Return messages per second, 0 when nothing arrived within the window."""
        while self._stamps and now - self._stamps[0] > self.window:
            self._stamps.popleft()
        if len(self._stamps) < 2:
            return 0.0
        span = max(self._stamps[-1] - self._stamps[0], 1e-6)
        if now - self._stamps[-1] > max(1.0, 3.0 * span / (len(self._stamps) - 1)):
            return 0.0
        return (len(self._stamps) - 1) / span


def parse_topic_specs(specs: list[str], robot_ns: str = 'robot') -> dict[str, float]:
    """Parse 'topic:min_hz' strings; '{robot_ns}' in a topic becomes the robot namespace."""
    out = {}
    for spec in specs:
        if not spec:
            continue
        spec = spec.replace('{robot_ns}', robot_ns.strip('/'))
        topic, _, hz = spec.rpartition(':')
        try:
            out[topic or spec] = float(hz) if topic else 0.0
        except ValueError:
            out[spec] = 0.0
    return out


def cpu_temperature() -> float | None:
    """Return the hottest thermal zone in degrees C, or None."""
    temps = []
    for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
        try:
            with open(path) as f:
                temps.append(int(f.read().strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return max(temps) if temps else None


def memory_kb(path: str = '/proc/meminfo') -> tuple | None:
    """Return (used, total) memory in kB (used = total - available), or None."""
    try:
        with open(path) as f:
            info = {line.split(':')[0]: int(line.split()[1]) for line in f if ':' in line}
        return info['MemTotal'] - info['MemAvailable'], info['MemTotal']
    except (OSError, KeyError, ValueError, IndexError):
        return None


def memory_percent() -> float | None:
    """Return used memory in percent from /proc/meminfo, or None."""
    memory = memory_kb()
    return 100.0 * memory[0] / memory[1] if memory and memory[1] else None


class CpuUsage:
    """Share of the CPU time in use since the previous reading (all cores), from /proc/stat."""

    def __init__(self, path: str = '/proc/stat'):
        self._path = path
        self._last = None

    def percent(self) -> float | None:
        """Return the busy share in percent since the last call; None on the first call."""
        try:
            with open(self._path) as f:
                fields = [int(x) for x in f.readline().split()[1:9]]
        except (OSError, ValueError):
            return None
        if len(fields) < 4:
            return None
        idle, total = fields[3] + (fields[4] if len(fields) > 4 else 0), sum(fields)
        last, self._last = self._last, (idle, total)
        if last is None or total <= last[1]:
            return None
        return 100.0 * (1.0 - (idle - last[0]) / (total - last[1]))


# Jetson GPU load (0..1000) in sysfs: Orin on JetPack 6, Orin on JetPack 5, Nano / Xavier
GPU_LOAD_PATTERNS = (
    '/sys/devices/platform/bus@0/*.gpu/load',
    '/sys/devices/platform/gpu.0/load',
    '/sys/devices/gpu.0/load',
    '/sys/devices/platform/*.gpu/load',
    '/sys/devices/*.ga10b/load',
    '/sys/devices/*.gv11b/load',
)


def gpu_load_path(patterns=GPU_LOAD_PATTERNS) -> str | None:
    """Return the readable GPU load file of a Jetson, or None (no Jetson GPU)."""
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            if os.access(path, os.R_OK):
                return path
    return None


def gpu_percent(path: str | None) -> float | None:
    """Return the GPU load in percent from a Jetson load file (per mille), or None."""
    if path is None:
        return None
    try:
        with open(path) as f:
            return min(int(f.read().strip()) / 10.0, 100.0)
    except (OSError, ValueError):
        return None


def disk_free_gb(path: str) -> float | None:
    """Return free space in GB at path, or None."""
    try:
        return shutil.disk_usage(os.path.expanduser(path)).free / 1e9
    except OSError:
        return None


def missing_nodes(expected: list[str], running) -> list[str]:
    """Return the expected node names (with or without a leading /) that are not running."""
    names = {'/' + n.lstrip('/') for n in running}
    return [n for n in expected if n and '/' + n.lstrip('/') not in names]


def state_issues(odometry: str, field: str, estop: bool) -> list[str]:
    """Return issues from the odometry, protective-field and e-stop states."""
    issues = []
    if odometry.startswith(('lost', 'degraded')):
        issues.append(f'odometry {odometry}')
    if field == 'stale':
        issues.append('safety scan stale: navigation stopped')
    if estop:
        issues.append('e-stop engaged')
    return issues


def overall_level(issues: list[str], critical: list[str]) -> str:
    """Return 'ok', 'warn' (something degraded) or 'error' (the robot cannot patrol)."""
    if critical:
        return 'error'
    return 'warn' if issues else 'ok'
