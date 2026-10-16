from molly_health.checks import (
    CpuUsage,
    disk_free_gb,
    gpu_load_path,
    gpu_percent,
    memory_kb,
    missing_nodes,
    overall_level,
    parse_topic_specs,
    RateTracker,
    state_issues,
)


def test_rate_of_a_10hz_topic():
    """Ten messages a second read as ~10 Hz."""
    tracker = RateTracker(window_sec=5.0)
    for i in range(30):
        tracker.tick(i * 0.1)
    assert abs(tracker.rate(2.95) - 10.0) < 0.5


def test_silent_topic_drops_to_zero():
    """A topic that stopped publishing reads as 0 Hz."""
    tracker = RateTracker(window_sec=5.0)
    for i in range(20):
        tracker.tick(i * 0.1)
    assert tracker.rate(10.0) == 0.0


def test_topic_specs():
    """'topic:hz' strings parse, bare topics get 0 Hz."""
    expected = {'/a': 5.0, '/b/c': 12.5, '/d': 0.0}
    assert parse_topic_specs(['/a:5', '/b/c:12.5', '/d', '']) == expected
    assert parse_topic_specs(['/{robot_ns}/odom:10'], '/dog/') == {'/dog/odom': 10.0}
    assert parse_topic_specs(['/{robot_ns}/odom:10']) == {'/robot/odom': 10.0}


def test_disk_free_is_reported():
    """Disk space of the home directory is readable."""
    assert disk_free_gb('~') > 0


def test_missing_nodes_ignore_leading_slash():
    """Expected names match running names with or without a leading slash."""
    running = ['control_mux_node', '/path_follower_node']
    expected = ['/control_mux_node', 'path_follower_node', 'teleop_udp_node']
    assert missing_nodes(expected, running) == ['teleop_udp_node']
    assert missing_nodes([''], running) == []


def test_state_issues_and_level():
    """Lost odometry, a stale safety scan or the e-stop make the robot unable to patrol."""
    assert state_issues('ok', 'clear', False) == []
    assert state_issues('', '', False) == []
    issues = state_issues('lost: no odometry for 2.0 s', 'stale', True)
    assert len(issues) == 3
    assert overall_level([], []) == 'ok'
    assert overall_level(['/livox/lidar 2.0/5 Hz'], []) == 'warn'
    assert overall_level(issues, issues) == 'error'


def test_cpu_usage_between_two_readings(tmp_path):
    """The busy share comes from the /proc/stat counters between two readings."""
    stat = tmp_path / 'stat'
    stat.write_text('cpu  100 0 100 700 100 0 0 0 0 0\ncpu0 1 2 3 4\n')
    cpu = CpuUsage(str(stat))
    assert cpu.percent() is None  # nothing to compare with yet
    stat.write_text('cpu  200 0 200 1300 100 0 0 0 0 0\n')
    assert cpu.percent() == 25.0
    assert CpuUsage(str(tmp_path / 'missing')).percent() is None


def test_jetson_gpu_load_and_memory(tmp_path):
    """The Jetson GPU load is per mille; no load file means no GPU figure."""
    load = tmp_path / 'gpu.0' / 'load'
    load.parent.mkdir()
    load.write_text('412\n')
    path = gpu_load_path([str(tmp_path / 'nope' / 'load'), str(tmp_path / '*.0' / 'load')])
    assert path == str(load) and gpu_percent(path) == 41.2
    assert gpu_load_path([str(tmp_path / 'nope' / 'load')]) is None and gpu_percent(None) is None
    meminfo = tmp_path / 'meminfo'
    meminfo.write_text('MemTotal: 16000000 kB\nMemFree: 1000 kB\nMemAvailable: 12000000 kB\n')
    assert memory_kb(str(meminfo)) == (4000000, 16000000)
