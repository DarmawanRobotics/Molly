"""
End to end: the simulated robot guides a tour, maps a new site and walks back with the path finder.

Starts `ros2 launch molly sim.launch.py` with a fresh sites directory (the mall_loop site is
drawn from the world, with the demo tour). The tour walks to both POIs and narrates there; then a
new site is mapped and saved; then the mall_loop site is loaded again and the path finder walks
the robot back to its start. Finally no node may have died or printed a traceback. Set
MOLLY_SKIP_SIM=1 to skip it.
"""

import math
import os
import signal
import subprocess
import tempfile
import time

import pytest

rclpy = pytest.importorskip('rclpy')
qos = pytest.importorskip('rclpy.qos')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
std_msgs = pytest.importorskip('std_msgs.msg')
dharma_msg = pytest.importorskip('dharma_interfaces.msg')
msg = pytest.importorskip('molly_interfaces.msg')
dharma_srv = pytest.importorskip('dharma_interfaces.srv')
srv = pytest.importorskip('molly_interfaces.srv')
std_srvs = pytest.importorskip('std_srvs.srv')

pytestmark = pytest.mark.skipif(
    os.environ.get('MOLLY_SKIP_SIM') == '1', reason='MOLLY_SKIP_SIM=1'
)
DOMAIN = 100 + os.getpid() % 30
FOUNTAIN = (1.5, 3.0, math.pi)     # the last POI of the demo tour (dharma_sim/site_fixture.py)
START = (-4.5, -3.0, 0.0)          # the world's start pose


class Guide:
    """Talks to the stack like the dashboard does and keeps what it reports."""

    def __init__(self, node):
        """Subscribe to the tour, speech, site, mapping, planner and the true robot pose."""
        self.node = node
        self.tour, self.speech, self.poses = [], [], []
        self.site = self.mapping = self.planner = None
        self.localization = 'none'
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(msg.TourState, '/tour/state', self.tour.append, latched)
        node.create_subscription(
            msg.SpeechStatus, 'interaction/speech_status', self.speech.append, latched
        )
        node.create_subscription(dharma_msg.SiteStatus, 'site/status', self.on_site, latched)
        node.create_subscription(
            dharma_msg.MappingStatus, 'mapping/status', self.on_mapping, latched
        )
        node.create_subscription(
            dharma_msg.PlannerStatus, 'planner/status', self.on_planner, latched
        )
        node.create_subscription(
            std_msgs.String, 'localization/status', self.on_localization, latched
        )
        node.create_subscription(geometry_msgs.PoseStamped, 'sim/ground_truth', self.on_pose, 10)

    def on_site(self, status):
        """Keep the latest site status."""
        self.site = status

    def on_mapping(self, status):
        """Keep the latest mapping status."""
        self.mapping = status

    def on_planner(self, status):
        """Keep the latest planner status."""
        self.planner = status

    def on_localization(self, status):
        """Keep the localization source."""
        self.localization = status.data

    def on_pose(self, pose):
        """Keep (x, y, yaw) of the true robot pose."""
        q = pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.poses.append((pose.pose.position.x, pose.pose.position.y, yaw))

    def spin_until(self, condition, timeout: float) -> bool:
        """Spin until condition() is true or the timeout passes."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return True
        return False

    def call(self, kind, name: str, request, timeout: float = 30.0):
        """Call a service and return its response (None when it did not answer)."""
        client = self.node.create_client(kind, name)
        if not self.spin_until(client.service_is_ready, 30.0):
            return None
        future = client.call_async(request)
        self.spin_until(future.done, timeout)
        return future.result() if future.done() else None

    def near(self, goal, tolerance: float, yaw_tolerance: float) -> bool:
        """Return True when the true robot pose is at goal (x, y, yaw)."""
        if not self.poses:
            return False
        x, y, yaw = self.poses[-1]
        dyaw = abs(math.remainder(yaw - goal[2], math.tau))
        return math.hypot(x - goal[0], y - goal[1]) < tolerance and dyaw < yaw_tolerance


def default_sigint():
    """Give the launch the default SIGINT action, even if this test runs as a background job."""
    signal.signal(signal.SIGINT, signal.SIG_DFL)


@pytest.fixture(scope='module')
def guide():
    """Run the simulation with a fresh sites directory; check the logs at the end."""
    sites = tempfile.mkdtemp(prefix='molly_sites_')
    log = tempfile.NamedTemporaryFile('w+', prefix='molly_guide_', suffix='.log', delete=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    proc = subprocess.Popen(
        ['ros2', 'launch', 'molly', 'sim.launch.py', f'sites_dir:={sites}'],
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        preexec_fn=default_sigint,
    )
    rclpy.init(domain_id=DOMAIN)
    node = rclpy.create_node('sim_guide_test')
    recorder = Guide(node)
    recorder.sites = sites
    try:
        yield recorder
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.kill(proc.pid, signal.SIGINT)  # ros2 launch only, as `systemctl stop` does
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        log.seek(0)
        output = log.read()
        log.close()
        problems = [
            line for line in output.splitlines()
            if 'process has died' in line or 'Traceback' in line
        ]
        assert not problems, '\n'.join(problems) + f'\nfull log: {log.name}'


def test_tour_walks_to_every_poi_and_narrates(guide):
    """The demo tour goes POI by POI, says each narration and ends idle at the last POI."""
    assert guide.spin_until(
        lambda: guide.site is not None and guide.site.site == 'mall_loop' and guide.site.has_map,
        60.0,
    ), 'the mall_loop site did not load'
    assert guide.spin_until(lambda: guide.localization != 'none', 60.0), 'not localized'
    assert guide.spin_until(lambda: bool(guide.tour), 30.0), 'no /tour/state'
    command = msg.TourCommand(kind=msg.TourCommand.KIND_START, tour_id='demo')
    request = srv.TourControl.Request(command=command)
    answer = guide.call(srv.TourControl, '/tour/control', request)
    for _ in range(20):  # the guide hears of the localization a moment after this test does
        if answer is None or answer.success or 'not localized' not in answer.message:
            break
        guide.spin_until(lambda: False, 0.5)
        answer = guide.call(srv.TourControl, '/tour/control', request)
    assert answer is not None and answer.success, answer.message if answer else 'no answer'
    running = msg.TourState.STATE_RUNNING
    assert guide.spin_until(lambda: guide.tour[-1].state == running, 30.0), 'tour did not run'
    ended = guide.spin_until(
        lambda: guide.tour[-1].state in (msg.TourState.STATE_IDLE, msg.TourState.STATE_ERROR),
        240.0,
    )
    states = [(s.state, s.current_poi_id, s.activity) for s in guide.tour]
    assert ended and guide.tour[-1].state == msg.TourState.STATE_IDLE, f'tour: {states[-8:]}'
    narrated = [s.current_poi_id for s in guide.tour if s.activity == 'narrating']
    assert 'east_gate' in narrated and 'fountain' in narrated, states
    said = ' '.join(s.text for s in guide.speech)
    assert 'east gate' in said and 'fountain' in said, said
    assert guide.near(FOUNTAIN, 0.3, math.radians(15.0)), f'robot at {guide.poses[-1]}'


def test_new_site_is_mapped_and_saved(guide):
    """mapping/start builds a grid from the LiDAR; mapping/save writes it into the new site."""
    started = guide.call(
        dharma_srv.StartMapping, 'mapping/start',
        dharma_srv.StartMapping.Request(site='scratch', reset=True),
    )
    assert started is not None and started.success, started.message if started else 'no answer'
    assert guide.spin_until(
        lambda: guide.mapping is not None and guide.mapping.active and guide.mapping.scans >= 3
        and guide.mapping.known_area_m2 > 5.0,
        30.0,
    ), f'mapping: {guide.mapping}'
    saved = guide.call(std_srvs.Trigger, 'mapping/save', std_srvs.Trigger.Request())
    assert saved is not None and saved.success, saved.message if saved else 'no answer'
    assert os.path.isfile(os.path.join(guide.sites, 'scratch', 'map.yaml'))
    stopped = guide.call(std_srvs.Trigger, 'mapping/stop', std_srvs.Trigger.Request())
    assert stopped is not None and stopped.success
    assert guide.spin_until(
        lambda: guide.site.site == 'scratch' and guide.site.has_map and not guide.site.mapping,
        15.0,
    ), f'site: {guide.site}'


def test_path_finder_walks_back_to_the_start(guide):
    """Back on mall_loop, planner/go_to finds a way around the shops to the start pose."""
    loaded = guide.call(
        dharma_srv.LoadSite, 'site/load', dharma_srv.LoadSite.Request(site='mall_loop')
    )
    assert loaded is not None and loaded.success, loaded.message if loaded else 'no answer'
    assert guide.spin_until(lambda: guide.localization != 'none', 30.0), 'not localized again'
    request = dharma_srv.GoTo.Request(poi_id='', x=START[0], y=START[1], yaw=START[2])
    gone = guide.call(dharma_srv.GoTo, 'planner/go_to', request)
    for _ in range(20):  # the map and map -> base_link come back a moment after the site loads
        if gone is None or gone.success or not gone.message.startswith(('robot pose', 'no map')):
            break
        guide.spin_until(lambda: False, 0.5)
        gone = guide.call(dharma_srv.GoTo, 'planner/go_to', request)
    assert gone is not None and gone.success, gone.message if gone else 'no answer'
    assert guide.spin_until(
        lambda: guide.planner is not None and guide.planner.state in ('arrived', 'failed'),
        180.0,
    ), f'planner: {guide.planner}'
    assert guide.planner.state == 'arrived', guide.planner.message
    assert guide.near(START, 0.3, math.radians(15.0)), f'robot at {guide.poses[-1]}'
