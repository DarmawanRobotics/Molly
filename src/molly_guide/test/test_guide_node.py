"""In-process test of guide_node against fake planner, speech and gesture services."""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
executors = pytest.importorskip('rclpy.executors')
QoS = pytest.importorskip('rclpy.qos')
std_msgs = pytest.importorskip('std_msgs.msg')
std_srvs = pytest.importorskip('std_srvs.srv')
dharma_msgs = pytest.importorskip('dharma_interfaces.msg')
msgs = pytest.importorskip('molly_interfaces.msg')
dharma_srvs = pytest.importorskip('dharma_interfaces.srv')
srvs = pytest.importorskip('molly_interfaces.srv')
GuideNode = pytest.importorskip('molly_guide.guide_node').GuideNode


def latched():
    """Return the QoS of the latched topics."""
    return QoS.QoSProfile(
        depth=1,
        durability=QoS.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoS.QoSReliabilityPolicy.RELIABLE,
    )


class FakeRobot:
    """Planner, speech and gesture services that answer like the real nodes, quickly."""

    def __init__(self, node):
        """Serve the fakes on a helper node and publish the site and robot state."""
        self.node = node
        self.walks, self.said, self.gestures, self.pauses = [], [], [], []
        self.cancels = 0
        self.planner_pub = node.create_publisher(
            dharma_msgs.PlannerStatus, 'planner/status', latched()
        )
        self.speech_pub = node.create_publisher(
            msgs.SpeechStatus, 'interaction/speech_status', latched()
        )
        node.create_service(dharma_srvs.GoTo, 'planner/go_to', self.on_go_to)
        node.create_service(std_srvs.Trigger, 'planner/cancel', self.on_cancel)
        node.create_service(std_srvs.SetBool, 'navigation/pause', self.on_pause)
        node.create_service(srvs.Say, 'interaction/say', self.on_say)
        node.create_service(std_srvs.Trigger, 'interaction/stop_speech', self.on_cancel)
        node.create_service(srvs.Gesture, 'interaction/gesture', self.on_gesture)
        self.later = []  # (time, callable)
        node.create_timer(0.02, self.run_later)
        self.publish_site()
        self.localization = node.create_publisher(std_msgs.String, 'localization/status',
                                                  latched())
        self.mode = node.create_publisher(std_msgs.String, 'bridge/mode', latched())
        self.estop = node.create_publisher(std_msgs.Bool, 'bridge/estop_engaged', latched())
        self.localization.publish(std_msgs.String(data='tag:base_map'))
        self.mode.publish(std_msgs.String(data='auto'))
        self.estop.publish(std_msgs.Bool(data=False))

    def publish_site(self):
        """Publish two POIs and a tour."""
        pois = dharma_msgs.PoiArray(site='museum')
        pois.pois = [
            dharma_msgs.Poi(id='lobby', name='Lobby', narration='Ini lobi.', gesture='nod',
                            dwell_sec=0.2),
            dharma_msgs.Poi(id='hall', name='Hall', narration='This is the hall.', language='en'),
        ]
        tours = dharma_msgs.TourArray(site='museum')
        tours.tours = [dharma_msgs.Tour(id='main', name='Main', poi_ids=['lobby', 'hall'])]
        self.node.create_publisher(dharma_msgs.PoiArray, 'site/pois', latched()).publish(pois)
        self.node.create_publisher(dharma_msgs.TourArray, 'site/tours', latched()).publish(tours)

    def after(self, seconds, action):
        """Run an action a little later on the helper's timer."""
        self.later.append((time.monotonic() + seconds, action))

    def run_later(self):
        """Run the due actions."""
        now = time.monotonic()
        due = [a for t, a in self.later if t <= now]
        self.later = [(t, a) for t, a in self.later if t > now]
        for action in due:
            action()

    def status(self, state, goal):
        """Publish a planner status for a goal."""
        msg = dharma_msgs.PlannerStatus(state=state, goal_id=goal)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.planner_pub.publish(msg)

    def on_go_to(self, request, response):
        """Walk: following now, arrived 0.3 s later."""
        self.walks.append(request.poi_id)
        self.status('following', request.poi_id)
        self.after(0.3, lambda: self.status('arrived', request.poi_id))
        response.success, response.message = True, 'walking'
        return response

    def on_cancel(self, _request, response):
        """Count cancels (walk or speech)."""
        self.cancels += 1
        response.success = True
        return response

    def on_pause(self, request, response):
        """Record holds and continues."""
        self.pauses.append(request.data)
        response.success = True
        return response

    def on_say(self, request, response):
        """Speak: speaking now, done 0.1 s later."""
        self.said.append((request.text, request.language))
        uid = len(self.said)
        self.speech_pub.publish(msgs.SpeechStatus(id=uid, state='speaking', text=request.text))
        self.after(0.1, lambda: self.speech_pub.publish(
            msgs.SpeechStatus(id=uid, state='done', text=request.text)
        ))
        response.success, response.id = True, uid
        return response

    def on_gesture(self, request, response):
        """Gesture: done right away."""
        self.gestures.append(request.name)
        response.success, response.message = True, 'done'
        return response


@pytest.fixture
def rig():
    """Run the guide with the fake robot on a multi-threaded executor."""
    rclpy.init(args=['--ros-args', '-p', 'wait_for_visitors:=false', '-p',
                     'intro_texts:=["id:Halo"]', '-p', 'outro_texts:=["id:Dadah"]'])
    guide = GuideNode()
    helper = rclpy.create_node('guide_test_helper')
    robot = FakeRobot(helper)
    states = []
    helper.create_subscription(msgs.TourState, '/tour/state', states.append, 100)
    executor = executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(guide)
    executor.add_node(helper)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def control(kind, tour_id='', poi_ids=()):
        client = helper.create_client(srvs.TourControl, '/tour/control')
        assert client.wait_for_service(timeout_sec=5.0)
        request = srvs.TourControl.Request()
        request.command = msgs.TourCommand(kind=kind, tour_id=tour_id, poi_ids=list(poi_ids))
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        assert done.wait(5.0)
        return future.result()

    def wait_for(condition, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.02)
        return False

    time.sleep(0.3)  # latched inputs reach the guide
    yield robot, states, control, wait_for
    executor.shutdown()
    thread.join(timeout=5.0)
    guide.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def test_tour_runs_end_to_end(rig):
    """A saved tour of two POIs: walk, gesture, narrate, dwell, walk, narrate, back to IDLE."""
    robot, states, control, wait_for = rig
    answer = control(msgs.TourCommand.KIND_START, tour_id='main')
    assert answer.success, answer.message
    assert wait_for(lambda: len(robot.said) >= 4 and states[-1].state == 0)
    assert robot.walks == ['lobby', 'hall']
    assert robot.gestures == ['nod']
    assert robot.said == [('Halo', 'id'), ('Ini lobi.', 'id'), ('This is the hall.', 'en'),
                          ('Dadah', 'id')]
    activities = []
    for s in states:
        if s.activity and (not activities or activities[-1] != (s.current_poi_id, s.activity)):
            activities.append((s.current_poi_id, s.activity))
    assert activities == [
        ('lobby', 'narrating'), ('lobby', 'navigating'), ('lobby', 'performing_motion'),
        ('lobby', 'narrating'), ('lobby', 'dwelling'), ('hall', 'navigating'),
        ('hall', 'narrating'),
    ]
    running = [s for s in states if s.state == msgs.TourState.STATE_RUNNING]
    assert running and running[0].tour_name == 'Main' and running[0].total_pois == 2


def test_start_is_refused_without_localization_or_auto(rig):
    """START needs a localized robot in AUTO and known POIs; the reason comes back."""
    robot, states, control, wait_for = rig
    unknown = control(msgs.TourCommand.KIND_START, poi_ids=['lobby', 'nowhere'])
    assert not unknown.success and 'nowhere' in unknown.message
    robot.mode.publish(std_msgs.String(data='remote'))
    time.sleep(0.2)
    remote = control(msgs.TourCommand.KIND_START, poi_ids=['lobby'])
    assert not remote.success and 'AUTO' in remote.message
    robot.mode.publish(std_msgs.String(data='auto'))
    robot.localization.publish(std_msgs.String(data='none'))
    time.sleep(0.2)
    lost = control(msgs.TourCommand.KIND_START, poi_ids=['lobby'])
    assert not lost.success and 'localized' in lost.message
    pause = control(msgs.TourCommand.KIND_PAUSE)
    assert not pause.success and 'IDLE' in pause.message
    assert robot.walks == []


def test_stop_and_get_state(rig):
    """STOP ends a running tour; /tour/get_state answers the current state."""
    robot, states, control, wait_for = rig
    assert control(msgs.TourCommand.KIND_START, poi_ids=['hall', 'lobby']).success
    assert wait_for(lambda: robot.walks == ['hall'])
    assert control(msgs.TourCommand.KIND_STOP).success
    assert wait_for(lambda: states[-1].state == msgs.TourState.STATE_IDLE)
    assert robot.cancels >= 1
    client = robot.node.create_client(srvs.GetTourState, '/tour/get_state')
    assert client.wait_for_service(timeout_sec=5.0)
    done = threading.Event()
    future = client.call_async(srvs.GetTourState.Request())
    future.add_done_callback(lambda _: done.set())
    assert done.wait(5.0) and future.result().state.state == msgs.TourState.STATE_IDLE
