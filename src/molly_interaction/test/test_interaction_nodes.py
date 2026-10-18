"""In-process tests of speech_node, interaction_node and conversation_node."""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
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
srvs = pytest.importorskip('molly_interfaces.srv')
SpeechNode = pytest.importorskip('molly_interaction.speech_node').SpeechNode
InteractionNode = pytest.importorskip('molly_interaction.interaction_node').InteractionNode
ConversationNode = pytest.importorskip('molly_interaction.conversation_node').ConversationNode


class Rig:
    """Spins the nodes under test plus a helper node on a multi-threaded executor."""

    def __init__(self, nodes):
        """Start spinning in a background thread."""
        self.helper = rclpy.create_node('interaction_test_helper')
        self.nodes = nodes
        self.executor = executors.MultiThreadedExecutor(num_threads=4)
        for node in [*nodes, self.helper]:
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def call(self, srv_type, name, request, timeout=10.0):
        """Call a service from the helper node and return the response."""
        client = self.helper.create_client(srv_type, name)
        assert client.wait_for_service(timeout_sec=5.0), f'{name} not available'
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        assert done.wait(timeout), f'{name} did not answer'
        return future.result()

    def wait_for(self, condition, timeout=10.0):
        """Wait until condition() is true."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.02)
        return False

    def close(self):
        """Stop spinning and destroy every node."""
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        for node in [*self.nodes, self.helper]:
            node.destroy_node()


def latched():
    """Return the QoS of the latched topics."""
    return QoS.QoSProfile(
        depth=1,
        durability=QoS.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoS.QoSReliabilityPolicy.RELIABLE,
    )


@pytest.fixture
def speech():
    """Provide a speech node with the silent engine at 50 characters per second."""
    rclpy.init(args=['--ros-args', '-p', 'engine:=none', '-p', 'none_chars_per_sec:=50.0'])
    rig = Rig([SpeechNode()])
    statuses = []
    rig.helper.create_subscription(
        msgs.SpeechStatus, 'interaction/speech_status', statuses.append, 50
    )
    yield rig, statuses
    rig.close()
    rclpy.shutdown()


def test_say_reports_speaking_then_done(speech):
    """Each utterance goes speaking -> done under its id, in order."""
    rig, statuses = speech
    first = rig.call(srvs.Say, 'interaction/say', srvs.Say.Request(text='Halo semua'))
    second = rig.call(
        srvs.Say, 'interaction/say', srvs.Say.Request(text='Ayo ikut', language='en')
    )
    assert first.success and second.success and second.id == first.id + 1
    assert rig.wait_for(lambda: any(s.id == second.id and s.state == 'done' for s in statuses))
    states = [(s.id, s.state) for s in statuses if s.id]
    assert states == [(first.id, 'speaking'), (first.id, 'done'),
                      (second.id, 'speaking'), (second.id, 'done')]
    assert [s.language for s in statuses if s.id == second.id] == ['en', 'en']
    assert all(s.engine == 'none' for s in statuses if s.id)


def test_interrupt_and_stop_cancel(speech):
    """Cancel what is said and what waits with interrupt, the rest with stop_speech."""
    rig, statuses = speech
    long = rig.call(srvs.Say, 'interaction/say', srvs.Say.Request(text='x' * 500))
    queued = rig.call(srvs.Say, 'interaction/say', srvs.Say.Request(text='y' * 500))
    assert rig.wait_for(lambda: any(s.id == long.id and s.state == 'speaking' for s in statuses))
    urgent = rig.call(srvs.Say, 'interaction/say',
                      srvs.Say.Request(text='Awas!', interrupt=True))
    assert rig.wait_for(lambda: any(s.id == urgent.id and s.state == 'done' for s in statuses))
    final = {s.id: s.state for s in statuses}
    assert final[long.id] == 'cancelled' and final[queued.id] == 'cancelled'
    again = rig.call(srvs.Say, 'interaction/say', srvs.Say.Request(text='z' * 500))
    assert rig.wait_for(lambda: any(s.id == again.id and s.state == 'speaking' for s in statuses))
    stop = rig.call(std_srvs.Trigger, 'interaction/stop_speech', std_srvs.Trigger.Request())
    assert stop.success
    assert rig.wait_for(lambda: any(s.id == again.id and s.state == 'cancelled'
                                    for s in statuses))
    empty = rig.call(srvs.Say, 'interaction/say', srvs.Say.Request(text=' \n '))
    assert not empty.success and empty.id == 0


def test_say_text_topic(speech):
    """Speak a text from interaction/say_text, without an answer."""
    rig, statuses = speech
    pub = rig.helper.create_publisher(std_msgs.String, 'interaction/say_text', 10)
    assert rig.wait_for(lambda: pub.get_subscription_count() > 0)
    pub.publish(std_msgs.String(data='Selamat datang'))
    assert rig.wait_for(lambda: any(s.text == 'Selamat datang' and s.state == 'done'
                                    for s in statuses))


@pytest.fixture
def interaction(tmp_path):
    """Provide an interaction node with a fake driver gesture and speech service."""
    rclpy.init(args=['--ros-args', '-p', f'state_file:={tmp_path / "greeter.json"}', '-p',
                     'dwell_sec:=0.3', '-p', 'cooldown_sec:=60.0', '-p',
                     'gestures:=[nod, hello]'])
    rig = Rig([InteractionNode()])
    said, gestures = [], []

    def on_say(request, response):
        said.append(request.text)
        response.success, response.id = True, len(said)
        return response

    def on_nod(_request, response):
        time.sleep(0.3)  # the driver answers when the gesture is done
        gestures.append('nod')
        response.success, response.message = True, 'nod done'
        return response

    rig.helper.create_service(srvs.Say, 'interaction/say', on_say)
    rig.helper.create_service(std_srvs.Trigger, '/robot/nod', on_nod)
    greeter = []
    rig.helper.create_subscription(
        std_msgs.String, 'interaction/greeter', lambda m: greeter.append(json.loads(m.data)),
        latched(),
    )
    yield rig, said, gestures, greeter
    rig.close()
    rclpy.shutdown()


def visitors(front: int, nearest_front: float):
    """Return a Visitors message with people in front."""
    msg = dharma_msgs.Visitors()
    msg.count = msg.front = front
    msg.nearest_front = nearest_front
    msg.nearest = nearest_front
    msg.nearest_behind = -1.0
    return msg


def test_gesture_waits_for_the_driver(interaction):
    """A listed gesture calls /robot/<name> and answers when it is done; others are refused."""
    rig, _, gestures, _ = interaction
    start = time.monotonic()
    done = rig.call(srvs.Gesture, 'interaction/gesture', srvs.Gesture.Request(name='nod'))
    assert done.success and done.message == 'nod done' and gestures == ['nod']
    assert time.monotonic() - start >= 0.3
    refused = rig.call(srvs.Gesture, 'interaction/gesture', srvs.Gesture.Request(name='dance'))
    assert not refused.success and 'unknown gesture' in refused.message
    missing = rig.call(srvs.Gesture, 'interaction/gesture', srvs.Gesture.Request(name='hello'))
    assert not missing.success and 'not available' in missing.message


def test_greeter_greets_a_visitor_who_stays(interaction):
    """Somebody 1 m in front for dwell_sec is greeted once; switching off is kept."""
    rig, said, _, greeter = interaction
    pub = rig.helper.create_publisher(dharma_msgs.Visitors, 'perception/visitors', 10)
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        pub.publish(visitors(1, 1.0))
        time.sleep(0.05)
    assert len(said) == 1 and 'Molly' in said[0]
    assert rig.wait_for(lambda: greeter and greeter[-1]['last_greet_age'] is not None)
    off = rig.call(
        std_srvs.SetBool, 'interaction/set_greeter', std_srvs.SetBool.Request(data=False)
    )
    assert off.success
    assert rig.wait_for(lambda: greeter[-1]['enabled'] is False and greeter[-1]['state'] == 'off')
    now = rig.call(std_srvs.Trigger, 'interaction/greet', std_srvs.Trigger.Request())
    assert now.success and len(said) == 2


def test_greeter_choice_survives_a_restart(tmp_path):
    """The live on / off is restored from the state file."""
    (tmp_path / 'greeter.json').write_text('{"enabled": false}')
    rclpy.init(args=['--ros-args', '-p', f'state_file:={tmp_path / "greeter.json"}'])
    node = InteractionNode()
    try:
        assert node.get_parameter('enabled').value is False
        assert node._greeter.enabled is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def conversation():
    """Provide a conversation node against a fake Ollama and speech service."""

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):  # noqa: N802
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            system = body['messages'][0]['content']
            content = 'Kafe ada di lantai dua.' if 'Kafe' in system else 'Saya tidak tahu.'
            data = json.dumps({'message': {'role': 'assistant', 'content': content}}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *args):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rclpy.init(args=['--ros-args', '-p', f'url:=http://127.0.0.1:{server.server_address[1]}'])
    rig = Rig([ConversationNode()])
    said, log = [], []

    def on_say(request, response):
        said.append(request.text)
        response.success, response.id = True, 1
        return response

    rig.helper.create_service(srvs.Say, 'interaction/say', on_say)
    rig.helper.create_subscription(
        std_msgs.String, 'interaction/conversation', lambda m: log.append(json.loads(m.data)), 10
    )
    yield rig, said, log
    rig.close()
    rclpy.shutdown()
    server.shutdown()
    server.server_close()


def test_ask_uses_the_site_and_speaks(conversation):
    """The answer uses the POIs as knowledge, is spoken and logged."""
    rig, said, log = conversation
    pois = dharma_msgs.PoiArray(site='mall')
    pois.pois = [dharma_msgs.Poi(id='cafe', name='Kafe', description='Kopi di lantai dua')]
    pub = rig.helper.create_publisher(dharma_msgs.PoiArray, 'site/pois', latched())
    pub.publish(pois)
    assert rig.wait_for(lambda: rig.nodes[0]._pois)
    answer = rig.call(srvs.Ask, 'interaction/ask',
                      srvs.Ask.Request(question='Di mana kafe?', speak=True))
    assert answer.success and answer.answer == 'Kafe ada di lantai dua.'
    assert rig.wait_for(lambda: said == ['Kafe ada di lantai dua.'] and log)
    assert log[-1]['question'] == 'Di mana kafe?' and log[-1]['ok'] is True
    empty = rig.call(srvs.Ask, 'interaction/ask', srvs.Ask.Request(question='  '))
    assert not empty.success and empty.message == 'empty question'
