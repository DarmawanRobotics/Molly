"""interaction_node in-process: gestures through a fake driver, the greeter, its choice."""

from __future__ import annotations

import time

from interaction_rig import Rig
from molly_interaction.interaction_node import InteractionNode
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool, Trigger

from dharma_utils.ros import LATCHED
from dharma_utils.testing import Collector, wait_for
from molly_interfaces.msg import GreeterStatus, Visitors
from molly_interfaces.srv import Gesture, Say


def interaction(tmp_path, name):
    params = [
        Parameter('state_file', value=str(tmp_path / 'greeter.json')),
        Parameter('dwell_sec', value=0.3),
        Parameter('cooldown_sec', value=60.0),
        Parameter('gestures', value=['nod', 'hello']),
    ]
    rig = Rig([InteractionNode(parameter_overrides=params)], name)
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

    rig.helper.create_service(Say, 'interaction/say', on_say)
    rig.helper.create_service(Trigger, '/robot/nod', on_nod)
    return rig, said, gestures


def test_gesture_waits_for_the_driver(ros, tmp_path):
    rig, _, gestures = interaction(tmp_path, 'interaction_helper_gesture')
    try:
        start = time.monotonic()
        done = rig.call(Gesture, 'interaction/gesture', name='nod')
        assert done.success and done.message == 'nod done' and gestures == ['nod']
        assert time.monotonic() - start >= 0.3
        refused = rig.call(Gesture, 'interaction/gesture', name='dance')
        assert not refused.success and 'unknown gesture' in refused.message
        missing = rig.call(Gesture, 'interaction/gesture', name='hello')
        assert not missing.success and 'not available' in missing.message
    finally:
        rig.close()


def test_greeter_greets_a_visitor_who_stays(ros, tmp_path):
    rig, said, _ = interaction(tmp_path, 'interaction_helper_greeter')
    status = Collector(rig.helper, GreeterStatus, 'interaction/greeter', LATCHED)
    pub = rig.helper.create_publisher(Visitors, 'perception/visitors', 10)
    try:
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            pub.publish(Visitors(count=1, front=1, nearest=1.0, nearest_front=1.0))
            time.sleep(0.05)
        assert len(said) == 1 and 'Molly' in said[0]
        assert wait_for(lambda: status.last is not None and status.last.last_greet_age >= 0, 5.0)
        assert rig.call(SetBool, 'interaction/set_greeter', data=False).success
        assert wait_for(
            lambda: not status.last.enabled and status.last.state == GreeterStatus.OFF, 5.0
        )
        assert (tmp_path / 'greeter.json').read_text().strip() == '{\n  "enabled": false\n}'
        assert rig.call(Trigger, 'interaction/greet').success and len(said) == 2
    finally:
        rig.close()


def test_greeter_choice_survives_a_restart(ros, tmp_path):
    (tmp_path / 'greeter.json').write_text('{"enabled": false}')
    node = InteractionNode(
        parameter_overrides=[Parameter('state_file', value=str(tmp_path / 'greeter.json'))]
    )
    try:
        assert node.get_parameter('enabled').value is False
        assert node.greeter.enabled is False
    finally:
        node.destroy_node()
