"""speech_node in-process with the silent engine."""

from __future__ import annotations

from interaction_rig import Rig
from molly_interaction.speech_node import SpeechNode
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dharma_utils.ros import LATCHED
from dharma_utils.testing import Collector, wait_for
from molly_interfaces.msg import SpeechStatus
from molly_interfaces.srv import Say

SILENT = [Parameter('engine', value='none'), Parameter('none_chars_per_sec', value=50.0)]


def states(collector, uid):
    return [s.state for s in collector.messages if s.id == uid]


def test_say_reports_speaking_then_done(ros):
    rig = Rig([SpeechNode(parameter_overrides=SILENT)], 'speech_helper_done')
    status = Collector(rig.helper, SpeechStatus, 'interaction/speech_status', 50)
    try:
        answer = rig.call(Say, 'interaction/say', text=' Halo\n semua ', language='id')
        assert answer.success and answer.id == 1
        assert wait_for(lambda: SpeechStatus.DONE in states(status, 1), 5.0)
        assert states(status, 1)[:2] == [SpeechStatus.SPEAKING, SpeechStatus.DONE]
        said = next(s for s in status.messages if s.id == 1)
        assert (said.text, said.engine) == ('Halo semua', 'none')
        assert not rig.call(Say, 'interaction/say', text='   ').success
    finally:
        rig.close()


def test_interrupt_and_stop_cancel(ros):
    rig = Rig([SpeechNode(parameter_overrides=SILENT)], 'speech_helper_stop')
    status = Collector(rig.helper, SpeechStatus, 'interaction/speech_status', 50)
    try:
        first = rig.call(Say, 'interaction/say', text='x' * 500).id
        second = rig.call(Say, 'interaction/say', text='y' * 500).id
        assert wait_for(lambda: SpeechStatus.SPEAKING in states(status, first), 5.0)
        third = rig.call(Say, 'interaction/say', text='urgent', interrupt=True).id
        assert wait_for(lambda: SpeechStatus.DONE in states(status, third), 5.0)
        assert SpeechStatus.CANCELLED in states(status, first)
        assert SpeechStatus.CANCELLED in states(status, second)
        rig.call(Say, 'interaction/say', text='z' * 500)
        assert rig.call(Trigger, 'interaction/stop_speech').success
    finally:
        rig.close()


def test_say_text_topic(ros):
    rig = Rig([SpeechNode(parameter_overrides=SILENT)], 'speech_helper_topic')
    status = Collector(rig.helper, SpeechStatus, 'interaction/speech_status', LATCHED)
    pub = rig.helper.create_publisher(String, 'interaction/say_text', 10)
    try:
        assert wait_for(lambda: pub.get_subscription_count() > 0, 5.0)
        pub.publish(String(data='Selamat datang'))
        assert wait_for(
            lambda: (
                status.last is not None
                and status.last.text == 'Selamat datang'
                and status.last.state == SpeechStatus.DONE
            ),
            5.0,
        )
    finally:
        rig.close()
