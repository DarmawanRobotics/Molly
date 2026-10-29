"""guide_node in-process against a fake planner, speech, gestures and control mux."""

from __future__ import annotations

from guide_rig import FakeRobot, spin
from molly_guide.guide_node import GuideNode
from rclpy.parameter import Parameter

from dharma_utils.ros import LATCHED
from dharma_utils.testing import Collector, wait_for
from molly_interfaces.msg import TourCommand, TourState
from molly_interfaces.srv import GetTourState, GoToPoi, TourControl

QUIET = [
    Parameter('wait_for_visitors', value=False),
    Parameter('intro_texts', value=['id:Halo']),
    Parameter('outro_texts', value=['id:Dadah']),
]


def start(spinner, node, **fields):
    request = TourControl.Request(command=TourCommand(**fields))
    client = node.create_client(TourControl, 'tour/control')
    return spinner.call(client, request)


def test_a_tour_walks_gestures_and_narrates(ros):
    guide = GuideNode(parameter_overrides=QUIET)
    robot = FakeRobot('fake_robot_tour')
    states = Collector(robot.node, TourState, 'tour/state', LATCHED)
    spinner = spin(guide, robot.node)
    try:
        assert wait_for(lambda: guide.stops and guide.robot.localized, 5.0)
        answer = start(spinner, robot.node, kind=TourCommand.START, tour_id='main')
        assert answer.success, answer.message
        done = lambda: states.last is not None and states.last.state == TourState.IDLE  # noqa: E731
        assert wait_for(lambda: len(robot.said) >= 4 and done(), 15.0), robot.said
        assert robot.walks == [(1.0, 2.0), (4.0, 0.0)]
        assert robot.gestures == ['nod']
        assert robot.said == ['Halo', 'Ini lobi.', 'This is the hall.', 'Dadah']
        activities = {s.activity for s in states.messages}
        assert {TourState.ACTIVITY_NAVIGATING, TourState.ACTIVITY_NARRATING} <= activities
        assert 'finished' in states.last.message
        get = robot.node.create_client(GetTourState, 'tour/get_state')
        assert spinner.call(get, GetTourState.Request()).state.state == TourState.IDLE
    finally:
        spinner.stop()
        guide.destroy_node()
        robot.node.destroy_node()


def test_start_needs_auto_and_no_estop_and_the_estop_pauses(ros):
    guide = GuideNode(parameter_overrides=QUIET)
    robot = FakeRobot('fake_robot_estop', arrive_after=30.0)
    spinner = spin(guide, robot.node)
    try:
        assert wait_for(lambda: guide.stops and guide.robot.localized, 5.0)
        robot.set_control(auto=False, estop=False)
        assert wait_for(lambda: guide.robot.auto is False, 5.0)
        answer = start(spinner, robot.node, kind=TourCommand.START, poi_ids=['hall'])
        assert not answer.success and 'AUTO' in answer.message
        robot.set_control(auto=True, estop=False)
        assert wait_for(lambda: guide.robot.auto, 5.0)
        assert start(spinner, robot.node, kind=TourCommand.START, poi_ids=['hall']).success
        assert wait_for(lambda: robot.walks, 5.0)
        robot.set_control(auto=True, estop=True)
        assert wait_for(lambda: guide.engine.state.name == 'PAUSED', 5.0)
        assert 'navigation/pause' in robot.navigation
        refused = start(spinner, robot.node, kind=TourCommand.RESUME)
        assert not refused.success and 'e-stop' in refused.message
        robot.set_control(auto=True, estop=False)
        assert wait_for(lambda: not guide.robot.estop, 5.0)
        assert start(spinner, robot.node, kind=TourCommand.RESUME).success
        assert robot.navigation[-1] == 'navigation/resume'
        assert start(spinner, robot.node, kind=TourCommand.STOP).success
        assert 'planner/cancel' in robot.navigation
    finally:
        spinner.stop()
        guide.destroy_node()
        robot.node.destroy_node()


def test_go_to_poi(ros):
    guide = GuideNode(parameter_overrides=QUIET)
    robot = FakeRobot('fake_robot_go_to')
    spinner = spin(guide, robot.node)
    try:
        assert wait_for(lambda: guide.stops and guide.robot.auto, 5.0)
        client = robot.node.create_client(GoToPoi, 'guide/go_to_poi')
        answer = spinner.call(client, GoToPoi.Request(id='hall'))
        assert answer.success and robot.walks == [(4.0, 0.0)]
        assert not spinner.call(client, GoToPoi.Request(id='nope')).success
    finally:
        spinner.stop()
        guide.destroy_node()
        robot.node.destroy_node()
