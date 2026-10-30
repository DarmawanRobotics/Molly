"""A fake robot for guide_node: planner, navigation, speech and gesture services, the site."""

from __future__ import annotations

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from dharma_interfaces.msg import ControlStatus, LocalizationStatus, PlannerStatus
from dharma_interfaces.srv import GoTo
from dharma_utils.ros import LATCHED
from dharma_utils.testing import Spinner
from molly_interfaces.msg import Poi, PoiArray, SpeechStatus, Tour, TourArray
from molly_interfaces.srv import Gesture, Say

POIS = [
    Poi(
        id='lobby', name='Lobby', x=1.0, y=2.0, narration='Ini lobi.', gesture='nod', dwell_sec=0.2
    ),
    Poi(id='hall', name='Hall', x=4.0, y=0.0, narration='This is the hall.', language='en'),
]


class FakeRobot:
    """Answers like the real nodes, quickly; the test reads what was asked."""

    def __init__(self, name: str, arrive_after: float = 0.3, site: bool = True):
        self.node = rclpy.create_node(name)
        self.arrive_after = arrive_after
        self.walks, self.said, self.gestures, self.navigation = [], [], [], []
        self.lock = threading.Lock()
        self.later: list = []
        group = ReentrantCallbackGroup()
        node = self.node
        self.planner_pub = node.create_publisher(PlannerStatus, 'planner/status', LATCHED)
        self.speech_pub = node.create_publisher(SpeechStatus, 'interaction/speech_status', LATCHED)
        self.control_pub = node.create_publisher(ControlStatus, 'control/status', LATCHED)
        self.loc_pub = node.create_publisher(LocalizationStatus, 'localization/status', LATCHED)
        self.pois_pub = node.create_publisher(PoiArray, 'site/pois', LATCHED)
        self.tours_pub = node.create_publisher(TourArray, 'site/tours', LATCHED)
        node.create_service(GoTo, 'planner/go_to', self.on_go_to, callback_group=group)
        for service in ('planner/cancel', 'navigation/pause', 'navigation/resume',
                        'interaction/stop_speech'):  # fmt: skip
            node.create_service(Trigger, service, self.trigger(service), callback_group=group)
        node.create_service(Say, 'interaction/say', self.on_say, callback_group=group)
        node.create_service(Gesture, 'interaction/gesture', self.on_gesture, callback_group=group)
        node.create_timer(0.02, self.run_later)
        if site:
            self.publish_site()
        self.set_control(auto=True, estop=False)
        self.loc_pub.publish(LocalizationStatus(state=LocalizationStatus.LOCALIZED))
        self.tf = TransformBroadcaster(node)
        node.create_timer(0.05, self.publish_pose)

    def publish_site(self):
        pois = PoiArray(site='museum', pois=POIS)
        main = Tour(id='main', name='Main', poi_ids=['lobby', 'hall'])
        tours = TourArray(site='museum', tours=[main])
        self.pois_pub.publish(pois)
        self.tours_pub.publish(tours)

    def publish_pose(self):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = 0.5, -0.5
        t.transform.rotation.z, t.transform.rotation.w = math.sin(0.25), math.cos(0.25)
        self.tf.sendTransform(t)

    def set_control(self, auto: bool, estop: bool):
        mode = ControlStatus.MODE_AUTO if auto else ControlStatus.MODE_REMOTE
        self.control_pub.publish(ControlStatus(mode=mode, estop=estop))

    def after(self, seconds: float, action):
        with self.lock:
            self.later.append((time.monotonic() + seconds, action))

    def run_later(self):
        now = time.monotonic()
        with self.lock:
            due = [a for t, a in self.later if t <= now]
            self.later = [(t, a) for t, a in self.later if t > now]
        for action in due:
            action()

    def status(self, state: int, x: float, y: float):
        msg = PlannerStatus(state=state, goal_x=x, goal_y=y)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.planner_pub.publish(msg)

    def on_go_to(self, request, response):
        self.walks.append((request.x, request.y))
        self.status(PlannerStatus.FOLLOWING, request.x, request.y)
        x, y = request.x, request.y
        self.after(self.arrive_after, lambda: self.status(PlannerStatus.ARRIVED, x, y))
        response.success, response.message = True, 'walking'
        return response

    def trigger(self, name: str):
        def handle(_request, response):
            self.navigation.append(name)
            response.success = True
            return response

        return handle

    def on_say(self, request, response):
        self.said.append(request.text)
        uid = len(self.said)
        msg = SpeechStatus(id=uid, state=SpeechStatus.DONE, text=request.text)
        self.after(0.1, lambda: self.speech_pub.publish(msg))
        response.success, response.id = True, uid
        return response

    def on_gesture(self, request, response):
        self.gestures.append(request.name)
        response.success = True
        return response


def spin(*nodes, threads: int = 4) -> Spinner:
    """Spin nodes on a multi-threaded executor in the background."""
    executor = MultiThreadedExecutor(num_threads=threads)
    for node in nodes:
        executor.add_node(node)
    return Spinner(executor)
