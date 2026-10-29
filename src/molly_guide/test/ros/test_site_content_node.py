"""site_content_node in-process: follows the active site, saves POIs and tours."""

from __future__ import annotations

import json
import os

from guide_rig import FakeRobot, spin
from molly_guide.site_content_node import SiteContentNode
from rclpy.parameter import Parameter

from dharma_interfaces.msg import SiteStatus
from dharma_utils.ros import LATCHED
from dharma_utils.testing import Collector, wait_for
from molly_interfaces.msg import Poi, PoiArray, Tour, TourArray
from molly_interfaces.srv import DeleteById, SavePoi, SaveTour


def test_pois_and_tours_of_the_active_site(ros, tmp_path):
    os.makedirs(tmp_path / 'museum')
    with open(tmp_path / 'museum' / 'pois.json', 'w', encoding='utf-8') as f:
        json.dump({'pois': [{'id': 'gate', 'name': 'Gate', 'x': 2.0, 'y': 1.0}]}, f)
    node = SiteContentNode(parameter_overrides=[Parameter('sites_dir', value=str(tmp_path))])
    robot = FakeRobot('fake_robot_site', site=False)  # map -> base_link at (0.5, -0.5, 0.5 rad)
    site_pub = robot.node.create_publisher(SiteStatus, 'site/status', LATCHED)
    pois = Collector(robot.node, PoiArray, 'site/pois', LATCHED)
    tours = Collector(robot.node, TourArray, 'site/tours', LATCHED)
    spinner = spin(node, robot.node)
    try:
        site_pub.publish(SiteStatus(site='museum', has_map=True))
        assert wait_for(lambda: pois.last is not None and pois.last.site == 'museum', 5.0)
        assert [p.id for p in pois.last.pois] == ['gate']
        assert wait_for(lambda: node.robot_pose() is not None, 5.0), node.tf.error

        def save(**fields):
            client = robot.node.create_client(SavePoi, 'site/save_poi')
            return spinner.call(client, SavePoi.Request(**fields))

        here = save(poi=Poi(name='Here', narration='Halo'), at_robot_pose=True)
        assert here.success and here.poi.id == 'here'
        assert abs(here.poi.x - 0.5) < 1e-6 and abs(here.poi.yaw - 0.5) < 1e-6
        assert not save(poi=Poi(name='')).success
        tour_client = robot.node.create_client(SaveTour, 'site/save_tour')
        tour = Tour(name='Main', poi_ids=['gate', 'here'])
        assert spinner.call(tour_client, SaveTour.Request(tour=tour)).tour.id == 'main'
        assert wait_for(lambda: tours.last is not None and len(tours.last.tours) == 1, 5.0)
        delete = robot.node.create_client(DeleteById, 'site/delete_poi')
        assert spinner.call(delete, DeleteById.Request(id='here')).success
        assert wait_for(lambda: [p.id for p in pois.last.pois] == ['gate'], 5.0)
        assert list(tours.last.tours[0].poi_ids) == ['gate']
        site_pub.publish(SiteStatus(site=''))
        assert wait_for(lambda: pois.last.site == '' and not pois.last.pois, 5.0)
        assert not save(poi=Poi(name='Nowhere')).success
    finally:
        spinner.stop()
        node.destroy_node()
        robot.node.destroy_node()
