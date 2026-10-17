"""In-process test of site_manager_node against fake localization and grid mapper services."""

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))

rclpy = pytest.importorskip('rclpy')
MultiThreadedExecutor = pytest.importorskip('rclpy.executors').MultiThreadedExecutor
qos = pytest.importorskip('rclpy.qos')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
nav_msgs = pytest.importorskip('nav_msgs.msg')
std_msgs = pytest.importorskip('std_msgs.msg')
std_srvs = pytest.importorskip('std_srvs.srv')
tf2_ros = pytest.importorskip('tf2_ros')
msgs = pytest.importorskip('molly_interfaces.msg')
srvs = pytest.importorskip('molly_interfaces.srv')
SiteManagerNode = pytest.importorskip('molly_mapping.site_manager_node').SiteManagerNode
map_io = pytest.importorskip('molly_mapping.map_io')
Spinner = pytest.importorskip('ros_harness').Spinner


class FakeRobot:
    """Fake localization_node and grid_mapper_node services that record their calls."""

    def __init__(self, node):
        """Offer the services and a latched localization/status."""
        self.node = node
        self.calls = []
        self.localize_ok = False
        latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status = node.create_publisher(std_msgs.String, 'localization/status', latched)
        self.set_status('none')
        for srv, name, handler in (
            (srvs.LoadFile, 'localization/load_tags', self.load_tags),
            (std_srvs.Trigger, 'localization/assume_origin', self.assume_origin),
            (std_srvs.Trigger, 'localization/reset', self.reset),
            (std_srvs.Trigger, 'localization/start', self.localize),
            (std_srvs.SetBool, 'grid_mapper/start', self.mapper_start),
            (std_srvs.Trigger, 'grid_mapper/stop', self.mapper_stop),
            (srvs.LoadFile, 'grid_mapper/save', self.mapper_save),
            (srvs.LoadFile, 'grid_mapper/load', self.mapper_load),
        ):
            node.create_service(srv, name, handler)
        tf2_ros.StaticTransformBroadcaster(node).sendTransform(self.robot_tf(2.0, 1.0))

    @staticmethod
    def robot_tf(x, y):
        """Return a static map->base_link at (x, y) facing +y."""
        t = geometry_msgs.TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = np.sin(np.pi / 4), np.cos(np.pi / 4)
        return t

    def set_status(self, value):
        """Publish localization/status."""
        self.status.publish(std_msgs.String(data=value))

    def names(self):
        """Return the names of the calls so far."""
        return [c[0] for c in self.calls]

    def load_tags(self, request, response):
        """Create the file like localization_node does."""
        self.calls.append(('load_tags', request.path, request.create))
        if request.create and not os.path.exists(request.path):
            os.makedirs(os.path.dirname(request.path), exist_ok=True)
            with open(request.path, 'w') as f:
                f.write('{}')
        response.success, response.message = True, 'ok'
        return response

    def assume_origin(self, request, response):
        """Localize at the origin."""
        self.calls.append(('assume_origin',))
        self.set_status('origin')
        response.success = True
        return response

    def reset(self, request, response):
        """Drop localization."""
        self.calls.append(('reset',))
        self.set_status('none')
        response.success = True
        return response

    def localize(self, request, response):
        """Localize from a tag when allowed."""
        self.calls.append(('localize',))
        response.success = self.localize_ok
        response.message = 'tag base_map' if self.localize_ok else 'no fresh detection'
        if self.localize_ok:
            self.set_status('tag:base_map')
        return response

    def mapper_start(self, request, response):
        """Record the reset flag."""
        self.calls.append(('mapper_start', request.data))
        response.success = True
        return response

    def mapper_stop(self, request, response):
        """Stop."""
        self.calls.append(('mapper_stop',))
        response.success = True
        return response

    def mapper_save(self, request, response):
        """Write a small map into the site directory."""
        self.calls.append(('mapper_save', request.path))
        data = np.zeros((20, 30), dtype=np.int8)
        data[0, :] = 100
        map_io.save_map(request.path, map_io.GridMap(data, 0.1, -1.0, -1.0))
        response.success, response.message = True, 'saved'
        return response

    def mapper_load(self, request, response):
        """Record the continued map."""
        self.calls.append(('mapper_load', request.path))
        response.success = True
        return response


@pytest.fixture
def world(tmp_path):
    """Run the site manager next to the fakes; yield (spinner, helper, fake, recorded, root)."""
    root = tmp_path / 'map'
    rclpy.init(args=['--ros-args', '-p', f'sites_dir:={root}',
                     '-p', f'state_file:={tmp_path / "state" / "site"}',
                     '-p', 'service_timeout_sec:=3.0', '-p', 'refresh_sec:=0.5'])
    helper = rclpy.create_node('site_test_helper')
    fake = FakeRobot(helper)
    manager = SiteManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(helper)
    executor.add_node(manager)
    recorded = {'map': [], 'status': [], 'pois': [], 'tours': []}
    latched = qos.QoSProfile(depth=1, durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
    for msg_type, topic, key in (
        (nav_msgs.OccupancyGrid, '/map', 'map'),
        (msgs.SiteStatus, 'site/status', 'status'),
        (msgs.PoiArray, 'site/pois', 'pois'),
        (msgs.TourArray, 'site/tours', 'tours'),
    ):
        helper.create_subscription(msg_type, topic, recorded[key].append, latched)
    spinner = Spinner(executor)
    helper.manager = manager
    yield spinner, helper, fake, recorded, root
    spinner.stop()
    manager.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def call(spinner, helper, srv_type, name, **fields):
    """Call a site manager service with request fields."""
    request = srv_type.Request()
    for key, value in fields.items():
        setattr(request, key, value)
    return spinner.call(helper.create_client(srv_type, name), request, 15.0)


def test_map_a_new_site_add_pois_tours_and_save(world):
    """A new site takes the current pose as origin, POIs land at the robot, the map is saved."""
    spinner, helper, fake, recorded, root = world
    started = call(spinner, helper, srvs.StartMapping, 'mapping/start', site='lobby', reset=True)
    assert started.success, started.message
    assert ('load_tags', str(root / 'lobby' / 'tags.json'), True) in fake.calls
    assert 'assume_origin' in fake.names()
    assert ('mapper_start', True) in fake.calls
    assert spinner.wait(lambda: recorded['status'][-1].mapping, 3.0)
    assert recorded['status'][-1].site == 'lobby'

    manager = helper.manager
    assert spinner.wait(lambda: manager.localized() and manager.robot_pose() is not None, 3.0)
    poi = msgs.Poi(name='Main Entrance', narration='Selamat datang', dwell_sec=3.0)
    saved = call(spinner, helper, srvs.SavePoi, 'site/save_poi', poi=poi, at_robot_pose=True)
    assert saved.success, saved.message
    assert saved.poi.id == 'main-entrance'
    assert (saved.poi.x, saved.poi.y) == pytest.approx((2.0, 1.0))
    assert saved.poi.yaw == pytest.approx(np.pi / 2)
    second = call(spinner, helper, srvs.SavePoi, 'site/save_poi',
                  poi=msgs.Poi(name='Fountain', x=5.0, y=-2.0))
    assert second.success
    bad = call(spinner, helper, srvs.SavePoi, 'site/save_poi',
               poi=msgs.Poi(name='Nowhere', x=float('nan')))
    assert not bad.success
    tour = call(spinner, helper, srvs.SaveTour, 'site/save_tour',
                tour=msgs.Tour(name='Short', poi_ids=['main-entrance', 'fountain']))
    assert tour.success and tour.tour.id == 'short'
    assert not call(spinner, helper, srvs.SaveTour, 'site/save_tour',
                    tour=msgs.Tour(name='Bad', poi_ids=['nope'])).success
    assert spinner.wait(lambda: len(recorded['pois'][-1].pois) == 2, 3.0)
    assert spinner.wait(lambda: len(recorded['tours'][-1].tours) == 1, 3.0)

    assert call(spinner, helper, std_srvs.Trigger, 'mapping/save').success
    assert ('mapper_save', str(root / 'lobby')) in fake.calls
    assert call(spinner, helper, std_srvs.Trigger, 'mapping/stop').success
    assert spinner.wait(lambda: recorded['map'] and recorded['map'][-1].info.width == 30, 3.0)
    assert spinner.wait(lambda: not recorded['status'][-1].mapping, 3.0)
    status = recorded['status'][-1]
    assert status.has_map and status.pois == 2 and status.tours == 1
    with open(root / 'lobby' / 'pois.json') as f:
        assert [p['id'] for p in json.load(f)['pois']] == ['main-entrance', 'fountain']

    deleted = call(spinner, helper, srvs.DeleteById, 'site/delete_poi', id='fountain')
    assert deleted.success
    assert spinner.wait(lambda: recorded['tours'][-1].tours[0].poi_ids == ['main-entrance'], 3.0)
    listed = call(spinner, helper, srvs.ListSites, 'site/list')
    assert list(listed.sites) == ['lobby'] and listed.active == 'lobby'


def test_existing_site_needs_its_tags_or_a_reset(world):
    """Continuing another site's map needs a tag of it; reset starts it over in a backup."""
    spinner, helper, fake, recorded, root = world
    assert call(spinner, helper, srvs.StartMapping, 'mapping/start', site='lobby',
                reset=True).success
    assert call(spinner, helper, srvs.SavePoi, 'site/save_poi', poi=msgs.Poi(name='A', x=1.0),
                at_robot_pose=False).success
    assert call(spinner, helper, std_srvs.Trigger, 'mapping/save').success
    assert call(spinner, helper, std_srvs.Trigger, 'mapping/stop').success
    assert call(spinner, helper, srvs.StartMapping, 'mapping/start', site='hall',
                reset=False).success
    assert call(spinner, helper, std_srvs.Trigger, 'mapping/stop').success

    fake.calls.clear()
    refused = call(spinner, helper, srvs.StartMapping, 'mapping/start', site='lobby', reset=False)
    assert not refused.success and 'AprilTags' in refused.message
    assert 'reset' in fake.names() and 'localize' in fake.names()
    assert 'mapper_start' not in fake.names()

    fake.localize_ok = True
    fake.calls.clear()
    resumed = call(spinner, helper, srvs.StartMapping, 'mapping/start', site='lobby', reset=False)
    assert resumed.success, resumed.message
    assert ('mapper_load', str(root / 'lobby')) in fake.calls
    assert ('mapper_start', False) in fake.calls
    assert 'assume_origin' not in fake.names()
    assert call(spinner, helper, std_srvs.Trigger, 'mapping/stop').success

    assert call(spinner, helper, srvs.LoadSite, 'site/load', site='hall').success
    fake.localize_ok = False
    fake.calls.clear()
    over = call(spinner, helper, srvs.StartMapping, 'mapping/start', site='lobby', reset=True)
    assert over.success, over.message
    backups = [n for n in os.listdir(root / 'lobby') if n.startswith('backup-')]
    assert len(backups) == 1
    assert sorted(os.listdir(root / 'lobby' / backups[0])) == [
        'map.pgm', 'map.yaml', 'pois.json', 'routes', 'tags.json'
    ]
    assert ('mapper_start', True) in fake.calls
    assert spinner.wait(lambda: recorded['status'][-1].pois == 0, 3.0)


def test_load_refuses_unknown_sites_and_mapping_twice(world):
    """Unknown sites, bad names and a second mapping/start are refused with a reason."""
    spinner, helper, fake, recorded, root = world
    assert not call(spinner, helper, srvs.LoadSite, 'site/load', site='nowhere').success
    assert not call(spinner, helper, srvs.StartMapping, 'mapping/start', site='Bad Name').success
    assert not call(spinner, helper, srvs.SavePoi, 'site/save_poi',
                    poi=msgs.Poi(name='x')).success  # no site yet
    assert call(spinner, helper, srvs.StartMapping, 'mapping/start', site='lobby').success
    again = call(spinner, helper, srvs.StartMapping, 'mapping/start', site='hall')
    assert not again.success and 'mapping/stop' in again.message
    assert not call(spinner, helper, srvs.LoadSite, 'site/load', site='lobby').success
