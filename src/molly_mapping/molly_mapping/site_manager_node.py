#!/usr/bin/env python3
"""
Sites: the active site's map, AprilTags, POIs and tours, their editing services, and mapping.

Publishes the active site latched (/map, site/pois, site/tours, site/status) and serves
site/list, site/load, site/save_poi, site/delete_poi, site/save_tour, site/delete_tour and
mapping/start, mapping/stop, mapping/save. While mapping, /map relays the grid mapper's live
grid. A site's map frame is anchored by its AprilTags: mapping a new site takes the current map
frame (or the current pose when not localized), continuing an existing one needs the robot
localized on one of its tags first.
"""

import array
import math
import os
import threading

from molly_interfaces.msg import Poi, PoiArray, SiteStatus, Tour, TourArray
from molly_interfaces.srv import (
    DeleteById,
    ListSites,
    LoadFile,
    LoadSite,
    SavePoi,
    SaveTour,
    StartMapping,
)
from molly_mapping.cloud import transform_matrix, yaw_of
from molly_mapping.map_io import GridMap, load_map, MapError
from molly_mapping.site_store import check_name, SiteError, SiteStore, TAGS_FILE, valid_name
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULTS = {
    'sites_dir': '~/dev/Molly/map',
    'site': '',
    'state_file': '~/.molly/site',
    'map_frame': 'map',
    'base_frame': 'base_link',
    'service_timeout_sec': 5.0,
    'refresh_sec': 2.0,
    'start_mapping': False,
}


def poi_to_msg(poi: dict) -> Poi:
    """Return a Poi message from a stored POI."""
    return Poi(
        id=poi['id'],
        name=poi['name'],
        description=poi['description'],
        x=float(poi['x']),
        y=float(poi['y']),
        yaw=float(poi['yaw']),
        narration=poi['narration'],
        language=poi['language'],
        gesture=poi['gesture'],
        dwell_sec=float(poi['dwell_sec']),
    )


def poi_from_msg(msg: Poi) -> dict:
    """Return a POI dict from a Poi message (validated later by the store)."""
    return {
        'id': msg.id.strip(),
        'name': msg.name,
        'description': msg.description,
        'x': msg.x,
        'y': msg.y,
        'yaw': msg.yaw,
        'narration': msg.narration,
        'language': msg.language.strip(),
        'gesture': msg.gesture.strip(),
        'dwell_sec': msg.dwell_sec,
    }


def tour_to_msg(tour: dict) -> Tour:
    """Return a Tour message from a stored tour."""
    return Tour(
        id=tour['id'],
        name=tour['name'],
        description=tour['description'],
        poi_ids=list(tour['poi_ids']),
        return_poi_id=tour['return_poi_id'],
    )


def tour_from_msg(msg: Tour) -> dict:
    """Return a tour dict from a Tour message."""
    return {
        'id': msg.id.strip(),
        'name': msg.name,
        'description': msg.description,
        'poi_ids': list(msg.poi_ids),
        'return_poi_id': msg.return_poi_id.strip(),
    }


class SiteManagerNode(Node):
    """Own the active site and every file of it; orchestrate mapping sessions."""

    def __init__(self):
        """Declare parameters, interfaces and load the remembered site once spinning."""
        super().__init__('site_manager_node')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._store = SiteStore(str(p('sites_dir')))
        self._state_file = os.path.expanduser(str(p('state_file')))
        self._map_frame, self._base_frame = str(p('map_frame')), str(p('base_frame'))
        self._timeout = float(p('service_timeout_sec'))
        self._initial_site = str(p('site')).strip()
        self._start_mapping = bool(p('start_mapping'))
        self._auto_mapping = None

        self._site = ''
        self._map = None              # GridMap of the active site, None when it has none
        self._pois, self._tours = [], []
        self._mapping = False
        self._loc_status = 'none'
        self._tags_pending = ''       # tags file still to hand to localization
        self._message = 'no site loaded'
        self._last_status = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._service_group = MutuallyExclusiveCallbackGroup()  # one site operation at a time
        self._client_group = ReentrantCallbackGroup()
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self._status_pub = self.create_publisher(SiteStatus, 'site/status', latched)
        self._pois_pub = self.create_publisher(PoiArray, 'site/pois', latched)
        self._tours_pub = self.create_publisher(TourArray, 'site/tours', latched)
        self.create_subscription(OccupancyGrid, 'mapping/grid', self.on_grid, latched)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)

        def client(srv_type, name):
            return self.create_client(srv_type, name, callback_group=self._client_group)

        self._load_tags = client(LoadFile, 'localization/load_tags')
        self._assume_origin = client(Trigger, 'localization/assume_origin')
        self._reset_localization = client(Trigger, 'localization/reset')
        self._localize = client(Trigger, 'localization/start')
        self._mapper_start = client(SetBool, 'grid_mapper/start')
        self._mapper_stop = client(Trigger, 'grid_mapper/stop')
        self._mapper_save = client(LoadFile, 'grid_mapper/save')
        self._mapper_load = client(LoadFile, 'grid_mapper/load')

        for srv_type, name, callback in (
            (ListSites, 'site/list', self.list_callback),
            (LoadSite, 'site/load', self.load_callback),
            (SavePoi, 'site/save_poi', self.save_poi_callback),
            (DeleteById, 'site/delete_poi', self.delete_poi_callback),
            (SaveTour, 'site/save_tour', self.save_tour_callback),
            (DeleteById, 'site/delete_tour', self.delete_tour_callback),
            (StartMapping, 'mapping/start', self.mapping_start_callback),
            (Trigger, 'mapping/stop', self.mapping_stop_callback),
            (Trigger, 'mapping/save', self.mapping_save_callback),
        ):
            self.create_service(srv_type, name, callback, callback_group=self._service_group)

        self._startup = self.create_timer(0.1, self.startup, callback_group=self._service_group)
        self.create_timer(
            max(float(p('refresh_sec')), 0.5), self.refresh, callback_group=self._service_group
        )
        self.publish_status()
        self.get_logger().info(f'sites in {self._store.root}: {self._store.list_sites()}')

    # ------------------------------------------------------------------ helpers

    def call(self, client, request, timeout=None):
        """Call a service and wait (other executor threads take the answer); (result, error)."""
        timeout = self._timeout if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None, f'{client.srv_name} not available'
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            future.cancel()
            return None, f'{client.srv_name} timed out'
        try:
            return future.result(), ''
        except Exception as error:  # noqa: B902  a failed call is an answer too
            return None, f'{client.srv_name} failed: {error}'

    def localized(self) -> bool:
        """Return True when localization reports a source."""
        return self._loc_status not in ('', 'none')

    def robot_pose(self):
        """Return (x, y, yaw) of base_link in the map, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        m = transform_matrix(tf)
        return float(m[0, 3]), float(m[1, 3]), yaw_of(m)

    def tags_path(self, site: str) -> str:
        """Return the tags file of a site."""
        return self._store.path(site, TAGS_FILE)

    def remember(self, site: str):
        """Store the active site for the next start."""
        try:
            os.makedirs(os.path.dirname(self._state_file) or '.', exist_ok=True)
            tmp = f'{self._state_file}.tmp'
            with open(tmp, 'w') as f:
                f.write(site + '\n')
            os.replace(tmp, self._state_file)
        except OSError as error:
            self.get_logger().warn(f'cannot remember the site in {self._state_file}: {error}')

    def remembered(self) -> str:
        """Return the site of the last run, or ''."""
        try:
            with open(self._state_file) as f:
                site = f.read().strip()
        except OSError:
            return ''
        return site if valid_name(site) else ''

    def hand_tags(self, create: bool):
        """Give localization the active site's tags file; (ok, message). Retried when absent."""
        path = self.tags_path(self._site)
        result, error = self.call(self._load_tags, LoadFile.Request(path=path, create=create))
        if result is None or not result.success:
            self._tags_pending = '' if result is not None else path
            return False, error or result.message
        self._tags_pending = ''
        return True, result.message

    def activate(self, site: str, reset_localization: bool):
        """Make site active: map, POIs, tours, tags; return notes for the caller."""
        notes = []
        try:
            self._map = load_map(self._store.site_dir(site)) if self._store.has_map(site) else None
        except MapError as error:
            self._map = None
            notes.append(f'map not loaded: {error}')
        self._pois, problems = self._store.load_pois(site)
        tours, tour_problems = self._store.load_tours(site, [q['id'] for q in self._pois])
        self._tours = tours
        for problem in problems + tour_problems:
            self.get_logger().warn(f'{site}: {problem}')
        changed = site != self._site
        self._site = site
        self.remember(site)
        if reset_localization and changed:
            result, error = self.call(self._reset_localization, Trigger.Request())
            if result is None:
                notes.append(f'localization not reset: {error}')
            else:
                self._loc_status = 'none'  # the latched status follows a moment later
        ok, message = self.hand_tags(create=False)
        if not ok:
            notes.append(f'tags not loaded yet: {message}')
        self.publish_site()
        return notes

    def grid_msg(self, grid: GridMap) -> OccupancyGrid:
        """Return a GridMap as an OccupancyGrid in the map frame."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info.map_load_time = msg.header.stamp
        if grid is None:
            return msg  # empty: this site has no map yet
        msg.info.resolution = float(grid.resolution)
        msg.info.width, msg.info.height = grid.width, grid.height
        msg.info.origin.position.x = float(grid.origin_x)
        msg.info.origin.position.y = float(grid.origin_y)
        msg.info.origin.orientation.z = math.sin(grid.origin_yaw / 2.0)
        msg.info.origin.orientation.w = math.cos(grid.origin_yaw / 2.0)
        msg.data = array.array('b', grid.data.astype('int8').tobytes())
        return msg

    # ------------------------------------------------------------------ publishing

    def publish_site(self):
        """Publish the map (unless mapping), POIs, tours and the status."""
        if not self._mapping:
            self._map_pub.publish(self.grid_msg(self._map))
        self.publish_pois()
        self.publish_tours()
        self.publish_status(force=True)

    def publish_pois(self):
        """Publish site/pois."""
        msg = PoiArray(site=self._site, pois=[poi_to_msg(q) for q in self._pois])
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        self._pois_pub.publish(msg)

    def publish_tours(self):
        """Publish site/tours."""
        msg = TourArray(site=self._site, tours=[tour_to_msg(t) for t in self._tours])
        msg.header.stamp = self.get_clock().now().to_msg()
        self._tours_pub.publish(msg)

    def publish_status(self, force: bool = False):
        """Publish site/status when it changed (or always with force)."""
        site = self._site
        values = (
            site,
            tuple(self._store.list_sites()),
            self._map is not None,
            self._mapping,
            len(self._pois),
            len(self._tours),
            self._store.tags_count(site) if site else 0,
            self._message,
        )
        if values == self._last_status and not force:
            return
        self._last_status = values
        msg = SiteStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.site, msg.sites, msg.has_map, msg.mapping = site, list(values[1]), values[2], values[3]
        msg.pois, msg.tours, msg.tags, msg.message = values[4], values[5], values[6], values[7]
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------ timers, inputs

    def startup(self):
        """Load the site of the parameter or of the last run, once the executor spins."""
        self._startup.cancel()
        if self._start_mapping and self.start_mapping_soon():
            return
        site = self._initial_site or self.remembered()
        if not site:
            self.get_logger().info('no site yet: mapping/start creates one')
            return
        if not valid_name(site) or not self._store.exists(site):
            self.get_logger().warn(f'site {site!r} does not exist in {self._store.root}')
            self._message = f'site {site!r} not found'
            self.publish_status()
            return
        self._site_message(site, self.activate(site, reset_localization=False), 'loaded')

    def start_mapping_soon(self) -> bool:
        """Map the launch site at start in mapping mode; False when it is not a new site."""
        site = self._initial_site
        if not valid_name(site) or self._store.has_content(site):
            self.get_logger().warn(
                f'start_mapping: {site!r} is not a new site name, loading it instead '
                '(mapping/start extends or resets an existing site)'
            )
            return False
        self._auto_mapping = self.create_timer(
            1.0, self.auto_mapping, callback_group=self._service_group
        )
        return True

    def auto_mapping(self):
        """Start mapping the new site as soon as localization and the grid mapper answer."""
        clients = (self._load_tags, self._assume_origin, self._mapper_start)
        if not all(c.service_is_ready() for c in clients):
            return
        self._auto_mapping.cancel()
        request = StartMapping.Request(site=self._initial_site, reset=True)
        result = self.mapping_start_callback(request, StartMapping.Response())
        if result.success:
            self.get_logger().info(f'start_mapping: {result.message}')
        else:
            self.get_logger().error(f'start_mapping: {result.message}')

    def refresh(self):
        """Retry handing over the tags file and republish the status when files changed."""
        if self._tags_pending and self._site and self._load_tags.service_is_ready():
            ok, message = self.hand_tags(create=False)
            if ok:
                self.get_logger().info(f'{self._site}: {message}')
        self.publish_status()

    def on_grid(self, msg: OccupancyGrid):
        """Relay the live grid to /map while mapping."""
        if self._mapping:
            self._map_pub.publish(msg)

    def on_localization(self, msg: String):
        """Track whether the robot is localized."""
        self._loc_status = msg.data

    def _site_message(self, site: str, notes: list, verb: str) -> str:
        """Log, store and return the outcome of a site change."""
        message = f'{verb} {site}: {len(self._pois)} POIs, {len(self._tours)} tours'
        message += ', map' if self._map is not None else ', no map yet'
        if notes:
            message += ' (' + '; '.join(notes) + ')'
        self._message = message
        self.get_logger().info(message)
        self.publish_status(force=True)
        return message

    # ------------------------------------------------------------------ site services

    def list_callback(self, request, response):
        """Service: every site and the active one."""
        response.sites = self._store.list_sites()
        response.active = self._site
        return response

    def load_callback(self, request, response):
        """Service: make a site active (map, POIs, tours, tags)."""
        site = request.site.strip()
        if self._mapping:
            response.success, response.message = False, 'mapping: call mapping/stop first'
            return response
        if not valid_name(site) or not self._store.exists(site):
            response.success, response.message = False, f'no site {site!r}'
            return response
        notes = self.activate(site, reset_localization=True)
        response.success = True
        response.message = self._site_message(site, notes, 'loaded')
        return response

    def save_poi_callback(self, request, response):
        """Service: create or update a POI of the active site."""
        if not self._site:
            response.success, response.message = False, 'no site loaded'
            return response
        poi = poi_from_msg(request.poi)
        if request.at_robot_pose:
            pose = self.robot_pose()
            if pose is None or not self.localized():
                response.success = False
                response.message = 'robot pose unknown: localize first'
                return response
            poi['x'], poi['y'], poi['yaw'] = pose
        try:
            saved, self._pois = self._store.upsert_poi(self._site, poi)
        except (SiteError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        self.publish_pois()
        self._message = f'saved POI {saved["id"]}'
        self.publish_status()
        response.success, response.message = True, self._message
        response.poi = poi_to_msg(saved)
        return response

    def delete_poi_callback(self, request, response):
        """Service: remove a POI (and from every tour)."""
        if not self._site:
            response.success, response.message = False, 'no site loaded'
            return response
        try:
            self._pois, self._tours = self._store.delete_poi(self._site, request.id.strip())
        except (SiteError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        self.publish_pois()
        self.publish_tours()
        self._message = f'deleted POI {request.id.strip()}'
        self.publish_status()
        response.success, response.message = True, self._message
        return response

    def save_tour_callback(self, request, response):
        """Service: create or update a tour of the active site."""
        if not self._site:
            response.success, response.message = False, 'no site loaded'
            return response
        try:
            saved, self._tours = self._store.upsert_tour(self._site, tour_from_msg(request.tour))
        except (SiteError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        self.publish_tours()
        self._message = f'saved tour {saved["id"]}'
        self.publish_status()
        response.success, response.message = True, self._message
        response.tour = tour_to_msg(saved)
        return response

    def delete_tour_callback(self, request, response):
        """Service: remove a tour."""
        if not self._site:
            response.success, response.message = False, 'no site loaded'
            return response
        try:
            self._tours = self._store.delete_tour(self._site, request.id.strip())
        except (SiteError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        self.publish_tours()
        self._message = f'deleted tour {request.id.strip()}'
        self.publish_status()
        response.success, response.message = True, self._message
        return response

    # ------------------------------------------------------------------ mapping services

    def mapping_start_callback(self, request, response):
        """Service: map a site, new or existing (see the module docstring)."""
        site = request.site.strip()

        def fail(message):
            response.success, response.message = False, message
            self.get_logger().error(f'mapping/start {site}: {message}')
            return response

        try:
            check_name(site)
        except SiteError as error:
            return fail(str(error))
        if self._mapping:
            return fail(f'already mapping {self._site}: call mapping/stop first')
        notes = []
        same_frame = site == self._site and self.localized()
        if not same_frame and self._store.has_content(site):
            if request.reset:
                notes.append(f'previous {site} moved to {self._store.archive(site)}')
            else:
                notes += self.activate(site, reset_localization=True)
                located, error = self.call(self._localize, Trigger.Request())
                if located is None or not located.success:
                    return fail(
                        f'{site} has a map already: let the robot see one of its AprilTags '
                        f'and retry, or start it over with reset '
                        f'({error or located.message})'
                    )
                self._loc_status = 'tag'  # the latched status follows a moment later
                same_frame = True
        try:
            self._store.create(site)
        except OSError as error:
            return fail(f'cannot create {site}: {error}')
        if site != self._site or not same_frame:
            notes += self.activate(site, reset_localization=False)
        ok, message = self.hand_tags(create=True)
        if not ok:
            return fail(f'localization cannot take the tags file: {message}')
        if not self.localized():
            result, error = self.call(self._assume_origin, Trigger.Request())
            if result is None or not result.success:
                return fail(f'cannot set the map origin: {error or result.message}')
            self._loc_status = 'origin'
            notes.append('map origin at the current pose')
        resume = not request.reset and same_frame and self._store.has_map(site)
        if resume:
            result, error = self.call(
                self._mapper_load, LoadFile.Request(path=self._store.site_dir(site))
            )
            if result is None or not result.success:
                return fail(f'cannot continue the map: {error or result.message}')
        result, error = self.call(self._mapper_start, SetBool.Request(data=not resume))
        if result is None or not result.success:
            return fail(f'grid mapper did not start: {error or result.message}')
        self._mapping = True
        verb = 'continuing the map of' if resume else 'mapping'
        response.success = True
        response.message = self._site_message(site, notes, verb)
        return response

    def mapping_stop_callback(self, request, response):
        """Service: stop integrating scans; /map shows the saved map again."""
        result, error = self.call(self._mapper_stop, Trigger.Request())
        was = self._mapping
        self._mapping = False
        if self._site and self._store.has_map(self._site):
            try:
                self._map = load_map(self._store.site_dir(self._site))
                self._map_pub.publish(self.grid_msg(self._map))
            except MapError as error_text:
                self.get_logger().error(f'{self._site}: {error_text}')
        self._message = 'mapping stopped' if was else 'not mapping'
        if result is None:
            self._message += f' ({error})'
        self.publish_status(force=True)
        response.success, response.message = True, self._message
        return response

    def mapping_save_callback(self, request, response):
        """Service: write the grid into the active site and load it as the site map."""
        if not self._site:
            response.success, response.message = False, 'no site: call mapping/start first'
            return response
        result, error = self.call(
            self._mapper_save, LoadFile.Request(path=self._store.site_dir(self._site))
        )
        if result is None or not result.success:
            response.success = False
            response.message = error or result.message
            return response
        try:
            self._map = load_map(self._store.site_dir(self._site))
        except MapError as error_text:
            response.success, response.message = False, f'saved, but cannot read it: {error_text}'
            return response
        if not self._mapping:
            self._map_pub.publish(self.grid_msg(self._map))
        self._message = f'map of {self._site} saved'
        self.publish_status(force=True)
        response.success, response.message = True, result.message
        return response


def main(args=None):
    """Spin the site manager; service calls to other nodes need the extra threads."""
    rclpy.init(args=args)
    node = SiteManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
