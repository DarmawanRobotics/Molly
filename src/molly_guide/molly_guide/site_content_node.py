"""
site_content_node: the POIs and tours of the active site, stored next to its map.

Follows dharma_mapping's site/status: when the site changes it loads <site>/pois.json and
tours.json and publishes them latched on site/pois and site/tours. The site/save_* and
site/delete_* services change them. A mapping reset of the site moves them into its backup.
"""

from __future__ import annotations

import os

from dharma_interfaces.msg import SiteStatus
from dharma_utils.io import valid_name
from dharma_utils.ros import LATCHED, DharmaNode, run
from molly_guide.core.config import SiteContentConfig
from molly_guide.core.site import archive
from molly_guide.core.site.store import SiteContent
from molly_guide.core.site.validate import SiteError, check_name
from molly_guide.utils import convert
from molly_interfaces.msg import PoiArray, TourArray
from molly_interfaces.srv import DeleteById, SavePoi, SaveTour

POSE_MAX_AGE_SEC = 1.0  # at_robot_pose needs a pose at most this old


class SiteContentNode(DharmaNode):
    """site/pois, site/tours and the services that change them."""

    def __init__(self, **kwargs):
        super().__init__('site_content_node', **kwargs)
        self.config = self.declare_config(SiteContentConfig)
        self.robot = self.declare_robot()
        self.tf.lookup(self.robot.frames.map, self.robot.frames.base)  # listen from the start
        self.content: SiteContent | None = None
        self.stamp = None
        self.known_backups: set[str] = set()
        self.notes: list[str] = []
        self.pois_pub = self.latched(PoiArray, 'site/pois')
        self.tours_pub = self.latched(TourArray, 'site/tours')
        self.create_subscription(SiteStatus, 'site/status', self.on_site, LATCHED)
        self.create_service(SavePoi, 'site/save_poi', self.on_save_poi)
        self.create_service(SaveTour, 'site/save_tour', self.on_save_tour)
        self.create_service(DeleteById, 'site/delete_poi', self.on_delete_poi)
        self.create_service(DeleteById, 'site/delete_tour', self.on_delete_tour)
        self.every(self.config.refresh_sec, self.refresh)
        self.publish([], [])
        self.log.info('site content ready', sites_dir=self.config.sites_dir)

    def on_site(self, msg: SiteStatus):
        """Load the POIs and tours of a newly active site."""
        site = msg.site if valid_name(msg.site) else ''
        if self.content is not None and site == self.content.site:
            return
        self.content = SiteContent(self.config.sites_dir, site) if site else None
        self.known_backups = archive.backups(self.content.layout.root) if site else set()
        self.log.info('active site', site=site or '(none)')
        self.reload()

    def reload(self):
        """Read the files and publish them (problems are logged once per change)."""
        if self.content is None:
            self.stamp, self.notes = None, []
            self.publish([], [])
            return
        self.stamp = self.content.stamp()
        pois, problems = self.content.load_pois()
        tours, more = self.content.load_tours([p['id'] for p in pois])
        notes = problems + more
        if notes != self.notes:
            for note in notes:
                self.log.warning('site content entry skipped', site=self.content.site, note=note)
        self.notes = notes
        self.publish(pois, tours)

    def publish(self, pois: list, tours: list):
        """Publish site/pois and site/tours (latched)."""
        site = self.content.site if self.content is not None else ''
        stamp = self.get_clock().now().to_msg()
        msg = PoiArray(site=site, pois=[convert.poi_msg(p) for p in pois])
        msg.header.stamp, msg.header.frame_id = stamp, self.robot.frames.map
        self.pois_pub.publish(msg)
        tours_msg = TourArray(site=site, tours=[convert.tour_msg(t) for t in tours])
        tours_msg.header.stamp = stamp
        self.tours_pub.publish(tours_msg)

    def refresh(self):
        """Follow a mapping reset, and files changed by hand."""
        if self.content is None:
            return
        root = self.content.layout.root
        if self.config.archive_with_map:
            for backup in archive.new_resets(root, self.known_backups):
                moved = archive.archive_content(root, backup)
                if moved:
                    self.log.warning(
                        'site reset: POIs and tours archived with the old map',
                        site=self.content.site,
                        backup=os.path.join(root, backup),
                    )
        self.known_backups = archive.backups(root)
        if self.content.stamp() != self.stamp:
            self.reload()

    # ---------------------------------------------------------------- services
    def robot_pose(self) -> tuple[float, float, float] | None:
        """Return the robot pose (x, y, yaw) in the map, or None when not fresh."""
        frames = self.robot.frames
        result = self.tf.fresh(frames.map, frames.base, POSE_MAX_AGE_SEC)
        if result is None:
            return None
        pose = result.pose2d
        return pose.x, pose.y, pose.yaw

    def editable(self) -> SiteContent:
        """Return the active site's content, or raise SiteError."""
        if self.content is None:
            raise SiteError('no site loaded: site/load or mapping/start first')
        return self.content

    def on_save_poi(self, request, response):
        """Service: create or update a POI (at_robot_pose: where the robot stands)."""
        try:
            content = self.editable()
            raw = convert.poi_dict(request.poi)
            if request.at_robot_pose:
                pose = self.robot_pose()
                if pose is None:
                    raise SiteError(f'robot pose unknown (no fresh map -> base): {self.tf.error}')
                raw['x'], raw['y'], raw['yaw'] = pose
            poi = content.upsert_poi(raw)
        except (SiteError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        self.reload()
        response.success, response.message = True, f'POI {poi["id"]} saved'
        response.poi = convert.poi_msg(poi)
        self.log.info('POI saved', site=content.site, poi=poi['id'])
        return response

    def on_save_tour(self, request, response):
        """Service: create or update a tour."""
        try:
            tour = self.editable().upsert_tour(convert.tour_dict(request.tour))
        except (SiteError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        self.reload()
        response.success, response.message = True, f'tour {tour["id"]} saved'
        response.tour = convert.tour_msg(tour)
        return response

    def on_delete_poi(self, request, response):
        """Service: remove a POI (and from every tour)."""
        return self.delete(request, response, 'POI', lambda c, i: c.delete_poi(i))

    def on_delete_tour(self, request, response):
        """Service: remove a tour."""
        return self.delete(request, response, 'tour', lambda c, i: c.delete_tour(i))

    def delete(self, request, response, what: str, remove):
        """Run a delete and answer."""
        try:
            remove(self.editable(), check_name(request.id, f'{what} id'))
        except (SiteError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        self.reload()
        response.success, response.message = True, f'{what} {request.id} deleted'
        return response


def main(args=None):
    """Run the node."""
    run(SiteContentNode, args=args)


if __name__ == '__main__':
    main()
