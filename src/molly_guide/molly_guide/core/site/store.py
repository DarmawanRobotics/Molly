"""
The POIs and tours of a site, stored next to its map (Molly's part of the site directory).

    <sites_dir>/<site>/pois.json    {"pois": [...]}    fields of molly_interfaces/Poi
    <sites_dir>/<site>/tours.json   {"tours": [...]}   fields of molly_interfaces/Tour

Files are written atomically with the previous version as .bak (dharma_utils.io). Broken
entries are skipped with a note, never a crash.
"""

from __future__ import annotations

import os

from dharma_utils.io import SiteLayout, read_json, write_json_atomic
from molly_guide.core.site.validate import (
    MAX_POIS,
    MAX_TOURS,
    SiteError,
    check_name,
    clean_poi,
    clean_tour,
    slugify,
)

POIS_FILE = 'pois.json'
TOURS_FILE = 'tours.json'
FILES = (POIS_FILE, TOURS_FILE)


def _entries(data, key: str) -> list:
    items = data.get(key, []) if isinstance(data, dict) else []
    return items if isinstance(items, list) else []


class SiteContent:
    """POIs and tours of one site."""

    def __init__(self, sites_dir: str, site: str):
        check_name(site)
        self.layout = SiteLayout(sites_dir, site)
        self.site = site

    def path(self, name: str) -> str:
        """Return a file of the site directory."""
        return os.path.join(self.layout.root, name)

    def stamp(self) -> tuple:
        """Return the modification times of the files (a change means: read them again)."""
        stamps = []
        for name in FILES:
            try:
                stamps.append(os.stat(self.path(name)).st_mtime_ns)
            except OSError:
                stamps.append(None)
        return tuple(stamps)

    def _read(self, name: str, key: str) -> tuple[list, list[str]]:
        data, error = read_json(self.path(name))
        problems = [error] if error else []
        if data is not None and not isinstance(data, dict):
            problems.append(f'{name}: expected an object with "{key}"')
        return _entries(data, key), problems

    def load_pois(self) -> tuple[list[dict], list[str]]:
        """Return (valid POIs in order, problems)."""
        items, problems = self._read(POIS_FILE, 'pois')
        pois, seen = [], set()
        for i, raw in enumerate(items):
            try:
                poi = clean_poi(raw)
                if not poi['id'] or poi['id'] in seen:
                    raise SiteError(f'missing or duplicate id {poi["id"]!r}')
            except SiteError as error:
                problems.append(f'{POIS_FILE} entry {i}: {error}')
                continue
            seen.add(poi['id'])
            pois.append(poi)
        return pois, problems

    def load_tours(self, poi_ids=None) -> tuple[list[dict], list[str]]:
        """Return (valid tours, problems); tours with unknown POIs are skipped."""
        if poi_ids is None:
            poi_ids = [p['id'] for p in self.load_pois()[0]]
        items, problems = self._read(TOURS_FILE, 'tours')
        tours, seen = [], set()
        for i, raw in enumerate(items):
            try:
                tour = clean_tour(raw, poi_ids)
                if not tour['id'] or tour['id'] in seen:
                    raise SiteError(f'missing or duplicate id {tour["id"]!r}')
            except SiteError as error:
                problems.append(f'{TOURS_FILE} entry {i}: {error}')
                continue
            seen.add(tour['id'])
            tours.append(tour)
        return tours, problems

    def save_pois(self, pois: list[dict]):
        """Write the POIs."""
        write_json_atomic(self.path(POIS_FILE), {'pois': pois})

    def save_tours(self, tours: list[dict]):
        """Write the tours."""
        write_json_atomic(self.path(TOURS_FILE), {'tours': tours})

    def upsert_poi(self, raw: dict) -> dict:
        """Create (empty id: from the name) or update a POI; return it as saved."""
        poi = clean_poi(raw)
        pois, _ = self.load_pois()
        ids = [p['id'] for p in pois]
        if not poi['id']:
            poi['id'] = slugify(poi['name'], set(ids), 'poi')
        if poi['id'] in ids:
            pois[ids.index(poi['id'])] = poi
        elif len(pois) >= MAX_POIS:
            raise SiteError(f'a site has at most {MAX_POIS} POIs')
        else:
            pois.append(poi)
        self.save_pois(pois)
        return poi

    def delete_poi(self, poi_id: str):
        """Remove a POI and every reference to it in the tours (empty tours go too)."""
        pois, _ = self.load_pois()
        if poi_id not in [p['id'] for p in pois]:
            raise SiteError(f'no POI {poi_id!r} in {self.site}')
        self.save_pois([p for p in pois if p['id'] != poi_id])
        data, _ = read_json(self.path(TOURS_FILE))
        kept, changed = [], False
        for tour in _entries(data, 'tours'):
            if not isinstance(tour, dict):
                continue
            ids = [i for i in tour.get('poi_ids', []) if i != poi_id]
            if ids != tour.get('poi_ids', []) or tour.get('return_poi_id') == poi_id:
                changed = True
            if tour.get('return_poi_id') == poi_id:
                tour['return_poi_id'] = ''
            tour['poi_ids'] = ids
            if ids:
                kept.append(tour)
        if changed:
            self.save_tours(kept)

    def upsert_tour(self, raw: dict) -> dict:
        """Create (empty id: from the name) or update a tour; return it as saved."""
        poi_ids = [p['id'] for p in self.load_pois()[0]]
        tour = clean_tour(raw, poi_ids)
        tours, _ = self.load_tours(poi_ids)
        ids = [t['id'] for t in tours]
        if not tour['id']:
            tour['id'] = slugify(tour['name'], set(ids), 'tour')
        if tour['id'] in ids:
            tours[ids.index(tour['id'])] = tour
        elif len(tours) >= MAX_TOURS:
            raise SiteError(f'a site has at most {MAX_TOURS} tours')
        else:
            tours.append(tour)
        self.save_tours(tours)
        return tour

    def delete_tour(self, tour_id: str):
        """Remove a tour."""
        tours, _ = self.load_tours()
        if tour_id not in [t['id'] for t in tours]:
            raise SiteError(f'no tour {tour_id!r} in {self.site}')
        self.save_tours([t for t in tours if t['id'] != tour_id])
