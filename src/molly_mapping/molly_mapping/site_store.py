"""
Sites on disk: one directory per mapped place with its map, AprilTags, POIs, tours and routes.

    <root>/<site>/map.yaml + map.pgm   occupancy map (map_server format)
    <root>/<site>/tags.json            {tag_frame: {x, y, z, qx, qy, qz, qw}} (localization)
    <root>/<site>/pois.json            {"pois": [...]}   fields of molly_interfaces/Poi
    <root>/<site>/tours.json           {"tours": [...]}  fields of molly_interfaces/Tour
    <root>/<site>/routes/              recorded routes

Everything that comes from the network is validated here; JSON is written atomically with a
.bak of the previous version. No ROS imports.
"""

from datetime import datetime
import json
import math
import os
import re
import shutil
import unicodedata

NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,47}$')
LANGUAGE_RE = re.compile(r'^[a-z]{0,3}(-[a-zA-Z0-9]{1,8})*$')
GESTURE_RE = re.compile(r'^[a-z0-9_]{0,32}$')
TEXT_LIMITS = {'name': 64, 'description': 1000, 'narration': 2000}
MAX_DWELL_SEC = 600.0
MAX_COORDINATE = 10000.0
POIS_FILE, TOURS_FILE, TAGS_FILE, ROUTES_DIR = 'pois.json', 'tours.json', 'tags.json', 'routes'
CONTENT = ('map.yaml', 'map.pgm', TAGS_FILE, POIS_FILE, TOURS_FILE, ROUTES_DIR)


class SiteError(ValueError):
    """A request about a site, POI or tour is invalid."""


def valid_name(name) -> bool:
    """Return True for a valid site, POI or tour id."""
    return isinstance(name, str) and bool(NAME_RE.match(name))


def check_name(name, what: str = 'site') -> str:
    """Return name if valid, else raise SiteError."""
    if not valid_name(name):
        raise SiteError(
            f'{what} {name!r} must be 1-48 of a-z, 0-9, _ and -, starting with a letter or digit'
        )
    return name


def slugify(text: str, taken, fallback: str = 'poi') -> str:
    """Return an id from text (ASCII, a-z0-9 and -) that is not in taken."""
    ascii_text = unicodedata.normalize('NFKD', str(text)).encode('ascii', 'ignore').decode()
    base = re.sub(r'[^a-z0-9]+', '-', ascii_text.lower()).strip('-')[:40].strip('-')
    base = base or fallback
    if base not in taken:
        return base
    n = 2
    while f'{base}-{n}' in taken:
        n += 1
    return f'{base}-{n}'


def clean_text(value, field: str) -> str:
    """Return a trimmed string within the limit of its field."""
    if value is None:
        return ''
    if not isinstance(value, str):
        raise SiteError(f'{field} must be text')
    text = value.strip()
    limit = TEXT_LIMITS.get(field)
    if limit is not None and len(text) > limit:
        raise SiteError(f'{field} is longer than {limit} characters')
    return text


def clean_number(value, field: str, low=-MAX_COORDINATE, high=MAX_COORDINATE) -> float:
    """Return a finite float within [low, high]."""
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise SiteError(f'{field} must be a number') from error
    if not math.isfinite(number) or not low <= number <= high:
        raise SiteError(f'{field} must be finite and within [{low}, {high}]')
    return number


def clean_poi(raw: dict) -> dict:
    """Validate a POI (fields of molly_interfaces/Poi) and return it normalized."""
    if not isinstance(raw, dict):
        raise SiteError('a POI must be an object')
    poi = {
        'id': raw.get('id', '') or '',
        'name': clean_text(raw.get('name'), 'name'),
        'description': clean_text(raw.get('description'), 'description'),
        'x': clean_number(raw.get('x', 0.0), 'x'),
        'y': clean_number(raw.get('y', 0.0), 'y'),
        'yaw': math.remainder(clean_number(raw.get('yaw', 0.0), 'yaw', -100.0, 100.0), math.tau),
        'narration': clean_text(raw.get('narration'), 'narration'),
        'language': clean_text(raw.get('language'), 'language'),
        'gesture': clean_text(raw.get('gesture'), 'gesture'),
        'dwell_sec': clean_number(raw.get('dwell_sec', 0.0), 'dwell_sec', 0.0, MAX_DWELL_SEC),
    }
    if poi['id']:
        check_name(poi['id'], 'POI id')
    if not poi['name']:
        raise SiteError('a POI needs a name')
    if not LANGUAGE_RE.match(poi['language']) or len(poi['language']) > 16:
        raise SiteError(f'language {poi["language"]!r} is not a language tag like "id" or "en"')
    if not GESTURE_RE.match(poi['gesture']):
        raise SiteError(f'gesture {poi["gesture"]!r} must be a-z, 0-9 and _')
    return poi


def clean_tour(raw: dict, poi_ids) -> dict:
    """Validate a tour (fields of molly_interfaces/Tour) against the site's POI ids."""
    if not isinstance(raw, dict):
        raise SiteError('a tour must be an object')
    ids = raw.get('poi_ids') or []
    if not isinstance(ids, (list, tuple)) or not all(isinstance(i, str) for i in ids):
        raise SiteError('poi_ids must be a list of POI ids')
    tour = {
        'id': raw.get('id', '') or '',
        'name': clean_text(raw.get('name'), 'name'),
        'description': clean_text(raw.get('description'), 'description'),
        'poi_ids': [i.strip() for i in ids],
        'return_poi_id': clean_text(raw.get('return_poi_id'), 'return_poi_id'),
    }
    if tour['id']:
        check_name(tour['id'], 'tour id')
    if not tour['name']:
        raise SiteError('a tour needs a name')
    if not tour['poi_ids']:
        raise SiteError('a tour needs at least one POI')
    if len(tour['poi_ids']) > 200:
        raise SiteError('a tour has at most 200 stops')
    known = set(poi_ids)
    unknown = [i for i in tour['poi_ids'] if i not in known]
    if tour['return_poi_id'] and tour['return_poi_id'] not in known:
        unknown.append(tour['return_poi_id'])
    if unknown:
        raise SiteError(f'unknown POI id(s): {", ".join(unknown)}')
    return tour


def write_json_atomic(path: str, data) -> None:
    """Write JSON next to path and rename it over path, keeping the old file as .bak."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write('\n')
    if os.path.isfile(path):
        shutil.copyfile(path, f'{path}.bak')
    os.replace(tmp, path)


def read_json(path: str, default):
    """Read JSON, or return default when the file is missing."""
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return default
    except (OSError, ValueError) as error:
        raise SiteError(f'cannot read {path}: {error}') from error


class SiteStore:
    """The sites under one root directory."""

    def __init__(self, root: str):
        """Use root (created when missing) as the sites directory."""
        self.root = os.path.abspath(os.path.expanduser(root))

    def site_dir(self, site: str) -> str:
        """Return the directory of a site (validated name)."""
        return os.path.join(self.root, check_name(site))

    def path(self, site: str, name: str) -> str:
        """Return a file of a site."""
        return os.path.join(self.site_dir(site), name)

    def list_sites(self) -> list:
        """Return every site name, sorted."""
        try:
            entries = os.listdir(self.root)
        except OSError:
            return []
        return sorted(
            e for e in entries if valid_name(e) and os.path.isdir(os.path.join(self.root, e))
        )

    def exists(self, site: str) -> bool:
        """Return True when the site directory exists."""
        return os.path.isdir(self.site_dir(site))

    def create(self, site: str) -> str:
        """Create the site directory (and routes/) when missing; return it."""
        directory = self.site_dir(site)
        os.makedirs(os.path.join(directory, ROUTES_DIR), exist_ok=True)
        return directory

    def has_map(self, site: str) -> bool:
        """Return True when the site has a saved map."""
        return os.path.isfile(self.path(site, 'map.yaml'))

    def tags_count(self, site: str) -> int:
        """Return the number of AprilTags recorded for the site (0 for a broken file)."""
        try:
            tags = read_json(self.path(site, TAGS_FILE), {})
        except SiteError:
            return 0
        return len(tags) if isinstance(tags, dict) else 0

    def has_content(self, site: str) -> bool:
        """Return True when the site holds a map, tags or POIs (a frame worth keeping)."""
        if not self.exists(site):
            return False
        return self.has_map(site) or self.tags_count(site) > 0 or bool(self.load_pois(site)[0])

    def load_pois(self, site: str):
        """Return (valid POIs in order, problems) of a site."""
        data = read_json(self.path(site, POIS_FILE), {'pois': []})
        items = data.get('pois', []) if isinstance(data, dict) else []
        pois, problems, seen = [], [], set()
        for i, raw in enumerate(items if isinstance(items, list) else []):
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

    def load_tours(self, site: str, poi_ids):
        """Return (valid tours, problems) of a site; tours with unknown POIs are dropped."""
        data = read_json(self.path(site, TOURS_FILE), {'tours': []})
        items = data.get('tours', []) if isinstance(data, dict) else []
        tours, problems, seen = [], [], set()
        for i, raw in enumerate(items if isinstance(items, list) else []):
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

    def save_pois(self, site: str, pois: list) -> None:
        """Write the POIs of a site."""
        write_json_atomic(self.path(site, POIS_FILE), {'pois': pois})

    def save_tours(self, site: str, tours: list) -> None:
        """Write the tours of a site."""
        write_json_atomic(self.path(site, TOURS_FILE), {'tours': tours})

    def upsert_poi(self, site: str, raw: dict):
        """Create or update a POI; return (saved POI, all POIs)."""
        poi = clean_poi(raw)
        pois, _ = self.load_pois(site)
        ids = [p['id'] for p in pois]
        if not poi['id']:
            poi['id'] = slugify(poi['name'], set(ids), 'poi')
        if poi['id'] in ids:
            pois[ids.index(poi['id'])] = poi
        else:
            if len(pois) >= 500:
                raise SiteError('a site has at most 500 POIs')
            pois.append(poi)
        self.save_pois(site, pois)
        return poi, pois

    def delete_poi(self, site: str, poi_id: str):
        """Remove a POI and every reference to it in the tours; return (POIs, tours)."""
        pois, _ = self.load_pois(site)
        if poi_id not in [p['id'] for p in pois]:
            raise SiteError(f'no POI {poi_id!r} in {site}')
        pois = [p for p in pois if p['id'] != poi_id]
        self.save_pois(site, pois)
        data = read_json(self.path(site, TOURS_FILE), {'tours': []})
        items = data.get('tours', []) if isinstance(data, dict) else []
        changed, kept = False, []
        for tour in items if isinstance(items, list) else []:
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
            self.save_tours(site, kept)
        tours, _ = self.load_tours(site, [p['id'] for p in pois])
        return pois, tours

    def upsert_tour(self, site: str, raw: dict):
        """Create or update a tour; return (saved tour, all tours)."""
        pois, _ = self.load_pois(site)
        poi_ids = [p['id'] for p in pois]
        tour = clean_tour(raw, poi_ids)
        tours, _ = self.load_tours(site, poi_ids)
        ids = [t['id'] for t in tours]
        if not tour['id']:
            tour['id'] = slugify(tour['name'], set(ids), 'tour')
        if tour['id'] in ids:
            tours[ids.index(tour['id'])] = tour
        else:
            if len(tours) >= 100:
                raise SiteError('a site has at most 100 tours')
            tours.append(tour)
        self.save_tours(site, tours)
        return tour, tours

    def delete_tour(self, site: str, tour_id: str):
        """Remove a tour; return the remaining tours."""
        pois, _ = self.load_pois(site)
        tours, _ = self.load_tours(site, [p['id'] for p in pois])
        if tour_id not in [t['id'] for t in tours]:
            raise SiteError(f'no tour {tour_id!r} in {site}')
        tours = [t for t in tours if t['id'] != tour_id]
        self.save_tours(site, tours)
        return tours

    def archive(self, site: str) -> str:
        """Move the map, tags, POIs, tours and routes of a site into backup-<time>/; return it."""
        directory = self.site_dir(site)
        backup = os.path.join(directory, datetime.now().strftime('backup-%Y%m%d-%H%M%S'))
        os.makedirs(backup, exist_ok=True)
        for name in CONTENT:
            for item in (name, f'{name}.bak'):
                source = os.path.join(directory, item)
                if os.path.exists(source):
                    shutil.move(source, os.path.join(backup, item))
        os.makedirs(os.path.join(directory, ROUTES_DIR), exist_ok=True)
        return backup
