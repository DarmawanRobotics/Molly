"""POIs and tours stored next to the site's map, and their archive on a reset (no ROS)."""

from __future__ import annotations

import json
import math
import os

from molly_guide.core.site import archive
from molly_guide.core.site.store import POIS_FILE, TOURS_FILE, SiteContent
from molly_guide.core.site.validate import SiteError, clean_poi, slugify
import pytest


def poi(name, **fields):
    return {'name': name, 'x': 1.0, 'y': 2.0, 'yaw': 0.5, **fields}


@pytest.fixture
def content(tmp_path):
    return SiteContent(str(tmp_path), 'museum')


def test_new_pois_get_ids_from_their_names(content):
    first = content.upsert_poi(poi('Lobby Utama'))
    second = content.upsert_poi(poi('Lobby utama!'))
    assert (first['id'], second['id']) == ('lobby-utama', 'lobby-utama-2')
    pois, problems = content.load_pois()
    assert [p['id'] for p in pois] == ['lobby-utama', 'lobby-utama-2'] and problems == []
    with open(content.path(POIS_FILE), encoding='utf-8') as f:
        assert json.load(f)['pois'][0]['name'] == 'Lobby Utama'


def test_update_keeps_the_order(content):
    content.upsert_poi(poi('A'))
    content.upsert_poi(poi('B'))
    content.upsert_poi(poi('A', id='a', narration='Halo', x=3.0))
    pois, _ = content.load_pois()
    assert [(p['id'], p['x']) for p in pois] == [('a', 3.0), ('b', 1.0)]


def test_validation():
    assert clean_poi(poi('x', yaw=7.0))['yaw'] == pytest.approx(7.0 - 2 * math.pi)
    for bad in (
        {'x': 1.0},
        poi(''),
        poi('x', id='Bad Id'),
        poi('x', x=float('nan')),
        poi('x', x=True),
        poi('x', language='Indonesian'),
        poi('x', gesture='wave!'),
        poi('x', dwell_sec=601.0),
        poi('x' * 65),
        'not a dict',
    ):
        with pytest.raises(SiteError):
            clean_poi(bad)
    assert slugify('Café Kopi', set()) == 'cafe-kopi' and slugify('!!!', set(), 'tour') == 'tour'


def test_tours_check_their_pois(content):
    content.upsert_poi(poi('A'))
    content.upsert_poi(poi('B'))
    tour = content.upsert_tour({'name': 'Main', 'poi_ids': ['a', 'b'], 'return_poi_id': 'a'})
    assert tour['id'] == 'main'
    with pytest.raises(SiteError, match='unknown POI'):
        content.upsert_tour({'name': 'Bad', 'poi_ids': ['a', 'nope']})
    with pytest.raises(SiteError, match='at least one'):
        content.upsert_tour({'name': 'Empty', 'poi_ids': []})


def test_deleting_a_poi_updates_the_tours(content):
    for name in ('A', 'B'):
        content.upsert_poi(poi(name))
    content.upsert_tour({'name': 'Both', 'poi_ids': ['a', 'b'], 'return_poi_id': 'b'})
    content.upsert_tour({'name': 'Only B', 'poi_ids': ['b']})
    content.delete_poi('b')
    tours, _ = content.load_tours()
    assert [(t['id'], t['poi_ids'], t['return_poi_id']) for t in tours] == [('both', ['a'], '')]
    with pytest.raises(SiteError):
        content.delete_poi('b')
    content.delete_tour('both')
    assert content.load_tours()[0] == []


def test_broken_files_are_tolerated(content):
    os.makedirs(content.layout.root, exist_ok=True)
    with open(content.path(POIS_FILE), 'w') as f:
        json.dump({'pois': [poi('A', id='a'), poi('A again', id='a'), {'name': 3}]}, f)
    with open(content.path(TOURS_FILE), 'w') as f:
        f.write('{not json')
    pois, problems = content.load_pois()
    assert [p['id'] for p in pois] == ['a'] and len(problems) == 2
    tours, problems = content.load_tours()
    assert tours == [] and len(problems) == 1


def test_stamp_changes_with_the_files(content):
    before = content.stamp()
    content.upsert_poi(poi('A'))
    assert content.stamp() != before


def test_a_mapping_reset_archives_the_pois(content):
    content.upsert_poi(poi('A'))
    content.upsert_poi(poi('B'))  # a .bak exists now
    root = content.layout.root
    known = archive.backups(root)
    os.makedirs(os.path.join(root, 'backup-20261028-100000'))  # a backup without a map
    os.makedirs(os.path.join(root, 'backup-20261028-110000'))
    open(os.path.join(root, 'backup-20261028-110000', 'map.yaml'), 'w').close()
    resets = archive.new_resets(root, known)
    assert resets == ['backup-20261028-110000']
    moved = archive.archive_content(root, resets[0])
    assert sorted(moved) == [POIS_FILE, POIS_FILE + '.bak']
    assert content.load_pois()[0] == []
    assert archive.new_resets(root, archive.backups(root)) == []


def test_invalid_site_name(tmp_path):
    with pytest.raises(SiteError):
        SiteContent(str(tmp_path), '../etc')
