"""Sites on disk: names, slugs, validation, atomic writes, cascades and archives."""

import json
import math
import os

import pytest

site_store = pytest.importorskip('molly_mapping.site_store')


@pytest.fixture
def store(tmp_path):
    """Provide a store with one empty site."""
    s = site_store.SiteStore(str(tmp_path / 'map'))
    s.create('lobby')
    return s


def poi(name, **kw):
    """Return a POI dict."""
    return {'name': name, 'x': 1.0, 'y': 2.0, 'yaw': 0.5, **kw}


def test_names_and_listing(store):
    """Only valid names are sites; bad names raise SiteError."""
    store.create('hall-2')
    assert store.list_sites() == ['hall-2', 'lobby']
    for bad in ('', 'Lobby', '../etc', 'a b', '-x', 'x' * 49, None):
        assert not site_store.valid_name(bad)
        with pytest.raises(site_store.SiteError):
            store.site_dir(bad)


def test_slugs_are_unique_and_ascii():
    """Names become ids; clashes get -2, -3; non-ASCII falls back."""
    assert site_store.slugify('Main Entrance!', set()) == 'main-entrance'
    assert site_store.slugify('Main Entrance', {'main-entrance'}) == 'main-entrance-2'
    assert site_store.slugify('Café Ümlaut', set()) == 'cafe-umlaut'
    assert site_store.slugify('展示', set(), 'poi') == 'poi'
    assert site_store.slugify('展示', {'poi'}, 'poi') == 'poi-2'


def test_poi_create_update_and_validation(store):
    """A POI gets an id from its name, updates in place, and bad fields are refused."""
    saved, pois = store.upsert_poi('lobby', poi('Fountain', narration='Halo!', dwell_sec=5))
    assert saved['id'] == 'fountain' and len(pois) == 1
    saved['narration'] = 'Selamat datang'
    saved, pois = store.upsert_poi('lobby', saved)
    assert len(pois) == 1 and pois[0]['narration'] == 'Selamat datang'
    second, _ = store.upsert_poi('lobby', poi('Fountain'))
    assert second['id'] == 'fountain-2'
    assert saved['yaw'] == pytest.approx(0.5)
    wrapped = site_store.clean_poi(poi('w', yaw=3 * math.pi))
    assert abs(wrapped['yaw']) == pytest.approx(math.pi)
    for bad in (poi(''), poi('x', x=float('nan')), poi('x', dwell_sec=-1), poi('x', dwell_sec=601),
                poi('x', id='Bad Id'), poi('x', narration='a' * 2001), poi('x', gesture='wave!'),
                poi('x', language='indonesia'), poi('x', y='far')):
        with pytest.raises(site_store.SiteError):
            store.upsert_poi('lobby', bad)
    with open(store.path('lobby', 'pois.json')) as f:
        data = json.load(f)
    assert [p['id'] for p in data['pois']] == ['fountain', 'fountain-2']


def test_tours_refuse_unknown_pois_and_follow_deletes(store):
    """A tour needs known POIs; deleting a POI removes it from tours and empty tours vanish."""
    store.upsert_poi('lobby', poi('A'))
    store.upsert_poi('lobby', poi('B'))
    tour, _ = store.upsert_tour('lobby', {'name': 'Short', 'poi_ids': ['a', 'b'],
                                          'return_poi_id': 'a'})
    assert tour['id'] == 'short'
    store.upsert_tour('lobby', {'name': 'Only B', 'poi_ids': ['b']})
    with pytest.raises(site_store.SiteError):
        store.upsert_tour('lobby', {'name': 'Bad', 'poi_ids': ['nope']})
    with pytest.raises(site_store.SiteError):
        store.upsert_tour('lobby', {'name': 'Empty', 'poi_ids': []})
    pois, tours = store.delete_poi('lobby', 'a')
    assert [p['id'] for p in pois] == ['b']
    assert tours == [{'id': 'short', 'name': 'Short', 'description': '', 'poi_ids': ['b'],
                      'return_poi_id': ''}, tours[1]]
    pois, tours = store.delete_poi('lobby', 'b')
    assert pois == [] and tours == []
    with pytest.raises(site_store.SiteError):
        store.delete_poi('lobby', 'b')
    with pytest.raises(site_store.SiteError):
        store.delete_tour('lobby', 'short')


def test_broken_entries_are_skipped_not_fatal(store):
    """A hand-edited pois.json with a bad entry still loads the good ones."""
    with open(store.path('lobby', 'pois.json'), 'w') as f:
        json.dump({'pois': [poi('Good', id='good'), {'id': 'bad'}, poi('Dup', id='good'), 7]}, f)
    pois, problems = store.load_pois('lobby')
    assert [p['id'] for p in pois] == ['good'] and len(problems) == 3


def test_atomic_writes_keep_a_backup(store):
    """Every write leaves the previous file as .bak and no temporary file."""
    store.upsert_poi('lobby', poi('One'))
    store.upsert_poi('lobby', poi('Two'))
    with open(store.path('lobby', 'pois.json.bak')) as f:
        backup = json.load(f)
    assert [p['id'] for p in backup['pois']] == ['one']
    assert not any(n.endswith('.tmp') for n in os.listdir(store.site_dir('lobby')))


def test_content_and_archive(store):
    """A site with tags or POIs has content; archive moves it into a backup folder."""
    assert not store.has_content('lobby')
    with open(store.path('lobby', 'tags.json'), 'w') as f:
        json.dump({'base_map': {'x': 0}}, f)
    assert store.tags_count('lobby') == 1 and store.has_content('lobby')
    store.upsert_poi('lobby', poi('A'))
    backup = store.archive('lobby')
    assert not store.has_content('lobby')
    assert sorted(os.listdir(backup)) == ['pois.json', 'routes', 'tags.json']
