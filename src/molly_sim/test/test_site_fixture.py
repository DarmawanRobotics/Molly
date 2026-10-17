"""The simulator's ready site: the world as a map, demo POIs on free cells, a valid tour."""

import json
import math
import os

from molly_mapping.map_io import load_map
from molly_mapping.site_store import clean_poi, clean_tour
from molly_sim.site_fixture import DEMO_POIS, DEMO_TOUR, rasterize, write_site
from molly_sim.world import World

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD = os.path.join(os.path.dirname(HERE), 'config', 'worlds', 'mall_loop.yaml')


def cell(grid, x, y):
    """Return the occupancy value at a map position."""
    col = int((x - grid.origin_x) / grid.resolution)
    row = int((y - grid.origin_y) / grid.resolution)
    return int(grid.data[row, col])


def test_rasterized_world_has_walls_corridors_and_pillars():
    """Walls and pillars are occupied, the corridor is free, outside the walls is unknown."""
    grid = rasterize(World.from_yaml(WORLD))
    assert cell(grid, 6.0, 0.0) == 100          # outer wall
    assert cell(grid, 0.0, -1.5) == 100         # shop block
    assert cell(grid, 5.3, 3.8) == 100          # pillar
    assert cell(grid, -4.5, -3.0) == 0          # the start, in the corridor
    assert cell(grid, 0.0, 0.0) == 0            # inside the shops: free, walls stop the robot
    assert cell(grid, 6.3, 0.0) == -1           # beyond the outer wall


def test_demo_pois_and_tour_are_valid_and_reachable():
    """Every demo POI is a valid POI on a free cell well clear of walls; the tour names them."""
    grid = rasterize(World.from_yaml(WORLD))
    for poi in DEMO_POIS:
        clean_poi(poi)
        for dx in (-0.4, 0.0, 0.4):
            for dy in (-0.4, 0.0, 0.4):
                assert cell(grid, poi['x'] + dx, poi['y'] + dy) == 0, poi['id']
    clean_tour(DEMO_TOUR, [p['id'] for p in DEMO_POIS])


def test_write_site_once(tmp_path):
    """The site is written when it has no map and left alone afterwards."""
    world = World.from_yaml(WORLD)
    site = tmp_path / 'mall'
    assert write_site(world, str(site))
    grid = load_map(str(site))
    assert math.isclose(grid.resolution, 0.05)
    assert json.loads((site / 'tours.json').read_text())['tours'][0]['id'] == 'demo'
    (site / 'pois.json').write_text('{"pois": []}')
    assert not write_site(world, str(site))
    assert (site / 'pois.json').read_text() == '{"pois": []}'
    assert (site / 'routes').is_dir()
