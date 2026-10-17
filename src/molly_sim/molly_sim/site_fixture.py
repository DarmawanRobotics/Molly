"""
A ready site for the simulator: the world drawn as an occupancy map, demo POIs and a tour.

The guide can then run right away, without mapping first. Pure numpy, unit-tested.
"""

import json
import math
import os

from molly_mapping.map_io import GridMap, save_map
from molly_sim.world import World
import numpy as np

WALL_THICKNESS = 0.10   # m, drawn around every wall segment
MARGIN = 0.5            # m of unknown around the outer walls

# Demo POIs in the corridors of config/worlds/mall_loop.yaml and a tour through them
DEMO_POIS = [
    {
        'id': 'east_gate', 'name': 'East gate', 'description': 'Entrance on the east side',
        'x': 4.5, 'y': 0.0, 'yaw': math.pi / 2,
        'narration': 'This is the east gate.', 'language': 'en', 'gesture': '', 'dwell_sec': 1.0,
    },
    {
        'id': 'fountain', 'name': 'Fountain', 'description': 'The fountain in the north corridor',
        'x': 1.5, 'y': 3.0, 'yaw': math.pi,
        'narration': 'Here is the fountain.', 'language': 'en', 'gesture': '', 'dwell_sec': 1.0,
    },
]
DEMO_TOUR = {
    'id': 'demo', 'name': 'Demo tour', 'description': 'East gate, then the fountain',
    'poi_ids': ['east_gate', 'fountain'], 'return_poi_id': '',
}


def rasterize(world: World, resolution: float = 0.05, margin: float = MARGIN) -> GridMap:
    """Draw the walls and pillars as occupied cells; free inside the walls, unknown outside."""
    xy = np.concatenate([world.segments[:, :2], world.segments[:, 2:]]) if len(world.segments) \
        else np.zeros((1, 2))
    x0, y0 = xy.min(axis=0) - margin
    x1, y1 = xy.max(axis=0) + margin
    width = int(math.ceil((x1 - x0) / resolution))
    height = int(math.ceil((y1 - y0) / resolution))
    cx = x0 + (np.arange(width) + 0.5) * resolution
    cy = y0 + (np.arange(height) + 0.5) * resolution
    px, py = np.meshgrid(cx, cy)                      # row 0 at y0, like OccupancyGrid
    inner = xy.min(axis=0), xy.max(axis=0)
    data = np.full((height, width), -1, dtype=np.int8)
    inside = (px >= inner[0][0]) & (px <= inner[1][0]) & (py >= inner[0][1]) & (py <= inner[1][1])
    data[inside] = 0
    for sx0, sy0, sx1, sy1 in world.segments:
        dx, dy = sx1 - sx0, sy1 - sy0
        length2 = max(dx * dx + dy * dy, 1e-12)
        t = np.clip(((px - sx0) * dx + (py - sy0) * dy) / length2, 0.0, 1.0)
        dist = np.hypot(px - (sx0 + t * dx), py - (sy0 + t * dy))
        data[dist <= WALL_THICKNESS / 2.0 + resolution / 2.0] = 100
    for ox, oy, radius in world.circles:
        data[np.hypot(px - ox, py - oy) <= radius] = 100
    return GridMap(data, resolution, float(x0), float(y0), 0.0)


def write_site(world: World, directory: str, overwrite: bool = False) -> bool:
    """Write map, demo POIs, tour and an empty tags file into directory; False when present."""
    if not overwrite and os.path.isfile(os.path.join(directory, 'map.yaml')):
        return False
    save_map(directory, rasterize(world))
    os.makedirs(os.path.join(directory, 'routes'), exist_ok=True)
    files = {
        'pois.json': {'pois': DEMO_POIS},
        'tours.json': {'tours': [DEMO_TOUR]},
        'tags.json': {},
    }
    for name, content in files.items():
        with open(os.path.join(directory, name), 'w') as f:
            json.dump(content, f, indent=2)
    return True
