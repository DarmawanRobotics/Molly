"""The guide's pure parts of the gateway: map image, state summaries and command arguments."""

import math
import types

from molly_bridge import guide_commands, guide_state, map_image, protocol
import numpy as np
import pytest


def test_map_png_round_trip_and_orientation():
    """The PNG holds the grid top-down: occupied 0, free 254, unknown 205."""
    data = [0, 100, -1, 50,
            0, 0, 0, 0]
    image = map_image.grid_image(data, 4, 2)
    assert image.tolist() == [[254, 254, 254, 254], [254, 0, 205, 205]]
    assert map_image.decode_png(map_image.encode_png(image)).tolist() == image.tolist()


def test_downscale_keeps_walls_and_the_origin():
    """Blocks keep their darkest cell; padding goes on top and right, not under the origin."""
    image = np.full((5, 5), map_image.FREE, np.uint8)
    image[-1, 0] = map_image.OCCUPIED  # the cell at the origin
    small, factor = map_image.downscale(image, 2)
    assert factor == 3 and small.shape == (2, 2)
    assert small[-1, 0] == map_image.OCCUPIED and small[0, 1] == map_image.FREE


def test_map_frame_header_and_payload():
    """A grid becomes a map stream header + PNG that split_map_payload reads back."""
    info = types.SimpleNamespace(
        width=3, height=2, resolution=0.05,
        origin=types.SimpleNamespace(position=types.SimpleNamespace(x=-1.234, y=2.0)),
    )
    msg = types.SimpleNamespace(info=info, data=[0] * 6)
    header, png = map_image.map_frame(msg, 1024, 7, 'hall')
    assert header == {'t': 'map', 'rev': 7, 'res': 0.05, 'ox': -1.234, 'oy': 2.0, 'w': 3, 'h': 2,
                      'site': 'hall'}
    assert protocol.split_map_payload(protocol.map_payload(header, png)) == (header, png)
    assert protocol.split_map_payload(b'\x05') is None


def test_tour_summary_names_the_state():
    """Tour state numbers become names; an error shows its reason."""
    msg = types.SimpleNamespace(
        state=5, tour_id='main', tour_name='Main', current_poi_index=1, total_pois=3,
        current_poi_id='lobby', current_poi_name='Lobby', activity='', message='walking',
        error_reason='blocked',
    )
    summary = guide_state.tour_summary(msg)
    assert (summary['state'], summary['msg'], summary['index']) == ('error', 'blocked', 1)
    msg.state = 9
    assert guide_state.tour_summary(msg)['state'] == 'idle'


def test_visitors_summary_has_no_negative_distance():
    """-1 (nobody) becomes null."""
    msg = types.SimpleNamespace(count=0, front=0, behind=0, nearest=-1.0)
    assert guide_state.visitors_summary(msg)['nearest'] is None


def test_command_arguments_are_checked():
    """Ids, texts, numbers and gestures from the handheld are validated."""
    assert guide_commands.name('lobby-2') == 'lobby-2'
    for bad in ('', 'Lobby', '../x', 'a' * 49, 3, None):
        with pytest.raises(ValueError):
            guide_commands.name(bad)
    assert guide_commands.text('  a   b ', 'name') == 'a b'
    assert len(guide_commands.text('x' * 5000, 'text')) == 1000
    with pytest.raises(ValueError):
        guide_commands.text('  ', 'text', required=True)
    x, y, yaw = guide_commands.pose({'x': 1, 'y': -2, 'yaw': 3 * math.pi})
    assert (x, y) == (1.0, -2.0) and abs(abs(yaw) - math.pi) < 1e-9
    for bad in ({'x': float('inf'), 'y': 0}, {'x': True, 'y': 0}, {'y': 0}):
        with pytest.raises(ValueError):
            guide_commands.pose(bad)
    assert guide_commands.gesture('nod', ['nod']) == 'nod'
    with pytest.raises(ValueError):
        guide_commands.gesture('dance', ['nod'])


def test_poi_and_tour_fields():
    """Only the given POI fields change; a tour needs a name and valid stops."""
    assert guide_commands.poi_fields({'narration': ' hi '}, require_name=False) == {
        'narration': 'hi'
    }
    with pytest.raises(ValueError):
        guide_commands.poi_fields({'name': 'A', 'dwell': 601})
    tour = guide_commands.tour_fields({'name': 'Main', 'poi_ids': ['a', 'b']})
    assert tour == {'id': '', 'name': 'Main', 'description': '', 'poi_ids': ['a', 'b'],
                    'return_poi_id': ''}
    for bad in ({'name': 'x', 'poi_ids': []}, {'name': '', 'poi_ids': ['a']}, 'tour'):
        with pytest.raises(ValueError):
            guide_commands.tour_fields(bad)
    assert guide_commands.tour_start({'tour_id': 'main'}) == ('main', [])
    assert guide_commands.tour_start({'poi_ids': ['a']}) == ('', ['a'])
    with pytest.raises(ValueError):
        guide_commands.tour_start({})
