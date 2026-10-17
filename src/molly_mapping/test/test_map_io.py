"""map_server files: PGM P5/P2 reading, trinary conversion, save/load round trip, cropping."""

import numpy as np
import pytest

map_io = pytest.importorskip('molly_mapping.map_io')


def test_save_load_round_trip(tmp_path):
    """A saved grid loads back with the same cells, resolution and origin."""
    data = np.full((4, 6), map_io.UNKNOWN, dtype=np.int8)
    data[0, :] = map_io.OCCUPIED
    data[1:3, 1:5] = map_io.FREE
    grid = map_io.GridMap(data, 0.05, -1.25, 2.5)
    yaml_path = map_io.save_map(str(tmp_path / 'site'), grid)
    loaded = map_io.load_map(yaml_path)
    assert np.array_equal(loaded.data, data)
    assert loaded.resolution == pytest.approx(0.05)
    assert (loaded.origin_x, loaded.origin_y) == pytest.approx((-1.25, 2.5))
    assert map_io.load_map(str(tmp_path / 'site')).width == 6  # the directory works too


def test_image_row_zero_is_the_top(tmp_path):
    """Row 0 of the image is the largest y: the bottom grid row comes last in the file."""
    data = np.full((3, 2), map_io.FREE, dtype=np.int8)
    data[0, 0] = map_io.OCCUPIED  # bottom-left cell
    map_io.save_map(str(tmp_path), map_io.GridMap(data, 0.1, 0.0, 0.0))
    image = map_io.read_pgm(str(tmp_path / 'map.pgm'))
    assert image[-1, 0] == map_io.PIXEL_OCCUPIED
    assert image[0, 0] == map_io.PIXEL_FREE


def test_reads_map_server_files_with_comments_and_ascii(tmp_path):
    """A map_saver P5 with a comment and an ASCII P2 both read, thresholds as map_server."""
    p5 = tmp_path / 'a.pgm'
    pixels = np.array([[0, 254, 205]], dtype=np.uint8)
    p5.write_bytes(b'P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n3 1\n255\n' + pixels.tobytes())
    assert map_io.read_pgm(str(p5)).tolist() == [[0, 254, 205]]
    p2 = tmp_path / 'b.pgm'
    p2.write_text('P2\n3 1\n# a comment\n255\n0 254\n205\n')
    assert map_io.read_pgm(str(p2)).tolist() == [[0, 254, 205]]
    (tmp_path / 'map.yaml').write_text(
        'image: a.pgm\nresolution: 0.05\norigin: [-10.0, -5.0, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.196\n'
    )
    grid = map_io.load_map(str(tmp_path / 'map.yaml'))
    assert grid.data.tolist() == [[100, 0, -1]]
    assert grid.origin_x == -10.0


def test_sixteen_bit_and_negate(tmp_path):
    """16-bit PGMs are scaled to 8 bits; negate flips black and white."""
    path = tmp_path / 'c.pgm'
    path.write_bytes(b'P5 2 1 65535 ' + np.array([0, 65535], dtype='>u2').tobytes())
    assert map_io.read_pgm(str(path)).tolist() == [[0, 255]]
    image = np.array([[0, 255]], dtype=np.uint8)
    assert map_io.image_to_occupancy(image, negate=True).tolist() == [[0, 100]]


def test_broken_files_raise_map_error(tmp_path):
    """Missing, truncated or foreign files raise MapError, never anything else."""
    with pytest.raises(map_io.MapError):
        map_io.load_map(str(tmp_path / 'missing.yaml'))
    bad = tmp_path / 'bad.pgm'
    bad.write_bytes(b'P5\n10 10\n255\n' + b'\x00' * 5)
    with pytest.raises(map_io.MapError):
        map_io.read_pgm(str(bad))
    with pytest.raises(map_io.MapError):
        map_io.read_pgm(str(tmp_path / 'missing.pgm'))
    foreign = tmp_path / 'f.pgm'
    foreign.write_bytes(b'GIF89a')
    with pytest.raises(map_io.MapError):
        map_io.read_pgm(str(foreign))


def test_crop_known_keeps_cells_in_place():
    """Cropping to the known cells shifts the origin so every cell keeps its position."""
    data = np.full((100, 100), map_io.UNKNOWN, dtype=np.int8)
    data[40:45, 60:62] = map_io.FREE
    grid = map_io.GridMap(data, 0.1, -5.0, -5.0)
    cropped = map_io.crop_known(grid, 0.2)
    assert cropped.data.shape == (5 + 4, 2 + 4)
    assert cropped.origin_x == pytest.approx(-5.0 + 58 * 0.1)
    assert cropped.origin_y == pytest.approx(-5.0 + 38 * 0.1)
    assert map_io.crop_known(map_io.GridMap(np.full((3, 3), -1, np.int8), 1, 0, 0), 1).width == 3
