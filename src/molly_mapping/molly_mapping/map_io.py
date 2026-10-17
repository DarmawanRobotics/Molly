"""
Occupancy maps on disk in the map_server format (PGM + YAML) and as numpy grids.

No ROS imports, so everything here is unit-tested without a robot. Grid data uses the
nav_msgs/OccupancyGrid values (-1 unknown, 0 free, 100 occupied) with row 0 at the origin (lowest
y); images have row 0 at the top (highest y), as map_server reads and writes them.
"""

from dataclasses import dataclass
import math
import os

import numpy as np
import yaml

UNKNOWN, FREE, OCCUPIED = -1, 0, 100
PIXEL_OCCUPIED, PIXEL_FREE, PIXEL_UNKNOWN = 0, 254, 205
MAP_YAML, MAP_IMAGE = 'map.yaml', 'map.pgm'


class MapError(ValueError):
    """A map file is missing, broken or not understood."""


@dataclass
class GridMap:
    """An occupancy grid in the map frame: data[row, col] with row 0 at origin_y."""

    data: np.ndarray            # int8 (height, width): -1 unknown, 0..100 occupancy
    resolution: float           # m per cell
    origin_x: float             # m, map position of the corner of cell (0, 0)
    origin_y: float
    origin_yaw: float = 0.0     # rad

    @property
    def width(self) -> int:
        """Return the number of columns."""
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        """Return the number of rows."""
        return int(self.data.shape[0])


def _tokens(stream: bytes, count: int, start: int):
    """Read count whitespace-separated header tokens, skipping # comments; return (tokens, end)."""
    tokens, i, n = [], start, len(stream)
    while len(tokens) < count:
        while i < n and stream[i:i + 1].isspace():
            i += 1
        if i < n and stream[i:i + 1] == b'#':
            while i < n and stream[i:i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        j = i
        while j < n and not stream[j:j + 1].isspace() and stream[j:j + 1] != b'#':
            j += 1
        if j == i:
            raise MapError('truncated PGM header')
        tokens.append(stream[i:j])
        i = j
    return tokens, i


def read_pgm(path: str) -> np.ndarray:
    """Read a binary (P5) or ASCII (P2) PGM as uint8 rows top first (16-bit scaled to 8)."""
    try:
        with open(path, 'rb') as f:
            raw = f.read()
    except OSError as error:
        raise MapError(f'cannot read {path}: {error}') from error
    magic = raw[:2]
    if magic not in (b'P5', b'P2'):
        raise MapError(f'{path} is not a PGM (magic {magic!r})')
    (width, height, maxval), end = _tokens(raw, 3, 2)
    try:
        width, height, maxval = int(width), int(height), int(maxval)
    except ValueError as error:
        raise MapError(f'{path}: bad PGM header') from error
    if width <= 0 or height <= 0 or not 0 < maxval < 65536:
        raise MapError(f'{path}: bad PGM size {width}x{height} maxval {maxval}')
    count = width * height
    if magic == b'P5':
        body = raw[end + 1:]  # one whitespace byte after maxval
        dtype = np.dtype('>u2') if maxval > 255 else np.dtype('u1')
        expected = count * dtype.itemsize
        if len(body) < expected:
            raise MapError(f'{path}: {len(body)} bytes of pixels, expected {expected}')
        values = np.frombuffer(body, dtype=dtype, count=count).astype(np.float64)
    else:
        try:
            values = np.array(raw[end:].split()[:count], dtype=np.float64)
        except ValueError as error:
            raise MapError(f'{path}: bad ASCII pixels') from error
        if values.size < count:
            raise MapError(f'{path}: {values.size} pixels, expected {count}')
    if maxval != 255:
        values = np.round(values * 255.0 / maxval)
    return np.clip(values, 0, 255).astype(np.uint8).reshape(height, width)


def read_image(path: str) -> np.ndarray:
    """Read a map image as uint8 grey rows top first: PGM natively, other formats with OpenCV."""
    if path.lower().endswith('.pgm'):
        return read_pgm(path)
    try:
        import cv2
    except ImportError as error:
        raise MapError(f'{path}: only PGM maps can be read without OpenCV') from error
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise MapError(f'cannot read image {path}')
    return image


def write_pgm(path: str, image: np.ndarray, comment: str = '') -> None:
    """Write uint8 rows top first as a binary PGM (P5), atomically."""
    image = np.ascontiguousarray(image, dtype=np.uint8)
    height, width = image.shape
    header = b'P5\n'
    if comment:
        header += f'# {comment}\n'.encode()
    header += f'{width} {height}\n255\n'.encode()
    tmp = f'{path}.tmp'
    with open(tmp, 'wb') as f:
        f.write(header)
        f.write(image.tobytes())
    os.replace(tmp, path)


def image_to_occupancy(
    image: np.ndarray,
    negate: bool = False,
    occupied_thresh: float = 0.65,
    free_thresh: float = 0.25,
    mode: str = 'trinary',
) -> np.ndarray:
    """Convert an image (rows top first) to OccupancyGrid values (row 0 at the bottom)."""
    shade = image.astype(np.float64)
    if mode == 'raw':
        data = np.where(shade > 100.0, UNKNOWN, shade)
    else:
        p = shade / 255.0 if negate else (255.0 - shade) / 255.0
        data = np.full(p.shape, float(UNKNOWN))
        if mode == 'scale':
            span = max(occupied_thresh - free_thresh, 1e-9)
            data = np.round(99.0 * (p - free_thresh) / span)
        elif mode != 'trinary':
            raise MapError(f'unknown map mode {mode!r}, use trinary, scale or raw')
        data[p > occupied_thresh] = OCCUPIED
        data[p < free_thresh] = FREE
    return np.flipud(data).astype(np.int8)


def occupancy_to_image(
    data: np.ndarray, occupied_thresh: float = 0.65, free_thresh: float = 0.25
) -> np.ndarray:
    """Convert OccupancyGrid values (row 0 at the bottom) to a trinary image (rows top first)."""
    data = np.asarray(data, dtype=np.int16)
    image = np.full(data.shape, PIXEL_UNKNOWN, dtype=np.uint8)
    known = data >= 0
    image[known & (data >= occupied_thresh * 100.0)] = PIXEL_OCCUPIED
    image[known & (data <= free_thresh * 100.0)] = PIXEL_FREE
    return np.flipud(image)


def map_yaml_path(path: str) -> str:
    """Return the YAML of a map given its YAML or the directory holding map.yaml."""
    return os.path.join(path, MAP_YAML) if os.path.isdir(path) else path


def load_map(path: str) -> GridMap:
    """Load a map_server map from its YAML file (or a directory holding map.yaml)."""
    yaml_path = map_yaml_path(path)
    try:
        with open(yaml_path) as f:
            meta = yaml.safe_load(f) or {}
    except OSError as error:
        raise MapError(f'cannot read {yaml_path}: {error}') from error
    except yaml.YAMLError as error:
        raise MapError(f'{yaml_path}: {error}') from error
    if not isinstance(meta, dict):
        raise MapError(f'{yaml_path}: expected a mapping')
    try:
        image_name = str(meta['image'])
        resolution = float(meta['resolution'])
        origin = [float(v) for v in meta.get('origin', [0.0, 0.0, 0.0])]
        negate = bool(int(meta.get('negate', 0)))
        occupied = float(meta.get('occupied_thresh', 0.65))
        free = float(meta.get('free_thresh', 0.25))
        mode = str(meta.get('mode', 'trinary'))
    except (KeyError, TypeError, ValueError) as error:
        raise MapError(f'{yaml_path}: missing or bad field ({error})') from error
    if not (resolution > 0.0 and math.isfinite(resolution)) or len(origin) < 2:
        raise MapError(f'{yaml_path}: bad resolution or origin')
    image_path = image_name if os.path.isabs(image_name) else os.path.join(
        os.path.dirname(os.path.abspath(yaml_path)), image_name
    )
    data = image_to_occupancy(read_image(image_path), negate, occupied, free, mode)
    yaw = origin[2] if len(origin) > 2 else 0.0
    return GridMap(data, resolution, origin[0], origin[1], yaw)


def save_map(directory: str, grid: GridMap) -> str:
    """Write map.pgm + map.yaml (trinary) into directory; return the YAML path."""
    os.makedirs(directory, exist_ok=True)
    write_pgm(
        os.path.join(directory, MAP_IMAGE),
        occupancy_to_image(grid.data),
        f'molly_mapping {grid.resolution:.3f} m/pix',
    )
    meta = {
        'image': MAP_IMAGE,
        'mode': 'trinary',
        'resolution': float(grid.resolution),
        'origin': [float(grid.origin_x), float(grid.origin_y), float(grid.origin_yaw)],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,  # unknown (205) stays unknown in every map_server
    }
    yaml_path = os.path.join(directory, MAP_YAML)
    tmp = f'{yaml_path}.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    os.replace(tmp, yaml_path)
    return yaml_path


def known_bounds(data: np.ndarray):
    """Return (row0, row1, col0, col1) of the known cells (inclusive), or None when all unknown."""
    rows = np.flatnonzero((data >= 0).any(axis=1))
    cols = np.flatnonzero((data >= 0).any(axis=0))
    if rows.size == 0:
        return None
    return int(rows[0]), int(rows[-1]), int(cols[0]), int(cols[-1])


def crop_known(grid: GridMap, margin_m: float) -> GridMap:
    """Return the grid cut to its known cells plus a margin (unchanged when nothing is known)."""
    bounds = known_bounds(grid.data)
    if bounds is None:
        return grid
    pad = int(math.ceil(max(margin_m, 0.0) / grid.resolution))
    r0, r1, c0, c1 = bounds
    r0, c0 = max(r0 - pad, 0), max(c0 - pad, 0)
    r1, c1 = min(r1 + pad, grid.height - 1), min(c1 + pad, grid.width - 1)
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    dx, dy = c0 * grid.resolution, r0 * grid.resolution
    return GridMap(
        grid.data[r0:r1 + 1, c0:c1 + 1].copy(),
        grid.resolution,
        grid.origin_x + c * dx - s * dy,
        grid.origin_y + s * dx + c * dy,
        grid.origin_yaw,
    )
