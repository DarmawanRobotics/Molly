"""The handheld's map image: an occupancy grid as a small grayscale PNG (no OpenCV needed)."""

import math
import struct
import zlib

import numpy as np

OCCUPIED = 0
FREE = 254
UNKNOWN = 205


def grid_image(data, width: int, height: int, occupied: int = 65, free: int = 25) -> np.ndarray:
    """Return the grid as an image, row 0 at the top (largest y): 0 / 254 / 205 like map_server."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    image = np.full((height, width), UNKNOWN, np.uint8)
    image[(grid >= 0) & (grid <= free)] = FREE
    image[grid >= occupied] = OCCUPIED
    return np.ascontiguousarray(image[::-1])


def downscale(image: np.ndarray, max_px: int):
    """
    Shrink an image to at most max_px per side by whole factors; return (image, factor).

    Blocks keep their darkest cell (occupied over unknown over free), so thin walls survive.
    Rows are padded at the top and columns at the right, so the bottom-left corner (the map
    origin) stays where it was.
    """
    height, width = image.shape
    factor = max(1, math.ceil(max(height, width) / max(int(max_px), 1)))
    if factor == 1:
        return image, 1
    rows, cols = math.ceil(height / factor), math.ceil(width / factor)
    padded = np.full((rows * factor, cols * factor), FREE, np.uint8)
    padded[rows * factor - height:, :width] = image
    blocks = padded.reshape(rows, factor, cols, factor)
    return blocks.min(axis=(1, 3)), factor


def encode_png(image: np.ndarray) -> bytes:
    """Encode an 8-bit grayscale image as PNG with zlib only."""
    height, width = image.shape
    rows = np.zeros((height, width + 1), np.uint8)  # filter byte 0 (none) in front of each row
    rows[:, 1:] = image

    def block(kind: bytes, payload: bytes) -> bytes:
        crc = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', crc)

    header = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return (
        b'\x89PNG\r\n\x1a\n'
        + block(b'IHDR', header)
        + block(b'IDAT', zlib.compress(rows.tobytes(), 6))
        + block(b'IEND', b'')
    )


def decode_png(png: bytes) -> np.ndarray:
    """Decode a PNG written by encode_png (tests and tools)."""
    if png[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('not a PNG')
    pos, width, height, data = 8, 0, 0, b''
    while pos < len(png):
        (length,) = struct.unpack_from('>I', png, pos)
        kind = png[pos + 4:pos + 8]
        payload = png[pos + 8:pos + 8 + length]
        if kind == b'IHDR':
            width, height = struct.unpack_from('>II', payload)
        elif kind == b'IDAT':
            data += payload
        pos += 12 + length
    rows = np.frombuffer(zlib.decompress(data), np.uint8).reshape(height, width + 1)
    return rows[:, 1:].copy()


def map_frame(msg, max_px: int, rev: int, site: str = ''):
    """Return (header dict, png bytes) of an OccupancyGrid for the handheld's map stream."""
    info = msg.info
    image = grid_image(msg.data, info.width, info.height)
    image, factor = downscale(image, max_px)
    header = {
        't': 'map',
        'rev': rev,
        'res': round(info.resolution * factor, 4),
        'ox': round(info.origin.position.x, 3),
        'oy': round(info.origin.position.y, 3),
        'w': int(image.shape[1]),
        'h': int(image.shape[0]),
        'site': site,
    }
    return header, encode_png(image)
