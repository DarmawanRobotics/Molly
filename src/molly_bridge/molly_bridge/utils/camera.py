"""Camera frames for the handheld as small JPEGs (OpenCV optional for driver JPEGs)."""

from __future__ import annotations

import numpy as np

try:
    import cv2
except ImportError:  # raw images need OpenCV, small compressed ones do not
    cv2 = None


def compress(img, max_width: int, quality: int) -> bytes:
    """JPEG-encode a BGR image, no wider than max_width."""
    height, width = img.shape[:2]
    if width > max_width:
        size = (max_width, int(height * max_width / width))
        img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else b''


def shrink(jpeg: bytes, max_bytes: int, max_width: int, quality: int) -> tuple[bytes, str]:
    """
    Re-encode a driver JPEG bigger than max_bytes (drivers default to quality 95).

    A frame travels as a burst of 1400-byte datagrams and one lost datagram loses the frame, so
    a 150 kB JPEG rarely reaches the handheld whole over Wi-Fi. Returns (jpeg, note).
    """
    if len(jpeg) <= max_bytes:
        return jpeg, ''
    if cv2 is None:
        return jpeg, (
            f'camera frames of {len(jpeg)} B are too big for the handheld and OpenCV is missing '
            'to shrink them: install python3-opencv or lower the JPEG quality'
        )
    img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    return (compress(img, max_width, quality) if img is not None else b''), ''


def encode(msg, max_width: int, quality: int) -> tuple[bytes, str]:
    """JPEG-encode a raw rgb8 / bgr8 sensor_msgs/Image; returns (jpeg, note)."""
    if cv2 is None:
        return b'', 'raw camera images need OpenCV (python3-opencv)'
    if msg.encoding not in ('rgb8', 'bgr8'):
        return b'', f'camera encoding {msg.encoding} is not rgb8 or bgr8'
    img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
    img = img[:, : msg.width]
    if msg.encoding == 'rgb8':
        img = np.ascontiguousarray(img[:, :, ::-1])
    return compress(img, max_width, quality), ''
