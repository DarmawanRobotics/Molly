"""Person detector pre- and post-processing for Ultralytics YOLO ONNX exports (no model needed)."""

from molly_perception.detector import letterbox, nms, OnnxPersonDetector, person_boxes
import numpy as np
import pytest


def head(rows) -> np.ndarray:
    """Return a (1, 84, 100) YOLO head from rows of (cx, cy, w, h, class, score), rest empty."""
    out = np.zeros((1, 84, 100), np.float32)  # real heads have thousands of anchors
    for i, (cx, cy, w, h, cls, score) in enumerate(rows):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + cls, i] = score
    return out


def test_letterbox_keeps_the_aspect_ratio_and_centres_the_image():
    """A 640x480 frame becomes 416x312 with 52 px of grey above and below."""
    pytest.importorskip('cv2')
    frame = np.full((480, 640, 3), 255, np.uint8)
    blob, scale, padding = letterbox(frame, 416)
    assert blob.shape == (1, 3, 416, 416) and blob.dtype == np.float32
    assert scale == pytest.approx(0.65) and padding == (0, 52)
    assert blob[0, 0, 0, 0] == pytest.approx(114 / 255) and blob[0, 0, 200, 200] == 1.0


def test_people_are_mapped_back_to_the_frame_and_suppressed_once():
    """Overlaps merge, weak boxes and other classes go, pixels undo the letterbox padding."""
    out = head(
        [
            (208, 208, 65, 130, 0, 0.9),   # a person in the middle
            (210, 206, 65, 130, 0, 0.8),   # the same person again
            (100, 100, 20, 40, 0, 0.2),    # too unsure
            (300, 300, 50, 50, 2, 0.95),   # a car
        ]
    )
    boxes = person_boxes(out, 0.4, 0.65, (0, 52), (480, 640))
    assert len(boxes) == 1
    x1, y1, x2, y2, score = boxes[0]
    assert score == pytest.approx(0.9)
    assert (x1, y1, x2, y2) == pytest.approx((270, 140, 370, 340))
    assert person_boxes(out.transpose(0, 2, 1), 0.4, 0.65, (0, 52), (480, 640)) == boxes
    assert person_boxes(out, 0.99, 0.65, (0, 52), (480, 640)) == []


def test_nms_keeps_separate_people():
    """Two people side by side both stay; best first."""
    boxes = np.array([[0, 0, 10, 20], [30, 0, 40, 20], [1, 0, 11, 20]], np.float32)
    assert nms(boxes, np.array([0.6, 0.9, 0.5]), 0.45) == [1, 0]


def test_missing_model_names_the_install_command(tmp_path):
    """Without the model file the detector says how to get it."""
    with pytest.raises(FileNotFoundError, match='install people_model'):
        OnnxPersonDetector(str(tmp_path / 'yolo11n-416.onnx'))
