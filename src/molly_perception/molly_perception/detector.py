import os

import numpy as np

PERSON = 0  # COCO class of a person in the Ultralytics YOLO heads


def letterbox(bgr: np.ndarray, size: int):
    """
    Resize keeping the aspect ratio and pad to size x size with grey 114 (Ultralytics style).

    Returns the network input (1, 3, size, size) RGB float32 in 0..1, the scale and the padding
    (x, y) in pixels of the padded image.
    """
    import cv2

    height, width = bgr.shape[:2]
    scale = min(size / width, size / height)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    if (new_w, new_h) != (width, height):
        bgr = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    canvas = np.full((size, size, 3), 114, np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = bgr
    blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(blob), scale, (pad_x, pad_y)


def nms(boxes: np.ndarray, scores: np.ndarray, iou: float) -> list[int]:
    """Return the indices kept by non-maximum suppression, best score first."""
    x1, y1, x2, y2 = boxes.T
    areas = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        best, rest = order[0], order[1:]
        keep.append(int(best))
        w = np.clip(np.minimum(x2[best], x2[rest]) - np.maximum(x1[best], x1[rest]), 0, None)
        h = np.clip(np.minimum(y2[best], y2[rest]) - np.maximum(y1[best], y1[rest]), 0, None)
        inter = w * h
        overlap = inter / (areas[best] + areas[rest] - inter + 1e-9)
        order = rest[overlap <= iou]
    return keep


def person_boxes(
    output: np.ndarray,
    confidence: float,
    scale: float,
    padding: tuple,
    image_shape: tuple,
    iou: float = 0.45,
    max_people: int = 100,
) -> list[tuple]:
    """
    Decode a YOLOv8 / YOLO11 head (1, 4 + classes, anchors) into person boxes.

    Rows are centre x, centre y, width, height in the padded input, then one score per class.
    Returns (x1, y1, x2, y2, score) in pixels of the original image, best first.
    """
    out = np.asarray(output, dtype=np.float32)
    out = out.reshape(out.shape[-2:])
    if out.shape[0] > out.shape[1]:  # exported as (anchors, 4 + classes)
        out = out.T
    scores = out[4 + PERSON]
    keep = scores >= confidence
    if not keep.any():
        return []
    cx, cy, w, h = out[:4, keep]
    scores = scores[keep]
    height, width = image_shape[:2]
    boxes = np.stack(
        [
            np.clip((cx - w / 2 - padding[0]) / scale, 0, width),
            np.clip((cy - h / 2 - padding[1]) / scale, 0, height),
            np.clip((cx + w / 2 - padding[0]) / scale, 0, width),
            np.clip((cy + h / 2 - padding[1]) / scale, 0, height),
        ],
        axis=1,
    )
    return [
        (*map(float, boxes[i]), float(scores[i])) for i in nms(boxes, scores, iou)[:max_people]
    ]


class OnnxPersonDetector:
    """
    YOLO person detector on an Ultralytics ONNX export, without PyTorch.

    ONNX Runtime when installed (its CUDA provider when that is available), else OpenCV DNN.
    """

    def __init__(
        self,
        model_path: str,
        confidence: float = 0.4,
        image_size: int = 416,
        threads: int = 2,
        backend: str = 'auto',
    ):
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'no model at {model_path}: script/setup.sh install people_model'
            )
        self.confidence = confidence
        self._size = image_size
        self._session = self._net = None
        self.backend = ''
        missing = ''
        if backend in ('auto', 'onnxruntime'):
            try:
                import onnxruntime as ort
            except ImportError as error:
                missing = f'onnxruntime not installed ({error})'
            else:
                self.load_onnxruntime(ort, model_path, threads)
        if not self.backend and backend in ('auto', 'opencv'):
            import cv2

            cv2.setNumThreads(threads)
            self._net = cv2.dnn.readNetFromONNX(model_path)
            self.backend = 'opencv-cpu'
        if not self.backend:
            raise RuntimeError(missing or f'unknown backend {backend!r}')

    def load_onnxruntime(self, ort, model_path: str, threads: int):
        """Open the model in ONNX Runtime, on CUDA when this build has it."""
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        available = ort.get_available_providers()
        providers = [
            p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider') if p in available
        ]
        self._session = ort.InferenceSession(model_path, options, providers=providers)
        model_input = self._session.get_inputs()[0]
        self._input = model_input.name
        if isinstance(model_input.shape[2], int):
            self._size = model_input.shape[2]  # a static export knows its size
        cuda = self._session.get_providers()[0] == 'CUDAExecutionProvider'
        self.backend = 'onnxruntime-cuda' if cuda else 'onnxruntime-cpu'

    def detect(self, bgr: np.ndarray) -> list[tuple]:
        """Return person boxes as (x1, y1, x2, y2, score) in pixels."""
        blob, scale, padding = letterbox(bgr, self._size)
        if self._session is not None:
            output = self._session.run(None, {self._input: blob})[0]
        else:
            self._net.setInput(blob)
            output = self._net.forward()
        return person_boxes(output, self.confidence, scale, padding, bgr.shape)


class PersonDetector:
    """Ultralytics YOLO person detector (.pt, .onnx or a TensorRT .engine); needs PyTorch."""

    def __init__(self, model_path: str, confidence: float = 0.4, image_size: int = 640):
        from ultralytics import YOLO

        self._model = YOLO(model_path, task='detect')
        self.confidence = confidence
        self._image_size = image_size
        self.backend = 'ultralytics'

    def detect(self, bgr: np.ndarray) -> list[tuple]:
        """Return person boxes as (x1, y1, x2, y2, score) in pixels."""
        result = self._model.predict(
            bgr,
            conf=self.confidence,
            imgsz=self._image_size,
            classes=[PERSON],
            verbose=False,
        )[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        return [(*map(float, b), float(s)) for b, s in zip(xyxy, scores)]


def create_detector(
    model_path: str, confidence: float, image_size: int, threads: int, backend: str = 'auto'
):
    """Return the detector for a model file: ONNX without PyTorch, anything else Ultralytics."""
    if model_path.endswith('.onnx') and backend != 'ultralytics':
        return OnnxPersonDetector(model_path, confidence, image_size, threads, backend)
    return PersonDetector(model_path, confidence, image_size)
