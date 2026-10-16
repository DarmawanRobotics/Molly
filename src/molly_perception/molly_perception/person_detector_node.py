#!/usr/bin/env python3
import json
import math
import os
import time

from geometry_msgs.msg import Pose, PoseArray
from molly_perception.person_projection import feet_pixel, ground_point, pixel_ray
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import tf_transformations as tft

try:
    import cv2
except ImportError:
    cv2 = None

# Changed while running (handheld, `ros2 param set`); the rest is read at start.
LIVE_PARAMS = ('enabled', 'rate_hz', 'confidence')

STATUS_PERIOD_SEC = 2.0  # perception/people_status also when nothing changes
RELOAD_AFTER_SEC = 30.0  # retry a model that failed to load
NO_CAMERA_SEC = 3.0  # without frames for this long the status says so
# rate_hz 0: detections per second by where the model runs (smooth boxes on the GPU, light on CPU)
AUTO_RATE_GPU_HZ = 5.0
AUTO_RATE_CPU_HZ = 2.0
MAX_BOXES = 20  # person boxes in the status (fractions of the image, for the handheld)


class PersonDetectorNode(Node):
    """Detects people in the camera and places them on the floor in the map frame."""

    def __init__(self):
        """Declare params and wire topics; the model loads at the first frame while enabled."""
        super().__init__('person_detector_node')
        defaults = {
            'enabled': True,
            'state_file': '~/.molly/people.json',
            'image_topic': '/camera/camera/color/image_raw',
            'compressed': False,
            'camera_info_topic': '/camera/camera/color/camera_info',
            'model_path': 'yolo11n-416.onnx',
            'backend': 'auto',
            'threads': 2,
            'confidence': 0.4,
            'image_size': 416,
            'rate_hz': 0.0,
            'floor_z': -0.35,
            'max_range': 12.0,
            'map_frame': 'map',
            'base_frame': 'base_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        state_file = str(p('state_file')).strip()
        self._state_file = os.path.expanduser(state_file) if state_file else ''
        self.restore_choice()
        self._enabled = bool(p('enabled'))
        self._rate = max(float(p('rate_hz')), 0.0)  # 0: auto
        self._confidence = float(p('confidence'))
        self._model_path = str(p('model_path'))
        self._backend = str(p('backend'))
        self._threads = max(int(p('threads')), 1)
        self._image_size = int(p('image_size'))
        self._image_topic = str(p('image_topic'))
        self._floor_z, self._max_range = float(p('floor_z')), float(p('max_range'))
        self._map_frame, self._base_frame = p('map_frame'), p('base_frame')
        self._last = 0.0
        self._k = None
        self._t_base_cam = {}

        self._detector = None
        self._load_error = ''
        self._load_tried = -math.inf
        self._frame_time = -math.inf  # last camera frame
        self._result = None  # count, latency, time of the last detection

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._people_pub = self.create_publisher(PoseArray, 'perception/people', 10)
        self._status_pub = self.create_publisher(String, 'perception/people_status', latched)
        self.create_subscription(CameraInfo, p('camera_info_topic'), self.on_info, 10)
        kind = CompressedImage if p('compressed') else Image
        self.create_subscription(kind, self._image_topic, self.on_image, qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(STATUS_PERIOD_SEC, self.publish_status)
        self.publish_status()

    # ---------------------------------------------------------------- live parameters
    def on_parameters(self, params):
        """Accept enabled / rate_hz / confidence while running; the rest needs a restart."""
        updates = {}
        for param in params:
            if param.name not in LIVE_PARAMS:
                return SetParametersResult(
                    successful=False, reason=f'{param.name} is read at start, restart the node'
                )
            if param.name == 'enabled':
                updates['enabled'] = bool(param.value)
                continue
            value = float(param.value)
            if param.name == 'rate_hz' and not (
                math.isfinite(value) and (value == 0.0 or 0.1 <= value <= 30.0)
            ):
                return SetParametersResult(successful=False, reason='rate_hz: 0 (auto), 0.1 to 30')
            if param.name == 'confidence' and not 0.05 <= value <= 0.95:
                return SetParametersResult(successful=False, reason='confidence: 0.05 to 0.95')
            updates[param.name] = value
        if 'enabled' in updates:
            self._enabled = updates['enabled']
        if 'rate_hz' in updates:
            self._rate = updates['rate_hz']
        if 'confidence' in updates:
            self._confidence = updates['confidence']
            if self._detector is not None:
                self._detector.confidence = self._confidence
        self.get_logger().info(f'person detection {"on" if self._enabled else "off"}')
        self.save_choice()
        self.publish_status()
        return SetParametersResult(successful=True)

    def restore_choice(self):
        """Apply the on / off chosen live before a restart (state_file)."""
        if not self._state_file:
            return
        try:
            with open(self._state_file) as f:
                saved = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as error:
            self.get_logger().warn(f'ignoring {self._state_file}: {error}')
            return
        if isinstance(saved, dict) and isinstance(saved.get('enabled'), bool):
            self.set_parameters([Parameter('enabled', value=saved['enabled'])])
            self.get_logger().info(
                f'person detection {"on" if saved["enabled"] else "off"} '
                f'(restored from {self._state_file})'
            )

    def save_choice(self):
        """Keep the live on / off for the next start (state_file)."""
        if not self._state_file:
            return
        try:
            os.makedirs(os.path.dirname(self._state_file) or '.', exist_ok=True)
            partial = self._state_file + '.tmp'
            with open(partial, 'w') as f:
                json.dump({'enabled': self._enabled}, f)
            os.replace(partial, self._state_file)
        except OSError as error:
            self.get_logger().warn(f'cannot keep the person detection choice: {error}')

    # ---------------------------------------------------------------- status
    def status(self) -> dict:
        """Return what perception/people_status carries (the handheld shows it)."""
        now = time.monotonic()
        error = ''
        if not self._enabled:
            state = 'off'
        elif self._load_error:
            state, error = 'error', self._load_error
        elif self._frame_time == -math.inf:
            state = 'loading'  # the camera has not sent a frame yet
        elif now - self._frame_time > NO_CAMERA_SEC:
            state, error = 'error', f'no camera images on {self._image_topic}'
        elif self._detector is None:
            state = 'loading'
        else:
            state = 'running'
        result = self._result or {}
        return {
            'enabled': self._enabled,
            'state': state,
            'error': error,
            'backend': getattr(self._detector, 'backend', ''),
            'model': os.path.basename(self._model_path),
            'count': result.get('count'),
            'boxes': result.get('boxes', []),
            'placed': result.get('placed'),
            'latency_ms': result.get('latency_ms'),
            'at': result.get('at'),
            'rate_hz': round(self.detection_rate(), 2),
        }

    def detection_rate(self) -> float:
        """Return detections per second: rate_hz, or with 0 by where the model runs."""
        if self._rate > 0.0:
            return self._rate
        backend = getattr(self._detector, 'backend', '')
        on_gpu = backend.endswith('cuda') or backend == 'ultralytics'
        return AUTO_RATE_GPU_HZ if on_gpu else AUTO_RATE_CPU_HZ

    def publish_status(self):
        """Publish the status (latched): after every detection and every STATUS_PERIOD_SEC."""
        self._status_pub.publish(String(data=json.dumps(self.status())))

    # ---------------------------------------------------------------- detection
    def ensure_detector(self) -> bool:
        """Load the model once enabled (and again RELOAD_AFTER_SEC after a failure)."""
        if self._detector is not None:
            return True
        now = time.monotonic()
        if now - self._load_tried < RELOAD_AFTER_SEC:
            return False
        self._load_tried = now
        try:
            from molly_perception.detector import create_detector

            self._detector = create_detector(
                self._model_path, self._confidence, self._image_size, self._threads, self._backend
            )
        except Exception as error:  # noqa: B902  missing model or runtime: report, retry later
            self._load_error = str(error)
            self.get_logger().error(f'person detector not loaded: {error}')
            self.publish_status()
            return False
        self._load_error = ''
        self.get_logger().info(
            f'person detector {os.path.basename(self._model_path)} on {self._detector.backend}'
        )
        return True

    def on_info(self, msg: CameraInfo):
        """Keep the camera intrinsics."""
        self._k = np.array(msg.k, dtype=float).reshape(3, 3)

    def decode(self, msg) -> np.ndarray:
        """Return a BGR image from a compressed or rgb8/bgr8 message, or None."""
        if cv2 is None:
            return None
        if isinstance(msg, CompressedImage):
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if msg.encoding not in ('rgb8', 'bgr8'):
            return None
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)
        img = img[:, : msg.width]
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img

    def camera_extrinsic(self, frame_id: str):
        """Return the cached base->camera transform as a 4x4 matrix, or None."""
        if frame_id not in self._t_base_cam:
            try:
                t = self._tf_buffer.lookup_transform(self._base_frame, frame_id, Time())
            except TransformException:
                return None
            self._t_base_cam[frame_id] = to_matrix(t)
        return self._t_base_cam[frame_id]

    def on_image(self, msg):
        """Detect people at rate_hz; count them, and place them on the map when localized."""
        now = time.monotonic()
        self._frame_time = now
        if not self._enabled or now - self._last < 1.0 / self.detection_rate():
            return
        self._last = now
        if not self.ensure_detector():
            return
        image = self.decode(msg)
        if image is None:
            return
        start = time.monotonic()
        boxes = self._detector.detect(image)
        latency = time.monotonic() - start

        people = PoseArray()
        people.header.stamp = msg.header.stamp
        people.header.frame_id = self._map_frame
        t_base_cam = self.camera_extrinsic(msg.header.frame_id)
        t_map_base = None
        if t_base_cam is not None and self._k is not None:
            try:
                t_map_base = to_matrix(
                    self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
                )
            except TransformException:
                pass  # not localized: count the people, place none
        for box in boxes if t_map_base is not None else []:
            point = ground_point(
                pixel_ray(*feet_pixel(box), self._k), t_base_cam, self._floor_z, self._max_range
            )
            if point is None:
                continue
            x, y, _ = (t_map_base @ np.append(point, 1.0))[:3]
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.w = box[4]  # detection score, poses carry no heading
            people.poses.append(pose)
        if t_map_base is not None:
            self._people_pub.publish(people)
        height, width = image.shape[:2]
        self._result = {
            'count': len(boxes),
            'boxes': [
                [round(x1 / width, 3), round(y1 / height, 3), round(x2 / width, 3),
                 round(y2 / height, 3), round(score, 2)]
                for x1, y1, x2, y2, score in boxes[:MAX_BOXES]
            ],
            'placed': len(people.poses),
            'latency_ms': int(latency * 1000),
            'at': time.time(),
        }
        self.publish_status()


def to_matrix(t) -> np.ndarray:
    """Return a 4x4 matrix from a TransformStamped."""
    q, v = t.transform.rotation, t.transform.translation
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    m[:3, 3] = [v.x, v.y, v.z]
    return m


def main(args=None):
    """Spin the person detector node."""
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
