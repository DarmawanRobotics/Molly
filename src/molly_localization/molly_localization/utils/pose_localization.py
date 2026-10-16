import math

from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations as tft


def transform_to_matrix(t: TransformStamped) -> np.ndarray:
    """Convert a TransformStamped into a 4x4 homogeneous matrix."""
    trans = t.transform.translation
    rot = t.transform.rotation
    m = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    m[:3, 3] = [trans.x, trans.y, trans.z]
    return m


def matrix_to_transform(
    m: np.ndarray, parent_frame: str, child_frame: str, stamp
) -> TransformStamped:
    """Convert a 4x4 homogeneous matrix into a TransformStamped."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(m[0, 3])
    t.transform.translation.y = float(m[1, 3])
    t.transform.translation.z = float(m[2, 3])
    q = tft.quaternion_from_matrix(m)
    t.transform.rotation.x = float(q[0])
    t.transform.rotation.y = float(q[1])
    t.transform.rotation.z = float(q[2])
    t.transform.rotation.w = float(q[3])
    return t


def pose_values_to_matrix(values) -> np.ndarray:
    """Convert a [x, y, z, qx, qy, qz, qw] list into a 4x4 homogeneous matrix."""
    x, y, z, qx, qy, qz, qw = values
    m = tft.quaternion_matrix([qx, qy, qz, qw])
    m[:3, 3] = [x, y, z]
    return m


def flatten_to_yaw(m: np.ndarray) -> np.ndarray:
    """Project a pose onto the ground plane, keeping only x, y and yaw."""
    yaw = math.atan2(m[1, 0], m[0, 0])
    flat = tft.euler_matrix(0.0, 0.0, yaw)
    flat[0, 3] = m[0, 3]
    flat[1, 3] = m[1, 3]
    return flat


def map_odom_from_map_base(m_map_base: np.ndarray, m_odom_base: np.ndarray) -> np.ndarray:
    """2D map->odom so that base_link lands on m_map_base given the current odom->base_link."""
    return flatten_to_yaw(m_map_base) @ tft.inverse_matrix(flatten_to_yaw(m_odom_base))


def map_base_from_tag(
    m_map_tag: np.ndarray, m_base_cam: np.ndarray, m_cam_tag: np.ndarray
) -> np.ndarray:
    """map->base_link implied by seeing a tag whose map pose is known."""
    return m_map_tag @ tft.inverse_matrix(m_base_cam @ m_cam_tag)


def pose_msg_to_matrix(pose) -> np.ndarray:
    """Convert a geometry_msgs/Pose into a 4x4 homogeneous matrix."""
    p, q = pose.position, pose.orientation
    return pose_values_to_matrix([p.x, p.y, p.z, q.x, q.y, q.z, q.w])


def planar_delta(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """Return the translation (m) and absolute yaw (rad) difference between two poses."""
    dist = math.hypot(a[0, 3] - b[0, 3], a[1, 3] - b[1, 3])
    dyaw = math.atan2(a[1, 0], a[0, 0]) - math.atan2(b[1, 0], b[0, 0])
    dyaw = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
    return dist, dyaw
