import math


def wrap_angle(a: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a: float, b: float) -> float:
    """Signed difference a - b wrapped into (-pi, pi]."""
    return wrap_angle(a - b)


def clamp(value: float, limit: float) -> float:
    """Clamp value into [-limit, limit]."""
    return max(-limit, min(limit, value))


def yaw_from_quaternion(q) -> float:
    """Yaw angle of a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def to_local(dx: float, dy: float, yaw: float) -> tuple[float, float]:
    """Rotate a world-frame vector into a frame with heading yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy
