import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

STATE_COLORS = {
    'FOLLOWING': (0.2, 0.9, 0.3),
    'APPROACHING': (1.0, 0.85, 0.1),
    'JOINING': (1.0, 0.85, 0.1),
    'DWELLING': (0.2, 0.6, 1.0),
    'GOAL_REACHED': (0.4, 0.6, 1.0),
    'PAUSED': (0.7, 0.7, 0.7),
    'NAV_INACTIVE': (0.5, 0.5, 0.5),
    'BLOCKED': (1.0, 0.5, 0.0),
    'NO_PATH': (1.0, 0.5, 0.0),
}
ALARM = (1.0, 0.15, 0.15)


def rgba(rgb, a: float = 1.0) -> ColorRGBA:
    """Return a ColorRGBA from an (r, g, b) tuple."""
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def state_color(state: str) -> tuple:
    """RGB for a follower state; unknown states are alarms."""
    return STATE_COLORS.get(state, ALARM)


def speed_color(v: float, v_max: float) -> ColorRGBA:
    """Red (slow) to green (fast) colour for a speed."""
    t = max(0.0, min(1.0, v / v_max)) if v_max > 0 else 1.0
    return ColorRGBA(r=float(1.0 - t), g=float(0.3 + 0.7 * t), b=0.1, a=1.0)


def base(frame: str, stamp, ns: str, mid: int, kind: int) -> Marker:
    """Marker skeleton with identity orientation."""
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def pt(x: float, y: float, z: float = 0.0) -> Point:
    """geometry_msgs Point from floats."""
    return Point(x=float(x), y=float(y), z=float(z))


def circle(
    frame,
    stamp,
    ns,
    mid,
    cx,
    cy,
    radius,
    rgb,
    width=0.02,
    alpha=0.8,
    segments=48,
    start=0.0,
    sweep=2 * math.pi,
    z=0.02,
) -> Marker:
    """Circle or arc outline."""
    m = base(frame, stamp, ns, mid, Marker.LINE_STRIP)
    m.scale.x = width
    m.color = rgba(rgb, alpha)
    for i in range(segments + 1):
        a = start + sweep * i / segments
        m.points.append(pt(cx + radius * math.cos(a), cy + radius * math.sin(a), z))
    return m


def disc(frame, stamp, ns, mid, x, y, radius, rgb, alpha=0.35, height=0.01) -> Marker:
    """Flat filled disc."""
    m = base(frame, stamp, ns, mid, Marker.CYLINDER)
    m.pose.position = pt(x, y, height / 2)
    m.scale.x = m.scale.y = 2 * radius
    m.scale.z = height
    m.color = rgba(rgb, alpha)
    return m


def sphere(frame, stamp, ns, mid, x, y, size, rgb, z=0.05, alpha=1.0) -> Marker:
    """Small sphere."""
    m = base(frame, stamp, ns, mid, Marker.SPHERE)
    m.pose.position = pt(x, y, z)
    m.scale.x = m.scale.y = m.scale.z = size
    m.color = rgba(rgb, alpha)
    return m


def line(frame, stamp, ns, mid, points, rgb, width=0.03, alpha=1.0, z=0.03) -> Marker:
    """Polyline through (x, y) points."""
    m = base(frame, stamp, ns, mid, Marker.LINE_STRIP)
    m.scale.x = width
    m.color = rgba(rgb, alpha)
    m.points = [pt(x, y, z) for x, y in points]
    return m


def arrow(frame, stamp, ns, mid, x, y, yaw, length, rgb, width=0.05, z=0.05, alpha=1.0) -> Marker:
    """Arrow from (x, y) pointing along yaw."""
    m = base(frame, stamp, ns, mid, Marker.ARROW)
    m.points = [pt(x, y, z), pt(x + length * math.cos(yaw), y + length * math.sin(yaw), z)]
    m.scale.x = width
    m.scale.y = 2 * width
    m.scale.z = 2 * width
    m.color = rgba(rgb, alpha)
    return m


def label(frame, stamp, ns, mid, x, y, text, rgb=(1.0, 1.0, 1.0), size=0.15, z=0.4) -> Marker:
    """Small view-facing label."""
    m = base(frame, stamp, ns, mid, Marker.TEXT_VIEW_FACING)
    m.pose.position = pt(x, y, z)
    m.scale.z = size
    m.color = rgba(rgb)
    m.text = text
    return m


def arc_from_curvature(frame, stamp, ns, mid, curvature, length, rgb, segments=24) -> Marker:
    """Path the robot would drive at a constant curvature, in the robot frame."""
    points = []
    for i in range(segments + 1):
        s = length * i / segments
        if abs(curvature) < 1e-4:
            points.append((s, 0.0))
        else:
            points.append(
                (math.sin(curvature * s) / curvature, (1.0 - math.cos(curvature * s)) / curvature)
            )
    return line(frame, stamp, ns, mid, points, rgb, width=0.025, alpha=0.9)


def cone(frame, stamp, ns, mid, half_angle, radius, rgb, alpha=0.25, segments=24) -> Marker:
    """Return a filled sector centred on +x, in the robot frame."""
    m = base(frame, stamp, ns, mid, Marker.TRIANGLE_LIST)
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color = rgba(rgb, alpha)
    for i in range(segments):
        a0 = -half_angle + 2 * half_angle * i / segments
        a1 = -half_angle + 2 * half_angle * (i + 1) / segments
        m.points += [
            pt(0, 0, 0.01),
            pt(radius * math.cos(a0), radius * math.sin(a0), 0.01),
            pt(radius * math.cos(a1), radius * math.sin(a1), 0.01),
        ]
    return m


def delete_all(frame: str, stamp) -> Marker:
    """Return a marker that clears every marker of the receiving display."""
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.action = Marker.DELETEALL
    return m
