import math

Waypoint = tuple[float, float, float, float]  # x, y, yaw_deg, dwell_sec


def dedupe(points: list[tuple[float, float]], min_spacing: float) -> list[tuple[float, float]]:
    """Drop points closer than min_spacing to the previously kept one (last point always kept)."""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    for p in points[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_spacing:
            out.append(p)
    if math.hypot(points[-1][0] - out[-1][0], points[-1][1] - out[-1][1]) > 1e-6:
        out.append(points[-1])
    return out


def resample(points: list[tuple[float, float]], spacing: float) -> list[tuple[float, float]]:
    """Resample a polyline at a uniform arc-length spacing, keeping both endpoints."""
    if len(points) < 2 or spacing <= 0.0:
        return list(points)
    out = [points[0]]
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        s = spacing - carry
        while s <= seg:
            t = s / seg
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            s += spacing
        carry = seg - (s - spacing)
    if math.hypot(points[-1][0] - out[-1][0], points[-1][1] - out[-1][1]) > spacing * 0.3:
        out.append(points[-1])
    else:
        out[-1] = points[-1]
    return out


def smooth(
    points: list[tuple[float, float]],
    weight_data: float,
    weight_smooth: float,
    tolerance: float = 1e-5,
    max_iterations: int = 2000,
) -> list[tuple[float, float]]:
    """Gradient-descent smoothing with both endpoints fixed."""
    if len(points) < 3:
        return list(points)
    orig = [list(p) for p in points]
    new = [list(p) for p in points]
    for _ in range(max_iterations):
        change = 0.0
        for i in range(1, len(new) - 1):
            for d in (0, 1):
                prev = new[i][d]
                new[i][d] += weight_data * (orig[i][d] - new[i][d])
                new[i][d] += weight_smooth * (new[i - 1][d] + new[i + 1][d] - 2.0 * new[i][d])
                change += abs(prev - new[i][d])
        if change < tolerance:
            break
    return [(p[0], p[1]) for p in new]


def headings(points: list[tuple[float, float]]) -> list[float]:
    """Tangent heading (rad) at every point using central differences."""
    n = len(points)
    if n < 2:
        return [0.0] * n
    out = []
    for i in range(n):
        a, b = points[max(i - 1, 0)], points[min(i + 1, n - 1)]
        out.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return out


def curvatures(points: list[tuple[float, float]], span: int = 3) -> list[float]:
    """Unsigned Menger curvature (1/m) at every point from neighbours `span` indices away."""
    n = len(points)
    out = [0.0] * n
    for i in range(span, n - span):
        (ax, ay), (bx, by), (cx, cy) = points[i - span], points[i], points[i + span]
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ca = math.hypot(ax - cx, ay - cy)
        area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        denom = ab * bc * ca
        out[i] = 2.0 * area2 / denom if denom > 1e-9 else 0.0
    return out


def process_route(
    waypoints: list[Waypoint], spacing: float, weight_data: float, weight_smooth: float
) -> tuple[list[Waypoint], float]:
    """
    Resample and smooth a recorded route between anchors (ends and stop points).

    Returns the processed route and the maximum deviation from the recording in metres.
    """
    if len(waypoints) < 3:
        return list(waypoints), 0.0
    anchors = [0] + [
        i for i, w in enumerate(waypoints) if w[3] > 0.0 and 0 < i < len(waypoints) - 1
    ]
    anchors.append(len(waypoints) - 1)

    route: list[Waypoint] = []
    max_dev = 0.0
    for a, b in zip(anchors, anchors[1:]):
        raw = [(w[0], w[1]) for w in waypoints[a:b + 1]]
        pts = smooth(resample(dedupe(raw, spacing * 0.5), spacing), weight_data, weight_smooth)
        yaws = headings(pts)
        max_dev = max(max_dev, max_deviation(pts, raw))
        for j, (x, y) in enumerate(pts):
            if j == 0 and route:
                continue
            if j == 0:
                route.append(waypoints[a])
            elif j == len(pts) - 1:
                route.append(waypoints[b])
            else:
                route.append((x, y, math.degrees(yaws[j]), 0.0))
    return route, max_dev


def point_segment_distance(p, a, b) -> float:
    """Distance from point p to segment ab."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    t = (
        0.0
        if length_sq < 1e-12
        else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    )
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def max_deviation(path: list[tuple[float, float]], reference: list[tuple[float, float]]) -> float:
    """Largest distance from any path point to the reference polyline."""
    if len(reference) < 2:
        return 0.0
    segments = list(zip(reference, reference[1:]))
    return max(min(point_segment_distance(p, a, b) for a, b in segments) for p in path)


def velocity_profile(
    points: list[tuple[float, float]],
    stop_indices: list[int],
    cruise: float,
    min_speed: float,
    max_lateral_accel: float,
    decel: float,
    approach_speed: float,
) -> list[float]:
    """Return the speed limit per point: curvature cap, then braking into stops and the end."""
    n = len(points)
    if n == 0:
        return []
    kappa = curvatures(points)
    v = [
        max(min_speed, min(cruise, math.sqrt(max_lateral_accel / k) if k > 1e-6 else cruise))
        for k in kappa
    ]
    for i in stop_indices:
        v[i] = min(v[i], approach_speed)
    v[-1] = min(v[-1], approach_speed)
    for i in range(n - 2, -1, -1):
        ds = math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * decel * ds))
    return v


def parse_route_rows(rows) -> tuple[list[Waypoint], list[str]]:
    """
    Parse route CSV rows (x, y[, yaw_deg[, dwell_sec]]) into waypoints.

    Comments and blank rows are skipped. A row that is not numeric, or holds NaN or infinity, is
    dropped and reported: it would steer the robot nowhere. A negative dwell is reported and read
    as 0 (no stop).
    """
    waypoints, problems = [], []
    for line_no, row in enumerate(rows, start=1):
        if not row or not row[0].strip() or row[0].strip().startswith('#'):
            continue
        try:
            values = [float(v) for v in row[:4]]
            x, y = values[0], values[1]
        except (ValueError, IndexError):
            problems.append(f'line {line_no}: not a waypoint: {row}')
            continue
        yaw_deg = values[2] if len(values) > 2 else 0.0
        dwell_sec = values[3] if len(values) > 3 else 0.0
        if not all(math.isfinite(v) for v in (x, y, yaw_deg, dwell_sec)):
            problems.append(f'line {line_no}: not finite: {row}')
            continue
        if dwell_sec < 0.0:
            problems.append(f'line {line_no}: negative dwell {dwell_sec:g} s, not a stop')
            dwell_sec = 0.0
        waypoints.append((x, y, yaw_deg, dwell_sec))
    return waypoints, problems
