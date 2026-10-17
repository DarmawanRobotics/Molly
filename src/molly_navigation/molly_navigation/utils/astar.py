"""
Grid search on a costmap: A*, escaping a lethal start, snapping a goal, line-of-sight shortcuts.

Pure Python on flat lists (fast enough for a few hundred thousand cells), unit-tested. Cells are
(row, col); costs are per cell length, inf is lethal.
"""

import heapq
import math
import time

import numpy as np

SQRT2 = math.sqrt(2.0)


class SearchTimeout(RuntimeError):
    """The search ran out of time or expansions."""


def astar(cost: np.ndarray, start, goal, heuristic_weight: float = 1.0,
          max_expansions: int = 3_000_000, timeout_sec: float = 0.0):
    """Return the cheapest 8-connected path of cells from start to goal, or None."""
    rows, cols = cost.shape
    width = cols + 2
    padded = np.full((rows + 2, width), np.inf)
    padded[1:-1, 1:-1] = cost
    c = padded.ravel().tolist()
    s = (start[0] + 1) * width + start[1] + 1
    g_idx = (goal[0] + 1) * width + goal[1] + 1
    if not (math.isfinite(c[s]) and math.isfinite(c[g_idx])):
        return None
    gr, gc = divmod(g_idx, width)
    weight = max(1.0, float(heuristic_weight))
    straight = ((1, 1.0), (-1, 1.0), (width, 1.0), (-width, 1.0))
    diagonal = ((width + 1, 1, width), (width - 1, -1, width),
                (-width + 1, 1, -width), (-width - 1, -1, -width))
    inf = math.inf
    best = [inf] * len(c)
    parent = [-1] * len(c)
    closed = bytearray(len(c))
    best[s] = 0.0

    def h(i):
        r, k = divmod(i, width)
        dr, dc = abs(r - gr), abs(k - gc)
        return weight * (max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc))

    heap = [(h(s), 0.0, s)]
    deadline = time.monotonic() + timeout_sec if timeout_sec > 0.0 else None
    expansions = 0
    while heap:
        _, g, i = heapq.heappop(heap)
        if closed[i]:
            continue
        if i == g_idx:
            break
        closed[i] = 1
        expansions += 1
        if expansions > max_expansions or (
            deadline is not None and expansions % 4096 == 0 and time.monotonic() > deadline
        ):
            raise SearchTimeout(f'search stopped after {expansions} expansions')
        for step, length in straight:
            j = i + step
            cj = c[j]
            if cj == inf or closed[j]:
                continue
            ng = g + length * cj
            if ng < best[j]:
                best[j] = ng
                parent[j] = i
                heapq.heappush(heap, (ng + h(j), ng, j))
        for step, dc, dr in diagonal:
            j = i + step
            cj = c[j]
            # no corner cutting: both orthogonal neighbours must be passable
            if cj == inf or closed[j] or c[i + dc] == inf or c[i + dr] == inf:
                continue
            ng = g + SQRT2 * cj
            if ng < best[j]:
                best[j] = ng
                parent[j] = i
                heapq.heappush(heap, (ng + h(j), ng, j))
    if best[g_idx] == inf:
        return None
    path, i = [], g_idx
    while i != -1:
        r, k = divmod(i, width)
        path.append((r - 1, k - 1))
        i = parent[i]
    path.reverse()
    return path


def nearest_free(cost: np.ndarray, cell, max_cells: int):
    """Return the closest passable cell within max_cells of cell (cell itself first), or None."""
    rows, cols = cost.shape
    r0, c0 = cell
    for dr, dc, _ in _offsets(max_cells):
        r, c = r0 + dr, c0 + dc
        if 0 <= r < rows and 0 <= c < cols and math.isfinite(cost[r, c]):
            return r, c
    return None


_OFFSET_CACHE = {}


def _offsets(radius: int):
    """Return disk offsets sorted by distance (cached)."""
    radius = max(int(radius), 0)
    if radius not in _OFFSET_CACHE:
        _OFFSET_CACHE[radius] = sorted(
            ((dr, dc, math.hypot(dr, dc))
             for dr in range(-radius, radius + 1)
             for dc in range(-radius, radius + 1)
             if math.hypot(dr, dc) <= radius + 1e-9),
            key=lambda o: o[2],
        )
    return _OFFSET_CACHE[radius]


def line_cells(a, b):
    """Return (rows, cols) of the cells a line between the centres of cells a and b crosses."""
    (r0, c0), (r1, c1) = a, b
    n = 4 * max(abs(r1 - r0), abs(c1 - c0)) + 1
    t = np.linspace(0.0, 1.0, n + 1)
    rows = np.floor(r0 + 0.5 + t * (r1 - r0)).astype(np.int64)
    cols = np.floor(c0 + 0.5 + t * (c1 - c0)).astype(np.int64)
    return rows, cols


def line_ok(cost: np.ndarray, a, b, max_cost: float) -> bool:
    """Return True when every cell on the line a-b is passable and costs at most max_cost."""
    rows, cols = line_cells(a, b)
    height, width = cost.shape
    if rows.min() < 0 or cols.min() < 0 or rows.max() >= height or cols.max() >= width:
        return False
    values = cost[rows, cols]
    return bool(np.all(np.isfinite(values)) and np.all(values <= max_cost))


def shortcut(cost: np.ndarray, path):
    """
    Replace runs of the path by straight lines where nothing is in the way.

    A line may not cross a cell costlier than the most expensive cell of the stretch it replaces,
    so shortcuts never bring the robot closer to a wall than the search did.
    """
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    anchor = 0
    worst = float(cost[path[0]])
    for i in range(1, len(path)):
        worst = max(worst, float(cost[path[i]]))
        if i - anchor > 1 and not line_ok(cost, path[anchor], path[i], worst + 1e-9):
            out.append(path[i - 1])  # the search's own step from here to i is always kept
            anchor = i - 1
            worst = max(float(cost[path[anchor]]), float(cost[path[i]]))
    out.append(path[-1])
    return out
