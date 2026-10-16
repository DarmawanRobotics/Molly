import math

from molly_perception.visitors import (
    band_points,
    Cluster,
    cluster_labels,
    drop_static,
    fuse_camera,
    inflate,
    major_extent,
    person_clusters,
    StaticMap,
    summarize,
    VisitorTracker,
)
import numpy as np

BOX = (-0.4, 0.4, -0.28, 0.28)


def leg_pair(x: float, y: float, n: int = 12) -> np.ndarray:
    """Return points of two legs 0.2 m apart around (x, y)."""
    rng = np.random.default_rng(int(abs(x * 100 + y * 10)) + n)
    legs = []
    for dy in (-0.1, 0.1):
        legs.append(rng.normal([x, y + dy], 0.03, (n, 2)))
    return np.vstack(legs).astype(np.float32)


def test_band_points_keep_legs_only():
    """Floor, ceiling, the robot body and far points are dropped."""
    pts = np.array(
        [
            [2.0, 0.0, 0.0],    # leg height
            [2.0, 0.0, -0.4],   # floor
            [2.0, 0.0, 1.5],    # above the band
            [0.2, 0.1, 0.0],    # own body
            [9.0, 0.0, 0.0],    # too far
            [-1.5, 0.5, 0.5],   # torso behind
        ],
        dtype=np.float32,
    )
    xy = band_points(pts, -0.15, 1.0, BOX, 0.3, 5.0)
    assert xy.tolist() == [[2.0, 0.0], [-1.5, 0.5]]
    assert band_points(np.empty((0, 3)), -0.15, 1.0, BOX, 0.3, 5.0).shape == (0, 2)


def test_cluster_labels_split_far_groups():
    """Two groups 2 m apart get two labels, points of one group one label."""
    xy = np.vstack([leg_pair(1.5, 0.0), leg_pair(-1.5, 0.5)])
    labels = cluster_labels(xy, 0.25)
    assert len(set(labels.tolist())) == 2
    assert len(set(labels[:24].tolist())) == 1
    assert cluster_labels(np.empty((0, 2)), 0.25).size == 0


def test_person_clusters_reject_walls_and_specks():
    """A pair of legs is a person; a 3 m wall and a 2-point speck are not."""
    wall = np.stack([np.linspace(-1.5, 1.5, 120), np.full(120, 2.5)], axis=1)
    speck = np.array([[0.0, -2.0], [0.02, -2.0]])
    xy = np.vstack([leg_pair(1.5, 0.0), wall, speck]).astype(np.float32)
    clusters = person_clusters(xy, 0.25, 4, 0.1, 1.0)
    assert len(clusters) == 1
    assert math.hypot(clusters[0].x - 1.5, clusters[0].y) < 0.05
    assert 0.15 < clusters[0].size < 0.5


def test_major_extent_of_a_diagonal_line():
    """A 1 m diagonal segment is 1 m long whatever its direction."""
    t = np.linspace(0.0, 1.0, 50)
    xy = np.stack([t * math.cos(0.7), t * math.sin(0.7)], axis=1)
    assert abs(major_extent(xy) - 1.0) < 1e-6
    assert major_extent(np.zeros((1, 2))) == 0.0


def test_inflate_grows_by_a_disc():
    """One occupied cell grows to a disc of the given radius in cells."""
    grid = np.zeros((9, 9), dtype=bool)
    grid[4, 4] = True
    grown = inflate(grid, 2)
    assert grown[4, 6] and grown[2, 4] and grown[5, 5]
    assert not grown[4, 7] and not grown[2, 2]
    assert inflate(grid, 0).sum() == 1


def test_static_map_drops_pillars_not_visitors():
    """A cluster on an occupied map cell (a pillar) is dropped, one in free space kept."""
    data = np.zeros((40, 40), dtype=np.int8)
    data[20, 30] = 100  # pillar at map (3.05, 2.05) with 0.1 m cells and origin (0, 0)
    static = StaticMap.from_grid(data.ravel().tolist(), 40, 40, 0.1, 0.0, 0.0, 0.0, 0.25)
    pillar, person = Cluster(2.0, 1.0, 0.3, 10), Cluster(0.5, -0.5, 0.3, 10)
    robot = (1.0, 1.0, 0.0)  # the pillar is 2 m ahead and 1 m left of the robot
    kept = drop_static([pillar, person], static, robot)
    assert kept == [person]
    assert drop_static([pillar], None, robot) == [pillar]
    assert drop_static([pillar], static, None) == [pillar]


def test_fuse_camera_raises_or_adds():
    """A camera person next to a cluster raises its confidence, a lonely one is added."""
    clusters = [Cluster(2.0, 0.0, 0.3, 10, 0.4)]
    fused = fuse_camera(clusters, [(2.1, 0.1, 0.9), (0.0, 3.0, 0.8)], 0.6)
    assert len(fused) == 2 and fused[0].confidence > 0.8
    assert (fused[1].x, fused[1].y, fused[1].confidence) == (0.0, 3.0, 0.8)
    assert clusters[0].confidence == 0.4  # inputs untouched


def test_tracker_confirms_after_two_scans_and_forgets():
    """A visitor counts from its second scan and disappears after max_misses empty scans."""
    tracker = VisitorTracker(confirm_scans=2, max_misses=1, assoc_distance=0.5)
    assert tracker.update([Cluster(1.0, 0.0, 0.3, 10)]) == []
    confirmed = tracker.update([Cluster(1.1, 0.05, 0.3, 10)])
    assert len(confirmed) == 1 and confirmed[0].x == 1.1
    assert tracker.update([]) == []  # missed once: not reported this scan
    assert len(tracker.tracks) == 1
    tracker.update([])
    assert tracker.tracks == []


def test_summarize_front_and_behind():
    """Counts and nearest distances per sector; nobody gives -1."""
    positions = [(2.0, 0.0), (1.0, 0.2), (-1.5, 0.0), (0.0, 2.0)]  # front x2, behind, side
    s = summarize(positions, math.radians(90), math.radians(150))
    assert (s['count'], s['front'], s['behind']) == (4, 2, 1)
    assert abs(s['nearest_front'] - math.hypot(1.0, 0.2)) < 1e-9
    assert s['nearest_behind'] == 1.5
    empty = summarize([], math.radians(90), math.radians(150))
    assert empty == {
        'count': 0, 'front': 0, 'behind': 0, 'nearest': -1.0, 'nearest_front': -1.0,
        'nearest_behind': -1.0,
    }
