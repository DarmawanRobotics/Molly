import math

from molly_sim.world import Body, cast_rays, lio_pose, scan_points, World
import numpy as np

BOX = World(
    segments=np.array([[-2, -2, 2, -2], [2, -2, 2, 2], [2, 2, -2, 2], [-2, 2, -2, -2]], float),
    circles=np.array([[1.0, 0.0, 0.25]]),
)


def test_rays_hit_walls_and_pillars():
    """From the centre: the pillar ahead at 0.75 m, walls at 2 m elsewhere."""
    d = cast_rays(BOX, 0.0, 0.0, np.array([0.0, math.pi / 2, math.pi, -math.pi / 2]), 10.0)
    assert np.allclose(d, [0.75, 2.0, 2.0, 2.0])


def test_dynamic_obstacles_and_max_range():
    """Extra circles are seen; empty space reads max range."""
    world = World()
    angles = np.array([0.0, math.pi])
    assert np.allclose(cast_rays(world, 0, 0, angles, 5.0), [5.0, 5.0])
    d = cast_rays(world, 0, 0, angles, 5.0, np.array([[-3.0, 0.0, 0.5]]))
    assert np.allclose(d, [5.0, 2.5])


def test_body_follows_commands_with_acceleration_limits():
    """1 m/s forward needs 1 s to reach (accel 1 m/s^2), then integrates the pose."""
    body = Body(accel=1.0)
    for _ in range(100):
        body.step((1.0, 0.0, 0.0), 0.01)
    assert abs(body.vx - 1.0) < 1e-9 and abs(body.x - 0.505) < 0.01
    body = Body(yaw=math.pi / 2)
    for _ in range(300):
        body.step((0.0, 0.5, 0.0), 0.01)  # sideways (+left) while facing +y: moves to -x
    assert body.x < -0.5 and abs(body.y) < 1e-9


def test_scan_points_are_in_base_link():
    """Rotating the robot rotates the world in its scan, not the points' frame."""
    rng = np.random.default_rng(0)
    pts = scan_points(BOX, Body(), (0.0, 0.0), 8, 10.0, [0.2], 0.0, rng)
    ahead = pts[np.argmin(np.abs(np.arctan2(pts[:, 1], pts[:, 0])))]
    assert abs(ahead[0] - 0.75) < 1e-6 and ahead[2] == np.float32(0.2)
    turned = scan_points(BOX, Body(yaw=math.pi), (0.0, 0.0), 8, 10.0, [0.2], 0.0, rng)
    behind = turned[np.argmin(np.abs(np.abs(np.arctan2(turned[:, 1], turned[:, 0])) - math.pi))]
    assert abs(behind[0] + 0.75) < 1e-6


def test_lio_pose_closes_the_tf_chain():
    """map->odom(start) * B * P * B^-1 must give the true base pose (B = base->imu)."""
    def mat(x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])

    start, pose, imu = (-4.5, -3.0, 0.3), (1.0, 2.0, 2.0), (0.12, -0.05)
    b = mat(imu[0], imu[1], 0.0)
    p = mat(*lio_pose(start, pose, imu))
    chain = mat(*start) @ b @ p @ np.linalg.inv(b)
    assert np.allclose(chain, mat(*pose))
    assert np.allclose(lio_pose(start, start, imu), (0.0, 0.0, 0.0))


def test_world_file_loads(tmp_path):
    """Polylines become segments, start yaw is in degrees."""
    f = tmp_path / 'w.yaml'
    f.write_text('walls: [[[0, 0], [1, 0], [1, 1]]]\npillars: [[2, 2, 0.3]]\nstart: [1, 2, 90]\n')
    world = World.from_yaml(str(f))
    assert world.segments.shape == (2, 4) and world.circles.shape == (1, 3)
    assert world.start[2] == math.pi / 2
