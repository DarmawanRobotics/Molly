import math

from molly_odometry.checks import looks_like_restart, stillness, yaw_from_quaternion
from molly_odometry.wait_still import split_args
import numpy as np


def still_samples(n=200, tilt_deg=0.0, noise=0.0, gyro=0.0):
    rng = np.random.default_rng(0)
    t = math.radians(tilt_deg)
    acc = np.tile([math.sin(t), 0.0, math.cos(t)], (n, 1)) + rng.normal(0.0, noise, (n, 3))
    return np.full((n, 3), gyro), acc


def test_still_level_imu_passes():
    gyro_max, acc_std, tilt = stillness(*still_samples(noise=0.002))
    assert gyro_max < 0.03 and acc_std < 0.01 and tilt < 1.0


def test_tilt_and_rotation_are_reported():
    _, _, tilt = stillness(*still_samples(tilt_deg=15.0))
    assert abs(tilt - 15.0) < 0.1
    gyro_max, _, _ = stillness(*still_samples(gyro=0.1))
    assert gyro_max > 0.1


def test_shaking_is_unit_free():
    g_units = stillness(*still_samples(noise=0.05))[1]
    gyro, acc = still_samples(noise=0.05)
    si_units = stillness(gyro, acc * 9.81)[1]
    assert g_units > 0.01 and abs(g_units - si_units) < 1e-9


def test_yaw():
    half = math.radians(30.0) / 2
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert abs(yaw - math.radians(30)) < 1e-9


def test_restart_detection():
    assert looks_like_restart(5.0, 0.0, 0.0, 0.0, 0.05, 0.3)
    assert looks_like_restart(0.0, math.radians(90), 0.01, 0.0, 0.05, 0.3)
    assert not looks_like_restart(0.02, 0.0, 0.0, 0.0, 0.05, 0.3)   # standing at the start
    assert not looks_like_restart(5.0, 0.0, 4.9, 0.0, 0.05, 0.3)    # normal motion


def test_split_args():
    ros, cmd = split_args(['--params-file', 'p.yaml', '--', '/bin/fl', '--ros-args', '-r', 'x'])
    assert ros == ['--ros-args', '--params-file', 'p.yaml']
    assert cmd == ['/bin/fl', '--ros-args', '-r', 'x']
    assert split_args(['--', 'a'])[0] == []
    ros, _ = split_args(['--params-file', 'a.yaml', '--params-file', 'site.yaml', '--', 'x'])
    assert ros == ['--ros-args', '--params-file', 'a.yaml', '--params-file', 'site.yaml']
