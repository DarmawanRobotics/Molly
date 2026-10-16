import json
import math
import os
import shutil

import numpy as np
import tf_transformations as tft


def average_poses(samples: list[np.ndarray]) -> tuple[np.ndarray, float, float]:
    """Average 4x4 poses; return (mean pose, max position std in m, yaw std in deg)."""
    positions = np.array([m[:3, 3] for m in samples])
    quats = np.array([tft.quaternion_from_matrix(m) for m in samples])
    quats[np.dot(quats, quats[0]) < 0.0] *= -1.0
    q = quats.mean(axis=0)
    q /= np.linalg.norm(q)
    mean = tft.quaternion_matrix(q)
    mean[:3, 3] = positions.mean(axis=0)
    yaws = np.array([math.atan2(m[1, 0], m[0, 0]) for m in samples])
    yaw_mean = math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean())
    yaw_dev = np.arctan2(np.sin(yaws - yaw_mean), np.cos(yaws - yaw_mean))
    return mean, float(positions.std(axis=0).max()), float(np.degrees(yaw_dev.std()))


def pose_to_dict(m: np.ndarray) -> dict:
    """Serialise a 4x4 pose in the tag_config.json format."""
    qx, qy, qz, qw = tft.quaternion_from_matrix(m)
    x, y, z = m[:3, 3]
    return {
        k: round(float(v), 4)
        for k, v in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), (x, y, z, qx, qy, qz, qw))
    }


def describe_pose(m: np.ndarray) -> str:
    """Return 'x y z | roll pitch yaw' with metres and degrees."""
    roll, pitch, yaw = (math.degrees(a) for a in tft.euler_from_matrix(m))
    x, y, z = m[:3, 3]
    return f'xyz=({x:.3f}, {y:.3f}, {z:.3f}) rpy=({roll:.1f}, {pitch:.1f}, {yaw:.1f})'


def write_json_atomic(path: str, data) -> None:
    """Write JSON to a temporary file next to path, then rename it over path."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2)
        f.write('\n')
    os.replace(tmp, path)


def create_empty_file(path: str) -> None:
    """Create an empty tag file ({}) unless one exists already."""
    if not os.path.exists(path):
        write_json_atomic(path, {})


def merge_into_file(path: str, poses: dict[str, np.ndarray]) -> dict:
    """Merge poses into a tag_config.json (keeping other tags), back up the old file, write it."""
    data = {}
    if os.path.isfile(path):
        with open(path, 'r') as f:
            data = json.load(f)
        if not isinstance(data, dict):
            data = {}
        shutil.copyfile(path, path + '.bak')
    data.update({frame: pose_to_dict(m) for frame, m in poses.items()})
    write_json_atomic(path, data)
    return data
