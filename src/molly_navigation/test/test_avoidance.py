import math

from molly_navigation.utils.avoidance.registry import AVOIDANCE_ALGORITHMS, create_avoidance
import pytest

N, ANGLE_MIN = 32, -math.pi / 2
INC = math.pi / N


def scan(fn):
    """Sector ranges from a function of the sector angle in degrees."""
    return [fn(math.degrees(ANGLE_MIN + (i + 0.5) * INC)) for i in range(N)]


def make(name):
    """Avoidance instance with smoothing disabled."""
    return create_avoidance(
        name,
        {
            'safe_distance': 1.5,
            'emergency_distance': 0.75,
            'emergency_cone_deg': 60.0,
            'smoothing': 0.0,
        },
    )


@pytest.mark.parametrize('name', sorted(AVOIDANCE_ALGORITHMS))
def test_clear_path_is_untouched(name):
    """No obstacle: no steering, no slowdown, no emergency."""
    r = make(name).update(scan(lambda a: 8.0), ANGLE_MIN, INC, 8.0)
    assert (
        abs(r.steering_bias) < 1e-6 and r.velocity_scale == pytest.approx(1.0) and not r.emergency
    )


@pytest.mark.parametrize('name', sorted(AVOIDANCE_ALGORITHMS))
def test_wall_ahead_triggers_emergency(name):
    """Anything inside the emergency distance in front must stop the robot."""
    assert make(name).update(scan(lambda a: 0.6), ANGLE_MIN, INC, 8.0).emergency


@pytest.mark.parametrize('name', ['vfh', 'potential_field', 'follow_gap'])
def test_obstacle_left_steers_right(name):
    """An obstacle slightly left of the heading steers the robot to the right."""
    r = make(name).update(scan(lambda a: 1.0 if 0 < a < 20 else 8.0), ANGLE_MIN, INC, 8.0)
    assert r.steering_bias < 0.0
