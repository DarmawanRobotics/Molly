from molly_navigation.utils.controllers.registry import CONTROLLERS, create_controller
from molly_navigation.utils.path_processing import process_route
import numpy as np
import pytest
from sim_utils import drive, LOOP, noisy_route, OUT_AND_BACK

PARAMS = {
    'holonomic': {'lookahead_distance': 0.8},
    'pure_pursuit': {'lookahead_distance': 1.0},
    'pid': {'lookahead_distance': 1.0},
    'mppi': {'max_angular_velocity': 1.0, 'temperature': 0.05},
    'lqr': {'max_angular_velocity': 1.0},
    'stanley': {'max_angular_velocity': 1.0},
}


def smoothed(corners):
    """Smoothed version of a noisy recorded route as x,y points."""
    route, _ = process_route(noisy_route(corners), 0.1, 0.05, 0.4)
    return [(w[0], w[1]) for w in route]


@pytest.mark.parametrize('name', sorted(CONTROLLERS))
@pytest.mark.parametrize('corners', [LOOP, OUT_AND_BACK], ids=['loop', 'out_and_back'])
def test_controller_completes_route(name, corners):
    """Every controller finishes closed loops and overlapping corridors close to the path."""
    np.random.seed(1)
    controller = create_controller(name, dict(goal_tolerance=0.3, **PARAMS[name]))
    finished, errors = drive(controller, smoothed(corners))
    assert finished
    assert np.mean(errors) < 0.1
    assert max(errors) < 0.5


def test_closed_loop_not_finished_at_start():
    """A loop starts and ends at the same point but must not be finished at the start."""
    controller = create_controller('pure_pursuit', {'lookahead_distance': 1.0})
    path = smoothed(LOOP)
    controller.set_path(path)
    from molly_navigation.utils.pose2d import Pose2D

    assert not controller.is_finished(Pose2D(0.0, 0.1, 0.0))
