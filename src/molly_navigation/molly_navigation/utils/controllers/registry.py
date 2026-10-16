from molly_navigation.utils.controllers.holonomic_controller import HolonomicController
from molly_navigation.utils.controllers.lqr_controller import LqrController
from molly_navigation.utils.controllers.mppi_controller import MppiController
from molly_navigation.utils.controllers.pid_controller import PidController
from molly_navigation.utils.controllers.pure_pursuit_controller import PurePursuitController
from molly_navigation.utils.controllers.stanley_controller import StanleyController

CONTROLLERS = {
    'holonomic': HolonomicController,
    'pure_pursuit': PurePursuitController,
    'pid': PidController,
    'mppi': MppiController,
    'lqr': LqrController,
    'stanley': StanleyController,
}


def create_controller(name: str, params: dict):
    """Build a controller instance by name using the given constructor kwargs."""
    if name not in CONTROLLERS:
        raise ValueError(f'unknown controller: {name!r}, options are {list(CONTROLLERS)}')
    return CONTROLLERS[name](**params)
