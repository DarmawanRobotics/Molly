from molly_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.pose2d import Pose2D
import numpy as np


class MppiController(PathController):
    """
    Simplified Model Predictive Path Integral controller.

    Samples angular-velocity sequences at the requested speed, rolls out a unicycle model and
    returns the cost-weighted first control (local path window + lookahead terminal cost).
    """

    def __init__(
        self,
        goal_tolerance=0.3,
        horizon_steps=15,
        dt=0.1,
        num_samples=200,
        angular_std=1.0,
        temperature=1.0,
        max_angular_velocity=1.0,
        window_points=60,
    ):
        self._goal_tolerance = goal_tolerance
        self._horizon_steps = horizon_steps
        self._rollout_dt = dt
        self._num_samples = num_samples
        self._angular_std = angular_std
        self._temperature = temperature
        self._max_angular_velocity = max_angular_velocity
        self._progress = PathProgress()
        self._window_points = window_points
        self._path = np.zeros((0, 2))
        self._last_angular = 0.0
        self._last_rollout = None

    def set_path(self, path_xy, start_index=None):
        """Replace the tracked path, optionally pinning progress to start_index."""
        self._progress.set_path(path_xy, start_index)
        self._path = np.zeros((0, 2))
        self._last_angular = 0.0

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once progress reached the end and the robot is within goal_tolerance."""
        return self._progress.is_finished(pose, self._goal_tolerance)

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        return self._progress.index

    def path_cost(self, xs: np.ndarray, ys: np.ndarray, target) -> np.ndarray:
        """Return the squared path-window distance plus the terminal lookahead distance."""
        dx = xs[:, :, None] - self._path[None, None, :, 0]
        dy = ys[:, :, None] - self._path[None, None, :, 1]
        tracking = (dx * dx + dy * dy).min(axis=2).mean(axis=1)
        terminal = (xs[:, -1] - target[0]) ** 2 + (ys[:, -1] - target[1]) ** 2
        return tracking + terminal

    def update(self, pose: Pose2D, dt: float, target_speed: float) -> ControllerOutput:
        """Sample control sequences, roll them out, and return a cost-weighted-average command."""
        if not self._progress.path:
            return ControllerOutput(0.0, 0.0)
        self._progress.update(pose)
        self._path = np.array(self._progress.window(self._window_points))
        speed = max(target_speed, 0.05)
        horizon_length = speed * self._rollout_dt * self._horizon_steps
        target = self._progress.lookahead_point(pose, horizon_length)

        angular_samples = np.random.normal(
            self._last_angular, self._angular_std, size=(self._num_samples, self._horizon_steps)
        )
        angular_samples = np.clip(
            angular_samples, -self._max_angular_velocity, self._max_angular_velocity
        )

        x = np.full(self._num_samples, pose.x)
        y = np.full(self._num_samples, pose.y)
        yaw = np.full(self._num_samples, pose.yaw)
        xs = np.zeros((self._num_samples, self._horizon_steps))
        ys = np.zeros((self._num_samples, self._horizon_steps))

        for step in range(self._horizon_steps):
            yaw = yaw + angular_samples[:, step] * self._rollout_dt
            x = x + speed * np.cos(yaw) * self._rollout_dt
            y = y + speed * np.sin(yaw) * self._rollout_dt
            xs[:, step] = x
            ys[:, step] = y

        costs = self.path_cost(xs, ys, target) + 0.01 * np.mean(angular_samples**2, axis=1)
        weights = np.exp(-(costs - costs.min()) / self._temperature)
        weights /= np.sum(weights) + 1e-9

        best_angular_sequence = np.sum(weights[:, None] * angular_samples, axis=0)
        self._last_angular = float(best_angular_sequence[0])
        self._last_rollout = (xs[np.argmax(weights)].tolist(), ys[np.argmax(weights)].tolist())

        return ControllerOutput(linear=target_speed, angular=self._last_angular)

    def debug_info(self) -> dict:
        """Expose the best-scoring rollout for visualization, once one has been computed."""
        if self._last_rollout is None:
            return {}
        xs, ys = self._last_rollout
        return {'rollout_xy': list(zip(xs, ys))}
