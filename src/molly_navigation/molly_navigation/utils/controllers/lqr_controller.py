from molly_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from molly_navigation.utils.path_progress import PathProgress
from molly_navigation.utils.pose2d import Pose2D
import numpy as np


class LqrController(PathController):
    """Discrete LQR on the path error [lateral, heading] with curvature feedforward."""

    def __init__(
        self,
        goal_tolerance=0.3,
        q_lateral=1.0,
        q_heading=0.5,
        r_angular=0.5,
        dt=0.05,
        max_angular_velocity=1.0,
    ):
        self._goal_tolerance = goal_tolerance
        self._Q = np.diag([q_lateral, q_heading])
        self._R = np.array([[r_angular]])
        self._dt = dt
        self._max_angular_velocity = max_angular_velocity
        self._progress = PathProgress()
        self._gains: dict[float, np.ndarray] = {}
        self._last_reference = None

    def set_path(self, path_xy, start_index=None):
        """Replace the tracked path."""
        self._progress.set_path(path_xy, start_index)

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once progress reached the end and the robot is within goal_tolerance."""
        return self._progress.is_finished(pose, self._goal_tolerance)

    def progress_index(self) -> int:
        """Return the current progress index along the path."""
        return self._progress.index

    def gain(self, speed: float) -> np.ndarray:
        """LQR gain for a forward speed, cached on a 5 cm/s grid."""
        key = round(max(speed, 0.05) / 0.05) * 0.05
        if key not in self._gains:
            A = np.array([[1.0, key * self._dt], [0.0, 1.0]])
            B = np.array([[0.0], [self._dt]])
            P = self._Q.copy()
            for _ in range(500):
                BtP = B.T @ P
                K = np.linalg.solve(self._R + BtP @ B, BtP @ A)
                P_next = self._Q + A.T @ P @ (A - B @ K)
                if np.max(np.abs(P_next - P)) < 1e-9:
                    break
                P = P_next
            self._gains[key] = K
        return self._gains[key]

    def update(self, pose: Pose2D, dt: float, target_speed: float) -> ControllerOutput:
        """Feedback on lateral/heading error plus v * curvature feedforward."""
        if not self._progress.path:
            return ControllerOutput(0.0, 0.0)
        self._progress.update(pose)
        ref = self._progress.reference(pose)
        self._last_reference = ref
        x = np.array([ref.lateral_error, ref.heading_error])
        feedback = float(-(self.gain(target_speed) @ x)[0])
        angular = target_speed * ref.curvature + feedback
        angular = max(-self._max_angular_velocity, min(self._max_angular_velocity, angular))
        return ControllerOutput(linear=target_speed, angular=angular)

    def debug_info(self) -> dict:
        """Expose the nearest path point for visualization."""
        path = self._progress.path
        return {'nearest_xy': path[self._progress.index]} if path else {}
