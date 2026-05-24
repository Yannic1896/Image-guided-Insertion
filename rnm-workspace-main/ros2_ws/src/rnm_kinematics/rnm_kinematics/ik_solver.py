from typing import Optional

import numpy as np

from rnm_kinematics.fk_solver import ForwardKinematicsSolver


class IKSolver:
    """
    Base class for Inverse Kinematic solvers.
    """

    def __init__(
        self,
        dh_params: Optional[np.ndarray] = None,
        joint_lower_limits: Optional[np.ndarray] = None,
        joint_upper_limits: Optional[np.ndarray] = None,
        max_step: float = 0.05,
    ):
        """

        Args:
            dh_params: Denavit-Hartenberg parameters for the robot
            joint_lower_limits: Lower joint limits ordered by actuated joint
            joint_upper_limits: Upper joint limits ordered by actuated joint
            max_step: Maximum per-iteration joint update in radians
        """
        self.dh_params = dh_params
        self._fk_solver = (
            ForwardKinematicsSolver(dh_params) if dh_params is not None else None
        )
        self._joint_count = self._infer_joint_count(dh_params)
        self._lower_limits, self._upper_limits = self._load_joint_limits(
            joint_lower_limits,
            joint_upper_limits,
        )
        self._max_step = float(max_step)
        if self._max_step <= 0.0:
            raise ValueError('max_step must be positive')

    @staticmethod
    def _infer_joint_count(dh_params: Optional[np.ndarray]) -> int:
        """Infer number of actuated joints from DH rows."""
        if dh_params is None:
            return 0

        dh_matrix = np.asarray(dh_params, dtype=float)
        if dh_matrix.ndim != 2 or dh_matrix.shape[0] == 0:
            return 0

        # Heuristic: trailing [d, 0, 0] row is a fixed tool/flange transform.
        if (
            dh_matrix.shape[0] > 1
            and np.isclose(dh_matrix[-1, 1], 0.0)
            and np.isclose(dh_matrix[-1, 2], 0.0)
        ):
            return int(dh_matrix.shape[0] - 1)

        return int(dh_matrix.shape[0])

    def _load_joint_limits(
        self,
        lower_limits: Optional[np.ndarray],
        upper_limits: Optional[np.ndarray],
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Validate optional joint limits for the actuated joints."""
        if lower_limits is None and upper_limits is None:
            return None, None
        if lower_limits is None or upper_limits is None:
            raise ValueError('Both lower and upper joint limits must be provided')

        lower = np.asarray(lower_limits, dtype=float)
        upper = np.asarray(upper_limits, dtype=float)
        expected_shape = (self._joint_count,)
        if lower.shape != expected_shape or upper.shape != expected_shape:
            raise ValueError(
                'Joint limit arrays must match the actuated joint count '
                f'{self._joint_count}.'
            )
        if np.any(lower >= upper):
            raise ValueError('Each lower joint limit must be smaller than its upper limit')

        return lower, upper

    def _initial_joint_angles(self, initial_guess: Optional[np.ndarray]) -> np.ndarray:
        """Build the starting joint vector, using limits when no guess is available."""
        if initial_guess is None:
            if self._lower_limits is not None and self._upper_limits is not None:
                return 0.5 * (self._lower_limits + self._upper_limits)
            return np.zeros(self._joint_count, dtype=float)

        initial = np.asarray(initial_guess, dtype=float)
        if initial.shape != (self._joint_count,):
            raise ValueError(
                f'initial_guess must be shape ({self._joint_count},), '
                f'got {initial.shape}'
            )

        return self._clip_to_limits(initial.copy())

    def _clip_to_limits(self, joint_angles: np.ndarray) -> np.ndarray:
        """Clip joint angles to configured physical limits when available."""
        if self._lower_limits is None or self._upper_limits is None:
            return joint_angles
        return np.clip(joint_angles, self._lower_limits, self._upper_limits)

    @staticmethod
    def _quat_to_rot_matrix(quaternion: np.ndarray) -> np.ndarray:
        """Convert quaternion [x, y, z, w] to rotation matrix."""
        q = np.asarray(quaternion, dtype=float)
        if q.shape != (4,):
            raise ValueError('Quaternion must be shape (4,) as [x, y, z, w]')

        norm = np.linalg.norm(q)
        if norm == 0.0:
            raise ValueError('Quaternion must be non-zero')
        q = q / norm

        x, y, z, w = q
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )

    @staticmethod
    def _orientation_error(current_rot: np.ndarray, target_rot: np.ndarray) -> np.ndarray:
        """Compute small-angle orientation error vector from current to target."""
        return 0.5 * (
            np.cross(current_rot[:, 0], target_rot[:, 0])
            + np.cross(current_rot[:, 1], target_rot[:, 1])
            + np.cross(current_rot[:, 2], target_rot[:, 2])
        )

    def _task_error(self, joint_angles: np.ndarray, target_pose: np.ndarray) -> np.ndarray:
        """Build 6D task-space error [position(3), orientation(3)]."""
        transform = self._fk_solver.compute(joint_angles)
        current_pos = transform[:3, 3]
        current_rot = transform[:3, :3]

        target_pos = target_pose[:3]
        target_rot = self._quat_to_rot_matrix(target_pose[3:])

        pos_error = target_pos - current_pos
        ori_error = self._orientation_error(current_rot, target_rot)
        return np.concatenate([pos_error, ori_error])

    def _numerical_jacobian(self, joint_angles: np.ndarray, epsilon: float = 1e-6) -> np.ndarray:
        """Compute task-space Jacobian numerically for [position, orientation]."""
        if self._fk_solver is None:
            raise ValueError('FK solver is not initialized')

        base_transform = self._fk_solver.compute(joint_angles)
        base_pos = base_transform[:3, 3]
        base_rot = base_transform[:3, :3]

        jacobian = np.zeros((6, self._joint_count), dtype=float)
        for i in range(self._joint_count):
            perturbed = joint_angles.copy()
            perturbed[i] += epsilon
            perturbed_transform = self._fk_solver.compute(perturbed)
            perturbed_pos = perturbed_transform[:3, 3]
            perturbed_rot = perturbed_transform[:3, :3]

            jacobian[:3, i] = (perturbed_pos - base_pos) / epsilon
            jacobian[3:, i] = self._orientation_error(base_rot, perturbed_rot) / epsilon

        return jacobian

    def solve(
        self,
        target_pose: np.ndarray,
        initial_guess: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Solve inverse kinematics for target end-effector pose.

        Args:
            target_pose: Target pose (position and orientation)
            initial_guess: Optional current/seed joint state

        Returns:
            Joint angles if solution found, None otherwise
        """
        if self._fk_solver is None or self._joint_count == 0:
            raise ValueError('Solver requires valid DH parameters')

        target = np.asarray(target_pose, dtype=float)
        if target.shape != (7,):
            raise ValueError('target_pose must be shape (7,) as [x, y, z, qx, qy, qz, qw]')

        max_iterations = 200
        tolerance = 1e-4
        damping = 1e-2

        joint_angles = self._initial_joint_angles(initial_guess)

        for _ in range(max_iterations):
            error = self._task_error(joint_angles, target)
            if np.linalg.norm(error) < tolerance:
                return joint_angles

            jacobian = self._numerical_jacobian(joint_angles)
            jjt = jacobian @ jacobian.T
            damped_inverse = np.linalg.inv(
                jjt + (damping ** 2) * np.eye(jjt.shape[0], dtype=float)
            )
            delta_q = jacobian.T @ damped_inverse @ error

            delta_q = np.clip(delta_q, -self._max_step, self._max_step)
            joint_angles += delta_q
            joint_angles = self._clip_to_limits(joint_angles)

            if np.linalg.norm(delta_q) < 1e-7:
                break

        return None
