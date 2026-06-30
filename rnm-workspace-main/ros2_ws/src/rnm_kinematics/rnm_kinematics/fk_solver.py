"""Forward kinematics utilities for the Franka Panda modified-DH model."""

from __future__ import annotations

from typing import Sequence

import numpy as np


class ForwardKinematicsSolver:
    """Compute the end-effector transform from a modified-DH table."""

    def __init__(self, dh_params: Sequence[Sequence[float]]) -> None:
        dh_matrix = np.asarray(dh_params, dtype=float)
        if dh_matrix.ndim != 2 or dh_matrix.shape[1] != 3:
            raise ValueError("DH parameters must be an Nx3 table of [d, a, alpha].")

        self._dh_params = dh_matrix

    @property
    def row_count(self) -> int:
        """Return the number of DH rows, including static tool/flange rows."""
        return int(self._dh_params.shape[0])

    def compute(self, joint_positions: Sequence[float]) -> np.ndarray:
        """
        Compute the homogeneous transform from base frame to end-effector frame.

        Dynamic rows consume the provided joint positions in order. Any remaining
        DH rows are treated as fixed transforms with theta = 0.0, which matches
        the Panda flange row in the provided DH table.
        """
        joint_values = np.asarray(joint_positions, dtype=float)
        if joint_values.ndim != 1:
            raise ValueError("Joint positions must be a one-dimensional sequence.")
        if joint_values.size > self.row_count:
            raise ValueError(
                f"Received {joint_values.size} joint values for {self.row_count} DH rows."
            )

        transform = np.eye(4)
        for index, (d_value, a_value, alpha_value) in enumerate(self._dh_params):
            theta = joint_values[index] if index < joint_values.size else 0.0
            transform = transform @ dh_transform(theta, d_value, a_value, alpha_value)

        return transform


def dh_transform(theta: float, d_value: float, a_value: float, alpha: float) -> np.ndarray:
    """Build one modified-DH transform matrix."""
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    return np.array(
        [
            [
                cos_theta,
                -sin_theta,
                0.0,
                a_value,
            ],
            [
                sin_theta * cos_alpha,
                cos_theta * cos_alpha,
                -sin_alpha,
                -d_value * sin_alpha,
            ],
            [
                sin_theta * sin_alpha,
                cos_theta * sin_alpha,
                cos_alpha,
                d_value * cos_alpha,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to [x, y, z, w]."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3.")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ],
            dtype=float,
        )
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        quat = np.array(
            [
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ],
            dtype=float,
        )
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        quat = np.array(
            [
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ],
            dtype=float,
        )
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        quat = np.array(
            [
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ],
            dtype=float,
        )

    norm = np.linalg.norm(quat)
    if norm == 0.0:
        raise ValueError("Rotation matrix produced a zero quaternion.")
    return quat / norm
