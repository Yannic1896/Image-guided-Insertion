"""Math helpers for camera calibration transforms."""

from __future__ import annotations

import numpy as np


def invert_rigid_transform(rotation, translation):
    """Return the inverse of a 3D rigid transform."""
    rotation_matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    translation_vector = np.asarray(translation, dtype=float).reshape(3)

    inverse_rotation = rotation_matrix.T
    inverse_translation = -inverse_rotation @ translation_vector
    return inverse_rotation, inverse_translation


def average_rotation_matrices(rotations):
    """Project the arithmetic mean of rotation matrices back onto SO(3)."""
    if not rotations:
        raise ValueError("At least one rotation matrix is required.")

    mean_rotation = np.zeros((3, 3), dtype=float)
    for rotation in rotations:
        mean_rotation += np.asarray(rotation, dtype=float).reshape(3, 3)

    u_matrix, _, vt_matrix = np.linalg.svd(mean_rotation)
    averaged = u_matrix @ vt_matrix
    if np.linalg.det(averaged) < 0.0:
        u_matrix[:, -1] *= -1.0
        averaged = u_matrix @ vt_matrix
    return averaged


def rotation_matrix_to_quaternion(rotation):
    """Convert a 3x3 rotation matrix to an x, y, z, w quaternion."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = np.trace(matrix)

    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w_value = 0.25 * scale
        x_value = (matrix[2, 1] - matrix[1, 2]) / scale
        y_value = (matrix[0, 2] - matrix[2, 0]) / scale
        z_value = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w_value = (matrix[2, 1] - matrix[1, 2]) / scale
        x_value = 0.25 * scale
        y_value = (matrix[0, 1] + matrix[1, 0]) / scale
        z_value = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w_value = (matrix[0, 2] - matrix[2, 0]) / scale
        x_value = (matrix[0, 1] + matrix[1, 0]) / scale
        y_value = 0.25 * scale
        z_value = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w_value = (matrix[1, 0] - matrix[0, 1]) / scale
        x_value = (matrix[0, 2] + matrix[2, 0]) / scale
        y_value = (matrix[1, 2] + matrix[2, 1]) / scale
        z_value = 0.25 * scale

    quaternion = np.array([x_value, y_value, z_value, w_value], dtype=float)
    return quaternion / np.linalg.norm(quaternion)
