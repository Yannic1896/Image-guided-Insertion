"""Small geometry helpers shared by mapping tools."""

from __future__ import annotations

import math

import numpy as np


class PoseSample:
    """Pose sample represented as target-frame transform data."""

    def __init__(
        self,
        stamp_sec: float,
        translation: np.ndarray,
        quaternion: np.ndarray,
        rotation: np.ndarray,
    ) -> None:
        self.stamp_sec = stamp_sec
        self.translation = translation
        self.quaternion = quaternion
        self.rotation = rotation


def unit_vector(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm <= 1e-12:
        raise RuntimeError("Cannot normalize a zero-length vector.")
    return vector / norm


def unit_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quaternion)
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion / norm


def checked_unit_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        raise ValueError("Received a zero-length TF quaternion.")
    return quaternion / norm


def rotation_matrix_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    x_value, y_value, z_value, w_value = quaternion
    xx_value = x_value * x_value
    yy_value = y_value * y_value
    zz_value = z_value * z_value
    xy_value = x_value * y_value
    xz_value = x_value * z_value
    yz_value = y_value * z_value
    wx_value = w_value * x_value
    wy_value = w_value * y_value
    wz_value = w_value * z_value

    return np.array(
        [
            [
                1.0 - 2.0 * (yy_value + zz_value),
                2.0 * (xy_value - wz_value),
                2.0 * (xz_value + wy_value),
            ],
            [
                2.0 * (xy_value + wz_value),
                1.0 - 2.0 * (xx_value + zz_value),
                2.0 * (yz_value - wx_value),
            ],
            [
                2.0 * (xz_value - wy_value),
                2.0 * (yz_value + wx_value),
                1.0 - 2.0 * (xx_value + yy_value),
            ],
        ],
        dtype=np.float64,
    )


def quaternion_angle(first: np.ndarray, second: np.ndarray) -> float:
    dot = abs(float(np.dot(first, second)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def pose_sample_from_transform(transform, stamp_sec: float) -> PoseSample:
    translation = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ],
        dtype=np.float64,
    )
    quaternion = np.array(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ],
        dtype=np.float64,
    )
    quaternion = checked_unit_quaternion(quaternion)
    return PoseSample(
        stamp_sec=stamp_sec,
        translation=translation,
        quaternion=quaternion,
        rotation=rotation_matrix_from_quaternion(quaternion),
    )


def transform_points(points: np.ndarray, pose_sample: PoseSample) -> np.ndarray:
    return points @ pose_sample.rotation.T + pose_sample.translation


def transform_points_homogeneous(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def transform_point_homogeneous(point: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return matrix[:3, :3] @ point + matrix[:3, 3]


def matrix_from_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def orthonormal_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    helper = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(np.dot(helper, normal))) > 0.9:
        helper = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    basis_u = unit_vector(np.cross(normal, helper))
    basis_v = unit_vector(np.cross(normal, basis_u))
    return basis_u, basis_v


def quaternion_from_z_axis(z_axis: np.ndarray) -> np.ndarray:
    z_axis = unit_vector(z_axis)
    helper = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(helper, z_axis))) > 0.95:
        helper = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    x_axis = unit_vector(np.cross(helper, z_axis))
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    return quaternion_from_matrix(rotation)


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ],
            dtype=np.float64,
        )

    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        scale = (
            np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            * 2.0
        )
        quat = np.array(
            [
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ],
            dtype=np.float64,
        )
    elif index == 1:
        scale = (
            np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            * 2.0
        )
        quat = np.array(
            [
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ],
            dtype=np.float64,
        )
    else:
        scale = (
            np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            * 2.0
        )
        quat = np.array(
            [
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ],
            dtype=np.float64,
        )
    return unit_quaternion(quat)


def sphere_points(
    center: np.ndarray,
    radius: float,
    phi_count: int = 16,
    theta_count: int = 32,
) -> np.ndarray:
    phi_values = np.linspace(0.0, np.pi, phi_count)
    theta_values = np.linspace(0.0, 2.0 * np.pi, theta_count, endpoint=False)
    points = []
    for phi in phi_values:
        for theta in theta_values:
            points.append(
                [
                    center[0] + radius * np.sin(phi) * np.cos(theta),
                    center[1] + radius * np.sin(phi) * np.sin(theta),
                    center[2] + radius * np.cos(phi),
                ]
            )
    return np.asarray(points, dtype=np.float64)


def line_points(start: np.ndarray, end: np.ndarray, count: int) -> np.ndarray:
    weights = np.linspace(0.0, 1.0, count).reshape(-1, 1)
    return start.reshape(1, 3) * (1.0 - weights) + end.reshape(1, 3) * weights


def limit_points(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def vector_string(vector: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:.6f}" for value in vector) + "]"
