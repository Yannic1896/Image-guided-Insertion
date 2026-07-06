"""Lightweight joint-path safety checks for needle planning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from rnm_kinematics.fk_solver import dh_transform


@dataclass(frozen=True)
class SafetyCheckResult:
    """Result returned by a lightweight safety check."""

    ok: bool
    reason: str = ""


class LightweightCollisionChecker:
    """Conservative capsule-style checks using only DH forward kinematics."""

    def __init__(
        self,
        dh_params: Sequence[Sequence[float]],
        joint_count: int,
        min_self_distance: float,
        min_table_z: float,
        forbidden_spheres: Sequence[tuple[Sequence[float], float]],
    ) -> None:
        self._dh_params = np.asarray(dh_params, dtype=float)
        self._joint_count = int(joint_count)
        self._min_self_distance = float(min_self_distance)
        self._min_table_z = float(min_table_z)
        self._forbidden_spheres = [
            (np.asarray(center, dtype=float), float(radius))
            for center, radius in forbidden_spheres
        ]

    def validate(self, joint_positions: Sequence[float]) -> SafetyCheckResult:
        points = self._link_points(joint_positions)

        if np.any(points[:, 2] < self._min_table_z):
            min_z = float(np.min(points[:, 2]))
            return SafetyCheckResult(
                False,
                f"link point below table/floor limit: z={min_z:.3f}m",
            )

        if self._min_self_distance > 0.0:
            result = self._check_self_distance(points)
            if not result.ok:
                return result

        for center, radius in self._forbidden_spheres:
            for index in range(len(points) - 1):
                distance = _point_to_segment_distance(center, points[index], points[index + 1])
                if distance < radius:
                    return SafetyCheckResult(
                        False,
                        "link segment intersects forbidden sphere "
                        f"{center.tolist()} r={radius:.3f}m",
                    )

        return SafetyCheckResult(True)

    def _link_points(self, joint_positions: Sequence[float]) -> np.ndarray:
        joints = np.asarray(joint_positions, dtype=float)
        if joints.shape != (self._joint_count,):
            raise ValueError(
                f"Expected {self._joint_count} joints, got shape {joints.shape}."
            )

        transform = np.eye(4)
        points = [transform[:3, 3].copy()]
        for index, (d_value, a_value, alpha_value) in enumerate(self._dh_params):
            theta = joints[index] if index < self._joint_count else 0.0
            transform = transform @ dh_transform(theta, d_value, a_value, alpha_value)
            points.append(transform[:3, 3].copy())

        return np.asarray(points, dtype=float)

    def _check_self_distance(self, points: np.ndarray) -> SafetyCheckResult:
        segment_count = len(points) - 1
        for first in range(segment_count):
            for second in range(first + 3, segment_count):
                if first == 0 and second == segment_count - 1:
                    continue
                distance = _segment_distance(
                    points[first],
                    points[first + 1],
                    points[second],
                    points[second + 1],
                )
                if distance < self._min_self_distance:
                    return SafetyCheckResult(
                        False,
                        "self-collision clearance too small between link segments "
                        f"{first}-{first + 1} and {second}-{second + 1}: "
                        f"{distance:.3f}m",
                    )
        return SafetyCheckResult(True)


def joint_limit_margin(
    joint_positions: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
) -> float:
    """Return the smallest distance from any joint to either configured limit."""
    joints = np.asarray(joint_positions, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    return float(np.min(np.minimum(joints - lower, upper - joints)))


def parse_forbidden_spheres(
    flat_centers: Sequence[float],
    radii: Sequence[float],
) -> list[tuple[list[float], float]]:
    """Parse flat xyz centers and per-sphere radii from ROS parameters."""
    centers = list(flat_centers or [])
    sphere_radii = list(radii or [])
    if not centers and not sphere_radii:
        return []
    if len(centers) % 3 != 0:
        raise ValueError("collision_forbidden_sphere_centers must be xyz triples.")
    sphere_count = len(centers) // 3
    if len(sphere_radii) != sphere_count:
        raise ValueError(
            "collision_forbidden_sphere_radii length must match the number of centers."
        )
    return [
        (centers[index * 3 : index * 3 + 3], sphere_radii[index])
        for index in range(sphere_count)
    ]


def _point_to_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    segment = end - start
    length_sq = float(np.dot(segment, segment))
    if length_sq == 0.0:
        return float(np.linalg.norm(point - start))
    alpha = np.clip(float(np.dot(point - start, segment) / length_sq), 0.0, 1.0)
    projection = start + alpha * segment
    return float(np.linalg.norm(point - projection))


def _segment_distance(
    p1: np.ndarray,
    q1: np.ndarray,
    p2: np.ndarray,
    q2: np.ndarray,
) -> float:
    """Shortest distance between two 3D line segments."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))

    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(p1 - p2))
    if a <= 1e-12:
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0)
    else:
        c = float(np.dot(d1, r))
        if e <= 1e-12:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(np.dot(d1, d2))
            denominator = a * e - b * b
            if denominator != 0.0:
                s = np.clip((b * f - c * e) / denominator, 0.0, 1.0)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)

    closest_1 = p1 + d1 * s
    closest_2 = p2 + d2 * t
    return float(np.linalg.norm(closest_1 - closest_2))
