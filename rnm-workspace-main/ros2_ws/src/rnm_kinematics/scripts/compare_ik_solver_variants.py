#!/usr/bin/env python3
"""Compare IK solver variants without starting ROS 2 or the simulator."""

from __future__ import annotations

import argparse
import ast
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rnm_kinematics.fk_solver import (  # noqa: E402
    ForwardKinematicsSolver,
    quaternion_from_rotation_matrix,
)


DEFAULT_TARGET = "0.328,0.105,0.481,0.924,-0.253,0.121,0.260"
DEFAULT_INITIAL_GUESS = "0.0,-0.785,0.0,-2.356,0.0,1.571,0.785"
VARIANT_ORDER = (
    "no_fixes",
    "joint_clamp",
    "joint_clamp_delta",
    "full_fixes",
)


@dataclass(frozen=True)
class SolverSettings:
    max_iterations: int
    tolerance: float
    damping: float
    step_scale: float
    max_step: float


@dataclass(frozen=True)
class SolverVariant:
    name: str
    description: str
    use_joint_limits: bool
    use_delta_clamp: bool
    use_initial_guess: bool
    wrap_to_pi: bool


@dataclass
class SolverResult:
    variant: SolverVariant
    joint_angles: np.ndarray
    iterations: int
    task_error_norm: float
    converged: bool


def parse_vector(text: str, expected_size: int, name: str) -> np.ndarray:
    """Parse a comma-separated or space-separated vector."""
    values = text.replace(",", " ").split()
    if len(values) != expected_size:
        raise ValueError(
            f"{name} must contain {expected_size} values, got {len(values)}."
        )
    return np.array([float(value) for value in values], dtype=float)


def read_yaml_array(path: Path, key: str) -> list[float]:
    """Read a simple one-line YAML array, without requiring PyYAML."""
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{key}:"):
            value_text = stripped.split(":", 1)[1].strip()
            return [float(value) for value in ast.literal_eval(value_text)]
    raise KeyError(f"Could not find '{key}' in {path}")


def load_dh_table(config_dir: Path) -> np.ndarray:
    """Load the package Panda DH table from panda_dh.yaml."""
    dh_path = config_dir / "panda_dh.yaml"
    d_values = read_yaml_array(dh_path, "dh_d")
    a_values = read_yaml_array(dh_path, "dh_a")
    alpha_values = read_yaml_array(dh_path, "dh_alpha")
    if not (len(d_values) == len(a_values) == len(alpha_values)):
        raise ValueError("DH arrays in panda_dh.yaml must have matching lengths.")
    return np.array([d_values, a_values, alpha_values], dtype=float).T


def load_joint_limits(config_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load Panda lower and upper joint limits from panda_joint_limits.yaml."""
    limits_path = config_dir / "panda_joint_limits.yaml"
    lower = np.array(read_yaml_array(limits_path, "joint_limit_lower"), dtype=float)
    upper = np.array(read_yaml_array(limits_path, "joint_limit_upper"), dtype=float)
    if lower.shape != upper.shape:
        raise ValueError("Joint lower/upper limit arrays must match.")
    return lower, upper


def infer_joint_count(dh_table: np.ndarray) -> int:
    """Infer actuated joint count using the same flange-row heuristic."""
    if (
        dh_table.shape[0] > 1
        and np.isclose(dh_table[-1, 1], 0.0)
        and np.isclose(dh_table[-1, 2], 0.0)
    ):
        return int(dh_table.shape[0] - 1)
    return int(dh_table.shape[0])


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert quaternion [x, y, z, w] to a rotation matrix."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise ValueError("Target quaternion must be non-zero.")
    x, y, z, w = q / norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


def orientation_error(current_rot: np.ndarray, target_rot: np.ndarray) -> np.ndarray:
    """Compute the small-angle orientation error vector."""
    return 0.5 * (
        np.cross(current_rot[:, 0], target_rot[:, 0])
        + np.cross(current_rot[:, 1], target_rot[:, 1])
        + np.cross(current_rot[:, 2], target_rot[:, 2])
    )


class VariantRunner:
    """Run solver variants against the same FK model and target."""

    def __init__(
        self,
        dh_table: np.ndarray,
        lower_limits: np.ndarray,
        upper_limits: np.ndarray,
        settings: SolverSettings,
    ) -> None:
        self._fk_solver = ForwardKinematicsSolver(dh_table)
        self._joint_count = infer_joint_count(dh_table)
        self._lower_limits = lower_limits
        self._upper_limits = upper_limits
        self._settings = settings

        if lower_limits.shape != (self._joint_count,):
            raise ValueError(
                "Joint limit arrays must match the actuated joint count "
                f"{self._joint_count}."
            )

    def run(
        self,
        variant: SolverVariant,
        target_pose: np.ndarray,
        initial_guess: np.ndarray,
    ) -> SolverResult:
        """Run one variant and return its final joint vector and metrics."""
        if variant.use_initial_guess:
            joint_angles = np.clip(
                initial_guess.copy(),
                self._lower_limits,
                self._upper_limits,
            )
        else:
            joint_angles = np.zeros(self._joint_count, dtype=float)

        for iteration in range(self._settings.max_iterations):
            task_error = self.task_error(joint_angles, target_pose)
            task_error_norm = float(np.linalg.norm(task_error))
            if task_error_norm < self._settings.tolerance:
                return SolverResult(
                    variant=variant,
                    joint_angles=joint_angles,
                    iterations=iteration,
                    task_error_norm=task_error_norm,
                    converged=True,
                )

            jacobian = self.numerical_jacobian(joint_angles)
            jjt = jacobian @ jacobian.T
            damped_inverse = np.linalg.inv(
                jjt
                + (self._settings.damping**2) * np.eye(jjt.shape[0], dtype=float)
            )
            delta_q = jacobian.T @ damped_inverse @ task_error

            if variant.use_delta_clamp:
                delta_q = np.clip(
                    delta_q,
                    -self._settings.max_step,
                    self._settings.max_step,
                )
                joint_angles += delta_q
            else:
                joint_angles += self._settings.step_scale * delta_q

            if variant.use_joint_limits:
                joint_angles = np.clip(
                    joint_angles,
                    self._lower_limits,
                    self._upper_limits,
                )
            elif variant.wrap_to_pi:
                joint_angles = (joint_angles + np.pi) % (2.0 * np.pi) - np.pi

            if np.linalg.norm(delta_q) < 1e-7:
                break

        task_error_norm = float(np.linalg.norm(self.task_error(joint_angles, target_pose)))
        return SolverResult(
            variant=variant,
            joint_angles=joint_angles,
            iterations=self._settings.max_iterations,
            task_error_norm=task_error_norm,
            converged=False,
        )

    def task_error(self, joint_angles: np.ndarray, target_pose: np.ndarray) -> np.ndarray:
        """Build 6D task-space error [position(3), orientation(3)]."""
        transform = self._fk_solver.compute(joint_angles)
        current_pos = transform[:3, 3]
        current_rot = transform[:3, :3]
        target_pos = target_pose[:3]
        target_rot = quaternion_to_rotation_matrix(target_pose[3:])
        return np.concatenate(
            [
                target_pos - current_pos,
                orientation_error(current_rot, target_rot),
            ]
        )

    def numerical_jacobian(
        self,
        joint_angles: np.ndarray,
        epsilon: float = 1e-6,
    ) -> np.ndarray:
        """Compute a numerical task-space Jacobian."""
        base_transform = self._fk_solver.compute(joint_angles)
        base_pos = base_transform[:3, 3]
        base_rot = base_transform[:3, :3]

        jacobian = np.zeros((6, self._joint_count), dtype=float)
        for index in range(self._joint_count):
            perturbed = joint_angles.copy()
            perturbed[index] += epsilon
            perturbed_transform = self._fk_solver.compute(perturbed)
            perturbed_pos = perturbed_transform[:3, 3]
            perturbed_rot = perturbed_transform[:3, :3]

            jacobian[:3, index] = (perturbed_pos - base_pos) / epsilon
            jacobian[3:, index] = (
                orientation_error(base_rot, perturbed_rot) / epsilon
            )

        return jacobian

    def pose_metrics(
        self,
        joint_angles: np.ndarray,
        target_pose: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float, float]:
        """Return FK pose plus position/orientation errors."""
        transform = self._fk_solver.compute(joint_angles)
        position = transform[:3, 3]
        quaternion = quaternion_from_rotation_matrix(transform[:3, :3])

        target_quaternion = target_pose[3:] / np.linalg.norm(target_pose[3:])
        quat_dot = abs(float(np.dot(quaternion, target_quaternion)))
        quat_dot = min(1.0, max(-1.0, quat_dot))
        orientation_error_rad = 2.0 * np.arccos(quat_dot)
        position_error_m = float(np.linalg.norm(target_pose[:3] - position))

        return position, quaternion, position_error_m, float(orientation_error_rad)


def build_variants() -> dict[str, SolverVariant]:
    """Return supported comparison variants."""
    variants = [
        SolverVariant(
            name="no_fixes",
            description="old zero start, no joint limits, no max-step clamp",
            use_joint_limits=False,
            use_delta_clamp=False,
            use_initial_guess=False,
            wrap_to_pi=True,
        ),
        SolverVariant(
            name="joint_clamp",
            description="zero start, Panda joint-limit clipping only",
            use_joint_limits=True,
            use_delta_clamp=False,
            use_initial_guess=False,
            wrap_to_pi=False,
        ),
        SolverVariant(
            name="joint_clamp_delta",
            description="zero start, Panda joint limits plus delta_q max-step clamp",
            use_joint_limits=True,
            use_delta_clamp=True,
            use_initial_guess=False,
            wrap_to_pi=False,
        ),
        SolverVariant(
            name="full_fixes",
            description="initial guess, Panda joint limits, delta_q max-step clamp",
            use_joint_limits=True,
            use_delta_clamp=True,
            use_initial_guess=True,
            wrap_to_pi=False,
        ),
    ]
    return {variant.name: variant for variant in variants}


def format_vector(vector: np.ndarray, precision: int = 6) -> str:
    """Format a vector compactly for terminal output."""
    return "[" + ", ".join(f"{value:.{precision}f}" for value in vector) + "]"


def print_result(
    result: SolverResult,
    runner: VariantRunner,
    target_pose: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
) -> None:
    """Print one variant result."""
    position, quaternion, pos_error_m, ori_error_rad = runner.pose_metrics(
        result.joint_angles,
        target_pose,
    )
    within_limits = bool(
        np.all(result.joint_angles >= lower_limits)
        and np.all(result.joint_angles <= upper_limits)
    )

    print(f"\n=== {result.variant.name} ===")
    print(result.variant.description)
    print(f"status: {'converged' if result.converged else 'not converged'}")
    print(f"iterations: {result.iterations}")
    print(f"task_error_norm: {result.task_error_norm:.9g}")
    print(f"within_panda_limits: {within_limits}")
    print(f"joint_angles: {format_vector(result.joint_angles)}")
    print(f"fk_position: {format_vector(position)}")
    print(f"fk_quaternion_xyzw: {format_vector(quaternion)}")
    print(f"position_error_m: {pos_error_m:.9g}")
    print(f"orientation_error_deg: {np.degrees(ori_error_rad):.9g}")

    if not within_limits:
        low_violation = np.minimum(result.joint_angles - lower_limits, 0.0)
        high_violation = np.maximum(result.joint_angles - upper_limits, 0.0)
        print(f"limit_violation_low: {format_vector(low_violation)}")
        print(f"limit_violation_high: {format_vector(high_violation)}")


def make_parser() -> argparse.ArgumentParser:
    """Build CLI parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Compare Panda IK solver variants using only math. "
            "No ROS nodes, topics, controllers, or simulator required."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("all", *VARIANT_ORDER),
        default="all",
        help="Which solver variant to run.",
    )
    parser.add_argument(
        "--target",
        default=DEFAULT_TARGET,
        help=(
            "Target pose as x,y,z,qx,qy,qz,qw. "
            f"Default: {DEFAULT_TARGET}"
        ),
    )
    parser.add_argument(
        "--initial",
        default=DEFAULT_INITIAL_GUESS,
        help=(
            "Initial/current joint state for full_fixes as seven comma-separated "
            f"joint values. Default: {DEFAULT_INITIAL_GUESS}"
        ),
    )
    parser.add_argument(
        "--max-step",
        type=float,
        default=0.05,
        help="Max per-iteration delta_q for variants using delta clamp.",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=200,
        help="Maximum IK iterations.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1e-4,
        help="Task-space convergence tolerance.",
    )
    parser.add_argument(
        "--damping",
        type=float,
        default=1e-2,
        help="Damped least-squares damping value.",
    )
    parser.add_argument(
        "--step-scale",
        type=float,
        default=0.5,
        help="Old solver step scale for variants without delta_q max-step clamp.",
    )
    parser.add_argument(
        "--config-dir",
        type=Path,
        default=PACKAGE_ROOT / "config",
        help="Directory containing panda_dh.yaml and panda_joint_limits.yaml.",
    )
    return parser


def main() -> None:
    """Run the requested comparison."""
    parser = make_parser()
    args = parser.parse_args()

    target_pose = parse_vector(args.target, 7, "target")
    initial_guess = parse_vector(args.initial, 7, "initial")
    settings = SolverSettings(
        max_iterations=args.max_iterations,
        tolerance=args.tolerance,
        damping=args.damping,
        step_scale=args.step_scale,
        max_step=args.max_step,
    )

    dh_table = load_dh_table(args.config_dir)
    lower_limits, upper_limits = load_joint_limits(args.config_dir)
    runner = VariantRunner(dh_table, lower_limits, upper_limits, settings)

    variants = build_variants()
    selected_names = VARIANT_ORDER if args.mode == "all" else (args.mode,)

    print("Target pose [x, y, z, qx, qy, qz, qw]:")
    print(format_vector(target_pose))
    print("Initial/current guess used by full_fixes:")
    print(format_vector(initial_guess))
    print(f"max_step: {args.max_step}")

    for name in selected_names:
        result = runner.run(variants[name], target_pose, initial_guess)
        print_result(result, runner, target_pose, lower_limits, upper_limits)


if __name__ == "__main__":
    main()
