"""Plan and publish a straight fixed-orientation needle insertion trajectory."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from tf2_ros import TransformBroadcaster

from rnm_kinematics.fk_solver import quaternion_from_rotation_matrix
from rnm_kinematics.ik_solver import IKSolver


DEFAULT_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
VECTOR_PATTERN = re.compile(r"^\s*([A-Za-z0-9_]+)\s*:\s*\[([^\]]+)\]")
SCALAR_PATTERN = re.compile(r"^\s*([A-Za-z0-9_]+)\s*:\s*([-+0-9.eE]+)")
SECTION_PATTERN = re.compile(r"^\s*\[([A-Za-z0-9_]+)\]\s*$")


@dataclass(frozen=True)
class NeedlePlan:
    """Resolved needle path geometry and fixed tool orientation."""

    entry: np.ndarray
    target: np.ndarray
    axis: np.ndarray
    rotation: np.ndarray
    quaternion: np.ndarray
    pre_entry: np.ndarray
    waypoints: list[np.ndarray]


class InsertionNode(Node):
    """Align the needle tip with an entry-target axis and publish joint trajectory."""

    def __init__(self) -> None:
        super().__init__("insertion_node")

        self._declare_parameters()

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._end_effector_frame = str(
            self.get_parameter("end_effector_frame").value
        )
        self._needle_tip_frame = str(self.get_parameter("needle_tip_frame").value)
        self._joint_names = [
            str(name) for name in list(self.get_parameter("joint_names").value or [])
        ]
        self._needle_axis_ee = self._unit_vector(
            self._array_parameter("needle_axis_in_ee_frame"),
            "needle_axis_in_ee_frame",
        )
        self._needle_tip_offset_m = float(
            self.get_parameter("needle_tip_offset_m").value
        )
        self._cartesian_step_m = float(self.get_parameter("cartesian_step_m").value)
        self._approach_distance_m = float(
            self.get_parameter("approach_distance_m").value
        )
        self._trajectory_rate_hz = float(self.get_parameter("trajectory_rate_hz").value)
        self._approach_speed_m_s = float(
            self.get_parameter("approach_speed_m_s").value
        )
        self._insertion_speed_m_s = float(
            self.get_parameter("insertion_speed_m_s").value
        )
        self._joint_transition_speed_rad_s = float(
            self.get_parameter("joint_transition_speed_rad_s").value
        )
        self._max_joint_speed_rad_s = float(
            self.get_parameter("max_joint_speed_rad_s").value
        )
        self._max_path_output_points = int(
            self.get_parameter("max_path_output_points").value
        )

        self._latest_joint_positions: Optional[np.ndarray] = None
        self._missing_joint_state_logged = False
        self._waiting_for_joint_state_logged = False
        self._planned_commands: list[np.ndarray] = []
        self._trajectory_published = False
        self._plan: Optional[NeedlePlan] = None

        self._ik_solver = IKSolver(
            dh_params=self._load_dh_params(),
            joint_lower_limits=np.array(
                self._array_parameter("joint_limit_lower"), dtype=float
            ),
            joint_upper_limits=np.array(
                self._array_parameter("joint_limit_upper"), dtype=float
            ),
            max_step=float(self.get_parameter("max_step").value),
        )

        self._command_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("joint_trajectory_topic").value),
            10,
        )
        self._entry_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("entry_pose_topic").value),
            10,
        )
        self._pre_entry_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("pre_entry_pose_topic").value),
            10,
        )
        self._target_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("target_pose_topic").value),
            10,
        )
        self._path_pub = self.create_publisher(
            PoseArray,
            str(self.get_parameter("planned_tip_path_topic").value),
            10,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_states_callback,
            10,
        )

        self.create_timer(1.0, self._publish_static_plan_outputs)
        self.create_timer(0.2, self._publish_needle_tip_tf)

        self.get_logger().info(
            "Needle calibration node ready. It will read target/entry files, "
            "align EE +Z with the insertion axis, validate IK, and publish a "
            "full fixed-orientation joint trajectory on launch."
        )

        self._build_plan_from_files()

    def _declare_parameters(self) -> None:
        self.declare_parameter("target_file", "target_output/target_location.txt")
        self.declare_parameter(
            "entry_file", "entry_point_output/needle_entry_point.txt"
        )
        self.declare_parameter("target_key", "scan_center_m")
        self.declare_parameter("candidate_id", 0)
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("end_effector_frame", "panda_fk_flange")
        self.declare_parameter("needle_tip_frame", "needle_tip")
        self.declare_parameter("needle_tip_offset_m", 0.16)
        self.declare_parameter("needle_axis_in_ee_frame", [0.0, 0.0, 1.0])
        self.declare_parameter("approach_distance_m", 0.03)
        self.declare_parameter("cartesian_step_m", 0.002)
        self.declare_parameter("trajectory_rate_hz", 1000.0)
        self.declare_parameter("approach_speed_m_s", 0.03)
        self.declare_parameter("insertion_speed_m_s", 0.005)
        self.declare_parameter("joint_transition_speed_rad_s", 0.5)
        self.declare_parameter("max_joint_speed_rad_s", 1.0)
        self.declare_parameter("max_path_output_points", 1000)
        self.declare_parameter(
            "joint_trajectory_topic",
            "/joint_position_example_controller/joint_trajectory_command",
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("entry_pose_topic", "/insertion/entry_pose")
        self.declare_parameter(
            "pre_entry_pose_topic",
            "/insertion/pre_entry_pose",
        )
        self.declare_parameter("target_pose_topic", "/insertion/target_pose")
        self.declare_parameter(
            "planned_tip_path_topic",
            "/insertion/planned_tip_path",
        )
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("dh_d", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("dh_a", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("dh_alpha", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_limit_lower", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_limit_upper", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("max_step", 0.05)

    def _build_plan_from_files(self) -> None:
        try:
            target_path = self._resolve_file(
                str(self.get_parameter("target_file").value)
            )
            entry_path = self._resolve_file(str(self.get_parameter("entry_file").value))
            target_key = str(self.get_parameter("target_key").value)
            candidate_id = int(self.get_parameter("candidate_id").value)

            target_values = self._parse_key_values(target_path)
            candidate_values = self._parse_key_values(
                entry_path,
                section=f"candidate_{candidate_id}",
            )

            target = np.array(target_values[target_key], dtype=float)
            entry = np.array(candidate_values["entry_point_m"], dtype=float)
            axis_from_file = np.array(
                candidate_values.get("insertion_axis_entry_to_target", []),
                dtype=float,
            )

            self._plan = self._make_plan(entry=entry, target=target)
            self._compare_file_axis(axis_from_file, self._plan.axis)
            self._log_state(
                "planned",
                f"Loaded target from {target_path} and candidate_{candidate_id} "
                f"from {entry_path}. Path length: "
                f"{np.linalg.norm(target - entry):.4f} m.",
            )
            self._publish_static_plan_outputs()
            self._validate_or_execute_plan()
        except Exception as exc:  # noqa: BLE001 - keep ROS node alive with status.
            self._log_state("error", f"Failed to build needle plan: {exc}")

    def _make_plan(self, entry: np.ndarray, target: np.ndarray) -> NeedlePlan:
        if entry.shape != (3,) or target.shape != (3,):
            raise ValueError("entry and target points must each have three values")

        delta = target - entry
        path_length = float(np.linalg.norm(delta))
        if path_length <= 0.0:
            raise ValueError("entry and target are identical")
        if self._cartesian_step_m <= 0.0:
            raise ValueError("cartesian_step_m must be positive")
        if self._trajectory_rate_hz <= 0.0:
            raise ValueError("trajectory_rate_hz must be positive")
        if self._approach_speed_m_s <= 0.0:
            raise ValueError("approach_speed_m_s must be positive")
        if self._insertion_speed_m_s <= 0.0:
            raise ValueError("insertion_speed_m_s must be positive")
        if self._joint_transition_speed_rad_s <= 0.0:
            raise ValueError("joint_transition_speed_rad_s must be positive")
        if self._max_joint_speed_rad_s <= 0.0:
            raise ValueError("max_joint_speed_rad_s must be positive")
        if self._max_path_output_points <= 0:
            raise ValueError("max_path_output_points must be positive")

        axis = delta / path_length
        rotation = self._rotation_with_local_axis_aligned(
            local_axis=self._needle_axis_ee,
            world_axis=axis,
        )
        quaternion = quaternion_from_rotation_matrix(rotation)
        pre_entry = entry - axis * self._approach_distance_m
        waypoints = self._linear_waypoints(
            pre_entry,
            entry,
            self._approach_speed_m_s,
        )
        waypoints.extend(
            self._linear_waypoints(entry, target, self._insertion_speed_m_s)[1:]
        )

        return NeedlePlan(
            entry=entry,
            target=target,
            axis=axis,
            rotation=rotation,
            quaternion=quaternion,
            pre_entry=pre_entry,
            waypoints=waypoints,
        )

    def _validate_or_execute_plan(self) -> None:
        if self._plan is None:
            return
        if self._trajectory_published:
            return
        if self._latest_joint_positions is None:
            if not self._waiting_for_joint_state_logged:
                self.get_logger().info(
                    "Plan is ready; waiting for /joint_states before IK validation."
                )
                self._waiting_for_joint_state_logged = True
            return

        insertion_commands = self._solve_waypoint_commands(self._plan)
        if not insertion_commands:
            return

        commands = self._with_joint_transition(insertion_commands)
        if not self._validate_joint_steps(commands):
            return

        self._planned_commands = commands
        self._log_state(
            "validated",
            f"IK validated {len(insertion_commands)} fixed-orientation insertion "
            f"samples and prepended {len(commands) - len(insertion_commands)} "
            "joint-transition samples.",
        )
        self._publish_full_joint_trajectory(commands)

    def _solve_waypoint_commands(self, plan: NeedlePlan) -> list[np.ndarray]:
        commands: list[np.ndarray] = []
        initial_guess = self._latest_joint_positions
        assert initial_guess is not None

        for index, tip_position in enumerate(plan.waypoints):
            ee_pose = self._ee_pose_from_tip_pose(tip_position, plan)
            solution = self._ik_solver.solve(ee_pose, initial_guess=initial_guess)
            if solution is None:
                self._log_state(
                    "ik_failed",
                    f"IK failed at waypoint {index + 1}/{len(plan.waypoints)}.",
                )
                return []
            commands.append(solution)
            initial_guess = solution

        return commands

    def _with_joint_transition(
        self,
        insertion_commands: list[np.ndarray],
    ) -> list[np.ndarray]:
        if not insertion_commands:
            return []
        assert self._latest_joint_positions is not None

        start = self._latest_joint_positions
        goal = insertion_commands[0]
        delta = goal - start
        max_step = self._joint_transition_speed_rad_s / self._trajectory_rate_hz
        step_count = max(1, int(math.ceil(float(np.max(np.abs(delta))) / max_step)))

        transition = [
            start + delta * (index / step_count)
            for index in range(step_count + 1)
        ]
        return transition + insertion_commands[1:]

    def _validate_joint_steps(self, commands: list[np.ndarray]) -> bool:
        if len(commands) < 2:
            return True

        max_allowed_step = self._max_joint_speed_rad_s / self._trajectory_rate_hz
        worst_step = 0.0
        worst_index = 0
        worst_joint = 0

        for index in range(1, len(commands)):
            deltas = np.abs(commands[index] - commands[index - 1])
            joint = int(np.argmax(deltas))
            step = float(deltas[joint])
            if step > worst_step:
                worst_step = step
                worst_index = index
                worst_joint = joint

        if worst_step > max_allowed_step:
            self._log_state(
                "blocked",
                "Joint trajectory exceeds configured speed limit: "
                f"sample {worst_index}, joint {worst_joint + 1}, "
                f"step {worst_step:.6f} rad > {max_allowed_step:.6f} rad "
                f"at {self._trajectory_rate_hz:.1f} Hz.",
            )
            return False

        self._log_state(
            "checked",
            f"Max joint step is {worst_step:.6f} rad "
            f"({worst_step * self._trajectory_rate_hz:.3f} rad/s).",
        )
        return True

    def _ee_pose_from_tip_pose(self, tip_position: np.ndarray, plan: NeedlePlan) -> np.ndarray:
        offset_world = plan.rotation @ (
            self._needle_axis_ee * self._needle_tip_offset_m
        )
        ee_position = tip_position - offset_world
        return np.array(
            [
                ee_position[0],
                ee_position[1],
                ee_position[2],
                plan.quaternion[0],
                plan.quaternion[1],
                plan.quaternion[2],
                plan.quaternion[3],
            ],
            dtype=float,
        )

    def _publish_full_joint_trajectory(self, commands: list[np.ndarray]) -> None:
        if not commands:
            self._log_state("blocked", "No validated joint trajectory to publish.")
            return

        command_msg = Float64MultiArray()
        point_count = len(commands)
        joint_count = len(self._joint_names)
        command_msg.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(
                    label="points",
                    size=point_count,
                    stride=point_count * joint_count,
                ),
                MultiArrayDimension(
                    label="joints",
                    size=joint_count,
                    stride=joint_count,
                ),
            ],
            data_offset=0,
        )
        command_msg.data = [
            float(value)
            for command in commands
            for value in command
        ]
        self._command_pub.publish(command_msg)
        self._trajectory_published = True
        duration_s = (point_count - 1) / self._trajectory_rate_hz
        self._log_state(
            "published",
            f"Published full trajectory once: {point_count} samples, "
            f"{len(command_msg.data)} floats, {duration_s:.3f} s at "
            f"{self._trajectory_rate_hz:.1f} Hz.",
        )

    def _joint_states_callback(self, msg: JointState) -> None:
        joint_map = {
            name: position for name, position in zip(msg.name, msg.position)
        }
        missing_joints = [
            joint_name for joint_name in self._joint_names if joint_name not in joint_map
        ]
        if missing_joints:
            if not self._missing_joint_state_logged:
                self.get_logger().warning(
                    f"Waiting for joint states containing: {', '.join(missing_joints)}"
                )
                self._missing_joint_state_logged = True
            return

        self._missing_joint_state_logged = False
        self._latest_joint_positions = np.array(
            [joint_map[joint_name] for joint_name in self._joint_names],
            dtype=float,
        )

        if self._plan is not None and not self._planned_commands:
            self._validate_or_execute_plan()

    def _publish_static_plan_outputs(self) -> None:
        if self._plan is None:
            return

        stamp = self.get_clock().now().to_msg()
        self._entry_pose_pub.publish(
            self._pose_stamped(self._plan.entry, self._plan.quaternion, stamp)
        )
        self._pre_entry_pose_pub.publish(
            self._pose_stamped(self._plan.pre_entry, self._plan.quaternion, stamp)
        )
        self._target_pose_pub.publish(
            self._pose_stamped(self._plan.target, self._plan.quaternion, stamp)
        )

        path_msg = PoseArray()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self._base_frame
        path_msg.poses = [
            self._pose_stamped(waypoint, self._plan.quaternion, stamp).pose
            for waypoint in self._path_output_waypoints()
        ]
        self._path_pub.publish(path_msg)

    def _path_output_waypoints(self) -> list[np.ndarray]:
        if self._plan is None:
            return []
        waypoints = self._plan.waypoints
        if len(waypoints) <= self._max_path_output_points:
            return waypoints

        stride = int(math.ceil(len(waypoints) / self._max_path_output_points))
        sampled = waypoints[::stride]
        if not np.array_equal(sampled[-1], waypoints[-1]):
            sampled.append(waypoints[-1])
        return sampled

    def _publish_needle_tip_tf(self) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._end_effector_frame
        tf_msg.child_frame_id = self._needle_tip_frame
        translation = self._needle_axis_ee * self._needle_tip_offset_m
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_msg)

    def _pose_stamped(self, position: np.ndarray, quaternion: np.ndarray, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])
        return msg

    def _load_dh_params(self) -> np.ndarray:
        d_values = self._array_parameter("dh_d")
        a_values = self._array_parameter("dh_a")
        alpha_values = self._array_parameter("dh_alpha")
        if not (d_values and a_values and alpha_values):
            raise ValueError("DH parameters are missing; load panda_dh.yaml.")
        if not (len(d_values) == len(a_values) == len(alpha_values)):
            raise ValueError("DH parameter arrays dh_d, dh_a, and dh_alpha must match.")
        return np.array([d_values, a_values, alpha_values], dtype=float).T

    def _array_parameter(self, name: str) -> list[float]:
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        return [float(item) for item in list(value or [])]

    def _resolve_file(self, filename: str) -> Path:
        path = Path(filename).expanduser()
        if path.is_absolute() and path.is_file():
            return path
        if path.is_file():
            return path.resolve()

        for parent in [Path.cwd(), *Path.cwd().parents]:
            candidate = parent / path
            if candidate.is_file():
                return candidate.resolve()

        raise FileNotFoundError(f"Could not find {filename}")

    def _parse_key_values(
        self,
        path: Path,
        section: Optional[str] = None,
    ) -> dict[str, list[float] | float]:
        values: dict[str, list[float] | float] = {}
        active_section: Optional[str] = None

        with path.open("r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.split("#", 1)[0].strip()
                if not line:
                    continue

                section_match = SECTION_PATTERN.match(line)
                if section_match:
                    active_section = section_match.group(1)
                    continue

                if section is not None and active_section != section:
                    continue

                vector_match = VECTOR_PATTERN.match(line)
                if vector_match:
                    values[vector_match.group(1)] = [
                        float(item.strip())
                        for item in vector_match.group(2).split(",")
                    ]
                    continue

                scalar_match = SCALAR_PATTERN.match(line)
                if scalar_match:
                    values[scalar_match.group(1)] = float(scalar_match.group(2))

        if not values:
            scope = f" section [{section}]" if section else ""
            raise ValueError(f"No key/value data found in {path}{scope}")
        return values

    def _linear_waypoints(
        self,
        start: np.ndarray,
        end: np.ndarray,
        speed_m_s: float,
    ) -> list[np.ndarray]:
        distance = float(np.linalg.norm(end - start))
        duration = distance / speed_m_s
        steps_from_rate = int(math.ceil(duration * self._trajectory_rate_hz))
        steps_from_distance = int(math.ceil(distance / self._cartesian_step_m))
        steps = max(1, steps_from_rate, steps_from_distance)
        return [
            start + (end - start) * (index / steps)
            for index in range(steps + 1)
        ]

    def _rotation_with_local_axis_aligned(
        self,
        local_axis: np.ndarray,
        world_axis: np.ndarray,
    ) -> np.ndarray:
        local_basis = self._orthonormal_basis_from_z(local_axis)
        world_basis = self._orthonormal_basis_from_z(world_axis)
        return world_basis @ local_basis.T

    def _orthonormal_basis_from_z(self, z_axis: np.ndarray) -> np.ndarray:
        z_unit = self._unit_vector(z_axis, "axis")
        reference = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(reference, z_unit))) > 0.95:
            reference = np.array([1.0, 0.0, 0.0], dtype=float)

        x_unit = np.cross(reference, z_unit)
        x_unit /= np.linalg.norm(x_unit)
        y_unit = np.cross(z_unit, x_unit)
        y_unit /= np.linalg.norm(y_unit)
        return np.column_stack((x_unit, y_unit, z_unit))

    def _unit_vector(self, values: list[float] | np.ndarray, name: str) -> np.ndarray:
        vector = np.asarray(values, dtype=float)
        if vector.shape != (3,):
            raise ValueError(f"{name} must contain exactly three values")
        norm = float(np.linalg.norm(vector))
        if norm <= 0.0:
            raise ValueError(f"{name} must be non-zero")
        return vector / norm

    def _compare_file_axis(self, file_axis: np.ndarray, computed_axis: np.ndarray) -> None:
        if file_axis.shape != (3,):
            return
        file_axis = self._unit_vector(file_axis, "insertion_axis_entry_to_target")
        angle = math.degrees(
            math.acos(float(np.clip(np.dot(file_axis, computed_axis), -1.0, 1.0)))
        )
        if angle > 1.0:
            self.get_logger().warning(
                "Candidate file axis differs from entry-target vector by "
                f"{angle:.2f} deg. Using computed entry-target axis."
            )

    def _log_state(self, state: str, detail: str) -> None:
        message = f"{state}: {detail}"
        if state in {"error", "ik_failed", "blocked"}:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[InsertionNode] = None
    try:
        node = InsertionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
