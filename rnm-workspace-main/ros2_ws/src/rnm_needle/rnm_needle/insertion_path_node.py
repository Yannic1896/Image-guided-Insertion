""" ROS 2 Node for the Needle-Path_calculation (Talker and Listener)"""
from __future__ import annotations
from pathlib import Path
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# Import Standard message types
from geometry_msgs.msg import Point, PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# Kinematik und Berechnungslogik importieren
from rnm_kinematics.ik_solver import IKSolver
from rnm_needle.insertion_path import InsertionPathCalculator
from rnm_needle.safety import LightweightCollisionChecker
from rnm_needle.safety import joint_limit_margin
from rnm_needle.safety import parse_forbidden_spheres

DEFAULT_REGISTRATION_TOPIC = '/needle_poses'
DEFAULT_BASE_FRAME = 'panda_link0'
DEFAULT_JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3', 
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
]

class InsertionPathNode(Node):
    """ ROS 2 Node for neddle-path-planning (combination of talker and listener). """
    def __init__(self) -> None:
        super().__init__('insertion_path_node')

        # declaring parameters
        self.declare_parameter('needle_offset_z', 0.167)
        self.declare_parameter('registration_topic', DEFAULT_REGISTRATION_TOPIC)
        self.declare_parameter('path_topic', '/needle_path')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('input_source', 'topic')
        self.declare_parameter('target_location_file', 'target_output/target_location.txt')
        self.declare_parameter('entry_location_file', 'entry_point_output/needle_entry_point.txt')
        self.declare_parameter('entry_candidate_index', 0)
        self.declare_parameter('file_poll_period', 0.5)
        self.declare_parameter('base_frame', DEFAULT_BASE_FRAME)
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        
        # declaring the DH- and Limit-Parameters, so we can load them from YAML/Launch
        self.declare_parameter('dh_d', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_a', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_alpha', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('joint_limit_lower', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('joint_limit_upper', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_step', 0.05)
        self.declare_parameter(
            'preferred_ik_seed',
            [0.0, -0.6, 0.0, -2.2, 0.0, 1.8, 0.8],
        )
        self.declare_parameter('ik_seed_delta', 0.35)
        self.declare_parameter('max_pre_entry_joint_delta', 2.0)
        self.declare_parameter('max_waypoint_joint_delta', 0.8)
        self.declare_parameter('min_joint_limit_margin', 0.05)
        self.declare_parameter('collision_check_enabled', True)
        self.declare_parameter('min_self_collision_distance', 0.03)
        self.declare_parameter('min_table_z', -0.05)
        self.declare_parameter(
            'collision_forbidden_sphere_centers',
            [0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'collision_forbidden_sphere_radii',
            [0.0],
        )
        
        # reading the parameters
        self._needle_offset_z = float(self.get_parameter('needle_offset_z').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._joint_names = [str(n) for n in self.get_parameter('joint_names').value]
        registration_topic = str(self.get_parameter('registration_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self._input_source = str(self.get_parameter('input_source').value)
        if self._input_source not in {'topic', 'file'}:
            raise ValueError("input_source must be 'topic' or 'file'.")
        self._target_location_file = Path(
            str(self.get_parameter('target_location_file').value)
        )
        self._entry_location_file = Path(
            str(self.get_parameter('entry_location_file').value)
        )
        self._entry_candidate_index = int(
            self.get_parameter('entry_candidate_index').value
        )
        self._file_poll_period = float(self.get_parameter('file_poll_period').value)

        # 2. IKSolver mit Parametern initialisieren
        dh_params = self._load_dh_params()
        lower_limits, upper_limits = self._load_joint_limits()
        max_step = float(self.get_parameter('max_step').value)
        self._joint_lower_limits = lower_limits
        self._joint_upper_limits = upper_limits
        self._preferred_ik_seed = self._load_optional_joint_array('preferred_ik_seed')
        self._ik_seed_delta = float(self.get_parameter('ik_seed_delta').value)
        self._max_pre_entry_joint_delta = float(
            self.get_parameter('max_pre_entry_joint_delta').value
        )
        self._max_waypoint_joint_delta = float(
            self.get_parameter('max_waypoint_joint_delta').value
        )
        self._min_joint_limit_margin = float(
            self.get_parameter('min_joint_limit_margin').value
        )
        self._collision_check_enabled = bool(
            self.get_parameter('collision_check_enabled').value
        )
        forbidden_spheres = parse_forbidden_spheres(
            self.get_parameter('collision_forbidden_sphere_centers').value or [],
            self.get_parameter('collision_forbidden_sphere_radii').value or [],
        )
        self._collision_checker = LightweightCollisionChecker(
            dh_params=dh_params,
            joint_count=len(self._joint_names),
            min_self_distance=float(
                self.get_parameter('min_self_collision_distance').value
            ),
            min_table_z=float(self.get_parameter('min_table_z').value),
            forbidden_spheres=forbidden_spheres,
        )
        
        self.ik_solver = IKSolver(
            dh_params=dh_params,
            joint_lower_limits=lower_limits,
            joint_upper_limits=upper_limits,
            max_step=max_step,
        )

        # saves reference for current robot state (IK startvalues)
        self._latest_joint_positions = None
        self._file_input_processed = False

        # 3. Calculator-class initializing 
        self.calculator = InsertionPathCalculator()

        # 4. LISTENER (Subscriber): listen to Model_registration (used PoseArray)
        self.registration_subsrcriber = None
        if self._input_source == 'topic':
            self.registration_subsrcriber = self.create_subscription(
                PoseArray,
                registration_topic,
                self.listener_callback,
                10
            )
        # 4.2. SUBSCRIBER: receive joint angles for the IK start value
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            10
        )

        # 5. TALKER (Publisher): sends path to Trajectory Planning
        self.path_publisher = self.create_publisher(
            JointTrajectory,
            path_topic,
            10
        )
        self._file_input_timer = None
        if self._input_source == 'file':
            self._file_input_timer = self.create_timer(
                max(self._file_poll_period, 0.1),
                self._file_input_timer_callback,
            )

        self.get_logger().info(
            f'rnm_needle: Kombi-Node initialisiert.\n'
            f' -> Input source: {self._input_source}\n'
            f' -> Listening on: {registration_topic if self._input_source == "topic" else "disabled"}\n'
            f' -> Target file: {self._target_location_file}\n'
            f' -> Entry file: {self._entry_location_file} '
            f'(candidate {self._entry_candidate_index})\n'
            f' -> Reading joints from: {joint_states_topic}\n'
            f' -> Publishing on: {path_topic}\n'
            f' -> Needle Offset Z: {self._needle_offset_z * 1000.0} mm'
        )
    def _load_dh_params(self) -> np.ndarray:
        """ Lädt DH-Parameter analog zum IKSolverNode """
        d = list(self.get_parameter('dh_d').value or [])
        a = list(self.get_parameter('dh_a').value or [])
        alpha = list(self.get_parameter('dh_alpha').value or [])
        if not (d and a and alpha):
            raise ValueError('DH-Parameter fehlen! Stelle sicher, dass panda_dh.yaml geladen ist.')
        if not (len(d) == len(a) == len(alpha)):
            raise ValueError('DH parameter arrays must have equal length.')
        return np.array([d, a, alpha], dtype=float).T

    def _load_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """ Lädt Gelenkgrenzen analog zum IKSolverNode """
        lower = list(self.get_parameter('joint_limit_lower').value or [])
        upper = list(self.get_parameter('joint_limit_upper').value or [])
        if not lower or not upper:
            raise ValueError('Gelenkgrenzen fehlen! Stelle sicher, dass panda_joint_limits.yaml geladen ist.')
        if len(lower) != len(self._joint_names) or len(upper) != len(self._joint_names):
            raise ValueError(
                'Joint limit arrays must match the configured joint_names length.'
            )
        return np.array(lower, dtype=float), np.array(upper, dtype=float)

    def _load_optional_joint_array(self, parameter_name: str) -> np.ndarray | None:
        values = list(self.get_parameter(parameter_name).value or [])
        if not values:
            return None
        if len(values) != len(self._joint_names):
            raise ValueError(
                f'{parameter_name} must contain {len(self._joint_names)} values.'
            )
        return np.array(values, dtype=float)

    def joint_states_callback(self, msg: JointState) -> None:
        """ Speichert den aktuellen Gelenkzustand des Roboters """
        joint_map = {name: pos for name, pos in zip(msg.name, msg.position)}
        if all(j in joint_map for j in self._joint_names):
            self._latest_joint_positions = np.array([joint_map[j] for j in self._joint_names], dtype=float)


    def listener_callback(self, msg: PoseArray) -> None:
        """Callback-Funtion: in action, when target- and goalpose are delivered"""
        """ Callback-Funktion: calculates path  -> calculates IK -> Publishes Trajectory """
        if self._latest_joint_positions is None:
            self.get_logger().warning('waiting for joint states from /joint_states... Calculation skipped.')
            return
        # Safety check: expecting 2 poses
        if len(msg.poses) < 2:
            self.get_logger().warning('Received PoseArray contains fewer than 2 poses. Calculation aborted.')
            return
            
        self.get_logger().info('Received entry and target positions from model registration.')
        # extraction of the positions in the PoseArray
        entry_point = msg.poses[0].position
        target_point = msg.poses[1].position
        self._calculate_and_publish_path(entry_point, target_point)

    def _file_input_timer_callback(self) -> None:
        if self._file_input_processed:
            if self._file_input_timer is not None:
                self._file_input_timer.cancel()
            return
        if self._latest_joint_positions is None:
            self.get_logger().info(
                'Waiting for joint states before computing insertion path from files.',
                throttle_duration_sec=5.0,
            )
            return

        try:
            entry_point, target_point = self._load_entry_target_from_files()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            self._file_input_processed = True
            if self._file_input_timer is not None:
                self._file_input_timer.cancel()
            return

        self._file_input_processed = True
        if self._file_input_timer is not None:
            self._file_input_timer.cancel()
        self.get_logger().info(
            'Loaded insertion entry/target from TXT reports; calculating path.'
        )
        self._calculate_and_publish_path(entry_point, target_point)

    def _load_entry_target_from_files(self) -> tuple[Point, Point]:
        target = self._read_vector_from_report(
            self._target_location_file,
            'scan_center_m',
        )
        entry = self._read_candidate_entry(
            self._entry_location_file,
            self._entry_candidate_index,
        )
        entry_point = Point(x=float(entry[0]), y=float(entry[1]), z=float(entry[2]))
        target_point = Point(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        self.get_logger().info(
            'Selected file entry/target: '
            f'entry={entry.tolist()}, target={target.tolist()}'
        )
        return entry_point, target_point

    def _read_vector_from_report(self, path: Path, key: str) -> np.ndarray:
        if not path.exists():
            raise RuntimeError(f'Report file does not exist: {path}')

        text = path.read_text(encoding='utf-8')
        return self._read_vector_from_text(text, key, path)

    def _read_candidate_entry(self, path: Path, candidate_index: int) -> np.ndarray:
        if candidate_index < 0:
            raise RuntimeError('entry_candidate_index must be non-negative.')
        if not path.exists():
            raise RuntimeError(f'Entry report file does not exist: {path}')

        text = path.read_text(encoding='utf-8')
        block = self._candidate_block(text, candidate_index)
        if block is not None:
            return self._read_vector_from_text(block, 'entry_point_m', path)
        if candidate_index == 0:
            return self._read_vector_from_text(text, 'entry_point_m', path)
        raise RuntimeError(f'Could not find [candidate_{candidate_index}] in {path}')

    def _candidate_block(self, text: str, candidate_index: int) -> str | None:
        pattern = (
            rf'^\[candidate_{candidate_index}\]\s*$'
            r'(?P<body>.*?)(?=^\[candidate_\d+\]\s*$|\Z)'
        )
        match = re.search(pattern, text, flags=re.MULTILINE | re.DOTALL)
        if match is None:
            return None
        return match.group('body')

    def _read_vector_from_text(self, text: str, key: str, path: Path) -> np.ndarray:
        match = re.search(rf'^{re.escape(key)}:\s*\[([^\]]+)\]', text, flags=re.MULTILINE)
        if match is None:
            raise RuntimeError(f'Could not find {key} in {path}')
        vector = np.fromstring(match.group(1), sep=',', dtype=np.float64)
        if len(vector) != 3:
            raise RuntimeError(f'Invalid {key} vector in {path}')
        return vector

    def _calculate_and_publish_path(self, entry_point: Point, target_point: Point) -> None:
        # calling the logic from insertion_path.py including the offset
        calculated_poses = self.calculator.compute_path(
            entry_point,
            target_point,
            self._needle_offset_z
        )
        if not calculated_poses:
            self.get_logger().warning('Calculated needle path is empty. Calculation aborted.')
            return

        # 2. JointTrajectory message preparation
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = self._base_frame
        trajectory_msg.joint_names = self._joint_names
        
        # start configuration for the IK
        current_q = self._latest_joint_positions.copy()

        # 3. loop for all calculated poses -> IK solve
        for i, pose in enumerate(calculated_poses):
            target_pose_array = np.array([
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ], dtype=float)

            if i == 0:
                q_sol = self._solve_pre_entry(target_pose_array, current_q)
            else:
                q_sol = self._solve_waypoint(target_pose_array, current_q, i)

            if q_sol is None:
                return

            if not self._validate_waypoint_transition(current_q, q_sol, i):
                return

            point = JointTrajectoryPoint()
            point.positions = q_sol.tolist()

            # Timestamp for the trajectory set (e.g., 0.5 seconds interval per point)
            duration = i * 0.5
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

            trajectory_msg.points.append(point)
            current_q = q_sol  # Update für den nächsten Punkt

        # 4.  joint trajectory publishen
        self.path_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published JointTrajectory with {len(trajectory_msg.points)} joint points to trajectory planning.')

    def _solve_pre_entry(
        self,
        target_pose_array: np.ndarray,
        current_q: np.ndarray,
    ) -> np.ndarray | None:
        candidates = []
        for seed in self._pre_entry_ik_seeds(current_q):
            try:
                q_sol = self.ik_solver.solve(target_pose_array, initial_guess=seed)
            except ValueError as exc:
                self.get_logger().error(f'IK input validation failed for pre-entry: {exc}')
                return None
            if q_sol is None:
                continue
            valid, reason = self._validate_candidate(q_sol)
            if not valid:
                self.get_logger().warn(f'Rejected pre-entry IK candidate: {reason}')
                continue
            score = self._score_pre_entry_candidate(q_sol, current_q)
            candidates.append((score, q_sol))

        if not candidates:
            self.get_logger().error('IK solver found no safe pre-entry solution.')
            return None

        candidates.sort(key=lambda item: item[0])
        best_score, best_q = candidates[0]
        self._log_pre_entry_validation(current_q, best_q, best_score)
        return best_q

    def _solve_waypoint(
        self,
        target_pose_array: np.ndarray,
        current_q: np.ndarray,
        waypoint_index: int,
    ) -> np.ndarray | None:
        try:
            q_sol = self.ik_solver.solve(target_pose_array, initial_guess=current_q)
        except ValueError as exc:
            self.get_logger().error(
                f'IK input validation failed for waypoint {waypoint_index}: {exc}'
            )
            return None

        if q_sol is None:
            self.get_logger().error(
                f'IK-Solver fand keine Lösung für Wegpunkt {waypoint_index}! '
                'Trajektorie abgebrochen.'
            )
            return None

        valid, reason = self._validate_candidate(q_sol)
        if not valid:
            self.get_logger().error(
                f'Waypoint {waypoint_index} failed safety validation: {reason}'
            )
            return None

        return q_sol

    def _pre_entry_ik_seeds(self, current_q: np.ndarray) -> list[np.ndarray]:
        seeds = [current_q]
        if self._preferred_ik_seed is not None:
            seeds.append(self._preferred_ik_seed)
        seeds.append(0.5 * (self._joint_lower_limits + self._joint_upper_limits))

        for joint_index in (1, 3, 5):
            for sign in (-1.0, 1.0):
                seed = current_q.copy()
                seed[joint_index] += sign * self._ik_seed_delta
                seed = np.clip(seed, self._joint_lower_limits, self._joint_upper_limits)
                seeds.append(seed)

        unique_seeds = []
        for seed in seeds:
            if not any(np.allclose(seed, existing) for existing in unique_seeds):
                unique_seeds.append(seed)
        return unique_seeds

    def _validate_candidate(self, q_sol: np.ndarray) -> tuple[bool, str]:
        margin = joint_limit_margin(
            q_sol,
            self._joint_lower_limits,
            self._joint_upper_limits,
        )
        if margin < self._min_joint_limit_margin:
            return (
                False,
                f'joint-limit margin {margin:.3f} rad is below '
                f'{self._min_joint_limit_margin:.3f} rad',
            )

        if self._collision_check_enabled:
            result = self._collision_checker.validate(q_sol)
            if not result.ok:
                return False, result.reason

        return True, ''

    def _validate_waypoint_transition(
        self,
        previous_q: np.ndarray,
        next_q: np.ndarray,
        waypoint_index: int,
    ) -> bool:
        deltas = np.abs(next_q - previous_q)
        max_delta = float(np.max(deltas))
        limit = (
            self._max_pre_entry_joint_delta
            if waypoint_index == 0
            else self._max_waypoint_joint_delta
        )
        if limit > 0.0 and max_delta > limit:
            self.get_logger().error(
                f'Waypoint {waypoint_index} joint jump too large: '
                f'{max_delta:.3f} rad > {limit:.3f} rad; deltas={deltas.tolist()}'
            )
            return False
        return True

    def _score_pre_entry_candidate(
        self,
        candidate_q: np.ndarray,
        current_q: np.ndarray,
    ) -> float:
        joint_distance = float(np.linalg.norm(candidate_q - current_q))
        margin = joint_limit_margin(
            candidate_q,
            self._joint_lower_limits,
            self._joint_upper_limits,
        )
        limit_penalty = 1.0 / max(margin, 1e-3)
        elbow_penalty = 0.25 * abs(candidate_q[3] - current_q[3])
        wrist_penalty = 0.15 * abs(candidate_q[5] - current_q[5])
        return joint_distance + 0.05 * limit_penalty + elbow_penalty + wrist_penalty

    def _log_pre_entry_validation(
        self,
        current_q: np.ndarray,
        pre_entry_q: np.ndarray,
        score: float,
    ) -> None:
        deltas = np.abs(pre_entry_q - current_q)
        margin = joint_limit_margin(
            pre_entry_q,
            self._joint_lower_limits,
            self._joint_upper_limits,
        )
        self.get_logger().info(
            'Selected pre-entry IK solution: '
            f'score={score:.3f}, max_delta={float(np.max(deltas)):.3f} rad, '
            f'limit_margin={margin:.3f} rad, deltas={deltas.tolist()}'
        )

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = InsertionPathNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

    
