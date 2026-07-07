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
from rnm_kinematics.fk_solver import ForwardKinematicsSolver
from rnm_needle.insertion_path import InsertionPathCalculator

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
        self.declare_parameter('needle_offset_z', 0.17)
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
        
        self.ik_solver = IKSolver(
            dh_params=dh_params,
            joint_lower_limits=lower_limits,
            joint_upper_limits=upper_limits,
            max_step=max_step,
        )

        # initialize fk solver
        self.fk_solver = ForwardKinematicsSolver(
            dh_params=dh_params
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
        return np.array([d, a, alpha], dtype=float).T

    def _load_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """ Lädt Gelenkgrenzen analog zum IKSolverNode """
        lower = list(self.get_parameter('joint_limit_lower').value or [])
        upper = list(self.get_parameter('joint_limit_upper').value or [])
        if not lower or not upper:
            raise ValueError('Gelenkgrenzen fehlen! Stelle sicher, dass panda_joint_limits.yaml geladen ist.')
        return np.array(lower, dtype=float), np.array(upper, dtype=float)

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

        # 2. JointTrajectory message preparation
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = self._base_frame
        trajectory_msg.joint_names = self._joint_names
        
        # start configuration for the IK 
        current_q = self._latest_joint_positions.copy()

        # find flange orientation
        T = self.fk_solver.compute(current_q)   # 4x4 transform base -> flange
        R = T[0:3, 0:3]                         # rotation part
        z_flange_base = R @ np.array([0.0, 0.0, 1.0])



        # 3. loop for all calculated poses -> IK solve
        for i, pose in enumerate(calculated_poses):
            target_pose_array = np.array([
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ], dtype=float)

            # IK calculation (uses method from ik_solver.py)
            q_sol = self.ik_solver.solve(target_pose_array, initial_guess=current_q)

            if q_sol is not None:
                point = JointTrajectoryPoint()
                point.positions = q_sol.tolist()
                
                # Timestamp for the trajectory set (e.g., 0.5 seconds interval per point)
                duration = i * 0.5
                point.time_from_start.sec = int(duration)
                point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
                
                trajectory_msg.points.append(point)
                current_q = q_sol  # Update für den nächsten Punkt
            else:
                self.get_logger().error(f'IK-Solver fand keine Lösung für Wegpunkt {i}! Trajektorie abgebrochen.')
                return

        # 4.  joint trajectory publishen
        self.path_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published JointTrajectory with {len(trajectory_msg.points)} joint points to trajectory planning.')

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

    
