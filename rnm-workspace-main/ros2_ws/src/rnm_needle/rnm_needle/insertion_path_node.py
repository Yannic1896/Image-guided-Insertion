""" ROS 2 Node for the Needle-Path_calculation (Talker and Listener)"""
from __future__ import annotations
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterUninitializedException

# Import Standard message types
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
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
        self.declare_parameter('needle_offset_z', 0.167)
        self.declare_parameter('registration_topic', DEFAULT_REGISTRATION_TOPIC)
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

        # 3. Calculator-class initializing 
        self.calculator = InsertionPathCalculator()

        # 4. LISTENER (Subscriber): listen to Model_registration (used PoseArray)
        self.registration_subsrcriber = self.create_subscription(
            PoseArray,
            registration_topic,
            self.listener_callback,
            10
        )
        # 4.2. SUBSCRIBER: receive joint angles for the IK start value
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # 5. TALKER (Publisher): sends path to Trajectory Planning
        path_topic = '/needle_path'
        self.path_publisher = self.create_publisher(
            JointTrajectory,
            path_topic,
            10
        )

        self.get_logger().info(
            f'rnm_needle: Kombi-Node initialisiert.\n'
            f' -> Listening on: {registration_topic}\n'
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

    