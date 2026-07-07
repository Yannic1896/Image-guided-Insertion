"""
ROS2 Node for Inverse Kinematic Solver.
Provides ROS2 services for IK computation.
"""

import numpy as np
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from rnm_kinematics.ik_solver import IKSolver

DEFAULT_COMMAND_TOPIC = '/ik_joint_goal'
#DEFAULT_COMMAND_TOPIC = '/joint_position_example_controller/joint_command'
DEFAULT_TARGET_POSE_TOPIC = '/target_pose'
DEFAULT_JOINT_STATES_TOPIC = '/joint_states'
DEFAULT_JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]


class IKSolverNode(Node):
    """ROS2 Node for Inverse Kinematic solving."""

    def __init__(self):
        super().__init__('ik_solver_node')

        self.declare_parameter('dh_d', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_a', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_alpha', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('command_topic', DEFAULT_COMMAND_TOPIC)
        self.declare_parameter('target_pose_topic', DEFAULT_TARGET_POSE_TOPIC)
        self.declare_parameter('joint_states_topic', DEFAULT_JOINT_STATES_TOPIC)
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('joint_limit_lower', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('joint_limit_upper', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_step', 0.05)

        dh_params = self._load_dh_params()
        self._joint_names = self._load_joint_names()
        lower_limits, upper_limits = self._load_joint_limits()
        max_step = float(self.get_parameter('max_step').value)
        self.ik_solver = IKSolver(
            dh_params=dh_params,
            joint_lower_limits=lower_limits,
            joint_upper_limits=upper_limits,
            max_step=max_step,
        )
        self._latest_joint_positions = None
        self._missing_joint_state_logged = False
        self._waiting_for_initial_state_logged = False

        command_topic = str(self.get_parameter('command_topic').value)
        target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)

        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            command_topic,
            10
        )

        # Subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self.target_pose_callback,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            10
        )

        self.get_logger().info(
            f'IK Solver Node initialized. Subscribing to {target_pose_topic}, '
            f'using current joints from {joint_states_topic}, '
            f'publishing joint commands to {command_topic}.'
        )

    def _load_dh_params(self) -> np.ndarray:
        d_values = self._get_array_parameter('dh_d')
        a_values = self._get_array_parameter('dh_a')
        alpha_values = self._get_array_parameter('dh_alpha')

        if not (d_values and a_values and alpha_values):
            raise ValueError(
                'DH parameters are missing. Use ik_solver.launch.py or pass '
                'panda_dh.yaml with --ros-args --params-file.'
            )
        if not (len(d_values) == len(a_values) == len(alpha_values)):
            raise ValueError('DH parameter arrays dh_d, dh_a, and dh_alpha must match.')

        dh_matrix = np.array(
            [d_values, a_values, alpha_values],
            dtype=float,
        ).T
        self.get_logger().info(f'Loaded DH table with {dh_matrix.shape[0]} rows')
        return dh_matrix

    def _load_joint_names(self) -> list[str]:
        joint_names = list(self.get_parameter('joint_names').value or [])
        if not joint_names:
            raise ValueError('joint_names parameter must contain the Panda joint names.')
        return [str(name) for name in joint_names]

    def _load_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        lower_values = self._get_array_parameter('joint_limit_lower')
        upper_values = self._get_array_parameter('joint_limit_upper')

        if not lower_values or not upper_values:
            raise ValueError(
                'Joint limits are missing. Use ik_solver.launch.py or pass '
                'panda_joint_limits.yaml with --ros-args --params-file.'
            )
        if len(lower_values) != len(self._joint_names):
            raise ValueError(
                'joint_limit_lower must have one value for each configured joint name.'
            )
        if len(upper_values) != len(self._joint_names):
            raise ValueError(
                'joint_limit_upper must have one value for each configured joint name.'
            )
        
        lower_limits = np.array(lower_values, dtype=float)
        upper_limits = np.array(upper_values, dtype=float)

        # Calculate the total range for each joint
        joint_ranges = upper_limits - lower_limits

        # Define a 5% safety buffer
        buffer = joint_ranges * 0.05

        # Shrink the bounds inward
        safe_lower_limits = lower_limits + buffer
        safe_upper_limits = upper_limits - buffer

        return safe_lower_limits, safe_upper_limits

    def _get_array_parameter(self, name: str) -> list[float]:
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        return list(value or [])

    def joint_states_callback(self, msg: JointState) -> None:
        joint_map = {
            name: position for name, position in zip(msg.name, msg.position)
        }
        missing_joints = [
            joint_name
            for joint_name in self._joint_names
            if joint_name not in joint_map
        ]
        if missing_joints:
            if not self._missing_joint_state_logged:
                self.get_logger().warning(
                    'Waiting for joint states containing: '
                    f'{", ".join(missing_joints)}'
                )
                self._missing_joint_state_logged = True
            return

        self._missing_joint_state_logged = False
        self._latest_joint_positions = np.array(
            [joint_map[joint_name] for joint_name in self._joint_names],
            dtype=float,
        )

    def target_pose_callback(self, msg: PoseStamped) -> None:
        self.get_logger().info('Received target pose')

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        target_pose = np.array([x, y, z, qx, qy, qz, qw], dtype=float)
        initial_guess = self._latest_joint_positions
        if initial_guess is None:
            if not self._waiting_for_initial_state_logged:
                self.get_logger().warning(
                    'No current joint state received yet; ignoring target poses '
                    'until a complete joint state arrives.'
                )
                self._waiting_for_initial_state_logged = True
            return

        self._waiting_for_initial_state_logged = False

        joint_angles = self.ik_solver.solve(target_pose, initial_guess=initial_guess)

        if joint_angles is None:
            self.get_logger().warning('IK failed: no solution found for target pose')
            return

        self._publish_joint_command(joint_angles)

    def _publish_joint_command(self, joint_angles: np.ndarray) -> None:
        command = [float(angle) for angle in joint_angles]
        if len(command) != len(self._joint_names):
            self.get_logger().error(
                f'IK produced {len(command)} joint angles; sim controller expects '
                f'{len(self._joint_names)}.'
            )
            return

        command_msg = Float64MultiArray()
        command_msg.data = command
        self.joint_command_pub.publish(command_msg)
        self.get_logger().info(f'Published joint command: {command}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IKSolverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
