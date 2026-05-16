"""
ROS2 Node for Inverse Kinematic Solver.
Provides ROS2 services for IK computation.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from rnm_kinematics.ik_solver import IKSolver


class IKSolverNode(Node):
    """ROS2 Node for Inverse Kinematic solving."""

    def __init__(self):
        super().__init__('ik_solver_node')

        self.declare_parameter('dh_d', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_a', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_alpha', Parameter.Type.DOUBLE_ARRAY)

        dh_params = self._load_dh_params()
        self.ik_solver = IKSolver(dh_params=dh_params)
        
        # Publishers
        self.joint_command_pub = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )
        
        # Subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.target_pose_callback,
            10
        )
        
        self.get_logger().info('IK Solver Node initialized')

    def _load_dh_params(self) -> np.ndarray | None:
        
        d_values = self.get_parameter('dh_d').value
        a_values = self.get_parameter('dh_a').value
        alpha_values = self.get_parameter('dh_alpha').value

        if not (d_values and a_values and alpha_values):
            self.get_logger().warn('DH parameters missing')
            return None


        dh_matrix = np.array(
            [d_values, a_values, alpha_values],
            dtype=float,
        ).T
        self.get_logger().info(f'Loaded DH table with {dh_matrix.shape[0]} rows')
        return dh_matrix

    def target_pose_callback(self, msg: PoseStamped) -> None:
        
        self.get_logger().info('Received target pose')

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        frame = msg.header.frame_id
        
        target_pose=np.array([x, y, z, qx, qy, qz, qw])
        joint_angles=self.ik_solver.solve(target_pose)

        if joint_angles is None:
            self.get_logger().warning('IK failed: no solution found for target pose')
            return

        self._publish_joint_state(joint_angles, msg)

    def _publish_joint_state(self, joint_angles: np.ndarray, pose_msg: PoseStamped) -> None:
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = pose_msg.header.stamp
        joint_state_msg.name = [
            f'panda_joint{i}' for i in range(1, len(joint_angles) + 1)
        ]
        joint_state_msg.position = [float(angle) for angle in joint_angles]
        self.joint_command_pub.publish(joint_state_msg)



def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
