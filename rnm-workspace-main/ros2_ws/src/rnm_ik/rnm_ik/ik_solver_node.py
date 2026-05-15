"""
ROS2 Node for Inverse Kinematic Solver.
Provides ROS2 services for IK computation.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from rnm_ik.ik_solver import IKSolver


class IKSolverNode(Node):
    """ROS2 Node for Inverse Kinematic solving."""

    def __init__(self):
        super().__init__('ik_solver_node')

        self.declare_parameter('dh_d', [])
        self.declare_parameter('dh_a', [])
        self.declare_parameter('dh_alpha', [])

        dh_params = self._load_dh_params()
        self.ik_solver = IKSolver(dh_params=dh_params)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
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
        """
        Callback for target pose subscription.
        
        Args:
            msg: Target pose message
        """
        self.get_logger().info('Received target pose')

        # TODO Implement IK computation
        pass


def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
