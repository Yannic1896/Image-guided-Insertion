"""
ROS2 Node for Inverse Kinematic Solver.
Provides ROS2 services for IK computation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from rnm_ik.ik_solver import IKSolver


class IKSolverNode(Node):
    """ROS2 Node for Inverse Kinematic solving."""

    def __init__(self):
        super().__init__('ik_solver_node')
  
        self.ik_solver = IKSolver()
        
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

    def target_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback for target pose subscription.
        
        Args:
            msg: Target pose message
        """
        self.get_logger().info(f'Received target pose')

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
