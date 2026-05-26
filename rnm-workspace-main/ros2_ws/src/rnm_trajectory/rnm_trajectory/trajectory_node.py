"""Trajectory planning node."""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryPlanningNode(Node):
    def __init__(self):
        super().__init__('trajectory_planning_node')
        
        # Parameters
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('ik_target_pose_topic', '/ik_target_pose')
        self.declare_parameter('ik_joint_goal_topic', '/ik_joint_goal')
        self.declare_parameter('joint_trajectory_topic', '/joint_position_example_controller/joint_trajectory_command')
        self.declare_parameter('planning_mode', 'joint') # cartesian or joint
        self.declare_parameter('joint_names', [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ])
        
        # Topic Names
        target_pose_topic = self.get_parameter('target_pose_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        ik_target_pose_topic = self.get_parameter('ik_target_pose_topic').value
        ik_joint_goal_topic = self.get_parameter('ik_joint_goal_topic').value
        joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value
        
        self.planning_mode = self.get_parameter('planning_mode').value
        self.joint_names = self.get_parameter('joint_names').value

        # Publishers
        self.ik_target_pub = self.create_publisher(PoseStamped, ik_target_pose_topic, 10)
        self.joint_traj_pub = self.create_publisher(JointTrajectory, joint_trajectory_topic, 10)
        self.publish_rate_hz = 1000 # Robot needs 1000Hz, for testing we can use lower rates like 100Hz or 500Hz
        self.trajectory_to_publish = None
        self.trajectory_publish_index = 0
        self.trajectory_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_trajectory_point)
        
        # Subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped, target_pose_topic, self.target_pose_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_states_callback, 10)
        self.ik_joint_goal_sub = self.create_subscription(
            Float64MultiArray, ik_joint_goal_topic, self.ik_joint_goal_callback, 10)
            
        self.current_joint_state = None
        self.last_target_pose = None
        
        self.get_logger().info('Trajectory Planning Node started.')

    def joint_states_callback(self, msg: JointState):
        """ Read Current Robot State """
        # Map joint names to positions to ensure correct ordering if they are out of order
        joint_map = {name: pos for name, pos in zip(msg.name, msg.position)}
        
        # Ensure we have all the required joints before saving state
        missing = [j for j in self.joint_names if j not in joint_map]
        if not missing:
            self.current_joint_state = [joint_map[j] for j in self.joint_names]

    def target_pose_callback(self, msg: PoseStamped):
        """ Receive Target Pose """
        if self.current_joint_state is None:
            return
            
        self.get_logger().info('Received target pose. Requesting IK solution...')
        self.last_target_pose = msg
        
        self.ik_target_pub.publish(msg)

    def ik_joint_goal_callback(self, msg: Float64MultiArray):
        """ Receive Goal Joint State from IK node """
        
        if self.current_joint_state is None:
            return
            
        goal_q = msg.data
        if len(goal_q) != len(self.joint_names):
            self.get_logger().error(f"Received {len(goal_q)} IK joints, expected {len(self.joint_names)}")
            return
            
        self.get_logger().info('Generating trajectory...')
        
       
        # Generate Trajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        self._plan_joint_trajectory(self.current_joint_state, goal_q, trajectory_msg)
            
        self.trajectory_to_publish = trajectory_msg
        self.trajectory_publish_index = 0
        self.get_logger().info(f' Publish trajectory with {len(trajectory_msg.points)} points to {self.joint_traj_pub.topic_name} at {self.publish_rate_hz} Hz.')

    def _publish_trajectory_point(self):
        if self.trajectory_to_publish is not None:
            points = self.trajectory_to_publish.points
            if self.trajectory_publish_index < len(points):
                msg = JointTrajectory()
                msg.joint_names = self.trajectory_to_publish.joint_names
                msg.points.append(points[self.trajectory_publish_index])
                self.joint_traj_pub.publish(msg)
                self.trajectory_publish_index += 1
            else:
                self.trajectory_to_publish = None
                self.trajectory_publish_index = 0
        

    def _plan_joint_trajectory(self, start_q, goal_q, trajectory_msg: JointTrajectory):
        """ Linear interpolation in joint space """
  

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
