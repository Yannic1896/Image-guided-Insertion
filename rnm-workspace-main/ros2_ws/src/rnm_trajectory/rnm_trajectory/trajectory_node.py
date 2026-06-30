"""Trajectory planning node."""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from .core.trajectory_generator import QuinticTrajectoryGenerator
from .core.joint_path import JointPath

class TrajectoryPlanningNode(Node):
    def __init__(self):
        super().__init__('trajectory_planning_node')
        
        # Parameters
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('joint_states_topic', '/franka_state_controller/joint_states_desired')
        self.declare_parameter('ik_target_pose_topic', '/ik_target_pose')
        self.declare_parameter('ik_joint_goal_topic', '/ik_joint_goal')
        self.declare_parameter('joint_trajectory_topic', '/joint_position_example_controller/joint_trajectory_command')
        self.declare_parameter('planning_mode', 'joint') # cartesian or joint
        self.declare_parameter('joint_names', [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ])
        self.declare_parameter('joint_velocity_limits', [0.0])
        self.declare_parameter('joint_acceleration_limits', [0.0])
        self.declare_parameter('joint_jerk_limits', [0.0])
        self.declare_parameter('safety_factor', 0.25)

        # Joint limits
        joint_velocity_limits = self.get_parameter('joint_velocity_limits').value
        joint_acceleration_limits = self.get_parameter('joint_acceleration_limits').value
        joint_jerk_limits = self.get_parameter('joint_jerk_limits').value
        self.safety_factor = self.get_parameter('safety_factor').value

        self.get_logger().info(f"--- PARAMETER CHECK ---")
        self.get_logger().info(f"Velocity limits: {joint_velocity_limits}")
        self.get_logger().info(f"Acceleration limits: {joint_acceleration_limits}")
        self.get_logger().info(f"Jerk limits: {joint_jerk_limits}")
        self.get_logger().info(f"Safety factor: {self.safety_factor}")
        
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
        self.joint_traj_pub = self.create_publisher(Float64MultiArray, joint_trajectory_topic, 10)

        self.publish_rate_hz = 1000 # Robot needs 1000Hz, for testing we can use lower rates like 100Hz or 500Hz
        self.trajectory_to_publish = []
        self.trajectory_publish_index = 0
        
        # Subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped, target_pose_topic, self.target_pose_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_states_callback, 10)
        self.ik_joint_goal_sub = self.create_subscription(
            Float64MultiArray, ik_joint_goal_topic, self.ik_joint_goal_callback, 10)
            
        self.current_joint_state = None
        self.last_target_pose = None
        
        self.generator = QuinticTrajectoryGenerator(joint_velocity_limits, joint_acceleration_limits, joint_jerk_limits)
        self.path_planner = JointPath()

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
        self.get_logger().info('Received IK joint goal, checking...')
        if self.current_joint_state is None:
            self.get_logger().error('Current joint state empty')
            return
            
        goal_q = msg.data
        self.get_logger().info(f"START_Q: {self.current_joint_state} | GOAL_Q: {goal_q}")
        if len(goal_q) != len(self.joint_names):
            self.get_logger().error(f"Received {len(goal_q)} IK joints, expected {len(self.joint_names)}")
            return
            
        self.get_logger().info('Generating trajectory...')
        
       
        # Generate Trajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        self._plan_joint_trajectory(self.current_joint_state, goal_q, trajectory_msg)

        # Put data into Float64MultiArray message
        flattened_data = []
        n_points = len(trajectory_msg.points)
        n_joints = len(self.joint_names)

        for point in trajectory_msg.points:
            flattened_data.extend(list(point.positions))

        array_msg = Float64MultiArray()

        dim_points = MultiArrayDimension()
        dim_points.label = "points"
        dim_points.size = n_points
        dim_points.stride = n_points * n_joints

        dim_joints = MultiArrayDimension()
        dim_joints.label = "joints"
        dim_joints.size = n_joints
        dim_joints.stride = n_joints

        array_msg.layout.dim = [dim_points, dim_joints]
        array_msg.layout.data_offset = 0
        array_msg.data = flattened_data

        # Publish full trajectory at once
        self.get_logger().info(f'Publishing full trajectory with {n_points} steps and ({len(flattened_data)} floats).')
        self.joint_traj_pub.publish(array_msg)        

    def _plan_joint_trajectory(self, start_q, goal_q, trajectory_msg: JointTrajectory):
        # compute path [start_q, waypoint1, waypoint2, ..., goal_q]
        path = self.path_planner.joint_path(start_q, goal_q)

        # compute trajectory
        raw_trajectory = self.generator.generate_trajectory(path, self.publish_rate_hz, safety_factor=0.05)

        for pt in raw_trajectory:
            point_msg = JointTrajectoryPoint()
        
            point_msg.positions = pt['positions']
            point_msg.velocities = pt['velocities']
            point_msg.accelerations = pt['accelerations']

            t = pt['time']
            duration_msg = Duration()
            duration_msg.sec = int(t)
            duration_msg.nanosec = int((t - int(t)) * 1e9)
            point_msg.time_from_start = duration_msg
        
            trajectory_msg.points.append(point_msg)

  

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
