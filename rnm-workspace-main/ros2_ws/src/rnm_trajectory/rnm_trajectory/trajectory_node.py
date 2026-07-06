"""Trajectory planning node."""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
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
        self.declare_parameter('needle_path_topic', '/needle_path')

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

        self.get_logger().info(f"PARAMETER CHECK")
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
        needle_path_topic = self.get_parameter('needle_path_topic').value
        self.joint_names = self.get_parameter('joint_names').value

        # Publishers
        self.ik_target_pub = self.create_publisher(PoseStamped, ik_target_pose_topic, 10)
        self.joint_traj_pub = self.create_publisher(Float64MultiArray, joint_trajectory_topic, 10)

        self.publish_rate_hz = 1000 # Robot needs 1000Hz
        self.trajectory_to_publish = []
        self.trajectory_publish_index = 0
        
        # Subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped, target_pose_topic, self.target_pose_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_states_callback, 10)
        self.ik_joint_goal_sub = self.create_subscription(
            Float64MultiArray, ik_joint_goal_topic, self.ik_joint_goal_callback, 10)
        self.needle_path_sub = self.create_subscription(
            JointTrajectory, needle_path_topic, self.needle_path_callback, 10)
            
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

        # Compute path [start_q, waypoint1, waypoint2, ..., goal_q]
        path = self.path_planner.joint_path(self.current_joint_state, goal_q)

        # Compute trajectory
        try:
            trajectory = self.generator.generate_trajectory(path, self.publish_rate_hz, self.safety_factor, min_duration=4.0)
            self._publish_trajectory(trajectory)
        except ValueError as e:
            self.get_logger().error(f"Trajectory validation failed: {str(e)}")

    def needle_path_callback(self, msg:JointTrajectory):
        """ Receive Needle Path"""
        self.get_logger().info('Received needle insertion path.')

        needle_path = []
        for point in msg.points:
            needle_path.append(list(point.positions))
            
        n_waypoints = len(needle_path)
        self.get_logger().info(f"{n_waypoints} waypoints in needle path")

        if n_waypoints < 2:
            self.get_logger().error("Not enough points in needle path.")
            return

        pre_entry_point = needle_path[0]
        entry_point = needle_path[1]

        try:
            self.get_logger().info('Generating approach trajectory...')
            approach_path = self.path_planner.joint_path(self.current_joint_state, pre_entry_point)
            approach_traj = self.generator.generate_trajectory(approach_path, self.publish_rate_hz, self.safety_factor, min_duration=4.0)

            self.get_logger().info('Generating pre-entry to entry trajectory...')
            pre_to_entry_path = self.path_planner.joint_path(pre_entry_point, entry_point)
            pre_to_entry_traj = self.generator.generate_trajectory(pre_to_entry_path, self.publish_rate_hz, self.safety_factor, min_duration=2.0)

            self.get_logger().info('Generating needle insertion trajectory...')
            insertion_path = needle_path[1:]
            insertion_traj = self.generator.generate_trajectory(insertion_path, self.publish_rate_hz, self.safety_factor, min_duration=0.3)

            final_trajectory = self._concatenate_trajectories([approach_traj, pre_to_entry_traj, insertion_traj])

            self._publish_trajectory(final_trajectory)
        except ValueError as e:
            self.get_logger().error(f"Needle trajectory verification failed: {str(e)}")

    def _concatenate_trajectories(self,trajectories):
        """
        Stitches multiple trajectories into one continuous trajectory.
        Args:
            trajectories (list): list of individual trajectories to be stitched
        Returns:
            list: fully stitched trajectory
        """
        full_trajectory = []
        t_offset = 0.0

        for idx, traj in enumerate(trajectories):
            if not traj:
                continue
            points = traj[1:] if idx > 0 else traj
            for point in points:
                full_trajectory.append({
                    'time': point['time'] + t_offset,
                    'positions': point['positions'],
                    'velocities': point['velocities'],
                    'accelerations': point['accelerations'],
                })
            t_offset = full_trajectory[-1]['time']

        return full_trajectory
        
    def _publish_trajectory(self, trajectory:list) -> None:
        """
        Flattens trajectory to Float64MultiArray and publishes it
        Args:
            trajectory (list): trajectory
        """
        if not trajectory:
            self.get_logger().warn("Trajectory is empty, nothing to publish.")
            return
        
        # Flatten data for Float64MultiArray
        flattened_data = []
        for pt in trajectory:
            flattened_data.extend(pt['positions'])

        array_msg = Float64MultiArray()
        array_msg.data = flattened_data

        n_points = len(trajectory)

        # Publish trajectory
        self.get_logger().info(f'Publishing trajectory with {n_points} steps.')
        self.joint_traj_pub.publish(array_msg)

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
