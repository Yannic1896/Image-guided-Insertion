"""Trajectory planning node."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt64
from trajectory_msgs.msg import JointTrajectory
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
        self.declare_parameter('trajectory_finished_topic', '/trajectory_finished')

        self.declare_parameter('joint_names', [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ])
        self.declare_parameter('joint_velocity_limits', [0.0])
        self.declare_parameter('joint_acceleration_limits', [0.0])
        self.declare_parameter('joint_jerk_limits', [0.0])
        self.declare_parameter('safety_factor', 0.10)
        self.declare_parameter('ik_min_duration', 6.0)
        self.declare_parameter('split_needle_approach', True)
        self.declare_parameter('use_staging_pose', False)
        self.declare_parameter('staging_joint_pose', [0.0, -0.6, 0.0, -2.2, 0.0, 1.8, 0.8])
        self.declare_parameter('needle_approach_min_duration', 8.0)
        self.declare_parameter('needle_pre_entry_min_duration', 2.0)
        self.declare_parameter('needle_insertion_min_duration', 0.3)
        self.declare_parameter('needle_approach_command_delay', 5.0)
        self.declare_parameter('needle_approach_max_retries', 5)
        self.declare_parameter('needle_motion_start_threshold', 0.002)
        self.declare_parameter('needle_approach_goal_tolerance', 0.01)

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
        trajectory_finished_topic = self.get_parameter('trajectory_finished_topic').value
        self.joint_names = self.get_parameter('joint_names').value
        self.ik_min_duration = float(self.get_parameter('ik_min_duration').value)
        self.split_needle_approach = bool(self.get_parameter('split_needle_approach').value)
        self.use_staging_pose = bool(self.get_parameter('use_staging_pose').value)
        self.staging_joint_pose = list(self.get_parameter('staging_joint_pose').value)
        self.needle_approach_min_duration = float(
            self.get_parameter('needle_approach_min_duration').value
        )
        self.needle_pre_entry_min_duration = float(
            self.get_parameter('needle_pre_entry_min_duration').value
        )
        self.needle_insertion_min_duration = float(
            self.get_parameter('needle_insertion_min_duration').value
        )
        self.needle_approach_command_delay = float(
            self.get_parameter('needle_approach_command_delay').value
        )
        self.needle_approach_max_retries = int(
            self.get_parameter('needle_approach_max_retries').value
        )
        self.needle_motion_start_threshold = float(
            self.get_parameter('needle_motion_start_threshold').value
        )
        self.needle_approach_goal_tolerance = float(
            self.get_parameter('needle_approach_goal_tolerance').value
        )

        expected_joint_count = len(self.joint_names)
        if len(self.staging_joint_pose) != expected_joint_count:
            raise ValueError(
                f"staging_joint_pose must contain {expected_joint_count} values, "
                f"got {len(self.staging_joint_pose)}."
            )
        for label, limits in (
            ("joint_velocity_limits", joint_velocity_limits),
            ("joint_acceleration_limits", joint_acceleration_limits),
            ("joint_jerk_limits", joint_jerk_limits),
        ):
            if len(limits) != expected_joint_count:
                raise ValueError(
                    f"{label} must contain {expected_joint_count} values, "
                    f"got {len(limits)}."
                )

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
        self.trajectory_finished_sub = self.create_subscription(
            UInt64, trajectory_finished_topic, self.trajectory_finished_callback, 10)
            
        self.current_joint_state = None
        self.last_target_pose = None
        self.last_finished_count = -1
        self.pending_needle_approach_trajectory = None
        self.pending_needle_insertion_trajectory = None
        self.waiting_for_needle_approach = False
        self.waiting_for_needle_approach_start = False
        self.waiting_for_needle_approach_finish = False
        self.needle_approach_motion_reference = None
        self.needle_approach_goal = None
        self.needle_approach_retry_timer = None
        self.needle_approach_retry_count = 0
        self.needle_approach_finish_count = -1
        
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
            if (
                self.waiting_for_needle_approach_start
                and self._needle_approach_motion_started()
            ):
                self._handle_needle_approach_started()

    def trajectory_finished_callback(self, msg: UInt64):
        self.last_finished_count = msg.data
        if (
            self.waiting_for_needle_approach
            and self.last_finished_count > self.needle_approach_finish_count
        ):
            if (
                self.waiting_for_needle_approach_start
                and not self._needle_approach_motion_started()
                and not self._needle_approach_goal_reached()
            ):
                self.needle_approach_finish_count = self.last_finished_count
                self.get_logger().warn(
                    'trajectory_finished changed before needle approach motion '
                    'was observed; continuing approach retries.'
                )
                return
            self._cancel_needle_approach_retry_timer()
            self.waiting_for_needle_approach = False
            self.waiting_for_needle_approach_start = False
            self.waiting_for_needle_approach_finish = False
            self.pending_needle_approach_trajectory = None
            insertion_trajectory = self.pending_needle_insertion_trajectory
            self.pending_needle_insertion_trajectory = None
            self.get_logger().info(
                'Needle approach finished; publishing insertion trajectory.'
            )
            self._publish_trajectory(insertion_trajectory)

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
            trajectory = self.generator.generate_trajectory(
                path,
                self.publish_rate_hz,
                self.safety_factor,
                min_duration=self.ik_min_duration,
            )
            self._publish_trajectory(trajectory)
        except ValueError as e:
            self.get_logger().error(f"Trajectory validation failed: {str(e)}")

    def needle_path_callback(self, msg:JointTrajectory):
        """ Receive Needle Path"""
        self.get_logger().info('Received needle insertion path.')

        if self.waiting_for_needle_approach:
            self.get_logger().error(
                'Needle approach is still active; rejecting new needle path.'
            )
            return

        if self.current_joint_state is None:
            self.get_logger().error('Current joint state empty')
            return

        needle_path = []
        for index, point in enumerate(msg.points):
            positions = list(point.positions)
            if len(positions) != len(self.joint_names):
                self.get_logger().error(
                    f"Needle waypoint {index} has {len(positions)} joints, "
                    f"expected {len(self.joint_names)}."
                )
                return
            needle_path.append(positions)
            
        n_waypoints = len(needle_path)
        self.get_logger().info(f"{n_waypoints} waypoints in needle path")

        if n_waypoints < 2:
            self.get_logger().error("Not enough points in needle path.")
            return

        pre_entry_point = needle_path[0]
        entry_point = needle_path[1]

        try:
            self.get_logger().info('Generating needle approach trajectory...')
            approach_path = self._needle_approach_path(pre_entry_point)
            approach_traj = self.generator.generate_trajectory(
                approach_path,
                self.publish_rate_hz,
                self.safety_factor,
                min_duration=self.needle_approach_min_duration,
            )

            self.get_logger().info('Generating pre-entry to entry trajectory...')
            pre_to_entry_path = self.path_planner.joint_path(pre_entry_point, entry_point)
            pre_to_entry_traj = self.generator.generate_trajectory(
                pre_to_entry_path,
                self.publish_rate_hz,
                self.safety_factor,
                min_duration=self.needle_pre_entry_min_duration,
            )

            self.get_logger().info('Generating needle insertion trajectory...')
            insertion_path = needle_path[1:]
            insertion_traj = []
            if len(insertion_path) > 1:
                insertion_traj = self.generator.generate_trajectory(
                    insertion_path,
                    self.publish_rate_hz,
                    self.safety_factor,
                    min_duration=self.needle_insertion_min_duration,
                )

            insertion_trajectory = self._concatenate_trajectories([
                pre_to_entry_traj,
                insertion_traj,
            ])

            if self.split_needle_approach:
                self.pending_needle_insertion_trajectory = insertion_trajectory
                self._start_needle_approach(approach_traj)
            else:
                final_trajectory = self._concatenate_trajectories([
                    approach_traj,
                    insertion_trajectory,
                ])
                self._publish_trajectory(final_trajectory)
        except ValueError as e:
            self.get_logger().error(f"Needle trajectory verification failed: {str(e)}")

    def _needle_approach_path(self, pre_entry_point):
        path = [self.current_joint_state]
        if self.use_staging_pose:
            path.append(self.staging_joint_pose)
        path.append(pre_entry_point)
        return path

    def _start_needle_approach(self, approach_trajectory):
        self.pending_needle_approach_trajectory = approach_trajectory
        self.waiting_for_needle_approach = True
        self.waiting_for_needle_approach_start = True
        self.waiting_for_needle_approach_finish = False
        self.needle_approach_retry_count = 0
        self.needle_approach_finish_count = self.last_finished_count
        self.needle_approach_motion_reference = (
            list(self.current_joint_state)
            if self.current_joint_state is not None
            else None
        )
        self.needle_approach_goal = list(approach_trajectory[-1]['positions'])
        self._cancel_needle_approach_retry_timer()
        self.get_logger().info(
            'Publishing needle approach trajectory; will retry until robot starts moving.'
        )
        self._publish_trajectory(approach_trajectory)
        self.needle_approach_retry_timer = self.create_timer(
            self.needle_approach_command_delay,
            self._needle_approach_retry_callback,
        )

    def _needle_approach_retry_callback(self):
        if not self.waiting_for_needle_approach_start:
            self._cancel_needle_approach_retry_timer()
            return

        if self._needle_approach_motion_started():
            self._handle_needle_approach_started()
            return

        if self.needle_approach_retry_count >= self.needle_approach_max_retries:
            self._cancel_needle_approach_retry_timer()
            self._clear_pending_needle_approach()
            self.get_logger().error(
                'Needle approach did not start after '
                f'{self.needle_approach_max_retries} retries; insertion aborted.'
            )
            return

        self.needle_approach_retry_count += 1
        self.get_logger().warn(
            'Needle approach has not started; resending approach trajectory '
            f'(attempt {self.needle_approach_retry_count + 1}/'
            f'{self.needle_approach_max_retries + 1}).'
        )
        self._publish_trajectory(self.pending_needle_approach_trajectory)

    def _needle_approach_motion_started(self) -> bool:
        if (
            self.current_joint_state is None
            or self.needle_approach_motion_reference is None
        ):
            return False
        deltas = [
            abs(current - reference)
            for current, reference in zip(
                self.current_joint_state,
                self.needle_approach_motion_reference,
            )
        ]
        return max(deltas, default=0.0) >= self.needle_motion_start_threshold

    def _needle_approach_goal_reached(self) -> bool:
        if self.current_joint_state is None or self.needle_approach_goal is None:
            return False
        deltas = [
            abs(current - goal)
            for current, goal in zip(self.current_joint_state, self.needle_approach_goal)
        ]
        return max(deltas, default=float('inf')) <= self.needle_approach_goal_tolerance

    def _handle_needle_approach_started(self):
        if not self.waiting_for_needle_approach_start:
            return
        self._cancel_needle_approach_retry_timer()
        self.waiting_for_needle_approach_start = False
        self.waiting_for_needle_approach_finish = True
        self.get_logger().info(
            'Needle approach started; waiting for trajectory_finished before insertion.'
        )

    def _cancel_needle_approach_retry_timer(self):
        if self.needle_approach_retry_timer is not None:
            self.needle_approach_retry_timer.cancel()
            self.needle_approach_retry_timer = None

    def _clear_pending_needle_approach(self):
        self.waiting_for_needle_approach = False
        self.waiting_for_needle_approach_start = False
        self.waiting_for_needle_approach_finish = False
        self.pending_needle_approach_trajectory = None
        self.pending_needle_insertion_trajectory = None
        self.needle_approach_motion_reference = None
        self.needle_approach_goal = None

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
