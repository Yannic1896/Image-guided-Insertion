"""Launch the trajectory planning node.
 Default: launch for real robot (ros2 launch rnm_trajectory start_trajectory.launch.py)
 Launch for simulation: ros2 launch rnm_trajectory start_trajectory.launch.py use_sim:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("rnm_trajectory")
    trajectory_config = os.path.join(package_share, "config", "trajectory_node.yaml")

    target_pose_topic_arg = DeclareLaunchArgument(
        "target_pose_topic",
        default_value="/target_pose",
        description="Target pose topic to subscribe to.",
    )
    joint_states_topic_arg = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/franka_state_controller/joint_states_desired",
        description="Joint states topic to subscribe to.",
    )
    ik_target_pose_topic_arg = DeclareLaunchArgument(
        "ik_target_pose_topic",
        default_value="/ik_target_pose",
        description="Topic to publish IK target pose.",
    )
    ik_joint_goal_topic_arg = DeclareLaunchArgument(
        "ik_joint_goal_topic",
        default_value="/ik_joint_goal",
        description="Topic to subscribe for IK joint goal.",
    )
    joint_trajectory_topic_arg = DeclareLaunchArgument(
        "joint_trajectory_topic",
        default_value="/joint_position_example_controller/joint_trajectory_command",
        description="Topic to publish joint trajectory.",
    )
    planning_mode_arg = DeclareLaunchArgument(
        "planning_mode",
        default_value="joint",
        description="Planning mode: 'joint' or 'cartesian'",
    )
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Set to true if running in simulation environment."
    )
    use_sim = LaunchConfiguration("use_sim")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    ik_target_pose_topic = LaunchConfiguration("ik_target_pose_topic")
    ik_joint_goal_topic = LaunchConfiguration("ik_joint_goal_topic")
    joint_trajectory_topic = LaunchConfiguration("joint_trajectory_topic")
    planning_mode = LaunchConfiguration("planning_mode")

    input_joint_topic = PythonExpression([
        "'/joint_states' if '", use_sim, "' == 'true' else '", joint_states_topic, "'"
    ])

    trajectory_node = Node(
        package="rnm_trajectory",
        executable="trajectory_node",
        name="trajectory_planning_node",
        parameters=[
            trajectory_config,
            {
            "use_sim_time": use_sim,
            "joint_states_topic": input_joint_topic,
            "target_pose_topic": target_pose_topic,
            "ik_target_pose_topic": ik_target_pose_topic,
            "ik_joint_goal_topic": ik_joint_goal_topic,
            "joint_trajectory_topic": joint_trajectory_topic,
            "planning_mode": planning_mode,
            }
        ],
        output="screen",
    )

    return LaunchDescription([
        target_pose_topic_arg,
        joint_states_topic_arg,
        ik_target_pose_topic_arg,
        ik_joint_goal_topic_arg,
        joint_trajectory_topic_arg,
        planning_mode_arg,
        use_sim_arg,
        trajectory_node
    ])
