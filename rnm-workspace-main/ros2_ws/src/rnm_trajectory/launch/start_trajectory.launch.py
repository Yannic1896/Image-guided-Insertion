"""Launch the trajectory planning node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    target_pose_topic_arg = DeclareLaunchArgument(
        "target_pose_topic",
        default_value="/target_pose",
        description="Target pose topic to subscribe to.",
    )
    joint_states_topic_arg = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/joint_states",
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

    trajectory_node = Node(
        package="rnm_trajectory",
        executable="trajectory_node",
        name="trajectory_planning_node",
        parameters=[
            "config/trajectory_node.yaml"
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
        trajectory_node
    ])
