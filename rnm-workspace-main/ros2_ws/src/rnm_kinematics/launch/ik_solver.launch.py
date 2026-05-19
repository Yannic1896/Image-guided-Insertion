"""
Launch file for IK Solver node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description."""
    package_share = get_package_share_directory('rnm_kinematics')
    dh_config = os.path.join(package_share, 'config', 'panda_dh.yaml')
    joint_limits_config = os.path.join(
        package_share,
        'config',
        'panda_joint_limits.yaml',
    )
    ik_config = os.path.join(package_share, 'config', 'ik_solver.yaml')

    target_pose_topic = LaunchConfiguration('target_pose_topic')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    command_topic = LaunchConfiguration('command_topic')
    max_step = LaunchConfiguration('max_step')

    ik_solver_node = Node(
        package='rnm_kinematics',
        executable='ik_solver',
        name='ik_solver_node',
        output='screen',
        parameters=[
            dh_config,
            joint_limits_config,
            ik_config,
            {
                'target_pose_topic': ParameterValue(target_pose_topic, value_type=str),
                'joint_states_topic': ParameterValue(joint_states_topic, value_type=str),
                'command_topic': ParameterValue(command_topic, value_type=str),
                'max_step': ParameterValue(max_step, value_type=float),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_pose_topic', default_value='target_pose'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/joint_position_example_controller/joint_command',
        ),
        DeclareLaunchArgument('max_step', default_value='0.05'),
        ik_solver_node,
    ])
