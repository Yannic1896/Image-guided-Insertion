"""
Launch file for IK Solver node.
 Default: launch for real robot (ros2 launch rnm_kinematics ik_solver.launch.py)
 Launch for simulation: ros2 launch rnm_kinematics ik_solver.launch.py use_sim:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    target_pose_topic_arg = DeclareLaunchArgument('target_pose_topic', default_value='/ik_target_pose')
    joint_states_topic_arg = DeclareLaunchArgument('joint_states_topic', default_value='/franka_state_controller/joint_states_desired')
    command_topic_arg = DeclareLaunchArgument('command_topic', default_value='/ik_joint_goal')
    max_step_arg = DeclareLaunchArgument('max_step', default_value='0.05')
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Set to true if running in simulation environment."
    )

    target_pose_topic = LaunchConfiguration('target_pose_topic')
    command_topic = LaunchConfiguration('command_topic')
    max_step = LaunchConfiguration('max_step')
    use_sim = LaunchConfiguration("use_sim")

    input_joint_topic = PythonExpression([
        "'/joint_states' if '", use_sim, "' == 'true' else '/franka_state_controller/joint_states_desired'"
    ])

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
                'use_sim_time': use_sim,
                'joint_states_topic': input_joint_topic,
                'target_pose_topic': target_pose_topic,
                'command_topic': command_topic,
                'max_step': max_step,
            },
        ],
    )

    return LaunchDescription([
        target_pose_topic_arg,
        joint_states_topic_arg,
        command_topic_arg,
        max_step_arg,
        use_sim_arg,
        ik_solver_node,
    ])
