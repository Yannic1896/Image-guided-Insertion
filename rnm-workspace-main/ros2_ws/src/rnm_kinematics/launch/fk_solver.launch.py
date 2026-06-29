"""Launch file for the FK solver node."""
"""
 Default: launch for real robot (ros2 launch rnm_kinematics fk_solver.launch.py)
 Launch for simulation: ros2 launch rnm_kinematics fk_solver.launch.py use_sim:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    package_share = get_package_share_directory("rnm_kinematics")
    dh_config = os.path.join(package_share, "config", "panda_dh.yaml")
    fk_config = os.path.join(package_share, "config", "fk_solver.yaml")

    # declare simulation argument (default: false)
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Set to true if running in simulation environment."
    )

    use_sim = LaunchConfiguration("use_sim")

    input_joint_topic = PythonExpression([
        "'/joint_states' if '", use_sim, "' == 'true' else '/franka_state_controller/joint_states_desired'"
    ])

    fk_solver_node = Node(
        package="rnm_kinematics",
        executable="fk_solver",
        name="fk_solver_node",
        output="screen",
        parameters=[
            dh_config,
            fk_config,
            {"use_sim_time": use_sim,
             "joint_states_topic": input_joint_topic
            }
        ],
    )

    return LaunchDescription([
        use_sim_arg,
        fk_solver_node,
    ])
