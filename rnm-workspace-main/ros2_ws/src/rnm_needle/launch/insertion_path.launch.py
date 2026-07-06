"""Launch the needle insertion path planning node with its configuration."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 1. getting the path to the configuration files
    kinematics_share = get_package_share_directory('rnm_kinematics') # alternativ mit:  # 1. Pfad zu DEINER eigenen needle_params.yaml holen
    needle_share = get_package_share_directory('rnm_needle')
    needle_params = os.path.join(needle_share, 'config', 'needle_params.yaml')
    panda_dh_params = os.path.join(kinematics_share, 'config', 'panda_dh.yaml')
    panda_limits = os.path.join(kinematics_share, 'config', 'panda_joint_limits.yaml')
    # 2. configuring the actual InsertionPathNode
    insertion_path_node = Node(
        package='rnm_needle',
        executable='insertion_path_node',
        name='insertion_path_node',
        output='screen',
        parameters=[
            panda_dh_params,
            panda_limits,
            needle_params,   # topics, DH-Parameters and limits
        ], 
        
    )

    return LaunchDescription([insertion_path_node])