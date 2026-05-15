"""
Launch file for IK Solver node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    package_share = get_package_share_directory('rnm_ik')
    dh_config = os.path.join(package_share, 'config', 'panda_dh.yaml')
    
    ik_solver_node = Node(
        package='rnm_ik',
        executable='ik_solver',
        name='ik_solver_node',
        output='screen',
        parameters=[
            dh_config,
        ],
    )
    
    return LaunchDescription([
        ik_solver_node,
    ])
