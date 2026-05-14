"""
Launch file for IK Solver node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    
    ik_solver_node = Node(
        package='rnm_ik',
        executable='ik_solver',
        name='ik_solver_node',
        output='screen',
        parameters=[
            {'robot_description': '/robot_description'},
            {'frame_id': 'base_link'},
            {'end_effector_frame': 'tool0'},
        ],
    )
    
    return LaunchDescription([
        ik_solver_node,
    ])
