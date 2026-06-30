"""Launch the needle insertion planning node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    insertion_share = get_package_share_directory('rnm_insertion')
    kinematics_share = get_package_share_directory('rnm_kinematics')

    insertion_config = os.path.join(
        insertion_share,
        'config',
        'insertion_params.yaml'
    )
    dh_config = os.path.join(kinematics_share, 'config', 'panda_dh.yaml')
    joint_limits_config = os.path.join(
        kinematics_share,
        'config',
        'panda_joint_limits.yaml',
    )

    insertion_node = Node(
        package='rnm_insertion',
        executable='insertion_node',
        name='insertion_node',
        parameters=[
            dh_config,
            joint_limits_config,
            insertion_config,
        ],
        output='screen',
    )

    return LaunchDescription([insertion_node])
