"""Launch file for the FK solver node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    package_share = get_package_share_directory("rnm_ik")
    dh_config = os.path.join(package_share, "config", "panda_dh.yaml")

    fk_solver_node = Node(
        package="rnm_ik",
        executable="fk_solver",
        name="fk_solver_node",
        output="screen",
        parameters=[
            dh_config,
        ],
    )

    return LaunchDescription([
        fk_solver_node,
    ])
