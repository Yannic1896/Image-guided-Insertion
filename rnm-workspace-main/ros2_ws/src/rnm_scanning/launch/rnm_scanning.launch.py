"""Launch the scanning node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("rnm_scanning")
    config_file = os.path.join(package_share, "config", "parameters.yaml")

    scanning_node = Node(
        package="rnm_scanning",
        executable="scanning_node",
        name="scanning_node",
        parameters=[config_file],
        output="screen",
    )

    return LaunchDescription([scanning_node])
