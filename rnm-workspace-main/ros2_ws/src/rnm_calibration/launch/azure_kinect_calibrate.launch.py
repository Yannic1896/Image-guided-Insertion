"""Launch the Azure Kinect calibration capture node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the calibration capture launch description."""
    package_share = get_package_share_directory("rnm_calibration")
    default_config = os.path.join(
        package_share,
        "config",
        "azure_kinect_calibration.yaml",
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the calibration ROS parameter file.",
        ),
        Node(
            package="rnm_calibration",
            executable="azure_kinect_calibrate",
            name="azure_kinect_calibrator",
            output="screen",
            parameters=[config_file],
        ),
    ])
