"""Launch the scanning node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scanning_mode_arg = DeclareLaunchArgument(
        "scanning_mode",
        default_value="hand_eye",
        description="Scanning mode: 'hand_eye' or 'model_registration'.",
    )

    scanning_node = Node(
        package="rnm_scanning",
        executable="scanning_node",
        name="scanning_node",
        parameters=[{"scanning_mode": LaunchConfiguration("scanning_mode")}],
        output="screen",
    )

    return LaunchDescription([scanning_mode_arg, scanning_node])
