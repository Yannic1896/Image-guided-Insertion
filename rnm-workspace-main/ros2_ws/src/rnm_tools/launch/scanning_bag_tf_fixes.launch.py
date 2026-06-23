"""Publish static TF fixes needed by the recorded scanning bag."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("parent_frame", default_value="panda_link7"),
            DeclareLaunchArgument("child_frame", default_value="panda_link8"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="panda_link7_to_link8_identity",
                output="screen",
                arguments=[
                    "--frame-id",
                    LaunchConfiguration("parent_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("child_frame"),
                ],
            ),
        ]
    )
