from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("viser_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("viser_port", default_value="8080"),
            Node(
                package="rnm_tools",
                executable="urdf_visualizer",
                name="urdf_visualizer",
                output="screen",
                parameters=[
                    {
                        "viser_host": LaunchConfiguration("viser_host"),
                        "viser_port": LaunchConfiguration("viser_port"),
                    }
                ],
            ),
        ]
    )
