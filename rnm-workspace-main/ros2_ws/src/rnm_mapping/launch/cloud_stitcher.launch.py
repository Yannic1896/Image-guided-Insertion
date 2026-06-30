"""Launch the RNM point cloud stitcher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("rnm_mapping"),
            "config",
            "cloud_stitcher.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to the cloud stitcher parameter YAML file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="rnm_mapping",
                executable="cloud_stitcher",
                name="cloud_stitcher",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
