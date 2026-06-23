"""Launch the RNM camera static transform publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("end_effector_frame", default_value="panda_EE"),
            DeclareLaunchArgument("rgb_camera_frame", default_value="rgb_camera_link"),
            DeclareLaunchArgument(
                "depth_camera_frame",
                default_value="depth_camera_link",
            ),
            DeclareLaunchArgument("invert_hand_eye", default_value="false"),
            DeclareLaunchArgument("invert_rgb_depth", default_value="false"),
            Node(
                package="rnm_tools",
                executable="camera_static_tf_publisher",
                name="camera_static_tf_publisher",
                output="screen",
                parameters=[
                    {
                        "end_effector_frame": LaunchConfiguration(
                            "end_effector_frame"
                        ),
                        "rgb_camera_frame": LaunchConfiguration("rgb_camera_frame"),
                        "depth_camera_frame": LaunchConfiguration(
                            "depth_camera_frame"
                        ),
                        "invert_hand_eye": ParameterValue(
                            LaunchConfiguration("invert_hand_eye"),
                            value_type=bool,
                        ),
                        "invert_rgb_depth": ParameterValue(
                            LaunchConfiguration("invert_rgb_depth"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
