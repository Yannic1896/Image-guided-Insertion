"""Launch both the talker and listener nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    topic_arg = DeclareLaunchArgument(
        "topic",
        default_value="chatter",
        description="Topic name for the talker/listener pair.",
    )
    rate_arg = DeclareLaunchArgument(
        "rate_hz",
        default_value="1.0",
        description="Publishing rate in Hz.",
    )

    topic = LaunchConfiguration("topic")
    rate_hz = LaunchConfiguration("rate_hz")

    talker_node = Node(
        package="rnm_sample",
        executable="talker",
        name="talker",
        parameters=[{"topic": topic, "rate_hz": rate_hz}],
        output="screen",
    )

    listener_node = Node(
        package="rnm_sample",
        executable="listener",
        name="listener",
        parameters=[{"topic": topic}],
        output="screen",
    )

    return LaunchDescription([topic_arg, rate_arg, talker_node, listener_node])
