"""Minimal publisher node – publishes a String message on a configurable topic."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    """Publishes a counter message at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("talker")

        self.declare_parameter("topic", "chatter")
        self.declare_parameter("rate_hz", 1.0)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value

        self._pub = self.create_publisher(String, topic, 10)
        self._count = 0
        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(f"Talker started, publishing on '{topic}' at {rate_hz} Hz.")

    def _publish(self) -> None:
        msg = String()
        msg.data = f"Hello RNM! count={self._count}"
        self._pub.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self._count += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
