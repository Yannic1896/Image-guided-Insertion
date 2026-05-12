"""Minimal subscriber node – listens on a configurable topic."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    """Subscribes to String messages and logs them."""

    def __init__(self) -> None:
        super().__init__("listener")

        self.declare_parameter("topic", "chatter")

        topic = self.get_parameter("topic").get_parameter_value().string_value

        self.create_subscription(String, topic, self._callback, 10)
        self.get_logger().info(f"Listener started, subscribed to '{topic}'.")

    def _callback(self, msg: String) -> None:
        self.get_logger().info(f"Received: '{msg.data}'")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
