"""Minimal publisher for the Franka joint position example controller."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointCommandPublisher(Node):
    """Publish a gentle sinusoidal joint command around the current pose."""

    def __init__(self) -> None:
        super().__init__("joint_command_publisher")

        self.declare_parameter("arm_id", "panda")
        self.declare_parameter(
            "command_topic", "/joint_position_example_controller/joint_command"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("amplitude_rad", 0.05)
        self.declare_parameter("motion_frequency_hz", 0.25)

        arm_id = self.get_parameter("arm_id").value
        command_topic = self.get_parameter("command_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._amplitude = float(self.get_parameter("amplitude_rad").value)
        self._motion_frequency = float(
            self.get_parameter("motion_frequency_hz").value
        )

        self._expected_joint_names = [f"{arm_id}_joint{i}" for i in range(1, 8)]
        self._initial_positions: Optional[list[float]] = None
        self._start_time: Optional[float] = None
        self._waiting_logged = False

        self._publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_states_callback,
            10,
        )
        self.create_timer(1.0 / rate_hz, self._publish_command)

        self.get_logger().info(
            f"Publishing gentle joint commands to {command_topic} after receiving {joint_states_topic}."
        )

    def _joint_states_callback(self, msg: JointState) -> None:
        joint_map = {
            name: position for name, position in zip(msg.name, msg.position)
        }
        if not all(name in joint_map for name in self._expected_joint_names):
            return

        if self._initial_positions is None:
            self._initial_positions = [joint_map[name] for name in self._expected_joint_names]
            self._start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                "Captured initial joint positions. Starting sinusoidal commands."
            )

    def _publish_command(self) -> None:
        if self._initial_positions is None or self._start_time is None:
            if not self._waiting_logged:
                self.get_logger().info("Waiting for the first complete /joint_states message.")
                self._waiting_logged = True
            return

        time_now = self.get_clock().now().nanoseconds / 1e9
        phase = 2.0 * math.pi * self._motion_frequency * (time_now - self._start_time)
        offset = self._amplitude * math.sin(phase)

        command = list(self._initial_positions)
        command[0] += offset
        command[3] -= offset

        msg = Float64MultiArray()
        msg.data = command
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
