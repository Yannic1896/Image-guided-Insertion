"""Synchronizes robot movement & image acquisition."""

import math
import os
import random

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, UInt64


class ScanningNode(Node):
    def __init__(self) -> None:
        super().__init__("scanning_node")

        self.declare_parameter(
            "scanning_mode", "hand_eye"
        )  # 'hand_eye' or 'model_registration'
        self.declare_parameter("max_samples", 30)
        self.declare_parameter("command_delay", 5.0)
        self.declare_parameter("max_retries", 5)

        # Bounding box of robot workspace
        self.declare_parameter("x_min", 0.40)
        self.declare_parameter("x_max", 0.85)
        self.declare_parameter("y_min", -0.30)
        self.declare_parameter("y_max", 0.30)
        self.declare_parameter("z_min", 0.40)
        self.declare_parameter("z_max", 0.70)

        self.declare_parameter("center_x", 0.2)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.6)

        self.declare_parameter("roll_deg", 138.8)
        self.declare_parameter("pitch_deg", 42.8)
        self.declare_parameter("yaw_deg", -104.8)

        _default_poses_file = os.path.join(
            get_package_share_directory("rnm_scanning"),
            "config",
            "stitching poses.yaml",
        )
        self.declare_parameter("stitching_poses_file", _default_poses_file)

        self.scanning_mode = self.get_parameter("scanning_mode").value
        self.max_samples = self.get_parameter("max_samples").value
        self.command_delay = self.get_parameter("command_delay").value
        self.max_retries = self.get_parameter("max_retries").value

        self.x_bounds = (
            self.get_parameter("x_min").value,
            self.get_parameter("x_max").value,
        )
        self.y_bounds = (
            self.get_parameter("y_min").value,
            self.get_parameter("y_max").value,
        )
        self.z_bounds = (
            self.get_parameter("z_min").value,
            self.get_parameter("z_max").value,
        )

        poses_file = self.get_parameter("stitching_poses_file").value
        self.model_reg_joints = self._load_stitching_poses(poses_file)

        self.start_roll = math.radians(self.get_parameter("roll_deg").value)
        self.start_pitch = math.radians(self.get_parameter("pitch_deg").value)
        self.start_yaw = math.radians(self.get_parameter("yaw_deg").value)
        self.max_orientation_tweak = math.radians(5.0)

        self.chessboard_visible = False
        self.startPoseSend = False
        self.sample_count = 0
        self.model_reg_index = 0
        self.resend_timer = None
        self.last_finished_count = -1
        self.count_at_last_send = -1
        self.number_retries = 0         #number of times we resend the same pose 
        self.current_target_msg = None  # for hand-eye
        self.current_joint_msg = None  # for model-registration

        # Subscribers
        self.detect_sub = self.create_subscription(
            Bool, "/chessboard_detected", self.detect_callback, 10
        )
        self.trajectory_finished_sub = self.create_subscription(
            UInt64, "/trajectory_finished", self.trajectory_finished_callback, 10
        )

        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.ik_joint_goal_pub = self.create_publisher(
            Float64MultiArray, "/ik_joint_goal", 10
        )
        self.trigger_image_pub = self.create_publisher(
            Bool, "/trigger_image_capture", 10
        )
        self.trigger_pointcloud_pub = self.create_publisher(
            Bool, "/trigger_pointcloud_capture", 10
        )

        self.get_logger().info(f"Scanning Node started in [{self.scanning_mode}] mode.")

        self.send_next_target()

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def detect_callback(self, msg: Bool) -> None:
        self.chessboard_visible = msg.data

    def trajectory_finished_callback(self, msg: UInt64) -> None:
        # /trajectory_finished is a UInt64 counter that increments for every trajectory completion
        self.last_finished_count = msg.data

    def _has_arrived(self) -> bool:
        # Compare if new trajectory finished
        return self.last_finished_count > self.count_at_last_send
    
    def _resend_current_command(self) -> None:
        if self._has_arrived() or self.number_retries >= self.max_retries:
            self.resend_timer.cancel()
            self.resend_timer = None
            self.scanning_loop()
            return
        self.number_retries +=1

        if self.scanning_mode == "hand_eye":
            self.get_logger().info(
                "Resending target pose (trajectory not finished yet)."
            )
            self.target_pub.publish(self.current_target_msg)
        elif self.scanning_mode == "model_registration":
            self.get_logger().info(
                "Resending joint goal (trajectory not finished yet)."
            )
            self.ik_joint_goal_pub.publish(self.current_joint_msg)


    def scanning_loop(self) -> None:
        if self.scanning_mode == "hand_eye":
            if self.chessboard_visible:
                self.get_logger().info(
                    f"Robot settled and chessboard visible — triggering image capture "
                    f"({self.sample_count + 1}/{self.max_samples})."
                )
                trigger_msg = Bool()
                trigger_msg.data = True
                self.trigger_image_pub.publish(trigger_msg)
                self.sample_count += 1

                if self.sample_count >= self.max_samples:
                    self.get_logger().info(
                        "Hand-eye calibration data collection complete."
                    )
                    return
            else:

                self.get_logger().warn("Chessboard not visible")

        elif self.scanning_mode == "model_registration":
            self.get_logger().info(
                f"Settled at scan point {self.model_reg_index + 1}/{len(self.model_reg_joints)} "
                f"— triggering point cloud capture."
            )
            trigger_msg = Bool()
            trigger_msg.data = True
            self.trigger_pointcloud_pub.publish(trigger_msg)
            self.model_reg_index += 1

            if self.model_reg_index >= len(self.model_reg_joints):
                self.get_logger().info("Scanning complete — all point clouds captured.")
                return

        self.send_next_target()

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def send_next_target(self) -> None:
        
        self.count_at_last_send = self.last_finished_count
        self.number_retries = 0

        if self.resend_timer is not None:
            self.resend_timer.cancel()
            self.resend_timer = None

        if self.scanning_mode == "hand_eye":
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = "panda_link0"

            if not self.startPoseSend:
                x = self.get_parameter("center_x").value
                y = self.get_parameter("center_y").value
                z = self.get_parameter("center_z").value
                roll = self.start_roll
                pitch = self.start_pitch
                yaw = self.start_yaw
                self.startPoseSend = True
            else:
                x = random.uniform(*self.x_bounds)
                y = random.uniform(*self.y_bounds)
                z = random.uniform(*self.z_bounds)
                t = self.max_orientation_tweak
                roll = self.start_roll + random.uniform(-t, t)
                pitch = self.start_pitch + random.uniform(-t, t)
                yaw = self.start_yaw + random.uniform(-t, t)
                self.get_logger().info(
                    f"Moving to hand-eye pose: XYZ,RYP=[{x:.3f}, {y:.3f}, {z:.3f}, {roll:.3f}, {pitch:.3f}, {yaw:.3f}]"
                )

            target_msg.pose.position.x = x
            target_msg.pose.position.y = y
            target_msg.pose.position.z = z
            target_msg.pose.orientation = self.euler_to_quaternion(roll, pitch, yaw)

            self.current_target_msg = target_msg
            self.target_pub.publish(target_msg)
            self.resend_timer = self.create_timer(
                self.command_delay, self._resend_current_command
            )

        elif self.scanning_mode == "model_registration":
            if self.model_reg_index >= len(self.model_reg_joints):
                self.get_logger().info("No more poses left.")
                return
            joints = self.model_reg_joints[self.model_reg_index]
            self.get_logger().info(
                f"Moving to scan point {self.model_reg_index + 1}/{len(self.model_reg_joints)}"
            )
            joint_msg = Float64MultiArray()
            joint_msg.data = joints

            self.current_joint_msg = joint_msg
            self.ik_joint_goal_pub.publish(joint_msg)
            self.resend_timer = self.create_timer(
                self.command_delay, self._resend_current_command
            )

    def _load_stitching_poses(self, path: str) -> list[list[float]]:
        """Parse a multi-document JointState YAML and return a list of position arrays."""
        with open(path, "r") as f:
            docs = [d for d in yaml.safe_load_all(f) if d and "position" in d]
        self.get_logger().info(f"Loaded {len(docs)} stitching poses from '{path}'")
        return [doc["position"] for doc in docs]

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
