"""Synchronizes robot movement & image acquisition."""

import numpy as np
import rclpy
from rclpy.node import Node
import random
import math

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion


class ScanningNode(Node):
    def __init__(self) -> None:
        super().__init__('scanning_node')

        self.declare_parameter('current_pose_topic', '/fk_pose')
        self.declare_parameter('trajectory_status_topic', '/trajectory_finished')
        self.declare_parameter('camera_pointcloud_topic', '/points2')

        self.declare_parameter('scanning_mode', 'hand_eye') # Options: 'hand_eye' or 'model_registration'

        #topic = self.get_parameter("topic").get_parameter_value().string_value

        self.scanning_mode = self.get_parameter('scanning_mode').value

        self.chessboard_visible = False
        self.trajectory_finished = True
        self.latest_fk_pose = None
        self.sample_count = 0
        self.max_samples = 30
        self.model_reg_index = 0

        # Subscribers
        self.fk_sub = self.create_subscription(
            PoseStamped,
            "/fk_pose",
            self.fk_callback,
            10
        )

        self.detect_sub = self.create_subscription(
            Bool,
            "/chessboard_detected",
            self.detect_callback,
            10
        )

        self.trajectory_status_sub = self.create_subscription(
            Bool,
            "/trajectory_finished",
            self.trajectory_callback,
            10
        )

        # Publishers
        self.target_pub = self.create_publisher(
            PoseStamped, 
            "/target_pose",
            10
        )

        self.trigger_image_pub = self.create_publisher(
            Bool,
            "/trigger_image_capture",
            10
        )

        self.trigger_pointcloud_pub = self.create_publisher(
            Bool,
            "/trigger_pointcloud_capture",
            10
        )

        self.loop_timer = self.create_timer(1.0, self.scanning.loop)
        self.get_logger().info(f"Scanning Node started in [{self.scanning_mode}] mode.")


    def fk_callback(self, msg: PoseStamped) -> None:
        self.latest_fk_pose = msg

    def detect_callback(self, msg: Bool) -> None:
        self.chessboard_visible = msg.data

    def trajectory_callback(self, msg: Bool) -> None:
        self.trajectory_finished = msg.data

    def scanning_loop(self) -> None:

        if not self.trajectory_finished:
            # Robot moving
            return
        
        if self.trajectory_finished:
            # Hand Eye Calibration
            if self.scanning_mode == 'hand_eye':
                if self.chessboard_visible:
                    self.get_logger().info("Robot settled and chessboard visible! Triggering capture...")
                    trigger_msg = Bool()
                    trigger_msg.data = True
                    self.trigger_image_pub.publish(trigger_msg)
                    self.sample_count += 1

                    if self.sample_count >= self.max_samples:
                        self.get_logger().info("Finished gathering hand-eye data.")
                        self.loop_timer.cancel()
                        return
                else:
                    self.get_logger().warn("Robot stopped, but chessboard not visible. Discarding pose.")

            # Model Registration
            elif self.scanning_mode == 'model_registration':
                self.get_logger().info("Settled at scan point. Triggering point cloud capture...")
                # Trigger point cloud collection code here if applicable
                self.model_reg_index += 1
                if self.model_reg_index >= len(self.model_reg_poses):
                    self.get_logger().info("Finished full scanning sweep around phantom!")
                    self.loop_timer.cancel()
                    return

            # Move on to next target
            self.send_next_target()

def send_next_target(self) -> None:
    """Calculates coordinates according to the chosen parameter tracking mode."""
    self.trajectory_finished = False # Lock loop until trajectory completes
    
    # Instantiate the correct message type
    target_msg = PoseStamped()
    target_msg.header.stamp = self.get_clock().now().to_msg()
    target_msg.header.frame_id = "panda_link0"  # Or your system's base frame ID
    
    # Hand Eye Calibration
    if self.scanning_mode == 'hand_eye':
        # Generate random coordinates in your bounding box
        x = random.uniform(*self.x_bounds)
        y = random.uniform(*self.y_bounds)
        z = random.uniform(*self.z_bounds)
        
        # Baseline looking downward orientation (Roll=180 deg) +/- 5 deg wiggle room
        max_tweak = math.radians(5.0)
        roll = math.radians(180.0) + random.uniform(-max_tweak, max_tweak)
        pitch = 0.0 + random.uniform(-max_tweak, max_tweak)
        yaw = 0.0 + random.uniform(-max_tweak, max_tweak)
        
        self.get_logger().info(f"Sending automated Hand-Eye Target: XYZ=[{x:.2f}, {y:.2f}, {z:.2f}]")
        
        # Populate position
        target_msg.pose.position.x = x
        target_msg.pose.position.y = y
        target_msg.pose.position.z = z
        
        # Populate orientation using the helper function
        target_msg.pose.orientation = self.euler_to_quaternion(roll, pitch, yaw)


    # Model Registration
    elif self.scanning_mode == 'model_registration':
        pose = self.model_reg_poses[self.model_reg_index]
        self.get_logger().info(f"Moving to phantom scan point {self.model_reg_index + 1}/{len(self.model_reg_poses)}")
        
        target_msg.pose.position.x = pose[0]
        target_msg.pose.position.y = pose[1]
        target_msg.pose.position.z = pose[2]
        
        # Convert degrees to radians for model_reg_poses
        r = math.radians(pose[3])
        p = math.radians(pose[4])
        y = math.radians(pose[5])
        target_msg.pose.orientation = self.euler_to_quaternion(r, p, y)

    # Publish target pose for trajectory generation
    self.target_pub.publish(target_msg)

def euler_to_quaternion(self, r, p, y):
    """Converts euler angles (radians) to a geometry_msgs Quaternion format."""
    
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)

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
