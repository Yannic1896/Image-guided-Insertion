"""Synchronizes robot movement & image acquisition."""

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Bool


class ScanningNode(Node):
    def __init__(self) -> None:
        super().__init__('scanning_node')

        self.declare_parameter('scanning_mode', 'hand_eye')  # 'hand_eye' or 'model_registration'
        self.declare_parameter('max_samples', 30)

        #TODO: Test parameters 
        self.declare_parameter('x_min', 0.40)
        self.declare_parameter('x_max', 0.85)
        self.declare_parameter('y_min', -0.30)
        self.declare_parameter('y_max', 0.30)
        self.declare_parameter('z_min', 0.40)
        self.declare_parameter('z_max', 0.70)

        self.declare_parameter('center_x', 0.2)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 0.6)

        self.declare_parameter('roll_deg', 138.8)
        self.declare_parameter('pitch_deg', 42.8)
        self.declare_parameter('yaw_deg', -104.8)

        self.scanning_mode = self.get_parameter('scanning_mode').value
        self.max_samples = self.get_parameter('max_samples').value
        self.x_bounds = (
            self.get_parameter('x_min').value,
            self.get_parameter('x_max').value,
        )
        self.y_bounds = (
            self.get_parameter('y_min').value,
            self.get_parameter('y_max').value,
        )
        self.z_bounds = (
            self.get_parameter('z_min').value,
            self.get_parameter('z_max').value,
        )

        # [x, y, z, roll_deg, pitch_deg, yaw_deg]
        self.model_reg_poses = [
            [0.40,  0.00, 0.50, 180.0,  0.0,   0.0],
            [0.45,  0.15, 0.50, 180.0,  0.0,  30.0],
            [0.45, -0.15, 0.50, 180.0,  0.0, -30.0],
            [0.35,  0.10, 0.55, 175.0,  5.0,  15.0],
            [0.35, -0.10, 0.55, 175.0,  5.0, -15.0],
            [0.50,  0.00, 0.45, 180.0,  0.0,   0.0],
        ]

        self.chessboard_visible = False
        self.trajectory_finished = False 
        self.latest_fk_pose = None
        self.startPoseSend= False
        self.sample_count = 0
        self.model_reg_index = 0

        # Subscribers
        self.fk_sub = self.create_subscription(
            PoseStamped, '/fk_pose', self.fk_callback, 10
        )
        self.detect_sub = self.create_subscription(
            Bool, '/chessboard_detected', self.detect_callback, 10
        )
        self.trajectory_status_sub = self.create_subscription(
            Bool, '/trajectory_finished', self.trajectory_callback, 10
        )

        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.trigger_image_pub = self.create_publisher(Bool, '/trigger_image_capture', 10)
        self.trigger_pointcloud_pub = self.create_publisher(Bool, '/trigger_pointcloud_capture', 10)

        self.loop_timer = self.create_timer(1.0, self.scanning_loop)
        self.get_logger().info(f"Scanning Node started in [{self.scanning_mode}] mode.")

        self.send_next_target()

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def fk_callback(self, msg: PoseStamped) -> None:
        self.latest_fk_pose = msg

    def detect_callback(self, msg: Bool) -> None:
        self.chessboard_visible = msg.data

    def trajectory_callback(self, msg: Bool) -> None:
        self.trajectory_finished = msg.data


    def scanning_loop(self) -> None:

        if not self.trajectory_finished:
            return

        if self.scanning_mode == 'hand_eye':
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
                    self.get_logger().info("Hand-eye calibration data collection complete.")
                    self.loop_timer.cancel()
                    return
            else:
                self.get_logger().warn(
                    "Chessboard not visible"
                )

        elif self.scanning_mode == 'model_registration':
            self.get_logger().info(
                f"Settled at scan point {self.model_reg_index + 1}/{len(self.model_reg_poses)} "
                f"— triggering point cloud capture."
            )
            trigger_msg = Bool()
            trigger_msg.data = True
            self.trigger_pointcloud_pub.publish(trigger_msg)
            self.model_reg_index += 1

            if self.model_reg_index >= len(self.model_reg_poses):
                self.get_logger().info("Scanning complete — all point clouds captured.")
                self.loop_timer.cancel()
                return

        self.send_next_target()

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def send_next_target(self) -> None:
        self.trajectory_finished = False

        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'panda_link0'

        if self.scanning_mode == 'hand_eye':

            if not self.startPoseSend:
                x = self.get_parameter('center_x').value
                y = self.get_parameter('center_y').value
                z = self.get_parameter('center_z').value
                roll = math.radians(self.get_parameter('roll_deg').value)
                pitch = math.radians(self.get_parameter('pitch_deg').value)
                yaw = math.radians(self.get_parameter('yaw_deg').value)
                self.startPoseSend= True
            else:
                x = random.uniform(*self.x_bounds)
                y = random.uniform(*self.y_bounds)
                z = random.uniform(*self.z_bounds)
                max_tweak = math.radians(5.0)
                roll = math.radians(180.0) + random.uniform(-max_tweak, max_tweak)
                pitch = random.uniform(-max_tweak, max_tweak)
                yaw = random.uniform(-max_tweak, max_tweak)
                self.get_logger().info(
                    f"Moving to hand-eye pose: XYZ,RYP=[{x:.3f}, {y:.3f}, {z:.3f}, {roll:.3f}, {pitch:.3f}, {yaw:.3f}]"
                )
                target_msg.pose.position.x = x
                target_msg.pose.position.y = y
                target_msg.pose.position.z = z
                target_msg.pose.orientation = self.euler_to_quaternion(roll, pitch, yaw)

        elif self.scanning_mode == 'model_registration':
            pose = self.model_reg_poses[self.model_reg_index]
            self.get_logger().info(
                f"Moving to scan point {self.model_reg_index + 1}/{len(self.model_reg_poses)}"
            )
            target_msg.pose.position.x = pose[0]
            target_msg.pose.position.y = pose[1]
            target_msg.pose.position.z = pose[2]
            target_msg.pose.orientation = self.euler_to_quaternion(
                math.radians(pose[3]),
                math.radians(pose[4]),
                math.radians(pose[5]),
            )

        self.target_pub.publish(target_msg)

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
