import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

import numpy as np
import os
import threading

from rnm_handeye.calc_handeye import calculate_handeye

from ament_index_python.packages import get_package_share_directory


class CollectorNode(Node):
    """Collects the Roboter Endeffector Pose and Chessboard Poses"""

    def __init__(self) -> None:
        super().__init__("collector_node")

        self.robot_poses = []
        self.camera_poses = []
        self.images = []

        self.latest_fk_pose = None
        self.latest_camera_pose = None
        self.latest_image = None
        self.chessboard_visible = False
        self.samples = 0

        self.bridge = CvBridge()

        # Safe location
        package_path = get_package_share_directory("rnm_handeye")
        self.save_path = os.path.join(package_path,"config")
        self.save_file = os.path.join(self.save_path,"handeye_samples.npz")

        os.makedirs(self.save_path, exist_ok=True)

        self.load_data()

        # Subscribers
        self.fk_sub = self.create_subscription(
            PoseStamped,
            "/fk_pose",
            self.fk_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            "/k4a/rgb/image_raw",
            self.image_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            PoseStamped,
            "/chessboard_pose",
            self.camera_callback,
            10
        )

        self.detect_sub = self.create_subscription(
            Bool,
            "/chessboard_detected",
            self.detect_callback,
            10
        )

        self.trigger_sub = self.create_subscription(
            Bool,
            "/trigger_image_capture",
            self.trigger_callback,
            10
        )

        self.get_logger().info("Handeye collector started")
        self.get_logger().info("Collector ready. ENTER for manual capture, 'exit' to quit")

        # Keyboard thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()


    def fk_callback(self,msg):
        self.latest_fk_pose = msg

    def camera_callback(self,msg):
        self.latest_camera_pose = msg

    def detect_callback(self,msg):
        self.chessboard_visible = msg.data

    def image_callback(self,msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def trigger_callback(self,msg):
        if msg.data:
            self.get_logger().info("Triggering image capture")
            self.capture_sample()


    def keyboard_loop(self):
        while rclpy.ok():
            command = input()
            if command == "":
                self.capture_sample()
            elif command.lower() == "exit":
                self.get_logger().info("Stopping collector node...")
                # self.destroy_node()
                rclpy.shutdown()
                break
            else:
                self.get_logger().info("Unknown command. Use ENTER for sample or 'exit' to quit")


    
    def capture_sample(self):
        if self.latest_fk_pose is None:
            self.get_logger().warn("No FK pose received")
            return
        
        if self.latest_camera_pose is None:
            self.get_logger().warn("No chessboard pose received")
            return
        
        if not self.chessboard_visible:
            self.get_logger().warn("Chessboard not detected")
            return
        
        if self.latest_image is None:
            self.get_logger().warn("No camera image received")
            return
        
        image = self.latest_image.copy()
        robot_T = self.pose_to_matrix(self.latest_fk_pose)
        camera_T = self.pose_to_matrix(self.latest_camera_pose)

        image_path = os.path.join(self.save_path, f"image_{len(self.images):03d}.png")
        cv2.imwrite(image_path, image)

        self.images.append(image_path)
        self.robot_poses.append(robot_T)
        self.camera_poses.append(camera_T)

        self.save_data()

        self.get_logger().info(f"Saved sample {len(self.robot_poses)}")

    def load_data(self):
        if os.path.exists(self.save_file):
            with np.load(self.save_file, allow_pickle=True) as data:
                self.robot_poses = list(data["gripper2base_poses"])
                self.camera_poses = list(data["target2cam_poses"])

                if "images" in data.files:
                    self.images=list(data["images"])

                self.get_logger().info(f"Loaded {len(self.robot_poses)} old samples")

    def save_data(self):
        np.savez(self.save_file, gripper2base_poses=np.array(self.robot_poses),
                    target2cam_poses=np.array(self.camera_poses),
                    images=np.array(self.images))
        
    def pose_to_matrix(self, pose):
        p = pose.pose.position
        q = pose.pose.orientation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            return np.eye(4)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = [p.x, p.y, p.z]
        return T

    def destroy_node(self):
        self.save_data()
        calculate_handeye()
        super().destroy_node()


    


def main(args = None):
    rclpy.init(args=args)
    node = CollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()