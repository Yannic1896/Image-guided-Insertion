import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from rnm_kinematics.fk_solver import quaternion_from_rotation_matrix



class ChessboardNode(Node):
    """Calculates Transformation from Camera to Chessboard"""

    def __init__(self) -> None:
        super().__init__("chessboard_node")

        # Chessboard Parameters
        self.rows = 7               # Number of inner corners
        self.cols = 9               # Not number of squares
        self.square_size = 0.025    # In meter

        # Camera data
        self.camera_matrix = None
        self.distortion_coeffs = None

        # OpenCV bridge for ROS
        self.bridge = CvBridge()

        # Create chessboard 3D points
        self.object_points = self.create_object_points()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            "/k4a/rgb/image_raw",
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/k4a/rgb/camera_info",
            self.camera_info_callback,
            10
        )

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/chessboard_pose",
            10
        )

        self.detected_pub = self.create_publisher(
            Bool,
            "/chessboard_detected",
            10
        )
        
        self.get_logger().info("Chessboard Node started")


    def create_object_points(self):
        '''Create the known 3D coordinates of the chessboard'''

        points = np.zeros((self.rows*self.cols,3),dtype=np.float32)
        points[:,:2] = np.mgrid[0:self.cols,0:self.rows].T.reshape(-1,2)
        points *= self.square_size
        return points

    def camera_info_callback(self,msg):
        '''Safe the intrinsics of the camera'''

        if self.camera_matrix is not None:
            return
        
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.distortion_coeffs = np.array(msg.d)
        self.get_logger().info("Camera calibration received")


    def image_callback(self,msg):
        '''Called for every Image'''

        if self.camera_matrix is None:
            return
        
        # Turn ROS image to OpenCV Image
        image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        detected_msg = Bool()

        # Chessboard detection
        found, corners = cv2.findChessboardCornersSB(gray, (self.cols,self.rows))
        if not found:
            detected_msg.data = False
            self.detected_pub.publish(detected_msg)
            return
        
        detected_msg.data = True
        self.detected_pub.publish(detected_msg)
        
        # More accurate corners
        # corners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),(
        #     cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
        # ))

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(self.object_points, corners, self.camera_matrix, self.distortion_coeffs)
        if not success:
            return
        
        # Publish the pose
        self.publish_pose(rvec,tvec,msg.header.stamp)


    def publish_pose(self, rvec, tvec, timestamp):
        # Rotation vector to Matrix
        R,_ = cv2.Rodrigues(rvec)

        # Matrix to Quaternion
        q = quaternion_from_rotation_matrix(R)

        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = ("rgb_camera_link")

        # Translation
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        # Rotation
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_pub.publish(pose)

def main(args = None):
    rclpy.init(args=args)
    node = ChessboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()