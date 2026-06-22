"""Publish saved Azure Kinect camera calibration data."""

from __future__ import annotations

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from tf2_ros import StaticTransformBroadcaster

from rnm_calibration.calibration_io import load_calibration_file
from rnm_calibration.calibration_io import matrix_from_field
from rnm_calibration.calibration_io import vector_from_field
from rnm_calibration.calibration_math import rotation_matrix_to_quaternion


class CalibrationPublisherNode(Node):
    """Load calibration YAML and publish CameraInfo plus static TF."""

    def __init__(self) -> None:
        super().__init__("azure_kinect_calibration_publisher")
        self._declare_parameters()

        calibration_file = self._parameter_string("calibration_file")
        self._calibration = load_calibration_file(calibration_file)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._rgb_info = _camera_info_from_section(
            self._calibration["rgb_camera"]
        )
        self._depth_info = _camera_info_from_section(
            self._calibration["depth_camera"]
        )
        self._rgb_info_pub = self.create_publisher(
            CameraInfo,
            self._parameter_string("rgb_camera_info_topic"),
            qos,
        )
        self._depth_info_pub = self.create_publisher(
            CameraInfo,
            self._parameter_string("depth_camera_info_topic"),
            qos,
        )

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        if self._parameter_bool("publish_tf"):
            self._broadcast_static_transform()

        publish_rate_hz = self._parameter_float("camera_info_publish_rate_hz")
        if publish_rate_hz > 0.0:
            self.create_timer(1.0 / publish_rate_hz, self._publish_camera_info)
        self._publish_camera_info()

        self.get_logger().info(
            f"Loaded calibration from {calibration_file} "
            "and started publishing."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "calibration_file",
            "~/.ros/rnm_calibration/azure_kinect_calibration.yaml",
        )
        self.declare_parameter("rgb_camera_info_topic", "/rgb/camera_info")
        self.declare_parameter("depth_camera_info_topic", "/depth/camera_info")
        self.declare_parameter("camera_info_publish_rate_hz", 1.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("tf_parent_frame_id", "")
        self.declare_parameter("tf_child_frame_id", "")

    def _publish_camera_info(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._rgb_info.header.stamp = stamp
        self._depth_info.header.stamp = stamp
        self._rgb_info_pub.publish(self._rgb_info)
        self._depth_info_pub.publish(self._depth_info)

    def _broadcast_static_transform(self) -> None:
        extrinsics = self._calibration["extrinsics"]
        parent_frame = self._parameter_string("tf_parent_frame_id")
        child_frame = self._parameter_string("tf_child_frame_id")
        if not parent_frame:
            parent_frame = str(extrinsics["parent_frame_id"])
        if not child_frame:
            child_frame = str(extrinsics["child_frame_id"])

        rotation = matrix_from_field(extrinsics["rotation_depth_to_rgb"])
        translation = vector_from_field(extrinsics["translation_depth_to_rgb"])
        quaternion = rotation_matrix_to_quaternion(rotation)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self._tf_broadcaster.sendTransform(transform)

    def _parameter_string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _parameter_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _parameter_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def _camera_info_from_section(section) -> CameraInfo:
    info = CameraInfo()
    info.header.frame_id = str(section["frame_id"])
    info.width = int(section["image_width"])
    info.height = int(section["image_height"])
    info.distortion_model = str(section["distortion_model"])
    info.k = _float_list(matrix_from_field(section["camera_matrix"]))
    info.d = _float_list(matrix_from_field(section["distortion_coefficients"]))
    info.r = _float_list(matrix_from_field(section["rectification_matrix"]))
    info.p = _float_list(matrix_from_field(section["projection_matrix"]))
    return info


def _float_list(values) -> list[float]:
    return [float(value) for value in values.reshape(-1)]


def main(args=None) -> None:
    """Run the calibration publisher node."""
    rclpy.init(args=args)
    node = CalibrationPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
