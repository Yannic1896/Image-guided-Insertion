"""Publish static camera transforms for the RNM Panda/Azure Kinect setup."""

from __future__ import annotations

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


EE_TO_RGB_ROTATION = (
    (0.672511, -0.0158563, 0.739904),
    (-0.740077, -0.0113785, 0.672421),
    (-0.00224321, -0.999799, -0.0193848),
)
EE_TO_RGB_TRANSLATION_M = (0.0576149, 0.0134255, 0.043355)

RGB_TO_DEPTH_ROTATION = (
    (0.999999, -0.000933465, -0.00143222),
    (0.00105493, 0.996182, 0.087296),
    (0.00134526, -0.0872974, 0.996181),
)
RGB_TO_DEPTH_TRANSLATION_M = (-0.0319686, -0.00227377, 0.00394809)


class CameraStaticTfPublisher(Node):
    """Publish the hand-eye and RGB/depth static transform chain."""

    def __init__(self) -> None:
        super().__init__("camera_static_tf_publisher")
        self.declare_parameter("end_effector_frame", "panda_EE")
        self.declare_parameter("rgb_camera_frame", "rgb_camera_link")
        self.declare_parameter("depth_camera_frame", "depth_camera_link")
        self.declare_parameter("invert_hand_eye", False)
        self.declare_parameter("invert_rgb_depth", False)

        ee_to_rgb_translation = EE_TO_RGB_TRANSLATION_M
        ee_to_rgb_rotation = EE_TO_RGB_ROTATION
        if self._parameter_bool("invert_hand_eye"):
            ee_to_rgb_translation, ee_to_rgb_rotation = _invert_transform(
                ee_to_rgb_translation,
                ee_to_rgb_rotation,
            )

        rgb_to_depth_translation = RGB_TO_DEPTH_TRANSLATION_M
        rgb_to_depth_rotation = RGB_TO_DEPTH_ROTATION
        if self._parameter_bool("invert_rgb_depth"):
            rgb_to_depth_translation, rgb_to_depth_rotation = _invert_transform(
                rgb_to_depth_translation,
                rgb_to_depth_rotation,
            )

        transforms = [
            self._make_transform(
                self._parameter_string("end_effector_frame"),
                self._parameter_string("rgb_camera_frame"),
                ee_to_rgb_translation,
                ee_to_rgb_rotation,
            ),
            self._make_transform(
                self._parameter_string("rgb_camera_frame"),
                self._parameter_string("depth_camera_frame"),
                rgb_to_depth_translation,
                rgb_to_depth_rotation,
            ),
        ]

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            "Published static transforms: "
            f"{transforms[0].header.frame_id} -> {transforms[0].child_frame_id}, "
            f"{transforms[1].header.frame_id} -> {transforms[1].child_frame_id}."
        )

    def _make_transform(
        self,
        parent_frame: str,
        child_frame: str,
        translation: tuple[float, float, float],
        rotation: tuple[tuple[float, float, float], ...],
    ) -> TransformStamped:
        quaternion = _rotation_matrix_to_quaternion(rotation)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        return transform

    def _parameter_string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _parameter_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def _invert_transform(
    translation: tuple[float, float, float],
    rotation: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], tuple[tuple[float, float, float], ...]]:
    inverse_rotation = tuple(
        tuple(rotation[row][col] for row in range(3)) for col in range(3)
    )
    inverse_translation = tuple(
        -sum(inverse_rotation[row][col] * translation[col] for col in range(3))
        for row in range(3)
    )
    return inverse_translation, inverse_rotation


def _rotation_matrix_to_quaternion(
    rotation: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to an x, y, z, w quaternion."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]

    if trace > 0.0:
        scale = (trace + 1.0) ** 0.5 * 2.0
        w_value = 0.25 * scale
        x_value = (rotation[2][1] - rotation[1][2]) / scale
        y_value = (rotation[0][2] - rotation[2][0]) / scale
        z_value = (rotation[1][0] - rotation[0][1]) / scale
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = (
            1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]
        ) ** 0.5 * 2.0
        w_value = (rotation[2][1] - rotation[1][2]) / scale
        x_value = 0.25 * scale
        y_value = (rotation[0][1] + rotation[1][0]) / scale
        z_value = (rotation[0][2] + rotation[2][0]) / scale
    elif rotation[1][1] > rotation[2][2]:
        scale = (
            1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]
        ) ** 0.5 * 2.0
        w_value = (rotation[0][2] - rotation[2][0]) / scale
        x_value = (rotation[0][1] + rotation[1][0]) / scale
        y_value = 0.25 * scale
        z_value = (rotation[1][2] + rotation[2][1]) / scale
    else:
        scale = (
            1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]
        ) ** 0.5 * 2.0
        w_value = (rotation[1][0] - rotation[0][1]) / scale
        x_value = (rotation[0][2] + rotation[2][0]) / scale
        y_value = (rotation[1][2] + rotation[2][1]) / scale
        z_value = 0.25 * scale

    norm = (
        x_value * x_value
        + y_value * y_value
        + z_value * z_value
        + w_value * w_value
    ) ** 0.5
    return (
        x_value / norm,
        y_value / norm,
        z_value / norm,
        w_value / norm,
    )


def main(args=None) -> None:
    """Run the camera static transform publisher node."""
    rclpy.init(args=args)
    node = CameraStaticTfPublisher()
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
