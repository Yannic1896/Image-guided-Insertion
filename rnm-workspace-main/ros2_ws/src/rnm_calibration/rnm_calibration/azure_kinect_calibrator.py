"""ROS 2 node that calibrates Azure Kinect RGB and depth cameras."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rnm_calibration.calibration_io import camera_section
from rnm_calibration.calibration_io import write_calibration_file
from rnm_calibration.calibration_math import average_rotation_matrices
from rnm_calibration.calibration_math import invert_rigid_transform
from rnm_calibration.calibration_targets import image_to_grayscale
from rnm_calibration.calibration_targets import make_target
from rnm_calibration.calibration_targets import paired_detection_points
from rnm_calibration.calibration_targets import TargetDetection


@dataclass(frozen=True)
class CalibrationSample:
    """One paired RGB/depth calibration target observation."""

    rgb_detection: TargetDetection
    depth_detection: TargetDetection


class AzureKinectCalibratorNode(Node):
    """Collect target observations and write an RGB/depth calibration YAML."""

    def __init__(self) -> None:
        super().__init__("azure_kinect_calibrator")
        self._declare_parameters()

        self._target_type = self._parameter_string("target_type")
        self._target = make_target(
            self._target_type,
            self._parameter_int("checkerboard_columns"),
            self._parameter_int("checkerboard_rows"),
            self._parameter_float("checkerboard_square_size_m"),
            self._parameter_int("charuco_squares_x"),
            self._parameter_int("charuco_squares_y"),
            self._parameter_float("charuco_square_size_m"),
            self._parameter_float("charuco_marker_size_m"),
            self._parameter_string("charuco_dictionary"),
            self._parameter_int("charuco_min_corners"),
        )

        self._bridge = CvBridge()
        self._samples: list[CalibrationSample] = []
        self._rgb_size: Optional[tuple[int, int]] = None
        self._depth_size: Optional[tuple[int, int]] = None
        self._last_sample_time = None
        self._last_reject_log_time = None
        self._complete = False

        self._sample_count = self._parameter_int("sample_count")
        self._min_sample_interval_s = self._parameter_float(
            "min_sample_interval_s"
        )
        self._min_common_points = self._parameter_int("min_common_points")
        self._shutdown_on_complete = self._parameter_bool(
            "shutdown_on_complete"
        )

        rgb_topic = self._parameter_string("rgb_image_topic")
        depth_topic = self._parameter_string("depth_calibration_image_topic")
        queue_size = self._parameter_int("sync_queue_size")
        slop = self._parameter_float("sync_slop_s")

        self._rgb_subscriber = Subscriber(
            self,
            Image,
            rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._depth_subscriber = Subscriber(
            self,
            Image,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_subscriber, self._depth_subscriber],
            queue_size,
            slop,
        )
        self._sync.registerCallback(self._image_callback)

        self.get_logger().info(
            "Calibrator waiting for paired target views on "
            f"{rgb_topic} and {depth_topic}."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("rgb_image_topic", "/rgb/image_raw")
        self.declare_parameter(
            "depth_calibration_image_topic",
            "/ir/image_raw",
        )
        self.declare_parameter("target_type", "checkerboard")
        self.declare_parameter("checkerboard_columns", 9)
        self.declare_parameter("checkerboard_rows", 6)
        self.declare_parameter("checkerboard_square_size_m", 0.025)
        self.declare_parameter("charuco_squares_x", 7)
        self.declare_parameter("charuco_squares_y", 5)
        self.declare_parameter("charuco_square_size_m", 0.04)
        self.declare_parameter("charuco_marker_size_m", 0.03)
        self.declare_parameter("charuco_dictionary", "DICT_4X4_50")
        self.declare_parameter("charuco_min_corners", 8)
        self.declare_parameter("sample_count", 25)
        self.declare_parameter("min_sample_interval_s", 0.75)
        self.declare_parameter("min_common_points", 8)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_s", 0.08)
        self.declare_parameter(
            "calibration_file",
            "~/.ros/rnm_calibration/azure_kinect_calibration.yaml",
        )
        self.declare_parameter("rgb_camera_name", "azure_kinect_rgb")
        self.declare_parameter("depth_camera_name", "azure_kinect_depth")
        self.declare_parameter(
            "rgb_frame_id",
            "rgb_camera_optical_frame",
        )
        self.declare_parameter(
            "depth_frame_id",
            "depth_camera_optical_frame",
        )
        self.declare_parameter("shutdown_on_complete", True)

    def _image_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        if self._complete:
            return
        if not self._sample_interval_elapsed():
            return

        try:
            rgb_gray = self._message_to_gray(rgb_msg)
            depth_gray = self._message_to_gray(depth_msg)
        except CvBridgeError as exc:
            self.get_logger().warning(f"cv_bridge conversion failed: {exc}")
            return

        rgb_detection = self._target.detect(rgb_gray)
        depth_detection = self._target.detect(depth_gray)
        if rgb_detection is None or depth_detection is None:
            self._log_rejection(
                "Waiting until the target is visible in both images."
            )
            return

        paired = paired_detection_points(rgb_detection, depth_detection)
        if paired is None or len(paired[0]) < self._min_common_points:
            self._log_rejection(
                "Target detected, but not enough common points were visible."
            )
            return

        if not self._accept_image_sizes(rgb_detection, depth_detection):
            return

        self._samples.append(
            CalibrationSample(
                rgb_detection=rgb_detection,
                depth_detection=depth_detection,
            )
        )
        self._last_sample_time = self.get_clock().now()
        self.get_logger().info(
            f"Accepted calibration sample {len(self._samples)}/"
            f"{self._sample_count}."
        )

        if len(self._samples) >= self._sample_count:
            self._complete = True
            self._calibrate_and_write()
            if self._shutdown_on_complete:
                self.create_timer(0.2, self._shutdown)

    def _sample_interval_elapsed(self) -> bool:
        if self._last_sample_time is None:
            return True
        elapsed = self.get_clock().now() - self._last_sample_time
        return elapsed.nanoseconds * 1e-9 >= self._min_sample_interval_s

    def _message_to_gray(self, msg: Image) -> np.ndarray:
        cv_image = self._bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough",
        )
        return image_to_grayscale(cv_image)

    def _accept_image_sizes(
        self,
        rgb_detection: TargetDetection,
        depth_detection: TargetDetection,
    ) -> bool:
        if self._rgb_size is None:
            self._rgb_size = rgb_detection.image_size
        if self._depth_size is None:
            self._depth_size = depth_detection.image_size

        if rgb_detection.image_size != self._rgb_size:
            self.get_logger().warning(
                "Skipping sample because the RGB image size changed from "
                f"{self._rgb_size} to {rgb_detection.image_size}."
            )
            return False
        if depth_detection.image_size != self._depth_size:
            self.get_logger().warning(
                "Skipping sample because the depth calibration image size "
                f"changed from {self._depth_size} "
                f"to {depth_detection.image_size}."
            )
            return False
        return True

    def _calibrate_and_write(self) -> None:
        rgb_rms, rgb_matrix, rgb_distortion = self._calibrate_intrinsics(
            [sample.rgb_detection for sample in self._samples],
            self._rgb_size,
            "RGB",
        )
        depth_rms, depth_matrix, depth_distortion = self._calibrate_intrinsics(
            [sample.depth_detection for sample in self._samples],
            self._depth_size,
            "depth",
        )

        (
            stereo_rms,
            rotation_rgb_to_depth,
            translation_rgb_to_depth,
        ) = self._calibrate_extrinsics(
            rgb_matrix,
            rgb_distortion,
            depth_matrix,
            depth_distortion,
        )
        (
            rotation_depth_to_rgb,
            translation_depth_to_rgb,
        ) = invert_rigid_transform(
            rotation_rgb_to_depth,
            translation_rgb_to_depth,
        )

        output_path = write_calibration_file(
            self._parameter_string("calibration_file"),
            self._target_type,
            len(self._samples),
            camera_section(
                self._parameter_string("rgb_camera_name"),
                self._parameter_string("rgb_frame_id"),
                self._rgb_size,
                rgb_matrix,
                rgb_distortion,
            ),
            camera_section(
                self._parameter_string("depth_camera_name"),
                self._parameter_string("depth_frame_id"),
                self._depth_size,
                depth_matrix,
                depth_distortion,
            ),
            {"rgb": rgb_rms, "depth": depth_rms, "stereo": stereo_rms},
            rotation_rgb_to_depth,
            translation_rgb_to_depth,
            rotation_depth_to_rgb,
            translation_depth_to_rgb,
        )
        self.get_logger().info(f"Wrote calibration result to {output_path}.")

    def _calibrate_intrinsics(
        self,
        detections: list[TargetDetection],
        image_size: tuple[int, int],
        label: str,
    ):
        object_points = [
            _object_points_cv(detection)
            for detection in detections
        ]
        image_points = [
            _image_points_cv(detection)
            for detection in detections
        ]
        rms, matrix, distortion, _, _ = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )
        self.get_logger().info(
            f"{label} calibration RMS reprojection error: {rms:.4f}"
        )
        return float(rms), matrix, distortion

    def _calibrate_extrinsics(
        self,
        rgb_matrix,
        rgb_distortion,
        depth_matrix,
        depth_distortion,
    ):
        object_points = []
        rgb_points = []
        depth_points = []
        for sample in self._samples:
            paired = paired_detection_points(
                sample.rgb_detection,
                sample.depth_detection,
            )
            if paired is None or len(paired[0]) < self._min_common_points:
                continue
            object_points.append(_object_points_cv_array(paired[0]))
            rgb_points.append(_image_points_cv_array(paired[1]))
            depth_points.append(_image_points_cv_array(paired[2]))

        if len(object_points) < 3:
            raise RuntimeError(
                "At least three paired observations are required "
                "for extrinsics."
            )

        try:
            result = cv2.stereoCalibrate(
                object_points,
                rgb_points,
                depth_points,
                rgb_matrix,
                rgb_distortion,
                depth_matrix,
                depth_distortion,
                self._rgb_size,
                flags=cv2.CALIB_FIX_INTRINSIC,
            )
            rms = float(result[0])
            rotation = result[5]
            translation = result[6].reshape(3)
            self.get_logger().info(
                f"Stereo calibration RMS reprojection error: {rms:.4f}"
            )
            return rms, rotation, translation
        except cv2.error as exc:
            self.get_logger().warning(
                "cv2.stereoCalibrate failed; falling back to solvePnP "
                f"extrinsics averaging. OpenCV error: {exc}"
            )
            return self._solve_pnp_extrinsics(
                object_points,
                rgb_points,
                depth_points,
                rgb_matrix,
                rgb_distortion,
                depth_matrix,
                depth_distortion,
            )

    def _solve_pnp_extrinsics(
        self,
        object_points,
        rgb_points,
        depth_points,
        rgb_matrix,
        rgb_distortion,
        depth_matrix,
        depth_distortion,
    ):
        rotations = []
        translations = []
        for obj_points, rgb_img_points, depth_img_points in zip(
            object_points,
            rgb_points,
            depth_points,
        ):
            ok_rgb, rgb_rvec, rgb_tvec = cv2.solvePnP(
                obj_points,
                rgb_img_points,
                rgb_matrix,
                rgb_distortion,
            )
            ok_depth, depth_rvec, depth_tvec = cv2.solvePnP(
                obj_points,
                depth_img_points,
                depth_matrix,
                depth_distortion,
            )
            if not ok_rgb or not ok_depth:
                continue

            rgb_rotation, _ = cv2.Rodrigues(rgb_rvec)
            depth_rotation, _ = cv2.Rodrigues(depth_rvec)
            rgb_translation = rgb_tvec.reshape(3)
            depth_translation = depth_tvec.reshape(3)

            rotation = depth_rotation @ rgb_rotation.T
            translation = depth_translation - rotation @ rgb_translation
            rotations.append(rotation)
            translations.append(translation)

        if not rotations:
            raise RuntimeError("solvePnP could not estimate any paired poses.")

        averaged_rotation = average_rotation_matrices(rotations)
        averaged_translation = np.mean(np.asarray(translations), axis=0)
        return math.nan, averaged_rotation, averaged_translation

    def _log_rejection(self, message: str) -> None:
        now = self.get_clock().now()
        if self._last_reject_log_time is not None:
            elapsed = now - self._last_reject_log_time
            if elapsed.nanoseconds * 1e-9 < 2.0:
                return
        self._last_reject_log_time = now
        self.get_logger().info(message)

    def _shutdown(self) -> None:
        rclpy.shutdown()

    def _parameter_string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _parameter_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _parameter_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _parameter_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def _object_points_cv(detection: TargetDetection) -> np.ndarray:
    return _object_points_cv_array(detection.object_points)


def _image_points_cv(detection: TargetDetection) -> np.ndarray:
    return _image_points_cv_array(detection.image_points)


def _object_points_cv_array(points) -> np.ndarray:
    return np.asarray(points, dtype=np.float32).reshape(-1, 1, 3)


def _image_points_cv_array(points) -> np.ndarray:
    return np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)


def main(args=None) -> None:
    """Run the Azure Kinect calibration capture node."""
    rclpy.init(args=args)
    node = AzureKinectCalibratorNode()
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
