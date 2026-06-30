"""Accumulate settled robot-mounted camera point clouds into one scan."""

from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import Optional

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from tf2_ros import TransformListener

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parents[1]))

from rnm_mapping.config_utils import optional_xyz_array
from rnm_mapping.config_utils import parameter_bool
from rnm_mapping.config_utils import parameter_float
from rnm_mapping.config_utils import parameter_int
from rnm_mapping.config_utils import parameter_string
from rnm_mapping.geometry_utils import PoseSample
from rnm_mapping.geometry_utils import pose_sample_from_transform
from rnm_mapping.geometry_utils import quaternion_angle
from rnm_mapping.geometry_utils import transform_points
from rnm_mapping.point_cloud_utils import create_cloud
from rnm_mapping.point_cloud_utils import crop_points
from rnm_mapping.point_cloud_utils import filter_radius_outliers
from rnm_mapping.point_cloud_utils import filter_source_points
from rnm_mapping.point_cloud_utils import merge_optional_colors
from rnm_mapping.point_cloud_utils import overlap_ratio
from rnm_mapping.point_cloud_utils import points_from_cloud
from rnm_mapping.point_cloud_utils import voxel_downsample
from rnm_mapping.point_cloud_utils import write_ascii_ply


class CloudStitcher(Node):
    """Transform accepted clouds into a fixed frame and accumulate them."""

    def __init__(self) -> None:
        super().__init__("cloud_stitcher")
        self._declare_parameters()

        self._cloud_topic = parameter_string(self, "cloud_topic")
        self._target_frame = parameter_string(self, "target_frame")
        self._stitched_cloud_topic = parameter_string(self, "stitched_cloud_topic")
        self._pose_capture_done_topic = parameter_string(
            self,
            "pose_capture_done_topic",
        )
        self._output_path = Path(parameter_string(self, "output_path")).expanduser()

        self._capture_once_per_settled_pose = parameter_bool(
            self,
            "capture_once_per_settled_pose",
        )
        self._max_clouds_per_settled_pose = parameter_int(
            self,
            "max_clouds_per_settled_pose",
        )
        self._settled_time_sec = parameter_float(self, "settled_time_sec")
        self._settled_translation_m = parameter_float(self, "settled_translation_m")
        self._settled_rotation_rad = parameter_float(self, "settled_rotation_rad")
        self._input_voxel_size_m = parameter_float(self, "input_voxel_size_m")
        self._accumulated_voxel_size_m = parameter_float(
            self,
            "accumulated_voxel_size_m",
        )
        self._min_depth_m = parameter_float(self, "min_depth_m")
        self._max_depth_m = parameter_float(self, "max_depth_m")
        self._max_range_m = parameter_float(self, "max_range_m")
        self._min_accumulated_observations = max(
            1,
            parameter_int(self, "min_accumulated_observations"),
        )
        self._input_outlier_radius_m = parameter_float(
            self,
            "input_outlier_radius_m",
        )
        self._input_outlier_min_neighbors = parameter_int(
            self,
            "input_outlier_min_neighbors",
        )
        self._output_outlier_radius_m = parameter_float(
            self,
            "output_outlier_radius_m",
        )
        self._output_outlier_min_neighbors = parameter_int(
            self,
            "output_outlier_min_neighbors",
        )
        self._min_cloud_overlap_ratio = parameter_float(
            self,
            "min_cloud_overlap_ratio",
        )
        self._overlap_distance_m = parameter_float(self, "overlap_distance_m")
        self._overlap_min_accumulated_points = parameter_int(
            self,
            "overlap_min_accumulated_points",
        )
        self._overlap_max_sample_points = parameter_int(
            self,
            "overlap_max_sample_points",
        )
        self._global_downsample_every_n_clouds = max(
            1,
            parameter_int(self, "global_downsample_every_n_clouds"),
        )
        self._tf_timeout = Duration(seconds=parameter_float(self, "tf_timeout_sec"))
        self._publish_after_accept = parameter_bool(self, "publish_after_accept")
        self._log_accepted_transforms = parameter_bool(
            self,
            "log_accepted_transforms",
        )
        self._reset_on_time_jump = parameter_bool(self, "reset_on_time_jump")
        self._crop_min = optional_xyz_array(self, "crop_min")
        self._crop_max = optional_xyz_array(self, "crop_max")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self._stitched_pub = self.create_publisher(
            PointCloud2,
            self._stitched_cloud_topic,
            1,
        )
        self._pose_capture_done_pub = self.create_publisher(
            Bool,
            self._pose_capture_done_topic,
            10,
        )
        self.create_service(Trigger, "reset_scan", self._reset_scan_callback)
        self.create_service(Trigger, "save_scan", self._save_scan_callback)

        self._accumulated_points: Optional[np.ndarray] = None
        self._accumulated_colors: Optional[np.ndarray] = None
        self._accumulated_weights: Optional[np.ndarray] = None
        self._settle_reference: Optional[PoseSample] = None
        self._stable_since_sec: Optional[float] = None
        self._captured_current_settle = False
        self._captured_current_settle_count = 0
        self._accepted_clouds = 0
        self._received_clouds = 0
        self._logged_cloud_layout = False
        self._last_cloud_stamp_sec: Optional[float] = None
        self._last_wait_log_sec = -math.inf
        self._publish_pose_capture_not_done()

        self.get_logger().info(
            f"Listening on {self._cloud_topic}, accumulating into "
            f"{self._target_frame}, publishing {self._stitched_cloud_topic}. "
            f"Pose capture feedback: {self._pose_capture_done_topic}."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("cloud_topic", "/points")
        self.declare_parameter("stitched_cloud_topic", "/stitched_cloud")
        self.declare_parameter(
            "pose_capture_done_topic",
            "/cloud_stitcher/pose_capture_done",
        )
        self.declare_parameter("target_frame", "panda_link0")
        self.declare_parameter(
            "output_path",
            "~/.ros/rnm_mapping/stitched_cloud.ply",
        )
        self.declare_parameter("capture_once_per_settled_pose", True)
        self.declare_parameter("max_clouds_per_settled_pose", 0)
        self.declare_parameter("settled_time_sec", 0.5)
        self.declare_parameter("settled_translation_m", 0.001)
        self.declare_parameter("settled_rotation_rad", 0.005)
        self.declare_parameter("input_voxel_size_m", 0.005)
        self.declare_parameter("accumulated_voxel_size_m", 0.005)
        self.declare_parameter("min_depth_m", 0.0)
        self.declare_parameter("max_depth_m", 0.0)
        self.declare_parameter("max_range_m", 0.0)
        self.declare_parameter("min_accumulated_observations", 1)
        self.declare_parameter("input_outlier_radius_m", 0.0)
        self.declare_parameter("input_outlier_min_neighbors", 3)
        self.declare_parameter("output_outlier_radius_m", 0.0)
        self.declare_parameter("output_outlier_min_neighbors", 3)
        self.declare_parameter("min_cloud_overlap_ratio", 0.0)
        self.declare_parameter("overlap_distance_m", 0.03)
        self.declare_parameter("overlap_min_accumulated_points", 1000)
        self.declare_parameter("overlap_max_sample_points", 10000)
        self.declare_parameter("global_downsample_every_n_clouds", 1)
        self.declare_parameter("crop_min", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("crop_max", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("tf_timeout_sec", 0.0)
        self.declare_parameter("publish_after_accept", True)
        self.declare_parameter("log_accepted_transforms", True)
        self.declare_parameter("reset_on_time_jump", True)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self._received_clouds += 1
        stamp = Time.from_msg(msg.header.stamp)
        stamp_sec = _time_to_sec(stamp)
        self._log_cloud_layout_once(msg)
        self._handle_time_jump(stamp_sec)

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                msg.header.frame_id,
                stamp,
                timeout=self._tf_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(
                f"Skipping cloud: no TF {self._target_frame} <- "
                f"{msg.header.frame_id} at stamp {stamp_sec:.3f}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        pose_sample = pose_sample_from_transform(transform, stamp_sec)
        if not self._pose_is_settled(pose_sample):
            self._log_waiting_for_settle(stamp_sec)
            return

        if self._settled_pose_capture_limit_reached():
            return

        points, colors = points_from_cloud(msg)
        points, colors = filter_source_points(
            points,
            colors,
            self._min_depth_m,
            self._max_depth_m,
            self._max_range_m,
        )
        if points.size == 0:
            self.get_logger().warning(
                "Skipping accepted pose with empty/filtered point cloud."
            )
            self._captured_current_settle = True
            return

        transformed = transform_points(points, pose_sample)
        transformed, colors = crop_points(
            transformed,
            colors,
            self._crop_min,
            self._crop_max,
        )
        transformed, colors, _ = voxel_downsample(
            transformed,
            self._input_voxel_size_m,
            colors,
        )
        transformed, colors = filter_radius_outliers(
            transformed,
            colors,
            self._input_outlier_radius_m,
            self._input_outlier_min_neighbors,
        )
        if not self._cloud_has_enough_overlap(transformed):
            return

        if transformed.size == 0:
            self.get_logger().warning(
                "Skipping cloud after crop/downsample removed all points."
            )
            self._captured_current_settle = True
            return

        weights = np.ones(len(transformed), dtype=np.uint32)
        self._append_points(transformed, colors, weights)
        self._accepted_clouds += 1
        self._captured_current_settle_count += 1
        self._captured_current_settle = True
        if self._current_pose_capture_done():
            self._publish_pose_capture_done_pulse()

        if self._publish_after_accept:
            self._publish_accumulated(stamp)

        point_count = 0
        if self._accumulated_points is not None:
            point_count = len(self._accumulated_points)
        output_points, _ = self._filtered_accumulated_points()
        self.get_logger().info(
            f"Accepted cloud {self._accepted_clouds} "
            f"({len(transformed)} points, accumulated {point_count}, "
            f"output {len(output_points)})."
        )
        self._log_accepted_transform(msg.header.frame_id, pose_sample)

    def _settled_pose_capture_limit_reached(self) -> bool:
        if self._capture_once_per_settled_pose and self._captured_current_settle:
            return True
        return (
            self._max_clouds_per_settled_pose > 0
            and self._captured_current_settle_count
            >= self._max_clouds_per_settled_pose
        )

    def _current_pose_capture_done(self) -> bool:
        return self._captured_current_settle_count >= self._required_pose_clouds()

    def _required_pose_clouds(self) -> int:
        if self._max_clouds_per_settled_pose > 0:
            return self._max_clouds_per_settled_pose
        return 1

    def _publish_pose_capture_done_pulse(self) -> None:
        msg = Bool()
        msg.data = True
        self._pose_capture_done_pub.publish(msg)
        self._publish_pose_capture_not_done()

    def _publish_pose_capture_not_done(self) -> None:
        msg = Bool()
        msg.data = False
        self._pose_capture_done_pub.publish(msg)

    def _cloud_has_enough_overlap(self, points: np.ndarray) -> bool:
        if (
            self._min_cloud_overlap_ratio <= 0.0
            or self._accumulated_points is None
            or len(self._accumulated_points) < self._overlap_min_accumulated_points
            or len(points) == 0
        ):
            return True

        ratio = overlap_ratio(
            points,
            self._accumulated_points,
            self._overlap_distance_m,
            self._overlap_max_sample_points,
        )
        if ratio >= self._min_cloud_overlap_ratio:
            return True

        self.get_logger().warning(
            "Rejected cloud with low overlap: "
            f"{ratio:.3f} < {self._min_cloud_overlap_ratio:.3f}.",
            throttle_duration_sec=2.0,
        )
        return False

    def _handle_time_jump(self, stamp_sec: float) -> None:
        if self._last_cloud_stamp_sec is None:
            self._last_cloud_stamp_sec = stamp_sec
            return

        if self._reset_on_time_jump and stamp_sec < self._last_cloud_stamp_sec - 0.5:
            self.get_logger().info(
                "Detected bag time jump backwards; resetting accumulated scan."
            )
            self._clear_scan()

        self._last_cloud_stamp_sec = stamp_sec

    def _log_cloud_layout_once(self, msg: PointCloud2) -> None:
        if self._logged_cloud_layout:
            return

        self._logged_cloud_layout = True
        field_names = ", ".join(field.name for field in msg.fields)
        self.get_logger().info(
            f"Input cloud frame is '{msg.header.frame_id}' with fields: {field_names}."
        )

    def _log_accepted_transform(
        self,
        source_frame: str,
        pose_sample: "PoseSample",
    ) -> None:
        if not self._log_accepted_transforms:
            return

        t = pose_sample.translation
        q = pose_sample.quaternion
        camera_z_axis = pose_sample.rotation[:, 2]
        self.get_logger().info(
            f"Accepted TF {self._target_frame} <- {source_frame}: "
            f"t=[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}], "
            f"q=[{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}], "
            f"source +Z in target=[{camera_z_axis[0]:.3f}, "
            f"{camera_z_axis[1]:.3f}, {camera_z_axis[2]:.3f}]"
        )

    def _pose_is_settled(self, pose_sample: "PoseSample") -> bool:
        if self._settle_reference is None:
            self._settle_reference = pose_sample
            self._stable_since_sec = pose_sample.stamp_sec
            self._captured_current_settle = False
            self._captured_current_settle_count = 0
            return False

        translation_delta = np.linalg.norm(
            pose_sample.translation - self._settle_reference.translation
        )
        rotation_delta = quaternion_angle(
            pose_sample.quaternion,
            self._settle_reference.quaternion,
        )
        if (
            translation_delta > self._settled_translation_m
            or rotation_delta > self._settled_rotation_rad
        ):
            self._settle_reference = pose_sample
            self._stable_since_sec = pose_sample.stamp_sec
            self._captured_current_settle = False
            self._captured_current_settle_count = 0
            return False

        if self._stable_since_sec is None:
            self._stable_since_sec = pose_sample.stamp_sec

        return (pose_sample.stamp_sec - self._stable_since_sec) >= (
            self._settled_time_sec
        )

    def _log_waiting_for_settle(self, stamp_sec: float) -> None:
        if stamp_sec - self._last_wait_log_sec < 2.0:
            return
        self._last_wait_log_sec = stamp_sec
        self.get_logger().info(
            "Waiting for a settled camera pose before accepting clouds..."
        )

    def _append_points(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray],
        weights: np.ndarray,
    ) -> None:
        if self._accumulated_points is None:
            self._accumulated_points = points
            self._accumulated_colors = colors
            self._accumulated_weights = weights
        else:
            self._accumulated_points = np.vstack((self._accumulated_points, points))
            self._accumulated_colors = merge_optional_colors(
                self._accumulated_colors,
                colors,
                len(self._accumulated_points),
                len(points),
            )
            self._accumulated_weights = np.concatenate(
                (self._accumulated_weights, weights)
            )

        next_cloud_count = self._accepted_clouds + 1
        if next_cloud_count % self._global_downsample_every_n_clouds == 0:
            (
                self._accumulated_points,
                self._accumulated_colors,
                self._accumulated_weights,
            ) = voxel_downsample(
                self._accumulated_points,
                self._accumulated_voxel_size_m,
                self._accumulated_colors,
                self._accumulated_weights,
            )

    def _publish_accumulated(self, stamp: Time) -> None:
        if self._accumulated_points is None:
            return

        points, colors = self._filtered_accumulated_points()
        if points.size == 0:
            return

        header = Header()
        header.stamp = stamp.to_msg()
        header.frame_id = self._target_frame
        cloud = create_cloud(header, points, colors)
        self._stitched_pub.publish(cloud)

    def _filtered_accumulated_points(self) -> tuple[np.ndarray, Optional[np.ndarray]]:
        if self._accumulated_points is None:
            return np.empty((0, 3), dtype=np.float64), None

        if (
            self._accumulated_weights is None
            or self._min_accumulated_observations <= 1
        ):
            points = self._accumulated_points
            colors = self._accumulated_colors
            return filter_radius_outliers(
                points,
                colors,
                self._output_outlier_radius_m,
                self._output_outlier_min_neighbors,
            )

        mask = self._accumulated_weights >= self._min_accumulated_observations
        colors = None
        if self._accumulated_colors is not None:
            colors = self._accumulated_colors[mask]
        return filter_radius_outliers(
            self._accumulated_points[mask],
            colors,
            self._output_outlier_radius_m,
            self._output_outlier_min_neighbors,
        )

    def _reset_scan_callback(self, request, response):
        del request
        self._clear_scan()
        response.success = True
        response.message = "Cleared accumulated point cloud."
        return response

    def _clear_scan(self) -> None:
        self._accumulated_points = None
        self._accumulated_colors = None
        self._accumulated_weights = None
        self._settle_reference = None
        self._stable_since_sec = None
        self._accepted_clouds = 0
        self._captured_current_settle = False
        self._captured_current_settle_count = 0
        self._publish_pose_capture_not_done()

    def _save_scan_callback(self, request, response):
        del request
        points, colors = self._filtered_accumulated_points()
        if points.size == 0:
            response.success = False
            response.message = "No accumulated points to save."
            return response

        try:
            write_ascii_ply(
                self._output_path,
                points,
                colors,
            )
        except OSError as exc:
            response.success = False
            response.message = f"Failed to save scan: {exc}"
            return response

        response.success = True
        response.message = f"Saved {len(points)} points to {self._output_path}."
        return response


def _time_to_sec(time: Time) -> float:
    return time.nanoseconds * 1e-9


def main(args=None) -> None:
    """Run the cloud stitcher node."""
    rclpy.init(args=args)
    node = CloudStitcher()
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
