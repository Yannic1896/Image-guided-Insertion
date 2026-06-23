"""Accumulate settled robot-mounted camera point clouds into one scan."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from scipy.spatial import cKDTree
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from tf2_ros import TransformListener


class CloudStitcher(Node):
    """Transform accepted clouds into a fixed frame and accumulate them."""

    def __init__(self) -> None:
        super().__init__("cloud_stitcher")
        self._declare_parameters()

        self._cloud_topic = self._parameter_string("cloud_topic")
        self._target_frame = self._parameter_string("target_frame")
        self._stitched_cloud_topic = self._parameter_string("stitched_cloud_topic")
        self._output_path = Path(self._parameter_string("output_path")).expanduser()

        self._capture_once_per_settled_pose = self._parameter_bool(
            "capture_once_per_settled_pose"
        )
        self._max_clouds_per_settled_pose = self._parameter_int(
            "max_clouds_per_settled_pose"
        )
        self._settled_time_sec = self._parameter_float("settled_time_sec")
        self._settled_translation_m = self._parameter_float("settled_translation_m")
        self._settled_rotation_rad = self._parameter_float("settled_rotation_rad")
        self._input_voxel_size_m = self._parameter_float("input_voxel_size_m")
        self._accumulated_voxel_size_m = self._parameter_float(
            "accumulated_voxel_size_m"
        )
        self._min_depth_m = self._parameter_float("min_depth_m")
        self._max_depth_m = self._parameter_float("max_depth_m")
        self._max_range_m = self._parameter_float("max_range_m")
        self._min_accumulated_observations = max(
            1,
            self._parameter_int("min_accumulated_observations"),
        )
        self._input_outlier_radius_m = self._parameter_float(
            "input_outlier_radius_m"
        )
        self._input_outlier_min_neighbors = self._parameter_int(
            "input_outlier_min_neighbors"
        )
        self._output_outlier_radius_m = self._parameter_float(
            "output_outlier_radius_m"
        )
        self._output_outlier_min_neighbors = self._parameter_int(
            "output_outlier_min_neighbors"
        )
        self._min_cloud_overlap_ratio = self._parameter_float(
            "min_cloud_overlap_ratio"
        )
        self._overlap_distance_m = self._parameter_float("overlap_distance_m")
        self._overlap_min_accumulated_points = self._parameter_int(
            "overlap_min_accumulated_points"
        )
        self._overlap_max_sample_points = self._parameter_int(
            "overlap_max_sample_points"
        )
        self._global_downsample_every_n_clouds = max(
            1,
            self._parameter_int("global_downsample_every_n_clouds"),
        )
        self._tf_timeout = Duration(seconds=self._parameter_float("tf_timeout_sec"))
        self._publish_after_accept = self._parameter_bool("publish_after_accept")
        self._log_accepted_transforms = self._parameter_bool(
            "log_accepted_transforms"
        )
        self._reset_on_time_jump = self._parameter_bool("reset_on_time_jump")
        self._crop_min = self._optional_xyz_array("crop_min")
        self._crop_max = self._optional_xyz_array("crop_max")

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

        self.get_logger().info(
            f"Listening on {self._cloud_topic}, accumulating into "
            f"{self._target_frame}, publishing {self._stitched_cloud_topic}."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("cloud_topic", "/points")
        self.declare_parameter("stitched_cloud_topic", "/stitched_cloud")
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

        pose_sample = _pose_sample_from_transform(transform, stamp_sec)
        if not self._pose_is_settled(pose_sample):
            self._log_waiting_for_settle(stamp_sec)
            return

        if self._settled_pose_capture_limit_reached():
            return

        points, colors = _points_from_cloud(msg)
        points, colors = self._filter_source_points(points, colors)
        if points.size == 0:
            self.get_logger().warning(
                "Skipping accepted pose with empty/filtered point cloud."
            )
            self._captured_current_settle = True
            return

        transformed = _transform_points(points, pose_sample)
        transformed, colors = self._crop_points(transformed, colors)
        transformed, colors, _ = _voxel_downsample(
            transformed,
            self._input_voxel_size_m,
            colors,
        )
        transformed, colors = _filter_radius_outliers(
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

    def _filter_source_points(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray],
    ) -> tuple[np.ndarray, Optional[np.ndarray]]:
        if points.size == 0:
            return points, colors

        mask = np.ones(len(points), dtype=bool)
        if self._min_depth_m > 0.0:
            mask &= points[:, 2] >= self._min_depth_m
        if self._max_depth_m > 0.0:
            mask &= points[:, 2] <= self._max_depth_m
        if self._max_range_m > 0.0:
            mask &= np.linalg.norm(points, axis=1) <= self._max_range_m

        filtered_colors = None if colors is None else colors[mask]
        return points[mask], filtered_colors

    def _cloud_has_enough_overlap(self, points: np.ndarray) -> bool:
        if (
            self._min_cloud_overlap_ratio <= 0.0
            or self._accumulated_points is None
            or len(self._accumulated_points) < self._overlap_min_accumulated_points
            or len(points) == 0
        ):
            return True

        sampled_points = _limit_point_count(points, self._overlap_max_sample_points)
        tree = cKDTree(self._accumulated_points)
        distances, _ = tree.query(sampled_points, k=1)
        overlap_ratio = float(np.mean(distances <= self._overlap_distance_m))
        if overlap_ratio >= self._min_cloud_overlap_ratio:
            return True

        self.get_logger().warning(
            "Rejected cloud with low overlap: "
            f"{overlap_ratio:.3f} < {self._min_cloud_overlap_ratio:.3f}.",
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
        rotation_delta = _quaternion_angle(
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
            self._accumulated_colors = _merge_optional_colors(
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
            ) = _voxel_downsample(
                self._accumulated_points,
                self._accumulated_voxel_size_m,
                self._accumulated_colors,
                self._accumulated_weights,
            )

    def _crop_points(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray],
    ) -> tuple[np.ndarray, Optional[np.ndarray]]:
        if self._crop_min is None and self._crop_max is None:
            return points, colors

        mask = np.ones(len(points), dtype=bool)
        if self._crop_min is not None:
            mask &= np.all(points >= self._crop_min, axis=1)
        if self._crop_max is not None:
            mask &= np.all(points <= self._crop_max, axis=1)
        cropped_colors = None if colors is None else colors[mask]
        return points[mask], cropped_colors

    def _publish_accumulated(self, stamp: Time) -> None:
        if self._accumulated_points is None:
            return

        points, colors = self._filtered_accumulated_points()
        if points.size == 0:
            return

        header = Header()
        header.stamp = stamp.to_msg()
        header.frame_id = self._target_frame
        cloud = _create_cloud(header, points, colors)
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
            return _filter_radius_outliers(
                points,
                colors,
                self._output_outlier_radius_m,
                self._output_outlier_min_neighbors,
            )

        mask = self._accumulated_weights >= self._min_accumulated_observations
        colors = None
        if self._accumulated_colors is not None:
            colors = self._accumulated_colors[mask]
        return _filter_radius_outliers(
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

    def _save_scan_callback(self, request, response):
        del request
        points, colors = self._filtered_accumulated_points()
        if points.size == 0:
            response.success = False
            response.message = "No accumulated points to save."
            return response

        try:
            _write_ascii_ply(
                self._output_path,
                points,
                colors,
            )
        except OSError as exc:
            response.success = False
            response.message = f"Failed to save scan: {exc}"
            return response

        response.success = True
        response.message = (
            f"Saved {len(points)} points to {self._output_path}."
        )
        return response

    def _parameter_string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _parameter_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _parameter_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _parameter_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _optional_xyz_array(self, name: str) -> Optional[np.ndarray]:
        try:
            value = list(self.get_parameter(name).value or [])
        except ParameterUninitializedException:
            return None

        if not value:
            return None
        if len(value) != 3:
            raise ValueError(f"{name} must be empty or contain exactly 3 values.")
        return np.array(value, dtype=np.float64)


class PoseSample:
    """Camera pose sample represented as target-frame transform data."""

    def __init__(
        self,
        stamp_sec: float,
        translation: np.ndarray,
        quaternion: np.ndarray,
        rotation: np.ndarray,
    ) -> None:
        self.stamp_sec = stamp_sec
        self.translation = translation
        self.quaternion = quaternion
        self.rotation = rotation


def _time_to_sec(time: Time) -> float:
    return time.nanoseconds * 1e-9


def _pose_sample_from_transform(transform, stamp_sec: float) -> PoseSample:
    translation = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ],
        dtype=np.float64,
    )
    quaternion = np.array(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ],
        dtype=np.float64,
    )
    quaternion = _normalize_quaternion(quaternion)
    return PoseSample(
        stamp_sec=stamp_sec,
        translation=translation,
        quaternion=quaternion,
        rotation=_rotation_matrix_from_quaternion(quaternion),
    )


def _points_from_cloud(msg: PointCloud2) -> tuple[np.ndarray, Optional[np.ndarray]]:
    color_field = _color_field_name(msg)
    field_names = ["x", "y", "z"]
    if color_field is not None:
        field_names.append(color_field)

    points = point_cloud2.read_points(
        msg,
        field_names=field_names,
        skip_nans=True,
    )

    if isinstance(points, np.ndarray):
        if points.dtype.names:
            xyz = np.column_stack((points["x"], points["y"], points["z"]))
            colors = _colors_from_structured_points(points, color_field)
        else:
            xyz = np.asarray(points, dtype=np.float64)
            if xyz.ndim == 1:
                xyz = xyz.reshape((-1, len(field_names)))
            colors = _colors_from_unstructured_points(xyz, color_field)
            xyz = xyz[:, :3]
    else:
        raw_points = list(points)
        if not raw_points:
            return np.empty((0, 3), dtype=np.float64), None
        xyz = np.array([point[:3] for point in raw_points], dtype=np.float64)
        colors = _colors_from_python_points(raw_points, color_field)

    if xyz.size == 0:
        return np.empty((0, 3), dtype=np.float64), None

    xyz = xyz.reshape((-1, 3)).astype(np.float64, copy=False)
    finite_mask = np.all(np.isfinite(xyz), axis=1)
    if colors is not None:
        colors = colors[finite_mask]
    return xyz[finite_mask], colors


def _transform_points(points: np.ndarray, pose_sample: PoseSample) -> np.ndarray:
    return points @ pose_sample.rotation.T + pose_sample.translation


def _limit_point_count(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def _filter_radius_outliers(
    points: np.ndarray,
    colors: Optional[np.ndarray],
    radius_m: float,
    min_neighbors: int,
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    if points.size == 0 or radius_m <= 0.0 or min_neighbors <= 0:
        return points, colors

    tree = cKDTree(points)
    counts = tree.query_ball_point(points, radius_m, return_length=True)
    mask = counts >= min_neighbors + 1
    filtered_colors = None if colors is None else colors[mask]
    return points[mask], filtered_colors


def _voxel_downsample(
    points: np.ndarray,
    voxel_size_m: float,
    colors: Optional[np.ndarray],
    weights: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    if points.size == 0 or voxel_size_m <= 0.0:
        return points, colors, weights

    voxel_indices = np.floor(points / voxel_size_m).astype(np.int64)
    _, inverse = np.unique(voxel_indices, axis=0, return_inverse=True)
    if weights is None:
        point_weights = np.ones(len(points), dtype=np.float64)
    else:
        point_weights = weights.astype(np.float64)

    weight_sums = np.bincount(inverse, weights=point_weights)
    sums = np.zeros((len(weight_sums), 3), dtype=np.float64)
    np.add.at(sums, inverse, points * point_weights[:, None])
    downsampled_points = sums / weight_sums[:, None]

    if colors is None:
        downsampled_colors = None
    else:
        color_sums = np.zeros((len(weight_sums), 3), dtype=np.float64)
        np.add.at(
            color_sums,
            inverse,
            colors.astype(np.float64) * point_weights[:, None],
        )
        downsampled_colors = np.rint(color_sums / weight_sums[:, None]).astype(
            np.uint8
        )

    if weights is None:
        return downsampled_points, downsampled_colors, None

    downsampled_weights = np.rint(weight_sums).astype(np.uint32)
    return downsampled_points, downsampled_colors, downsampled_weights


def _merge_optional_colors(
    accumulated_colors: Optional[np.ndarray],
    new_colors: Optional[np.ndarray],
    total_count: int,
    new_count: int,
) -> Optional[np.ndarray]:
    previous_count = total_count - new_count
    if accumulated_colors is None and new_colors is None:
        return None
    if accumulated_colors is None:
        accumulated_colors = np.full((previous_count, 3), 255, dtype=np.uint8)
    if new_colors is None:
        new_colors = np.full((new_count, 3), 255, dtype=np.uint8)
    return np.vstack((accumulated_colors, new_colors))


def _color_field_name(msg: PointCloud2) -> Optional[str]:
    field_names = {field.name for field in msg.fields}
    if "rgb" in field_names:
        return "rgb"
    if "rgba" in field_names:
        return "rgba"
    return None


def _colors_from_structured_points(
    points: np.ndarray,
    color_field: Optional[str],
) -> Optional[np.ndarray]:
    if color_field is None:
        return None
    return _unpack_rgb_values(points[color_field])


def _colors_from_unstructured_points(
    points: np.ndarray,
    color_field: Optional[str],
) -> Optional[np.ndarray]:
    if color_field is None or points.shape[1] < 4:
        return None
    return _unpack_rgb_values(points[:, 3])


def _colors_from_python_points(
    points: list,
    color_field: Optional[str],
) -> Optional[np.ndarray]:
    if color_field is None:
        return None
    return _unpack_rgb_values(np.array([point[3] for point in points]))


def _unpack_rgb_values(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values)
    if np.issubdtype(values.dtype, np.floating):
        packed = values.astype(np.float32).view(np.uint32)
    else:
        packed = values.astype(np.uint32)

    red = ((packed >> 16) & 0xFF).astype(np.uint8)
    green = ((packed >> 8) & 0xFF).astype(np.uint8)
    blue = (packed & 0xFF).astype(np.uint8)
    return np.column_stack((red, green, blue))


def _pack_rgb_float32(colors: np.ndarray) -> np.ndarray:
    colors = colors.astype(np.uint32)
    packed = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    return packed.astype(np.uint32).view(np.float32)


def _create_cloud(
    header: Header,
    points: np.ndarray,
    colors: Optional[np.ndarray],
) -> PointCloud2:
    if colors is None:
        return point_cloud2.create_cloud_xyz32(
            header,
            points.astype(np.float32).tolist(),
        )

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    rgb_values = _pack_rgb_float32(colors)
    cloud_points = np.column_stack((points.astype(np.float32), rgb_values))
    return point_cloud2.create_cloud(header, fields, cloud_points.tolist())


def _normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        raise ValueError("Received a zero-length TF quaternion.")
    return quaternion / norm


def _rotation_matrix_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    x_value, y_value, z_value, w_value = quaternion
    xx_value = x_value * x_value
    yy_value = y_value * y_value
    zz_value = z_value * z_value
    xy_value = x_value * y_value
    xz_value = x_value * z_value
    yz_value = y_value * z_value
    wx_value = w_value * x_value
    wy_value = w_value * y_value
    wz_value = w_value * z_value

    return np.array(
        [
            [
                1.0 - 2.0 * (yy_value + zz_value),
                2.0 * (xy_value - wz_value),
                2.0 * (xz_value + wy_value),
            ],
            [
                2.0 * (xy_value + wz_value),
                1.0 - 2.0 * (xx_value + zz_value),
                2.0 * (yz_value - wx_value),
            ],
            [
                2.0 * (xz_value - wy_value),
                2.0 * (yz_value + wx_value),
                1.0 - 2.0 * (xx_value + yy_value),
            ],
        ],
        dtype=np.float64,
    )


def _quaternion_angle(first: np.ndarray, second: np.ndarray) -> float:
    dot = abs(float(np.dot(first, second)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _write_ascii_ply(
    path: Path,
    points: np.ndarray,
    colors: Optional[np.ndarray],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write(f"element vertex {len(points)}\n")
        file.write("property float x\n")
        file.write("property float y\n")
        file.write("property float z\n")
        if colors is not None:
            file.write("property uchar red\n")
            file.write("property uchar green\n")
            file.write("property uchar blue\n")
        file.write("end_header\n")
        if colors is None:
            for point in points:
                file.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
        else:
            for point, color in zip(points, colors):
                file.write(
                    f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                    f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
                )


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
