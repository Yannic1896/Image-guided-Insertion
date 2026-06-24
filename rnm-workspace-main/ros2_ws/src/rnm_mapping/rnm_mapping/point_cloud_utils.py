"""Point-cloud array utilities shared by mapping nodes."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


def limit_point_count(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def filter_source_points(
    points: np.ndarray,
    colors: Optional[np.ndarray],
    min_depth_m: float,
    max_depth_m: float,
    max_range_m: float,
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    if points.size == 0:
        return points, colors

    mask = np.ones(len(points), dtype=bool)
    if min_depth_m > 0.0:
        mask &= points[:, 2] >= min_depth_m
    if max_depth_m > 0.0:
        mask &= points[:, 2] <= max_depth_m
    if max_range_m > 0.0:
        mask &= np.linalg.norm(points, axis=1) <= max_range_m

    filtered_colors = None if colors is None else colors[mask]
    return points[mask], filtered_colors


def crop_points(
    points: np.ndarray,
    colors: Optional[np.ndarray],
    crop_min: Optional[np.ndarray],
    crop_max: Optional[np.ndarray],
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    if crop_min is None and crop_max is None:
        return points, colors

    mask = np.ones(len(points), dtype=bool)
    if crop_min is not None:
        mask &= np.all(points >= crop_min, axis=1)
    if crop_max is not None:
        mask &= np.all(points <= crop_max, axis=1)
    cropped_colors = None if colors is None else colors[mask]
    return points[mask], cropped_colors


def overlap_ratio(
    points: np.ndarray,
    reference_points: np.ndarray,
    distance_m: float,
    max_sample_points: int,
) -> float:
    if len(points) == 0:
        return 0.0

    sampled_points = limit_point_count(points, max_sample_points)
    tree = cKDTree(reference_points)
    distances, _ = tree.query(sampled_points, k=1)
    return float(np.mean(distances <= distance_m))


def filter_radius_outliers(
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


def voxel_downsample(
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


def merge_optional_colors(
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


def points_from_cloud(msg: PointCloud2) -> tuple[np.ndarray, Optional[np.ndarray]]:
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


def create_cloud(
    header,
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


def write_ascii_ply(
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
