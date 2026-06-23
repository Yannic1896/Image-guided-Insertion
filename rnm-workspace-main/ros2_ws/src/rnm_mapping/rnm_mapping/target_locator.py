"""Locate a model-space spherical target in the registered scan frame."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from scipy import ndimage
from scipy.spatial import cKDTree
import trimesh


def main(args=None) -> None:
    parsed = _make_parser().parse_args(args=args)
    np.random.seed(parsed.random_seed)

    model_points = _sample_model(parsed.model, parsed.model_sample_count)
    model_points = model_points * parsed.model_scale
    target = _detect_spherical_component(
        model_points,
        voxel_size=parsed.detection_voxel_size,
        min_diameter=parsed.min_target_diameter,
        max_diameter=parsed.max_target_diameter,
        min_compactness=parsed.min_compactness,
    )

    model_to_scan = np.loadtxt(parsed.model_to_scan_transform)
    target_scan = _transform_point(target.center_model, model_to_scan)
    scan_points, scan_colors = _load_scan(parsed.scan)
    validation = _validate_target(
        target_scan,
        scan_points,
        bbox_margin=parsed.bbox_margin,
        support_radius=parsed.support_radius,
        min_support_points=parsed.min_support_points,
    )

    parsed.output_dir.mkdir(parents=True, exist_ok=True)
    _write_target_report(parsed.output_dir / "target_location.txt", target, target_scan, validation)
    _write_target_overlay(
        parsed.output_dir / "target_overlay.ply",
        scan_points,
        scan_colors,
        target_scan,
        max(target.radius_m, parsed.marker_radius),
    )

    print("Target localization complete.")
    print(
        "Model target center [m]: "
        f"{_vector_string(target.center_model)}, "
        f"radius ~= {target.radius_m:.4f} m"
    )
    print(f"Scan/base target center [m]: {_vector_string(target_scan)}")
    print(
        "Validation: "
        f"inside_bbox={validation.inside_bbox}, "
        f"nearest_scan_distance={validation.nearest_distance_m:.4f} m, "
        f"support_points={validation.support_points}"
    )
    print(f"Wrote outputs to: {parsed.output_dir}")

    if parsed.publish:
        _publish_target(target_scan, parsed)


class TargetCandidate:
    """Detected compact model component."""

    def __init__(
        self,
        center_model: np.ndarray,
        extents_m: np.ndarray,
        compactness: float,
        occupied_voxels: int,
    ) -> None:
        self.center_model = center_model
        self.extents_m = extents_m
        self.compactness = compactness
        self.occupied_voxels = occupied_voxels
        self.radius_m = float(np.max(extents_m) * 0.5)


class TargetValidation:
    """Target plausibility checks in scan frame."""

    def __init__(
        self,
        inside_bbox: bool,
        nearest_distance_m: float,
        support_points: int,
        support_radius_m: float,
        bbox_min: np.ndarray,
        bbox_max: np.ndarray,
    ) -> None:
        self.inside_bbox = inside_bbox
        self.nearest_distance_m = nearest_distance_m
        self.support_points = support_points
        self.support_radius_m = support_radius_m
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Locate the small spherical target from a registered STL model."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(
            "/workspaces/RNM/rnm-workspace-main/scanning_bag/Skeleton_Target.stl"
        ),
    )
    parser.add_argument(
        "--model-to-scan-transform",
        type=Path,
        default=Path(
            "/workspaces/RNM/rnm-workspace-main/registration_output/model_to_scan_transform.txt"
        ),
    )
    parser.add_argument(
        "--scan",
        type=Path,
        default=Path("/workspaces/RNM/rnm-workspace-main/stitched_cloud.ply"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/workspaces/RNM/rnm-workspace-main/target_output"),
    )
    parser.add_argument("--model-scale", type=float, default=0.001)
    parser.add_argument("--model-sample-count", type=int, default=180000)
    parser.add_argument("--detection-voxel-size", type=float, default=0.008)
    parser.add_argument("--min-target-diameter", type=float, default=0.012)
    parser.add_argument("--max-target-diameter", type=float, default=0.060)
    parser.add_argument("--min-compactness", type=float, default=0.45)
    parser.add_argument("--bbox-margin", type=float, default=0.03)
    parser.add_argument("--support-radius", type=float, default=0.05)
    parser.add_argument("--min-support-points", type=int, default=20)
    parser.add_argument("--marker-radius", type=float, default=0.015)
    parser.add_argument("--random-seed", type=int, default=7)
    parser.add_argument("--publish", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--target-frame", default="panda_link0")
    parser.add_argument("--target-point-topic", default="/target_point")
    parser.add_argument("--target-pose-topic", default="/target_pose")
    parser.add_argument("--publish-duration-sec", type=float, default=5.0)
    parser.add_argument("--publish-rate-hz", type=float, default=5.0)
    return parser


def _sample_model(model_path: Path, count: int) -> np.ndarray:
    mesh = trimesh.load_mesh(model_path, process=False)
    points, _ = trimesh.sample.sample_surface(mesh, count)
    return np.asarray(points, dtype=np.float64)


def _detect_spherical_component(
    points: np.ndarray,
    voxel_size: float,
    min_diameter: float,
    max_diameter: float,
    min_compactness: float,
) -> TargetCandidate:
    mins = points.min(axis=0) - voxel_size
    indices = np.floor((points - mins) / voxel_size).astype(np.int64)
    shape = indices.max(axis=0) + 3
    occupied = np.zeros(shape, dtype=bool)
    occupied[indices[:, 0], indices[:, 1], indices[:, 2]] = True

    labels, _ = ndimage.label(
        occupied,
        structure=np.ones((3, 3, 3), dtype=bool),
    )
    objects = ndimage.find_objects(labels)
    candidates: list[TargetCandidate] = []

    for label_index, slices in enumerate(objects, start=1):
        if slices is None:
            continue
        local_indices = np.argwhere(labels[slices] == label_index)
        if len(local_indices) < 5:
            continue

        starts = np.array([slice_item.start for slice_item in slices])
        voxel_coords = local_indices + starts
        component_points = mins + (voxel_coords + 0.5) * voxel_size
        extents = component_points.max(axis=0) - component_points.min(axis=0)
        max_extent = float(np.max(extents))
        min_extent = float(np.min(extents))
        if max_extent <= 0.0:
            continue

        compactness = min_extent / max_extent
        if (
            min_diameter <= max_extent <= max_diameter
            and compactness >= min_compactness
        ):
            candidates.append(
                TargetCandidate(
                    center_model=np.mean(component_points, axis=0),
                    extents_m=extents,
                    compactness=compactness,
                    occupied_voxels=len(local_indices),
                )
            )

    if not candidates:
        raise RuntimeError(
            "No spherical target candidate found. Try increasing "
            "--max-target-diameter or lowering --min-compactness."
        )

    candidates.sort(
        key=lambda candidate: (
            candidate.compactness,
            candidate.occupied_voxels,
            -abs(np.max(candidate.extents_m) - 0.025),
        ),
        reverse=True,
    )
    return candidates[0]


def _load_scan(path: Path) -> tuple[np.ndarray, Optional[np.ndarray]]:
    loaded = trimesh.load(path, process=False)
    points = np.asarray(loaded.vertices, dtype=np.float64)
    colors = None
    visual = getattr(loaded, "visual", None)
    vertex_colors = getattr(visual, "vertex_colors", None)
    if vertex_colors is not None and len(vertex_colors) == len(points):
        colors = np.asarray(vertex_colors[:, :3], dtype=np.uint8)
    return points, colors


def _validate_target(
    target_scan: np.ndarray,
    scan_points: np.ndarray,
    bbox_margin: float,
    support_radius: float,
    min_support_points: int,
) -> TargetValidation:
    bbox_min = scan_points.min(axis=0)
    bbox_max = scan_points.max(axis=0)
    inside_bbox = bool(
        np.all(target_scan >= bbox_min - bbox_margin)
        and np.all(target_scan <= bbox_max + bbox_margin)
    )

    tree = cKDTree(scan_points)
    nearest_distance, _ = tree.query(target_scan, k=1)
    support_points = len(tree.query_ball_point(target_scan, support_radius))
    if support_points < min_support_points:
        inside_bbox = False

    return TargetValidation(
        inside_bbox=inside_bbox,
        nearest_distance_m=float(nearest_distance),
        support_points=int(support_points),
        support_radius_m=support_radius,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
    )


def _transform_point(point: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return transform[:3, :3] @ point + transform[:3, 3]


def _write_target_report(
    path: Path,
    target: TargetCandidate,
    target_scan: np.ndarray,
    validation: TargetValidation,
) -> None:
    with path.open("w", encoding="utf-8") as file:
        file.write("# Target localized from registered STL model\n")
        file.write(f"model_center_m: {_vector_string(target.center_model)}\n")
        file.write(f"scan_center_m: {_vector_string(target_scan)}\n")
        file.write(f"estimated_radius_m: {target.radius_m:.6f}\n")
        file.write(f"model_component_extents_m: {_vector_string(target.extents_m)}\n")
        file.write(f"model_component_compactness: {target.compactness:.6f}\n")
        file.write(f"model_component_occupied_voxels: {target.occupied_voxels}\n")
        file.write(f"inside_scan_bbox_with_margin: {validation.inside_bbox}\n")
        file.write(f"nearest_scan_distance_m: {validation.nearest_distance_m:.6f}\n")
        file.write(f"support_radius_m: {validation.support_radius_m:.6f}\n")
        file.write(f"support_points: {validation.support_points}\n")
        file.write(f"scan_bbox_min_m: {_vector_string(validation.bbox_min)}\n")
        file.write(f"scan_bbox_max_m: {_vector_string(validation.bbox_max)}\n")


def _write_target_overlay(
    path: Path,
    scan_points: np.ndarray,
    scan_colors: Optional[np.ndarray],
    target_center: np.ndarray,
    marker_radius: float,
) -> None:
    marker_points = _sphere_points(target_center, marker_radius)
    if scan_colors is None:
        scan_colors = np.full((len(scan_points), 3), (180, 180, 180), dtype=np.uint8)
    marker_colors = np.full((len(marker_points), 3), (40, 255, 80), dtype=np.uint8)
    _write_ply(
        path,
        np.vstack((scan_points, marker_points)),
        np.vstack((scan_colors, marker_colors)),
    )


def _publish_target(target_scan: np.ndarray, parsed: argparse.Namespace) -> None:
    rclpy.init(args=None)
    node = rclpy.create_node("stl_target_locator")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE

    point_pub = node.create_publisher(PointStamped, parsed.target_point_topic, qos)
    pose_pub = node.create_publisher(PoseStamped, parsed.target_pose_topic, qos)

    publish_rate_hz = max(parsed.publish_rate_hz, 0.1)
    publish_period = Duration(seconds=1.0 / publish_rate_hz)
    end_time = node.get_clock().now() + Duration(
        seconds=max(parsed.publish_duration_sec, 0.0)
    )

    point_msg = PointStamped()
    point_msg.header.frame_id = parsed.target_frame
    point_msg.point.x = float(target_scan[0])
    point_msg.point.y = float(target_scan[1])
    point_msg.point.z = float(target_scan[2])

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = parsed.target_frame
    pose_msg.pose.position.x = float(target_scan[0])
    pose_msg.pose.position.y = float(target_scan[1])
    pose_msg.pose.position.z = float(target_scan[2])
    pose_msg.pose.orientation.w = 1.0

    print(
        "Publishing target on "
        f"{parsed.target_point_topic} and {parsed.target_pose_topic} "
        f"in frame {parsed.target_frame}."
    )

    try:
        while rclpy.ok() and node.get_clock().now() <= end_time:
            stamp = node.get_clock().now().to_msg()
            point_msg.header.stamp = stamp
            pose_msg.header.stamp = stamp
            point_pub.publish(point_msg)
            pose_pub.publish(pose_msg)
            rclpy.spin_once(node, timeout_sec=publish_period.nanoseconds * 1e-9)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _sphere_points(center: np.ndarray, radius: float) -> np.ndarray:
    phi_values = np.linspace(0.0, np.pi, 24)
    theta_values = np.linspace(0.0, 2.0 * np.pi, 48, endpoint=False)
    points = []
    for phi in phi_values:
        for theta in theta_values:
            points.append(
                [
                    center[0] + radius * np.sin(phi) * np.cos(theta),
                    center[1] + radius * np.sin(phi) * np.sin(theta),
                    center[2] + radius * np.cos(phi),
                ]
            )
    return np.asarray(points, dtype=np.float64)


def _write_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as file:
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write(f"element vertex {len(points)}\n")
        file.write("property float x\n")
        file.write("property float y\n")
        file.write("property float z\n")
        file.write("property uchar red\n")
        file.write("property uchar green\n")
        file.write("property uchar blue\n")
        file.write("end_header\n")
        for point, color in zip(points, colors):
            file.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def _vector_string(vector: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:.6f}" for value in vector) + "]"


if __name__ == "__main__":
    main()
