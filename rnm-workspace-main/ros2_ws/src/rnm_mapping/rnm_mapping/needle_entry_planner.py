"""Find and publish needle entry candidates from a registered STL model."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parents[1]))

from rnm_mapping.config_utils import load_ros_parameters
from rnm_mapping.config_utils import package_config_path
from rnm_mapping.geometry_utils import limit_points
from rnm_mapping.geometry_utils import line_points
from rnm_mapping.geometry_utils import quaternion_from_z_axis
from rnm_mapping.geometry_utils import sphere_points
from rnm_mapping.geometry_utils import unit_vector
from rnm_mapping.geometry_utils import vector_string
from rnm_mapping.mesh_io import load_ply_points
from rnm_mapping.mesh_io import load_registered_mesh
from rnm_mapping.mesh_io import write_ply
from rnm_mapping.needle_entry import EntryPlan
from rnm_mapping.needle_entry import find_entry_points
from rnm_mapping.needle_entry import plane_candidate_points
from rnm_mapping.needle_entry import sample_clearance_points


NODE_NAME = "needle_entry_planner"


def main(args=None) -> None:
    parsed = _parse_args(args)
    np.random.seed(parsed.random_seed)
    target_scan, target_radius = _load_target(parsed)

    mesh = load_registered_mesh(
        parsed.model,
        parsed.model_scale,
        parsed.model_to_scan_transform,
    )
    approach_axis = unit_vector(np.asarray(parsed.approach_axis, dtype=np.float64))
    clearance_points = sample_clearance_points(
        mesh,
        parsed.path_clearance_surface_sample_count,
    )
    clearance_tree = cKDTree(clearance_points) if len(clearance_points) > 0 else None
    plane_points = plane_candidate_points(
        vertices=np.asarray(mesh.vertices, dtype=np.float64),
        approach_axis=approach_axis,
        clearance_m=parsed.plane_clearance,
        sample_spacing_m=parsed.plane_sample_spacing,
        margin_m=parsed.plane_margin,
        max_samples=parsed.max_plane_samples,
    )

    plans = find_entry_points(
        mesh=mesh,
        target=target_scan,
        target_radius=target_radius,
        candidate_points=plane_points,
        approach_axis=approach_axis,
        needle_length_m=parsed.needle_length,
        needle_diameter_m=parsed.needle_diameter,
        target_exclusion_margin_m=parsed.target_exclusion_margin,
        horizontal_axis=parsed.horizontal_axis,
        min_path_elevation_deg=parsed.min_path_elevation_deg,
        max_path_elevation_deg=parsed.max_path_elevation_deg,
        clearance_points=clearance_points,
        clearance_tree=clearance_tree,
        min_path_clearance_m=parsed.min_path_clearance,
        preferred_path_clearance_m=parsed.preferred_path_clearance,
        path_clearance_sample_spacing_m=parsed.path_clearance_sample_spacing,
        path_clearance_entry_exclusion_m=parsed.path_clearance_entry_exclusion,
        path_clearance_target_exclusion_m=_target_clearance_exclusion(
            parsed,
            target_radius,
        ),
        entry_region_min=parsed.entry_region_min,
        entry_region_max=parsed.entry_region_max,
        max_entry_candidates=parsed.max_entry_candidates,
    )
    plan = plans[0]

    parsed.output_dir.mkdir(parents=True, exist_ok=True)
    _write_entry_report(parsed.output_dir / "needle_entry_point.txt", plans, parsed)
    if parsed.write_mesh_overlay:
        _write_mesh_entry_overlay(
            parsed.mesh_overlay_path,
            mesh,
            plans,
            parsed.marker_radius,
            parsed.candidate_marker_radius,
            parsed.overlay_model_sample_count,
        )
    if parsed.write_target_entry_overlay:
        _write_target_entry_overlay(
            parsed.target_entry_overlay_path,
            parsed.base_target_overlay_path,
            mesh,
            plans,
            parsed.marker_radius,
            parsed.candidate_marker_radius,
            parsed.overlay_model_sample_count,
        )

    print("Needle entry planning complete.")
    print(f"Target point [m]: {vector_string(plan.target)}")
    print(f"Entry point [m]: {vector_string(plan.entry)}")
    print(f"Needle axis entry->target [unit]: {vector_string(plan.insertion_axis)}")
    print(
        "Path: "
        f"length={plan.path_length_m:.4f} m, "
        f"path_elevation={plan.path_elevation_deg:.1f} deg, "
        f"axis_angle={plan.axis_angle_deg:.1f} deg, "
        f"min_clearance={plan.min_path_clearance_m:.4f} m"
    )
    print(f"Scored candidates: {plan.valid_candidates} / {len(plane_points)}")
    print(f"Recorded candidates: {len(plans)}")
    print(f"Wrote outputs to: {parsed.output_dir}")
    if parsed.write_target_entry_overlay:
        print(f"Wrote target/entry overlay to: {parsed.target_entry_overlay_path}")

    if parsed.publish:
        _publish_entry(plan, parsed)


def _parse_args(args=None) -> argparse.Namespace:
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument(
        "--config",
        type=Path,
        default=package_config_path(
            "rnm_mapping",
            __file__,
            "needle_entry_planner.yaml",
        ),
        help="YAML config file for needle entry planning.",
    )
    config_args, _ = config_parser.parse_known_args(args=args)
    defaults = load_ros_parameters(config_args.config, NODE_NAME)

    parser = _make_parser(config_parser)
    parser.set_defaults(**defaults)
    parsed = parser.parse_args(args=args)
    _normalize_parsed_config(parsed)
    return parsed


def _make_parser(config_parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Find needle entry candidates from a target and registered STL.",
        parents=[config_parser],
    )
    parser.add_argument("--model", type=Path, default=Path("scanning_bag/Skeleton_Target.stl"))
    parser.add_argument(
        "--model-to-scan-transform",
        type=Path,
        default=Path("registration_output/model_to_scan_transform.txt"),
    )
    parser.add_argument(
        "--target-location",
        type=Path,
        default=Path("target_output/target_location.txt"),
    )
    parser.add_argument("--target-point", type=float, nargs=3, default=None)
    parser.add_argument("--target-radius", type=float, default=None)
    parser.add_argument("--output-dir", type=Path, default=Path("entry_point_output"))
    parser.add_argument("--model-scale", type=float, default=0.001)
    parser.add_argument("--approach-axis", type=float, nargs=3, default=(0.0, 0.0, 1.0))
    parser.add_argument("--plane-clearance", type=float, default=0.005)
    parser.add_argument("--plane-sample-spacing", type=float, default=0.005)
    parser.add_argument("--plane-margin", type=float, default=0.02)
    parser.add_argument("--max-plane-samples", type=int, default=20000)
    parser.add_argument("--needle-length", type=float, default=0.164)
    parser.add_argument("--needle-diameter", type=float, default=0.005)
    parser.add_argument("--target-exclusion-margin", type=float, default=0.003)
    parser.add_argument("--horizontal-axis", type=float, nargs=3, default=(0.0, 0.0, 1.0))
    parser.add_argument("--min-path-elevation-deg", type=float, default=40.0)
    parser.add_argument("--max-path-elevation-deg", type=float, default=50.0)
    parser.add_argument("--path-clearance-radius", type=float, default=0.005)
    parser.add_argument("--min-path-clearance", type=float, default=None)
    parser.add_argument("--preferred-path-clearance", type=float, default=0.010)
    parser.add_argument("--path-clearance-sample-spacing", type=float, default=0.0025)
    parser.add_argument("--path-clearance-entry-exclusion", type=float, default=0.008)
    parser.add_argument("--path-clearance-target-exclusion", type=float, default=None)
    parser.add_argument("--path-clearance-surface-sample-count", type=int, default=120000)
    parser.add_argument("--entry-region-min", type=float, nargs=3, default=None)
    parser.add_argument("--entry-region-max", type=float, nargs=3, default=None)
    parser.add_argument("--random-seed", type=int, default=7)
    parser.add_argument("--max-entry-candidates", type=int, default=10)
    parser.add_argument("--marker-radius", type=float, default=0.006)
    parser.add_argument("--candidate-marker-radius", type=float, default=0.0035)
    parser.add_argument(
        "--mesh-overlay-path",
        type=Path,
        default=Path("entry_point_output/needle_entry_overlay.ply"),
    )
    parser.add_argument(
        "--base-target-overlay-path",
        type=Path,
        default=Path("target_output/target_overlay.ply"),
    )
    parser.add_argument(
        "--target-entry-overlay-path",
        type=Path,
        default=Path("target_output/target_entry_overlay.ply"),
    )
    parser.add_argument("--overlay-model-sample-count", type=int, default=120000)
    parser.add_argument(
        "--write-mesh-overlay",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--write-target-entry-overlay",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--publish", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--target-frame", default="panda_link0")
    parser.add_argument("--entry-point-topic", default="/needle_entry_point")
    parser.add_argument("--entry-pose-topic", default="/needle_entry_pose")
    parser.add_argument("--publish-duration-sec", type=float, default=5.0)
    parser.add_argument("--publish-rate-hz", type=float, default=5.0)
    return parser


def _normalize_parsed_config(parsed: argparse.Namespace) -> None:
    for name in (
        "model",
        "model_to_scan_transform",
        "target_location",
        "output_dir",
        "mesh_overlay_path",
        "base_target_overlay_path",
        "target_entry_overlay_path",
    ):
        value = getattr(parsed, name)
        if value is not None:
            setattr(parsed, name, Path(value))

    if parsed.target_point == []:
        parsed.target_point = None
    if parsed.target_point is not None:
        parsed.target_point = [float(value) for value in parsed.target_point]
    parsed.approach_axis = [float(value) for value in parsed.approach_axis]
    parsed.horizontal_axis = [float(value) for value in parsed.horizontal_axis]
    if parsed.min_path_clearance is None:
        parsed.min_path_clearance = parsed.path_clearance_radius
    parsed.entry_region_min = _optional_vector(parsed.entry_region_min)
    parsed.entry_region_max = _optional_vector(parsed.entry_region_max)


def _optional_vector(value: Any) -> np.ndarray | None:
    if value is None or value == []:
        return None
    return np.asarray(value, dtype=np.float64)


def _load_target(parsed: argparse.Namespace) -> tuple[np.ndarray, float]:
    if parsed.target_point is not None:
        target = np.asarray(parsed.target_point, dtype=np.float64)
        radius = 0.0 if parsed.target_radius is None else parsed.target_radius
        return target, radius

    target, radius = _read_target_location(parsed.target_location)
    if parsed.target_radius is not None:
        radius = parsed.target_radius
    return target, radius


def _read_target_location(path: Path) -> tuple[np.ndarray, float]:
    text = path.read_text(encoding="utf-8")
    center_match = re.search(r"scan_center_m:\s*\[([^\]]+)\]", text)
    radius_match = re.search(r"estimated_radius_m:\s*([0-9.eE+-]+)", text)
    if center_match is None:
        raise RuntimeError(f"Could not find scan_center_m in {path}")

    target = np.fromstring(center_match.group(1), sep=",", dtype=np.float64)
    if len(target) != 3:
        raise RuntimeError(f"Invalid scan_center_m in {path}")

    radius = 0.0
    if radius_match is not None:
        radius = float(radius_match.group(1))
    return target, radius


def _target_clearance_exclusion(parsed: argparse.Namespace, target_radius: float) -> float:
    if parsed.path_clearance_target_exclusion is not None:
        return float(parsed.path_clearance_target_exclusion)
    return float(target_radius + parsed.target_exclusion_margin)


def _write_entry_report(
    path: Path,
    plans: list[EntryPlan],
    parsed: argparse.Namespace,
) -> None:
    plan = plans[0]
    with path.open("w", encoding="utf-8") as file:
        file.write("# Needle entry point from registered STL model\n")
        file.write("# candidate_0 is the highest-scored candidate and is published.\n")
        file.write(f"target_point_m: {vector_string(plan.target)}\n")
        file.write(f"entry_point_m: {vector_string(plan.entry)}\n")
        file.write(
            "insertion_axis_entry_to_target: "
            f"{vector_string(plan.insertion_axis)}\n"
        )
        file.write(f"path_length_m: {plan.path_length_m:.6f}\n")
        file.write(f"needle_length_m: {parsed.needle_length:.6f}\n")
        file.write(f"needle_diameter_m: {parsed.needle_diameter:.6f}\n")
        file.write(f"plane_clearance_m: {parsed.plane_clearance:.6f}\n")
        file.write(f"horizontal_axis: {vector_string(np.asarray(parsed.horizontal_axis))}\n")
        file.write(f"min_path_elevation_deg: {parsed.min_path_elevation_deg:.6f}\n")
        file.write(f"max_path_elevation_deg: {parsed.max_path_elevation_deg:.6f}\n")
        file.write(f"path_elevation_deg: {plan.path_elevation_deg:.6f}\n")
        file.write(f"axis_angle_from_approach_deg: {plan.axis_angle_deg:.6f}\n")
        if parsed.entry_region_min is not None:
            file.write(f"entry_region_min_m: {vector_string(parsed.entry_region_min)}\n")
        if parsed.entry_region_max is not None:
            file.write(f"entry_region_max_m: {vector_string(parsed.entry_region_max)}\n")
        file.write(f"min_path_clearance_m: {parsed.min_path_clearance:.6f}\n")
        file.write(f"preferred_path_clearance_m: {parsed.preferred_path_clearance:.6f}\n")
        file.write(f"actual_min_path_clearance_m: {plan.min_path_clearance_m:.6f}\n")
        file.write(f"actual_mean_path_clearance_m: {plan.mean_path_clearance_m:.6f}\n")
        file.write(f"closest_bone_point_m: {vector_string(plan.closest_bone_point)}\n")
        file.write(
            "path_clearance_entry_exclusion_m: "
            f"{parsed.path_clearance_entry_exclusion:.6f}\n"
        )
        file.write(
            "path_clearance_target_exclusion_m: "
            f"{plan.path_clearance_target_exclusion_m:.6f}\n"
        )
        file.write(f"score: {plan.score:.6f}\n")
        file.write(f"valid_candidates: {plan.valid_candidates}\n")
        file.write(f"recorded_candidates: {len(plans)}\n")

        file.write("\n# Ranked entry candidates\n")
        for index, candidate in enumerate(plans):
            _write_candidate_report(file, index, candidate)


def _write_candidate_report(file, index: int, candidate: EntryPlan) -> None:
    file.write(f"[candidate_{index}]\n")
    file.write(f"entry_point_m: {vector_string(candidate.entry)}\n")
    file.write(
        "insertion_axis_entry_to_target: "
        f"{vector_string(candidate.insertion_axis)}\n"
    )
    file.write(f"path_length_m: {candidate.path_length_m:.6f}\n")
    file.write(f"path_elevation_deg: {candidate.path_elevation_deg:.6f}\n")
    file.write(f"axis_angle_from_approach_deg: {candidate.axis_angle_deg:.6f}\n")
    file.write(f"min_path_clearance_m: {candidate.min_path_clearance_m:.6f}\n")
    file.write(f"mean_path_clearance_m: {candidate.mean_path_clearance_m:.6f}\n")
    file.write(f"closest_bone_point_m: {vector_string(candidate.closest_bone_point)}\n")
    file.write(f"score: {candidate.score:.6f}\n")


def _write_mesh_entry_overlay(
    path: Path,
    mesh,
    plans: list[EntryPlan],
    marker_radius: float,
    candidate_marker_radius: float,
    model_sample_count: int,
) -> None:
    model_points = limit_points(np.asarray(mesh.vertices, dtype=np.float64), model_sample_count)
    model_colors = np.full((len(model_points), 3), (190, 190, 190), dtype=np.uint8)
    _write_entry_overlay_points(
        path,
        base_points=model_points,
        base_colors=model_colors,
        plans=plans,
        marker_radius=marker_radius,
        candidate_marker_radius=candidate_marker_radius,
        line_count=80,
    )


def _write_target_entry_overlay(
    path: Path,
    base_overlay_path: Path,
    mesh,
    plans: list[EntryPlan],
    marker_radius: float,
    candidate_marker_radius: float,
    model_sample_count: int,
) -> None:
    if base_overlay_path.exists():
        base_points, base_colors = load_ply_points(base_overlay_path)
    else:
        base_points = limit_points(
            np.asarray(mesh.vertices, dtype=np.float64),
            model_sample_count,
        )
        base_colors = np.full((len(base_points), 3), (190, 190, 190), dtype=np.uint8)

    _write_entry_overlay_points(
        path,
        base_points=base_points,
        base_colors=base_colors,
        plans=plans,
        marker_radius=marker_radius,
        candidate_marker_radius=candidate_marker_radius,
        line_count=100,
    )


def _write_entry_overlay_points(
    path: Path,
    base_points: np.ndarray,
    base_colors: np.ndarray,
    plans: list[EntryPlan],
    marker_radius: float,
    candidate_marker_radius: float,
    line_count: int,
) -> None:
    plan = plans[0]
    target_points = sphere_points(plan.target, marker_radius)
    target_colors = np.full((len(target_points), 3), (40, 255, 80), dtype=np.uint8)
    candidate_points = _candidate_marker_points(plans[1:], candidate_marker_radius)
    candidate_colors = np.full(
        (len(candidate_points), 3),
        (255, 220, 40),
        dtype=np.uint8,
    )
    entry_points = sphere_points(plan.entry, marker_radius)
    entry_colors = np.full((len(entry_points), 3), (40, 120, 255), dtype=np.uint8)
    path_points = line_points(plan.entry, plan.target, line_count)
    path_colors = np.full((len(path_points), 3), (255, 80, 40), dtype=np.uint8)

    write_ply(
        path,
        np.vstack((base_points, target_points, candidate_points, entry_points, path_points)),
        np.vstack((base_colors, target_colors, candidate_colors, entry_colors, path_colors)),
    )


def _candidate_marker_points(plans: list[EntryPlan], radius: float) -> np.ndarray:
    if not plans:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack([sphere_points(plan.entry, radius) for plan in plans])


def _publish_entry(plan: EntryPlan, parsed: argparse.Namespace) -> None:
    from geometry_msgs.msg import PointStamped
    from geometry_msgs.msg import PoseStamped
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy

    rclpy.init(args=None)
    node = rclpy.create_node(NODE_NAME)
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE

    point_pub = node.create_publisher(PointStamped, parsed.entry_point_topic, qos)
    pose_pub = node.create_publisher(PoseStamped, parsed.entry_pose_topic, qos)

    publish_rate_hz = max(parsed.publish_rate_hz, 0.1)
    publish_period = Duration(seconds=1.0 / publish_rate_hz)
    end_time = node.get_clock().now() + Duration(
        seconds=max(parsed.publish_duration_sec, 0.0)
    )

    point_msg = PointStamped()
    point_msg.header.frame_id = parsed.target_frame
    point_msg.point.x = float(plan.entry[0])
    point_msg.point.y = float(plan.entry[1])
    point_msg.point.z = float(plan.entry[2])

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = parsed.target_frame
    pose_msg.pose.position.x = float(plan.entry[0])
    pose_msg.pose.position.y = float(plan.entry[1])
    pose_msg.pose.position.z = float(plan.entry[2])
    orientation = quaternion_from_z_axis(plan.insertion_axis)
    pose_msg.pose.orientation.x = float(orientation[0])
    pose_msg.pose.orientation.y = float(orientation[1])
    pose_msg.pose.orientation.z = float(orientation[2])
    pose_msg.pose.orientation.w = float(orientation[3])

    print(
        "Publishing needle entry on "
        f"{parsed.entry_point_topic} and {parsed.entry_pose_topic} "
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


if __name__ == "__main__":
    main()
