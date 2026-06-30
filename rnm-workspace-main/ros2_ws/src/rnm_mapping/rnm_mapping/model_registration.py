"""Register an STL model against a stitched point cloud."""

from __future__ import annotations

import argparse
from itertools import permutations
from pathlib import Path
import sys
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree
import trimesh

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parents[1]))

from rnm_mapping.geometry_utils import limit_points
from rnm_mapping.geometry_utils import matrix_from_transform
from rnm_mapping.geometry_utils import transform_points_homogeneous
from rnm_mapping.mesh_io import load_ply_points
from rnm_mapping.mesh_io import write_ply
from rnm_mapping.point_cloud_utils import crop_points
from rnm_mapping.point_cloud_utils import filter_radius_outliers
from rnm_mapping.point_cloud_utils import voxel_downsample


def main(args=None) -> None:
    parser = _make_parser()
    parsed = parser.parse_args(args=args)
    np.random.seed(parsed.random_seed)

    scan_points, scan_colors = load_ply_points(parsed.scan, default_color=None)
    scan_points, scan_colors = crop_points(
        scan_points,
        scan_colors,
        _optional_vector(parsed.crop_min),
        _optional_vector(parsed.crop_max),
    )
    scan_points, scan_colors = filter_radius_outliers(
        scan_points,
        scan_colors,
        parsed.scan_outlier_radius,
        parsed.scan_outlier_min_neighbors,
    )
    scan_points, scan_colors, _ = voxel_downsample(
        scan_points,
        parsed.scan_voxel_size,
        scan_colors,
    )

    model_mesh = trimesh.load_mesh(parsed.model, process=False)
    model_points = _sample_model(model_mesh, parsed.model_sample_count)
    model_points = model_points * parsed.model_scale
    model_points, _, _ = voxel_downsample(
        model_points,
        parsed.model_voxel_size,
        None,
    )

    print(f"Loaded scan points: {len(scan_points)}")
    print(f"Loaded sampled model points: {len(model_points)}")
    print(f"Scan extents: {_extent_string(scan_points)}")
    print(f"Model extents: {_extent_string(model_points)}")

    initial = _best_pca_initial_transform(
        source_points=scan_points,
        target_points=model_points,
        sample_count=parsed.initial_sample_count,
        trim_fraction=parsed.initial_trim_fraction,
    )
    icp = _point_to_point_icp(
        source_points=scan_points,
        target_points=model_points,
        initial_rotation=initial.rotation,
        initial_translation=initial.translation,
        max_iterations=parsed.icp_iterations,
        max_correspondence_distance=parsed.icp_max_correspondence_distance,
        min_correspondences=parsed.icp_min_correspondences,
        trim_fraction=parsed.icp_trim_fraction,
    )

    scan_to_model = matrix_from_transform(icp.rotation, icp.translation)
    model_to_scan = np.linalg.inv(scan_to_model)
    aligned_model_points = transform_points_homogeneous(
        model_points,
        model_to_scan,
    )

    output_dir = parsed.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_dir / "model_to_scan_transform.txt", model_to_scan)
    np.savetxt(output_dir / "scan_to_model_transform.txt", scan_to_model)
    write_ply(output_dir / "scan_for_registration.ply", scan_points, scan_colors)
    write_ply(
        output_dir / "registered_model_points.ply",
        aligned_model_points,
        np.full((len(aligned_model_points), 3), (255, 80, 40), dtype=np.uint8),
    )
    write_ply(
        output_dir / "registration_overlay.ply",
        np.vstack((scan_points, aligned_model_points)),
        np.vstack(
            (
                _default_colors(scan_points, scan_colors),
                np.full((len(aligned_model_points), 3), (255, 80, 40), dtype=np.uint8),
            )
        ),
    )

    print("Registration complete.")
    print(f"Initial PCA score: {initial.rmse:.4f} m")
    print(
        "ICP: "
        f"iterations={icp.iterations}, "
        f"correspondences={icp.correspondences}, "
        f"rmse={icp.rmse:.4f} m"
    )
    print(f"Wrote outputs to: {output_dir}")


class RegistrationResult:
    """Rigid transform and fit stats."""

    def __init__(
        self,
        rotation: np.ndarray,
        translation: np.ndarray,
        iterations: int,
        correspondences: int,
        rmse: float,
    ) -> None:
        self.rotation = rotation
        self.translation = translation
        self.iterations = iterations
        self.correspondences = correspondences
        self.rmse = rmse


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Register an STL model against a stitched scan PLY."
    )
    parser.add_argument(
        "--scan",
        type=Path,
        default=Path("/workspaces/RNM/rnm-workspace-main/stitched_cloud.ply"),
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(
            "/workspaces/RNM/rnm-workspace-main/scanning_bag/Skeleton_Target.stl"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/workspaces/RNM/rnm-workspace-main/registration_output"),
    )
    parser.add_argument("--model-scale", type=float, default=0.001)
    parser.add_argument("--model-sample-count", type=int, default=120000)
    parser.add_argument("--scan-voxel-size", type=float, default=0.008)
    parser.add_argument("--model-voxel-size", type=float, default=0.008)
    parser.add_argument("--scan-outlier-radius", type=float, default=0.0)
    parser.add_argument("--scan-outlier-min-neighbors", type=int, default=4)
    parser.add_argument("--crop-min", type=float, nargs=3, default=None)
    parser.add_argument("--crop-max", type=float, nargs=3, default=None)
    parser.add_argument("--initial-sample-count", type=int, default=8000)
    parser.add_argument("--initial-trim-fraction", type=float, default=0.75)
    parser.add_argument("--icp-iterations", type=int, default=60)
    parser.add_argument("--icp-max-correspondence-distance", type=float, default=0.04)
    parser.add_argument("--icp-min-correspondences", type=int, default=300)
    parser.add_argument("--icp-trim-fraction", type=float, default=0.75)
    parser.add_argument("--random-seed", type=int, default=7)
    return parser


def _sample_model(mesh: trimesh.Trimesh, count: int) -> np.ndarray:
    if count <= 0:
        return np.asarray(mesh.vertices, dtype=np.float64)
    points, _ = trimesh.sample.sample_surface(mesh, count)
    return np.asarray(points, dtype=np.float64)


def _optional_vector(values: Optional[list[float]]) -> Optional[np.ndarray]:
    if values is None:
        return None
    return np.asarray(values, dtype=np.float64)


def _best_pca_initial_transform(
    source_points: np.ndarray,
    target_points: np.ndarray,
    sample_count: int,
    trim_fraction: float,
) -> RegistrationResult:
    source_sample = limit_points(source_points, sample_count)
    target_sample = limit_points(target_points, sample_count)
    source_centroid, source_axes = _pca_frame(source_sample)
    target_centroid, target_axes = _pca_frame(target_sample)
    target_tree = cKDTree(target_sample)

    best: Optional[RegistrationResult] = None
    for permutation in permutations(range(3)):
        permuted_axes = target_axes[:, permutation]
        for signs in _proper_sign_flips():
            candidate_axes = permuted_axes @ np.diag(signs)
            rotation = candidate_axes @ source_axes.T
            if np.linalg.det(rotation) < 0.0:
                continue
            translation = target_centroid - rotation @ source_centroid
            transformed = source_sample @ rotation.T + translation
            distances, _ = target_tree.query(transformed, k=1)
            rmse = _trimmed_rmse(distances, trim_fraction)
            candidate = RegistrationResult(
                rotation=rotation,
                translation=translation,
                iterations=0,
                correspondences=len(distances),
                rmse=rmse,
            )
            if best is None or candidate.rmse < best.rmse:
                best = candidate

    if best is None:
        raise RuntimeError("Failed to compute PCA initial transform.")
    return best


def _point_to_point_icp(
    source_points: np.ndarray,
    target_points: np.ndarray,
    initial_rotation: np.ndarray,
    initial_translation: np.ndarray,
    max_iterations: int,
    max_correspondence_distance: float,
    min_correspondences: int,
    trim_fraction: float,
) -> RegistrationResult:
    target_tree = cKDTree(target_points)
    rotation = initial_rotation.copy()
    translation = initial_translation.copy()
    last_rmse = np.inf
    last_correspondences = 0

    for iteration in range(max_iterations):
        transformed = source_points @ rotation.T + translation
        distances, indices = target_tree.query(transformed, k=1)
        mask = distances <= max_correspondence_distance
        if np.count_nonzero(mask) < min_correspondences:
            ordered = np.argsort(distances)
            keep_count = max(min_correspondences, int(len(distances) * trim_fraction))
            keep = ordered[:keep_count]
        else:
            inlier_indices = np.flatnonzero(mask)
            ordered = inlier_indices[np.argsort(distances[mask])]
            keep_count = max(min_correspondences, int(len(ordered) * trim_fraction))
            keep = ordered[:keep_count]

        source_corr = transformed[keep]
        target_corr = target_points[indices[keep]]
        delta_rotation, delta_translation = _rigid_transform(source_corr, target_corr)
        rotation = delta_rotation @ rotation
        translation = delta_rotation @ translation + delta_translation

        residuals = np.linalg.norm(
            (source_corr @ delta_rotation.T + delta_translation) - target_corr,
            axis=1,
        )
        rmse = float(np.sqrt(np.mean(residuals * residuals)))
        last_correspondences = len(keep)
        if abs(last_rmse - rmse) < 1e-5:
            return RegistrationResult(
                rotation,
                translation,
                iteration + 1,
                last_correspondences,
                rmse,
            )
        last_rmse = rmse

    return RegistrationResult(
        rotation,
        translation,
        max_iterations,
        last_correspondences,
        last_rmse,
    )


def _rigid_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid
    covariance = source_centered.T @ target_centered
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_centroid - rotation @ source_centroid
    return rotation, translation


def _pca_frame(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    covariance = centered.T @ centered / max(1, len(points) - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    axes = eigenvectors[:, order]
    if np.linalg.det(axes) < 0.0:
        axes[:, -1] *= -1.0
    return centroid, axes


def _proper_sign_flips() -> list[tuple[int, int, int]]:
    flips = []
    for x_sign in (-1, 1):
        for y_sign in (-1, 1):
            for z_sign in (-1, 1):
                if x_sign * y_sign * z_sign > 0:
                    flips.append((x_sign, y_sign, z_sign))
    return flips


def _trimmed_rmse(distances: np.ndarray, trim_fraction: float) -> float:
    trim_fraction = min(1.0, max(0.01, trim_fraction))
    keep_count = max(1, int(len(distances) * trim_fraction))
    kept = np.partition(distances, keep_count - 1)[:keep_count]
    return float(np.sqrt(np.mean(kept * kept)))


def _default_colors(points: np.ndarray, colors: Optional[np.ndarray]) -> np.ndarray:
    if colors is not None:
        return colors
    return np.full((len(points), 3), (210, 210, 210), dtype=np.uint8)


def _extent_string(points: np.ndarray) -> str:
    extents = points.max(axis=0) - points.min(axis=0)
    return "[" + ", ".join(f"{value:.4f}" for value in extents) + "]"


if __name__ == "__main__":
    main()
