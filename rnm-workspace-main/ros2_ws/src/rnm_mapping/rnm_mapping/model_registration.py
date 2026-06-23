"""Register an STL model against a stitched point cloud."""

from __future__ import annotations

import argparse
from itertools import permutations
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree
import trimesh


def main(args=None) -> None:
    parser = _make_parser()
    parsed = parser.parse_args(args=args)

    scan_points, scan_colors = _load_point_cloud(parsed.scan)
    scan_points, scan_colors = _crop_points(scan_points, scan_colors, parsed)
    scan_points, scan_colors = _radius_filter(
        scan_points,
        scan_colors,
        parsed.scan_outlier_radius,
        parsed.scan_outlier_min_neighbors,
    )
    scan_points, scan_colors = _voxel_downsample(
        scan_points,
        parsed.scan_voxel_size,
        scan_colors,
    )

    model_mesh = trimesh.load_mesh(parsed.model, process=False)
    model_points = _sample_model(model_mesh, parsed.model_sample_count)
    model_points = model_points * parsed.model_scale
    model_points, _ = _voxel_downsample(
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

    scan_to_model = _matrix_from_transform(icp.rotation, icp.translation)
    model_to_scan = np.linalg.inv(scan_to_model)
    aligned_model_points = _transform_points_homogeneous(
        model_points,
        model_to_scan,
    )

    output_dir = parsed.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_dir / "model_to_scan_transform.txt", model_to_scan)
    np.savetxt(output_dir / "scan_to_model_transform.txt", scan_to_model)
    _write_ply(output_dir / "scan_for_registration.ply", scan_points, scan_colors)
    _write_ply(
        output_dir / "registered_model_points.ply",
        aligned_model_points,
        np.full((len(aligned_model_points), 3), (255, 80, 40), dtype=np.uint8),
    )
    _write_ply(
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
    return parser


def _load_point_cloud(path: Path) -> tuple[np.ndarray, Optional[np.ndarray]]:
    loaded = trimesh.load(path, process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = trimesh.util.concatenate(tuple(loaded.geometry.values()))

    points = np.asarray(loaded.vertices, dtype=np.float64)
    colors = None
    visual = getattr(loaded, "visual", None)
    vertex_colors = getattr(visual, "vertex_colors", None)
    if vertex_colors is not None and len(vertex_colors) == len(points):
        colors = np.asarray(vertex_colors[:, :3], dtype=np.uint8)

    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if colors is not None:
        colors = colors[finite]
    return points, colors


def _sample_model(mesh: trimesh.Trimesh, count: int) -> np.ndarray:
    if count <= 0:
        return np.asarray(mesh.vertices, dtype=np.float64)
    points, _ = trimesh.sample.sample_surface(mesh, count)
    return np.asarray(points, dtype=np.float64)


def _crop_points(
    points: np.ndarray,
    colors: Optional[np.ndarray],
    parsed: argparse.Namespace,
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    mask = np.ones(len(points), dtype=bool)
    if parsed.crop_min is not None:
        mask &= np.all(points >= np.asarray(parsed.crop_min), axis=1)
    if parsed.crop_max is not None:
        mask &= np.all(points <= np.asarray(parsed.crop_max), axis=1)
    cropped_colors = None if colors is None else colors[mask]
    return points[mask], cropped_colors


def _best_pca_initial_transform(
    source_points: np.ndarray,
    target_points: np.ndarray,
    sample_count: int,
    trim_fraction: float,
) -> RegistrationResult:
    source_sample = _limit_points(source_points, sample_count)
    target_sample = _limit_points(target_points, sample_count)
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


def _voxel_downsample(
    points: np.ndarray,
    voxel_size: float,
    colors: Optional[np.ndarray],
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    if points.size == 0 or voxel_size <= 0.0:
        return points, colors

    voxel_indices = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(voxel_indices, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    downsampled_points = sums / counts[:, None]

    if colors is None:
        return downsampled_points, None

    color_sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(color_sums, inverse, colors.astype(np.float64))
    downsampled_colors = np.rint(color_sums / counts[:, None]).astype(np.uint8)
    return downsampled_points, downsampled_colors


def _radius_filter(
    points: np.ndarray,
    colors: Optional[np.ndarray],
    radius: float,
    min_neighbors: int,
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    if points.size == 0 or radius <= 0.0 or min_neighbors <= 0:
        return points, colors
    tree = cKDTree(points)
    counts = tree.query_ball_point(points, radius, return_length=True)
    mask = counts >= min_neighbors + 1
    filtered_colors = None if colors is None else colors[mask]
    return points[mask], filtered_colors


def _limit_points(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def _trimmed_rmse(distances: np.ndarray, trim_fraction: float) -> float:
    trim_fraction = min(1.0, max(0.01, trim_fraction))
    keep_count = max(1, int(len(distances) * trim_fraction))
    kept = np.partition(distances, keep_count - 1)[:keep_count]
    return float(np.sqrt(np.mean(kept * kept)))


def _matrix_from_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def _transform_points_homogeneous(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def _default_colors(points: np.ndarray, colors: Optional[np.ndarray]) -> np.ndarray:
    if colors is not None:
        return colors
    return np.full((len(points), 3), (210, 210, 210), dtype=np.uint8)


def _write_ply(path: Path, points: np.ndarray, colors: Optional[np.ndarray]) -> None:
    colors = _default_colors(points, colors)
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


def _extent_string(points: np.ndarray) -> str:
    extents = points.max(axis=0) - points.min(axis=0)
    return "[" + ", ".join(f"{value:.4f}" for value in extents) + "]"


if __name__ == "__main__":
    main()
