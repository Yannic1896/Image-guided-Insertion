"""Pure geometry for needle entry candidate generation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree
import trimesh

from rnm_mapping.geometry_utils import orthonormal_basis
from rnm_mapping.geometry_utils import unit_vector


@dataclass
class EntryPlan:
    target: np.ndarray
    entry: np.ndarray
    insertion_axis: np.ndarray
    path_length_m: float
    path_elevation_deg: float
    axis_angle_deg: float
    min_path_clearance_m: float
    mean_path_clearance_m: float
    closest_bone_point: np.ndarray
    path_clearance_target_exclusion_m: float
    axis_alignment_score: float
    elevation_score: float
    min_clearance_score: float
    mean_clearance_score: float
    length_score: float
    score: float
    valid_candidates: int


@dataclass
class PathClearance:
    min_distance_m: float
    mean_distance_m: float
    closest_point: np.ndarray


def sample_clearance_points(mesh: trimesh.Trimesh, count: int) -> np.ndarray:
    if count <= 0:
        return np.asarray(mesh.vertices, dtype=np.float64)
    points, _ = trimesh.sample.sample_surface(mesh, count)
    return np.asarray(points, dtype=np.float64)


def plane_candidate_points(
    vertices: np.ndarray,
    approach_axis: np.ndarray,
    clearance_m: float,
    sample_spacing_m: float,
    margin_m: float,
    max_samples: int,
) -> np.ndarray:
    basis_u, basis_v = orthonormal_basis(approach_axis)
    top = float(np.max(vertices @ approach_axis))
    plane_offset = top + clearance_m

    u_values = vertices @ basis_u
    v_values = vertices @ basis_v
    u_min = float(np.min(u_values) - margin_m)
    u_max = float(np.max(u_values) + margin_m)
    v_min = float(np.min(v_values) - margin_m)
    v_max = float(np.max(v_values) + margin_m)

    sample_spacing_m = max(sample_spacing_m, 1e-4)
    u_count = max(2, int(np.ceil((u_max - u_min) / sample_spacing_m)) + 1)
    v_count = max(2, int(np.ceil((v_max - v_min) / sample_spacing_m)) + 1)
    if max_samples > 0 and u_count * v_count > max_samples:
        scale = np.sqrt((u_count * v_count) / max_samples)
        u_count = max(2, int(np.ceil(u_count / scale)))
        v_count = max(2, int(np.ceil(v_count / scale)))

    grid_u, grid_v = np.meshgrid(
        np.linspace(u_min, u_max, u_count),
        np.linspace(v_min, v_max, v_count),
        indexing="xy",
    )
    return (
        grid_u.reshape(-1, 1) * basis_u
        + grid_v.reshape(-1, 1) * basis_v
        + plane_offset * approach_axis
    )


def find_entry_points(
    mesh: trimesh.Trimesh,
    target: np.ndarray,
    target_radius: float,
    candidate_points: np.ndarray,
    approach_axis: np.ndarray,
    needle_length_m: float,
    needle_diameter_m: float,
    target_exclusion_margin_m: float,
    horizontal_axis: np.ndarray,
    min_path_elevation_deg: float,
    max_path_elevation_deg: float,
    clearance_points: np.ndarray,
    clearance_tree: Optional[cKDTree],
    min_path_clearance_m: float,
    preferred_path_clearance_m: float,
    path_clearance_sample_spacing_m: float,
    path_clearance_entry_exclusion_m: float,
    path_clearance_target_exclusion_m: float,
    entry_region_min: Optional[np.ndarray],
    entry_region_max: Optional[np.ndarray],
    max_entry_candidates: int,
    score_axis_alignment_weight: float,
    score_elevation_weight: float,
    score_min_clearance_weight: float,
    score_mean_clearance_weight: float,
    score_length_weight: float,
) -> list[EntryPlan]:
    vectors = candidate_points - target
    distances = np.linalg.norm(vectors, axis=1)
    mask = distances > 1e-9
    entries = candidate_points[mask]
    path_lengths = distances[mask]
    directions = vectors[mask] / path_lengths[:, None]
    if len(directions) == 0:
        raise RuntimeError("No valid approach directions generated.")

    min_ray_distance = max(
        target_radius + target_exclusion_margin_m,
        0.5 * needle_diameter_m,
    )
    mesh_hits = ray_surface_hits(mesh, target, directions, min_ray_distance)
    horizontal_axis = unit_vector(np.asarray(horizontal_axis, dtype=np.float64))
    min_path_elevation_rad = np.deg2rad(min_path_elevation_deg)
    max_path_elevation_rad = np.deg2rad(max_path_elevation_deg)

    plans: list[EntryPlan] = []
    valid_count = 0
    for entry, path_length, ray_direction, mesh_hit in zip(
        entries,
        path_lengths,
        directions,
        mesh_hits,
    ):
        if not entry_in_region(entry, entry_region_min, entry_region_max):
            continue
        if path_length > needle_length_m:
            continue
        if mesh_hit_before_entry(mesh_hit, path_length):
            continue

        axis_alignment = float(np.clip(np.dot(ray_direction, approach_axis), -1.0, 1.0))
        if axis_alignment <= 0.0:
            continue

        vertical_alignment = abs(
            np.clip(np.dot(ray_direction, horizontal_axis), -1.0, 1.0)
        )
        path_elevation = float(np.arcsin(vertical_alignment))
        if (
            path_elevation < min_path_elevation_rad
            or path_elevation > max_path_elevation_rad
        ):
            continue

        clearance = path_clearance_stats(
            entry=entry,
            target=target,
            clearance_tree=clearance_tree,
            clearance_points=clearance_points,
            sample_spacing_m=path_clearance_sample_spacing_m,
            entry_exclusion_m=path_clearance_entry_exclusion_m,
            target_exclusion_m=path_clearance_target_exclusion_m,
        )
        if clearance.min_distance_m < min_path_clearance_m:
            continue

        insertion_axis = unit_vector(target - entry)
        length_score = 1.0 - min(path_length / max(needle_length_m, 1e-9), 1.0)
        min_clearance_score = min(
            clearance.min_distance_m / max(preferred_path_clearance_m, 1e-9),
            1.0,
        )
        mean_clearance_score = min(
            clearance.mean_distance_m / max(preferred_path_clearance_m, 1e-9),
            1.0,
        )
        elevation_center = 0.5 * (min_path_elevation_rad + max_path_elevation_rad)
        elevation_width = max(max_path_elevation_rad - min_path_elevation_rad, 1e-9)
        elevation_score = 1.0 - min(
            abs(path_elevation - elevation_center) / (0.5 * elevation_width),
            1.0,
        )
        score = (
            score_axis_alignment_weight * axis_alignment
            + score_elevation_weight * elevation_score
            + score_min_clearance_weight * min_clearance_score
            + score_mean_clearance_weight * mean_clearance_score
            + score_length_weight * length_score
        )
        valid_count += 1

        plans.append(
            EntryPlan(
                target=target,
                entry=np.asarray(entry, dtype=np.float64),
                insertion_axis=insertion_axis,
                path_length_m=float(path_length),
                path_elevation_deg=float(np.rad2deg(path_elevation)),
                axis_angle_deg=float(np.rad2deg(np.arccos(axis_alignment))),
                min_path_clearance_m=float(clearance.min_distance_m),
                mean_path_clearance_m=float(clearance.mean_distance_m),
                closest_bone_point=clearance.closest_point,
                path_clearance_target_exclusion_m=float(
                    path_clearance_target_exclusion_m
                ),
                axis_alignment_score=float(axis_alignment),
                elevation_score=float(elevation_score),
                min_clearance_score=float(min_clearance_score),
                mean_clearance_score=float(mean_clearance_score),
                length_score=float(length_score),
                score=float(score),
                valid_candidates=0,
            )
        )

    if not plans:
        raise RuntimeError(
            "No valid entry point found. Try increasing --needle-length, "
            "widening --min-path-elevation-deg/--max-path-elevation-deg, "
            "lowering --min-path-clearance, or increasing --max-plane-samples."
        )

    plans.sort(key=lambda candidate: candidate.score, reverse=True)
    kept = plans[: max(1, max_entry_candidates)]
    for candidate in kept:
        candidate.valid_candidates = valid_count
    return kept


def mesh_hit_before_entry(
    mesh_hit: Optional[tuple[np.ndarray, np.ndarray, float, np.ndarray]],
    path_length: float,
) -> bool:
    if mesh_hit is None:
        return False
    _, _, hit_distance, _ = mesh_hit
    return hit_distance < path_length - 1e-5


def entry_in_region(
    entry: np.ndarray,
    entry_region_min: Optional[np.ndarray],
    entry_region_max: Optional[np.ndarray],
) -> bool:
    if entry_region_min is not None and np.any(entry < entry_region_min):
        return False
    if entry_region_max is not None and np.any(entry > entry_region_max):
        return False
    return True


def path_clearance_stats(
    entry: np.ndarray,
    target: np.ndarray,
    clearance_tree: Optional[cKDTree],
    clearance_points: np.ndarray,
    sample_spacing_m: float,
    entry_exclusion_m: float,
    target_exclusion_m: float,
) -> PathClearance:
    if clearance_tree is None or len(clearance_points) == 0:
        return empty_clearance()

    segment = target - entry
    length = float(np.linalg.norm(segment))
    if length <= 1e-9:
        return empty_clearance()

    start_distance = max(0.0, entry_exclusion_m)
    end_distance = max(0.0, length - target_exclusion_m)
    if start_distance >= end_distance:
        return empty_clearance()

    spacing = max(sample_spacing_m, 1e-4)
    sample_count = max(2, int(np.ceil((end_distance - start_distance) / spacing)) + 1)
    distances = np.linspace(start_distance, end_distance, sample_count)
    centers = entry.reshape(1, 3) + distances.reshape(-1, 1) * (segment / length)

    nearest_distances, nearest_indices = clearance_tree.query(centers, k=1)
    nearest_distances = np.asarray(nearest_distances, dtype=np.float64)
    nearest_indices = np.asarray(nearest_indices, dtype=np.int64)
    closest_sample_index = int(np.argmin(nearest_distances))
    closest_point = clearance_points[int(nearest_indices[closest_sample_index])]
    return PathClearance(
        min_distance_m=float(nearest_distances[closest_sample_index]),
        mean_distance_m=float(np.mean(nearest_distances)),
        closest_point=np.asarray(closest_point, dtype=np.float64),
    )


def empty_clearance() -> PathClearance:
    return PathClearance(
        min_distance_m=float("inf"),
        mean_distance_m=float("inf"),
        closest_point=np.full(3, np.nan, dtype=np.float64),
    )


def ray_surface_hits(
    mesh: trimesh.Trimesh,
    origin: np.ndarray,
    directions: np.ndarray,
    min_distance: float,
) -> list[Optional[tuple[np.ndarray, np.ndarray, float, np.ndarray]]]:
    try:
        return ray_surface_hits_trimesh(mesh, origin, directions, min_distance)
    except BaseException as exc:
        print(f"trimesh ray intersector unavailable ({exc}); using fallback.")
        return ray_surface_hits_fallback(mesh, origin, directions, min_distance)


def ray_surface_hits_trimesh(
    mesh: trimesh.Trimesh,
    origin: np.ndarray,
    directions: np.ndarray,
    min_distance: float,
) -> list[Optional[tuple[np.ndarray, np.ndarray, float, np.ndarray]]]:
    origins = np.repeat(origin.reshape(1, 3), len(directions), axis=0)
    locations, ray_indices, triangle_indices = mesh.ray.intersects_location(
        origins,
        directions,
        multiple_hits=True,
    )
    hits: list[Optional[tuple[np.ndarray, np.ndarray, float, np.ndarray]]] = [
        None
    ] * len(directions)

    if len(locations) == 0:
        return hits

    distances = np.einsum("ij,ij->i", locations - origin, directions[ray_indices])
    order = np.argsort(distances)
    normals = np.asarray(mesh.face_normals, dtype=np.float64)
    for index in order:
        distance = float(distances[index])
        if distance <= min_distance:
            continue
        ray_index = int(ray_indices[index])
        if hits[ray_index] is not None:
            continue
        normal = oriented_normal(
            normals[int(triangle_indices[index])],
            directions[ray_index],
        )
        hits[ray_index] = (
            np.asarray(locations[index], dtype=np.float64),
            normal,
            distance,
            directions[ray_index],
        )
    return hits


def ray_surface_hits_fallback(
    mesh: trimesh.Trimesh,
    origin: np.ndarray,
    directions: np.ndarray,
    min_distance: float,
) -> list[Optional[tuple[np.ndarray, np.ndarray, float, np.ndarray]]]:
    triangles = np.asarray(mesh.triangles, dtype=np.float64)
    normals = np.asarray(mesh.face_normals, dtype=np.float64)
    vertex0 = triangles[:, 0, :]
    edge1 = triangles[:, 1, :] - vertex0
    edge2 = triangles[:, 2, :] - vertex0
    eps = 1e-9
    hits: list[Optional[tuple[np.ndarray, np.ndarray, float, np.ndarray]]] = []

    for direction in directions:
        h = np.cross(np.repeat(direction.reshape(1, 3), len(edge2), axis=0), edge2)
        a = np.einsum("ij,ij->i", edge1, h)
        valid = np.abs(a) > eps
        f = np.zeros_like(a)
        f[valid] = 1.0 / a[valid]
        s = origin - vertex0
        u = f * np.einsum("ij,ij->i", s, h)
        q = np.cross(s, edge1)
        v = f * np.einsum("j,ij->i", direction, q)
        t = f * np.einsum("ij,ij->i", edge2, q)
        valid &= (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t > min_distance)

        if not np.any(valid):
            hits.append(None)
            continue

        valid_indices = np.flatnonzero(valid)
        nearest_index = int(valid_indices[np.argmin(t[valid])])
        distance = float(t[nearest_index])
        entry = origin + direction * distance
        normal = oriented_normal(normals[nearest_index], direction)
        hits.append((entry, normal, distance, direction))

    return hits


def oriented_normal(normal: np.ndarray, ray_direction: np.ndarray) -> np.ndarray:
    normal = unit_vector(normal)
    if np.dot(normal, ray_direction) < 0.0:
        return -normal
    return normal
