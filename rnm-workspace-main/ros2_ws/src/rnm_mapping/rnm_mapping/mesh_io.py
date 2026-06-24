"""Mesh and PLY IO helpers for mapping tools."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import trimesh


def load_registered_mesh(
    model_path: Path,
    model_scale: float,
    transform_path: Path,
) -> trimesh.Trimesh:
    mesh = trimesh.load_mesh(model_path, process=False)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError(f"Could not load STL mesh from {model_path}")

    scaled = trimesh.Trimesh(
        vertices=np.asarray(mesh.vertices, dtype=np.float64) * model_scale,
        faces=np.asarray(mesh.faces),
        process=False,
    )
    model_to_scan = np.loadtxt(transform_path)
    scaled.apply_transform(model_to_scan)
    scaled.fix_normals()
    return scaled


def load_ply_points(
    path: Path,
    default_color: Optional[tuple[int, int, int]] = (190, 190, 190),
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    loaded = trimesh.load(path, process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = trimesh.util.concatenate(tuple(loaded.geometry.values()))

    points = np.asarray(loaded.vertices, dtype=np.float64)
    colors = None
    visual = getattr(loaded, "visual", None)
    vertex_colors = getattr(visual, "vertex_colors", None)
    if vertex_colors is not None and len(vertex_colors) == len(points):
        colors = np.asarray(vertex_colors[:, :3], dtype=np.uint8)
    elif default_color is not None:
        colors = np.full((len(points), 3), default_color, dtype=np.uint8)

    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if colors is not None:
        colors = colors[finite]
    return points, colors


def write_ply(
    path: Path,
    points: np.ndarray,
    colors: Optional[np.ndarray],
    default_color: tuple[int, int, int] = (210, 210, 210),
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if colors is None:
        colors = np.full((len(points), 3), default_color, dtype=np.uint8)

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
