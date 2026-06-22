"""Read and write camera calibration YAML files."""

from __future__ import annotations

from datetime import datetime, timezone
import os

import numpy as np
import yaml


def expand_path(path: str) -> str:
    """Expand user and environment markers in a filesystem path."""
    return os.path.abspath(os.path.expandvars(os.path.expanduser(path)))


def camera_section(
    camera_name: str,
    frame_id: str,
    image_size: tuple[int, int],
    camera_matrix,
    distortion_coefficients,
):
    """Build a serializable camera calibration section."""
    matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    distortion = np.asarray(distortion_coefficients, dtype=float).reshape(-1)
    projection = np.zeros((3, 4), dtype=float)
    projection[:3, :3] = matrix

    return {
        "camera_name": camera_name,
        "frame_id": frame_id,
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "distortion_model": "plumb_bob",
        "camera_matrix": _matrix_field(matrix),
        "distortion_coefficients": _matrix_field(distortion.reshape(1, -1)),
        "rectification_matrix": _matrix_field(np.eye(3, dtype=float)),
        "projection_matrix": _matrix_field(projection),
    }


def write_calibration_file(
    path: str,
    target_type: str,
    sample_count: int,
    rgb_section,
    depth_section,
    rms_errors,
    rotation_rgb_to_depth,
    translation_rgb_to_depth,
    rotation_depth_to_rgb,
    translation_depth_to_rgb,
):
    """Write an Azure Kinect calibration result file."""
    resolved_path = expand_path(path)
    os.makedirs(os.path.dirname(resolved_path), exist_ok=True)

    data = {
        "calibration": {
            "version": 1,
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "target_type": target_type,
            "sample_count": int(sample_count),
            "rms_errors": {
                "rgb": float(rms_errors["rgb"]),
                "depth": float(rms_errors["depth"]),
                "stereo": float(rms_errors["stereo"]),
            },
        },
        "rgb_camera": rgb_section,
        "depth_camera": depth_section,
        "extrinsics": {
            "parent_frame_id": rgb_section["frame_id"],
            "child_frame_id": depth_section["frame_id"],
            "rotation_depth_to_rgb": _matrix_field(rotation_depth_to_rgb),
            "translation_depth_to_rgb": _vector_field(
                translation_depth_to_rgb
            ),
            "rotation_rgb_to_depth": _matrix_field(rotation_rgb_to_depth),
            "translation_rgb_to_depth": _vector_field(
                translation_rgb_to_depth
            ),
        },
    }

    with open(resolved_path, "w", encoding="utf-8") as file_obj:
        yaml.safe_dump(data, file_obj, sort_keys=False)
    return resolved_path


def load_calibration_file(path: str):
    """Load a calibration result file."""
    resolved_path = expand_path(path)
    with open(resolved_path, "r", encoding="utf-8") as file_obj:
        data = yaml.safe_load(file_obj)
    if not isinstance(data, dict):
        raise ValueError(
            f"Calibration file is empty or invalid: {resolved_path}"
        )
    return data


def matrix_from_field(field) -> np.ndarray:
    """Read a matrix field from the calibration YAML schema."""
    rows = int(field["rows"])
    cols = int(field["cols"])
    return np.asarray(field["data"], dtype=float).reshape(rows, cols)


def vector_from_field(field) -> np.ndarray:
    """Read a vector field from the calibration YAML schema."""
    return np.asarray(field["data"], dtype=float).reshape(-1)


def _matrix_field(matrix):
    array = np.asarray(matrix, dtype=float)
    return {
        "rows": int(array.shape[0]),
        "cols": int(array.shape[1]),
        "data": [float(value) for value in array.reshape(-1)],
    }


def _vector_field(vector):
    array = np.asarray(vector, dtype=float).reshape(-1)
    return {
        "rows": int(array.size),
        "cols": 1,
        "data": [float(value) for value in array],
    }
