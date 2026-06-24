"""Configuration loading helpers for command-style mapping tools."""

from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Optional

import numpy as np
from rclpy.exceptions import ParameterUninitializedException
import yaml


def package_config_path(
    package_name: str,
    source_file: str,
    config_name: str,
) -> Path:
    source_config = Path(source_file).resolve().parents[1] / "config" / config_name
    if source_config.exists():
        return source_config

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name)) / "config" / config_name
    except Exception:
        return source_config


def load_ros_parameters(config_path: Path, node_name: str) -> dict[str, Any]:
    if config_path is None or not config_path.exists():
        return {}

    with config_path.open("r", encoding="utf-8") as file:
        loaded = yaml.safe_load(file) or {}

    if not isinstance(loaded, dict):
        raise RuntimeError(f"Config file must contain a mapping: {config_path}")

    if node_name in loaded:
        loaded = loaded[node_name] or {}
    if isinstance(loaded, dict) and "ros__parameters" in loaded:
        loaded = loaded["ros__parameters"] or {}
    if not isinstance(loaded, dict):
        raise RuntimeError(f"Invalid config shape for {node_name}: {config_path}")

    defaults = {}
    for key, value in loaded.items():
        defaults[str(key).replace("-", "_")] = value
    return defaults


def parameter_string(node, name: str) -> str:
    return str(node.get_parameter(name).value)


def parameter_bool(node, name: str) -> bool:
    return bool(node.get_parameter(name).value)


def parameter_float(node, name: str) -> float:
    return float(node.get_parameter(name).value)


def parameter_int(node, name: str) -> int:
    return int(node.get_parameter(name).value)


def optional_xyz_array(node, name: str) -> Optional[np.ndarray]:
    try:
        value = list(node.get_parameter(name).value or [])
    except ParameterUninitializedException:
        return None

    if not value:
        return None
    if len(value) != 3:
        raise ValueError(f"{name} must be empty or contain exactly 3 values.")
    return np.array(value, dtype=np.float64)
