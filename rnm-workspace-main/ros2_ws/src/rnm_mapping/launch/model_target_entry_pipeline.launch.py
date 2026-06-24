"""Launch model registration, target localization, then needle entry planning."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


REGISTRATION_ARGS = (
    "scan",
    "model",
    "output_dir",
    "model_scale",
    "model_sample_count",
    "scan_voxel_size",
    "model_voxel_size",
    "scan_outlier_radius",
    "scan_outlier_min_neighbors",
    "crop_min",
    "crop_max",
    "initial_sample_count",
    "initial_trim_fraction",
    "icp_iterations",
    "icp_max_correspondence_distance",
    "icp_min_correspondences",
    "icp_trim_fraction",
)

TARGET_LOCATOR_ARGS = (
    "model",
    "model_to_scan_transform",
    "scan",
    "output_dir",
    "model_scale",
    "model_sample_count",
    "detection_voxel_size",
    "min_target_diameter",
    "max_target_diameter",
    "min_compactness",
    "bbox_margin",
    "support_radius",
    "min_support_points",
    "marker_radius",
    "random_seed",
    "publish",
    "target_frame",
    "target_point_topic",
    "target_pose_topic",
    "publish_duration_sec",
    "publish_rate_hz",
)


def _load_pipeline_config(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        raise RuntimeError(f"Pipeline config does not exist: {config_path}")

    with config_path.open("r", encoding="utf-8") as file:
        loaded = yaml.safe_load(file) or {}

    if not isinstance(loaded, dict):
        raise RuntimeError(f"Pipeline config must contain a mapping: {config_path}")
    return loaded


def _section(config: dict[str, Any], name: str) -> dict[str, Any]:
    section = config.get(name) or {}
    if not isinstance(section, dict):
        raise RuntimeError(f"Pipeline config section must be a mapping: {name}")
    return section


def _cli_name(key: str) -> str:
    return f"--{key.replace('_', '-')}"


def _append_arg(command: list[str], key: str, value: Any) -> None:
    if value is None:
        return
    if isinstance(value, list) and not value:
        return

    if isinstance(value, bool):
        command.append(_cli_name(key) if value else f"--no-{key.replace('_', '-')}")
        return

    command.append(_cli_name(key))
    if isinstance(value, list):
        command.extend(str(item) for item in value)
    else:
        command.append(str(value))


def _build_stage_command(
    executable: str,
    config: dict[str, Any],
    keys: tuple[str, ...],
) -> list[str]:
    command = ["ros2", "run", "rnm_mapping", executable]
    for key in keys:
        if key in config:
            _append_arg(command, key, config[key])
    return command


def _build_entry_command(config: dict[str, Any], default_config: str) -> list[str]:
    command = ["ros2", "run", "rnm_mapping", "find_needle_entry"]
    command.extend(["--config", str(config.get("config") or default_config)])

    extra_args = config.get("extra_args") or []
    if not isinstance(extra_args, list):
        raise RuntimeError("needle_entry_planner.extra_args must be a list")
    command.extend(str(arg) for arg in extra_args)
    return command


def _stage_success_handler(next_action: ExecuteProcess, stage_name: str):
    def _handler(event: ProcessExited, _context: LaunchContext):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        f"[model_target_entry_pipeline] {stage_name} failed "
                        f"with return code {event.returncode}; stopping pipeline."
                    )
                ),
                Shutdown(reason=f"{stage_name} failed"),
            ]

        return [
            LogInfo(
                msg=(
                    f"[model_target_entry_pipeline] {stage_name} complete; "
                    f"starting next stage."
                )
            ),
            next_action,
        ]

    return _handler


def _final_stage_handler(event: ProcessExited, _context: LaunchContext):
    if event.returncode != 0:
        return [
            LogInfo(
                msg=(
                    "[model_target_entry_pipeline] needle entry planning failed "
                    f"with return code {event.returncode}."
                )
            ),
            Shutdown(reason="needle entry planning failed"),
        ]

    return [
        LogInfo(msg="[model_target_entry_pipeline] all stages complete."),
        Shutdown(reason="model target entry pipeline complete"),
    ]


def _launch_pipeline(context: LaunchContext):
    pipeline_config = Path(LaunchConfiguration("pipeline_config").perform(context))
    default_entry_config = LaunchConfiguration("needle_entry_config").perform(context)
    config = _load_pipeline_config(pipeline_config)

    registration = ExecuteProcess(
        cmd=_build_stage_command(
            "register_stl_to_scan",
            _section(config, "model_registration"),
            REGISTRATION_ARGS,
        ),
        output="screen",
    )
    target_locator = ExecuteProcess(
        cmd=_build_stage_command(
            "locate_stl_target",
            _section(config, "target_locator"),
            TARGET_LOCATOR_ARGS,
        ),
        output="screen",
    )
    entry_planner = ExecuteProcess(
        cmd=_build_entry_command(
            _section(config, "needle_entry_planner"),
            default_entry_config,
        ),
        output="screen",
    )

    return [
        LogInfo(
            msg=(
                "[model_target_entry_pipeline] starting model registration "
                f"with config: {pipeline_config}"
            )
        ),
        registration,
        RegisterEventHandler(
            OnProcessExit(
                target_action=registration,
                on_exit=_stage_success_handler(target_locator, "model registration"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=target_locator,
                on_exit=_stage_success_handler(entry_planner, "target localization"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=entry_planner,
                on_exit=_final_stage_handler,
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_pipeline_config = PathJoinSubstitution(
        [
            FindPackageShare("rnm_mapping"),
            "config",
            "model_target_entry_pipeline.yaml",
        ]
    )
    default_entry_config = PathJoinSubstitution(
        [
            FindPackageShare("rnm_mapping"),
            "config",
            "needle_entry_planner.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pipeline_config",
                default_value=default_pipeline_config,
                description="Path to the three-stage model/target/entry pipeline YAML.",
            ),
            DeclareLaunchArgument(
                "needle_entry_config",
                default_value=default_entry_config,
                description=(
                    "Fallback needle entry planner YAML used when the pipeline "
                    "config does not set needle_entry_planner.config."
                ),
            ),
            OpaqueFunction(function=_launch_pipeline),
        ]
    )
