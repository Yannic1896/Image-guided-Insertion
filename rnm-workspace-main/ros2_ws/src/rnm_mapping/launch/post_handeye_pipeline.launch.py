"""Launch the full RNM pipeline after hand-eye calibration."""

from __future__ import annotations

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_supervisor(context: LaunchContext):
    use_sim = (
        LaunchConfiguration("use_sim").perform(context).lower()
        in {"1", "true", "yes", "on"}
    )

    command = [
        "ros2",
        "run",
        "rnm_mapping",
        "post_handeye_pipeline_supervisor",
        "--cloud-stitcher-params",
        LaunchConfiguration("cloud_stitcher_params").perform(context),
        "--pipeline-config",
        LaunchConfiguration("pipeline_config").perform(context),
        "--needle-entry-config",
        LaunchConfiguration("needle_entry_config").perform(context),
        "--target-location-file",
        LaunchConfiguration("target_location_file").perform(context),
        "--entry-location-file",
        LaunchConfiguration("entry_location_file").perform(context),
        "--entry-candidate-index",
        LaunchConfiguration("entry_candidate_index").perform(context),
        "--scan-complete-topic",
        LaunchConfiguration("scan_complete_topic").perform(context),
        "--save-scan-service",
        LaunchConfiguration("save_scan_service").perform(context),
        "--startup-delay-sec",
        LaunchConfiguration("startup_delay_sec").perform(context),
        "--scan-timeout-sec",
        LaunchConfiguration("scan_timeout_sec").perform(context),
        "--save-service-timeout-sec",
        LaunchConfiguration("save_service_timeout_sec").perform(context),
        "--shutdown-timeout-sec",
        LaunchConfiguration("shutdown_timeout_sec").perform(context),
        "--needle-mount-delay-sec",
        LaunchConfiguration("needle_mount_delay_sec").perform(context),
    ]
    if use_sim:
        command.append("--use-sim")

    return [
        ExecuteProcess(
            cmd=command,
            output="screen",
            emulate_tty=True,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_cloud_config = PathJoinSubstitution(
        [FindPackageShare("rnm_mapping"), "config", "cloud_stitcher.yaml"]
    )
    default_pipeline_config = PathJoinSubstitution(
        [FindPackageShare("rnm_mapping"), "config", "model_target_entry_pipeline.yaml"]
    )
    default_entry_config = PathJoinSubstitution(
        [FindPackageShare("rnm_mapping"), "config", "needle_entry_planner.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument(
                "cloud_stitcher_params",
                default_value=default_cloud_config,
            ),
            DeclareLaunchArgument(
                "pipeline_config",
                default_value=default_pipeline_config,
            ),
            DeclareLaunchArgument(
                "needle_entry_config",
                default_value=default_entry_config,
            ),
            DeclareLaunchArgument(
                "target_location_file",
                default_value="/workspaces/rnm-documents-primary/target_output/target_location.txt",
            ),
            DeclareLaunchArgument(
                "entry_location_file",
                default_value="/workspaces/rnm-documents-primary/entry_point_output/needle_entry_point.txt",
            ),
            DeclareLaunchArgument("entry_candidate_index", default_value="0"),
            DeclareLaunchArgument(
                "scan_complete_topic",
                default_value="/scanning/complete",
            ),
            DeclareLaunchArgument("save_scan_service", default_value="/save_scan"),
            DeclareLaunchArgument("startup_delay_sec", default_value="2.0"),
            DeclareLaunchArgument(
                "scan_timeout_sec",
                default_value="0.0",
                description="0 disables the scan timeout.",
            ),
            DeclareLaunchArgument("save_service_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("shutdown_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("needle_mount_delay_sec", default_value="15.0"),
            OpaqueFunction(function=_launch_supervisor),
        ]
    )
