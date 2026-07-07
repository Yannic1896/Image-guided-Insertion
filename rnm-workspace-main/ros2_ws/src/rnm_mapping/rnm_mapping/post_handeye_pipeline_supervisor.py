"""Orchestrate the post-handeye RNM scan-to-insertion pipeline."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen


class PostHandeyePipelineSupervisor(Node):
    """Launch and sequence the lab pipeline after hand-eye calibration."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("post_handeye_pipeline_supervisor")
        self._args = args
        self._scan_complete = Event()
        self._processes: list[ManagedProcess] = []
        self.create_subscription(
            Bool,
            args.scan_complete_topic,
            self._scan_complete_callback,
            10,
        )
        self._save_scan_client = self.create_client(Trigger, args.save_scan_service)

    def run(self) -> int:
        try:
            self._start_long_running_stack()
            if not self._wait_for_scan_complete():
                return 1
            if not self._save_scan():
                return 1

            self._stop_process("scanning")
            self._stop_process("cloud_stitcher")

            if not self._run_blocking("model_target_entry_pipeline", self._mapping_command()):
                return 1

            if self._args.needle_mount_delay_sec > 0.0:
                self.get_logger().info(
                    "Mount the needle now. Starting insertion path planning in "
                    f"{self._args.needle_mount_delay_sec:.1f} seconds."
                )
                time.sleep(self._args.needle_mount_delay_sec)

            insertion = self._start_process("insertion_path", self._insertion_command())
            self.get_logger().info(
                "Insertion path node is running. Stop this launch with Ctrl+C when finished."
            )
            return insertion.process.wait()
        finally:
            self._stop_all_processes()

    def _start_long_running_stack(self) -> None:
        self._start_process("ik_solver", self._ik_command())
        time.sleep(self._args.startup_delay_sec)
        self._start_process("trajectory", self._trajectory_command())
        time.sleep(self._args.startup_delay_sec)
        self._start_process("cloud_stitcher", self._cloud_stitcher_command())
        time.sleep(self._args.startup_delay_sec)
        self._start_process("scanning", self._scanning_command())

    def _wait_for_scan_complete(self) -> bool:
        self.get_logger().info(
            f"Waiting for scan completion on {self._args.scan_complete_topic}."
        )
        deadline = (
            time.monotonic() + self._args.scan_timeout_sec
            if self._args.scan_timeout_sec > 0.0
            else None
        )
        while rclpy.ok() and not self._scan_complete.is_set():
            if deadline is not None and time.monotonic() > deadline:
                self.get_logger().error("Timed out waiting for scanning to complete.")
                return False
            if self._check_for_early_process_failure():
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._scan_complete.is_set()

    def _scan_complete_callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("Scanning reported completion.")
            self._scan_complete.set()

    def _save_scan(self) -> bool:
        self.get_logger().info(f"Waiting for save service {self._args.save_scan_service}.")
        if not self._save_scan_client.wait_for_service(
            timeout_sec=self._args.save_service_timeout_sec
        ):
            self.get_logger().error(
                f"Save service not available: {self._args.save_scan_service}"
            )
            return False

        self.get_logger().info("Calling cloud stitcher save service.")
        future = self._save_scan_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self._args.save_service_timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error("Timed out waiting for save_scan response.")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            self.get_logger().error(f"save_scan failed: {message}")
            return False
        self.get_logger().info(response.message)
        return True

    def _start_process(self, name: str, command: list[str]) -> ManagedProcess:
        self.get_logger().info(f"Starting {name}: {' '.join(command)}")
        process = subprocess.Popen(command, preexec_fn=os.setsid)
        managed = ManagedProcess(name=name, process=process)
        self._processes.append(managed)
        return managed

    def _run_blocking(self, name: str, command: list[str]) -> bool:
        managed = self._start_process(name, command)
        return_code = managed.process.wait()
        self._processes = [item for item in self._processes if item is not managed]
        if return_code != 0:
            self.get_logger().error(f"{name} failed with return code {return_code}.")
            return False
        self.get_logger().info(f"{name} completed.")
        return True

    def _stop_process(self, name: str) -> None:
        for managed in list(self._processes):
            if managed.name == name:
                self._terminate_process(managed)
                self._processes.remove(managed)

    def _stop_all_processes(self) -> None:
        for managed in reversed(self._processes):
            self._terminate_process(managed)
        self._processes.clear()

    def _terminate_process(self, managed: ManagedProcess) -> None:
        process = managed.process
        if process.poll() is not None:
            return
        self.get_logger().info(f"Stopping {managed.name}.")
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=self._args.shutdown_timeout_sec)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass

        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=self._args.shutdown_timeout_sec)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self.get_logger().warn(f"{managed.name} did not stop cleanly.")

    def _check_for_early_process_failure(self) -> bool:
        for managed in self._processes:
            return_code = managed.process.poll()
            if return_code is not None and return_code != 0:
                self.get_logger().error(
                    f"{managed.name} exited early with return code {return_code}."
                )
                return True
        return False

    def _ik_command(self) -> list[str]:
        return [
            "ros2",
            "launch",
            "rnm_kinematics",
            "ik_solver.launch.py",
            f"use_sim:={_bool_arg(self._args.use_sim)}",
        ]

    def _trajectory_command(self) -> list[str]:
        return [
            "ros2",
            "launch",
            "rnm_trajectory",
            "start_trajectory.launch.py",
            f"use_sim:={_bool_arg(self._args.use_sim)}",
        ]

    def _cloud_stitcher_command(self) -> list[str]:
        return [
            "ros2",
            "launch",
            "rnm_mapping",
            "cloud_stitcher.launch.py",
            f"params_file:={self._args.cloud_stitcher_params}",
        ]

    def _scanning_command(self) -> list[str]:
        return [
            "ros2",
            "launch",
            "rnm_scanning",
            "rnm_scanning.launch.py",
            "scanning_mode:=model_registration",
            f"scan_complete_topic:={self._args.scan_complete_topic}",
        ]

    def _mapping_command(self) -> list[str]:
        return [
            "ros2",
            "launch",
            "rnm_mapping",
            "model_target_entry_pipeline.launch.py",
            f"pipeline_config:={self._args.pipeline_config}",
            f"needle_entry_config:={self._args.needle_entry_config}",
        ]

    def _insertion_command(self) -> list[str]:
        return [
            "ros2",
            "launch",
            "rnm_needle",
            "insertion_path.launch.py",
            "input_source:=file",
            f"target_location_file:={self._args.target_location_file}",
            f"entry_location_file:={self._args.entry_location_file}",
            f"entry_candidate_index:={self._args.entry_candidate_index}",
        ]


def _bool_arg(value: bool) -> str:
    return "true" if value else "false"


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the RNM post-handeye scan, mapping, and insertion pipeline."
    )
    parser.add_argument("--use-sim", action="store_true")
    parser.add_argument("--cloud-stitcher-params", required=True)
    parser.add_argument("--pipeline-config", required=True)
    parser.add_argument("--needle-entry-config", required=True)
    parser.add_argument("--target-location-file", required=True)
    parser.add_argument("--entry-location-file", required=True)
    parser.add_argument("--entry-candidate-index", type=int, default=0)
    parser.add_argument("--scan-complete-topic", default="/scanning/complete")
    parser.add_argument("--save-scan-service", default="/save_scan")
    parser.add_argument("--startup-delay-sec", type=float, default=2.0)
    parser.add_argument("--scan-timeout-sec", type=float, default=0.0)
    parser.add_argument("--save-service-timeout-sec", type=float, default=20.0)
    parser.add_argument("--shutdown-timeout-sec", type=float, default=5.0)
    parser.add_argument("--needle-mount-delay-sec", type=float, default=15.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    parsed = _make_parser().parse_args(argv)
    for path_name in (
        "cloud_stitcher_params",
        "pipeline_config",
        "needle_entry_config",
    ):
        path = Path(getattr(parsed, path_name))
        if not path.exists():
            raise SystemExit(f"{path_name} does not exist: {path}")

    rclpy.init(args=None)
    node = PostHandeyePipelineSupervisor(parsed)
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
