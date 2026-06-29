"""Publish the selected needle entry and target as one PoseArray."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

import numpy as np

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parents[1]))

from rnm_mapping.geometry_utils import quaternion_from_z_axis
from rnm_mapping.geometry_utils import unit_vector
from rnm_mapping.geometry_utils import vector_string


NODE_NAME = "target_entry_pose_array_publisher"


def main(args=None) -> None:
    parsed = _make_parser().parse_args(args=args)
    target = _read_vector_from_report(parsed.target_location, "scan_center_m")
    entry = _read_candidate_entry(parsed.entry_location, parsed.entry_candidate_index)
    insertion_axis = unit_vector(target - entry)
    orientation = quaternion_from_z_axis(insertion_axis)

    print("Publishing target/entry PoseArray.")
    print(f"Entry pose [0] position [m]: {vector_string(entry)}")
    print(f"Target pose [1] position [m]: {vector_string(target)}")
    print(f"Needle axis entry->target [unit]: {vector_string(insertion_axis)}")
    _publish_pose_array(entry, target, orientation, parsed)


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Publish a PoseArray with poses[0] as selected needle entry and "
            "poses[1] as target."
        )
    )
    parser.add_argument(
        "--target-location",
        type=Path,
        default=Path("target_output/target_location.txt"),
    )
    parser.add_argument(
        "--entry-location",
        type=Path,
        default=Path("entry_point_output/needle_entry_point.txt"),
    )
    parser.add_argument("--entry-candidate-index", type=int, default=0)
    parser.add_argument("--pose-array-topic", default="/needle_entry_target_poses")
    parser.add_argument("--target-frame", default="panda_link0")
    parser.add_argument("--publish-duration-sec", type=float, default=5.0)
    parser.add_argument("--publish-rate-hz", type=float, default=5.0)
    return parser


def _read_vector_from_report(path: Path, key: str) -> np.ndarray:
    if not path.exists():
        raise RuntimeError(f"Report file does not exist: {path}")

    text = path.read_text(encoding="utf-8")
    match = re.search(rf"^{re.escape(key)}:\s*\[([^\]]+)\]", text, flags=re.MULTILINE)
    if match is None:
        raise RuntimeError(f"Could not find {key} in {path}")

    vector = np.fromstring(match.group(1), sep=",", dtype=np.float64)
    if len(vector) != 3:
        raise RuntimeError(f"Invalid {key} vector in {path}")
    return vector


def _read_candidate_entry(path: Path, candidate_index: int) -> np.ndarray:
    if not path.exists():
        raise RuntimeError(f"Entry report file does not exist: {path}")

    text = path.read_text(encoding="utf-8")
    block = _candidate_block(text, candidate_index)
    if block is not None:
        return _read_vector_from_text(block, "entry_point_m", path)

    if candidate_index == 0:
        return _read_vector_from_text(text, "entry_point_m", path)

    raise RuntimeError(f"Could not find [candidate_{candidate_index}] in {path}")


def _candidate_block(text: str, candidate_index: int) -> str | None:
    pattern = (
        rf"^\[candidate_{candidate_index}\]\s*$"
        r"(?P<body>.*?)(?=^\[candidate_\d+\]\s*$|\Z)"
    )
    match = re.search(pattern, text, flags=re.MULTILINE | re.DOTALL)
    if match is None:
        return None
    return match.group("body")


def _read_vector_from_text(text: str, key: str, path: Path) -> np.ndarray:
    match = re.search(rf"^{re.escape(key)}:\s*\[([^\]]+)\]", text, flags=re.MULTILINE)
    if match is None:
        raise RuntimeError(f"Could not find {key} in {path}")

    vector = np.fromstring(match.group(1), sep=",", dtype=np.float64)
    if len(vector) != 3:
        raise RuntimeError(f"Invalid {key} vector in {path}")
    return vector


def _publish_pose_array(
    entry: np.ndarray,
    target: np.ndarray,
    orientation: np.ndarray,
    parsed: argparse.Namespace,
) -> None:
    from geometry_msgs.msg import PoseArray
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
    publisher = node.create_publisher(PoseArray, parsed.pose_array_topic, qos)

    publish_rate_hz = max(parsed.publish_rate_hz, 0.1)
    publish_period = Duration(seconds=1.0 / publish_rate_hz)
    end_time = node.get_clock().now() + Duration(
        seconds=max(parsed.publish_duration_sec, 0.0)
    )

    message = PoseArray()
    message.header.frame_id = parsed.target_frame
    message.poses = [
        _pose_from_point(entry, orientation),
        _pose_from_point(target, orientation),
    ]

    print(
        "Publishing PoseArray on "
        f"{parsed.pose_array_topic} in frame {parsed.target_frame}; "
        "poses[0]=entry, poses[1]=target."
    )

    try:
        while rclpy.ok() and node.get_clock().now() <= end_time:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=publish_period.nanoseconds * 1e-9)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _pose_from_point(point: np.ndarray, orientation: np.ndarray):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = float(point[0])
    pose.position.y = float(point[1])
    pose.position.z = float(point[2])
    pose.orientation.x = float(orientation[0])
    pose.orientation.y = float(orientation[1])
    pose.orientation.z = float(orientation[2])
    pose.orientation.w = float(orientation[3])
    return pose


if __name__ == "__main__":
    main()
