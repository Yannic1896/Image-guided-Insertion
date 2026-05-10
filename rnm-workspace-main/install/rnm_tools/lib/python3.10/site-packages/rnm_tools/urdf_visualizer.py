"""Realtime Viser-based URDF visualization driven by ROS 2 topics."""

#test

from __future__ import annotations

import re
import tempfile
import threading
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy
import viser
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from viser.extras import ViserUrdf

PACKAGE_URI_PATTERN = re.compile(r"package://([^/]+)/([^\"'<>\\s]+)")


class UrdfVisualizerNode(Node):
    """Subscribe to robot_description and joint_states, then mirror them in Viser."""

    def __init__(self) -> None:
        super().__init__("urdf_visualizer")

        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("viser_host", "0.0.0.0")
        self.declare_parameter("viser_port", 8080)
        self.declare_parameter("load_meshes", True)
        self.declare_parameter("load_collision_meshes", False)

        host = self.get_parameter("viser_host").get_parameter_value().string_value
        port = self.get_parameter("viser_port").get_parameter_value().integer_value
        load_meshes = self.get_parameter("load_meshes").get_parameter_value().bool_value
        load_collision_meshes = (
            self.get_parameter("load_collision_meshes").get_parameter_value().bool_value
        )

        self._server = viser.ViserServer(host=host, port=port)
        self._server.scene.add_grid("/grid", width=2.0, height=2.0)

        self._load_meshes = load_meshes
        self._load_collision_meshes = load_collision_meshes
        self._lock = threading.Lock()
        self._current_urdf: Optional[ViserUrdf] = None
        self._joint_order: list[str] = []
        self._joint_positions: Dict[str, float] = {}
        self._description_path: Optional[Path] = None

        self.get_logger().info(f"Viser server listening on http://{host}:{port}")

        description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        robot_description_topic = (
            self.get_parameter("robot_description_topic")
            .get_parameter_value()
            .string_value
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )

        self.create_subscription(
            String,
            robot_description_topic,
            self._robot_description_callback,
            description_qos,
        )
        self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_states_callback,
            qos_profile_sensor_data,
        )

    def _robot_description_callback(self, msg: String) -> None:
        urdf_text = msg.data.strip()
        if not urdf_text:
            self.get_logger().warning("Received empty robot_description.")
            return

        try:
            resolved_urdf = self._resolve_package_uris(urdf_text)
            description_path = self._write_temp_urdf(resolved_urdf)
            urdf_handle = ViserUrdf(
                self._server,
                urdf_or_path=description_path,
                load_meshes=self._load_meshes,
                load_collision_meshes=self._load_collision_meshes,
                collision_mesh_color_override=(1.0, 0.0, 0.0, 0.35),
            )
            joint_order = list(urdf_handle.get_actuated_joint_limits().keys())
        except Exception as exc:  # pragma: no cover - runtime dependency failures
            self.get_logger().error(
                f"Failed to load robot_description into viser: {exc}"
            )
            return

        with self._lock:
            if self._current_urdf is not None:
                self._current_urdf.remove()
            self._current_urdf = urdf_handle
            self._joint_order = joint_order

            if self._description_path is not None and self._description_path.exists():
                self._description_path.unlink(missing_ok=True)
            self._description_path = description_path

            self._update_visualization_locked()

        self.get_logger().info(
            f"Loaded URDF with {len(self._joint_order)} actuated joints into viser."
        )

    def _joint_states_callback(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_positions[name] = position
            self._update_visualization_locked()

    def _update_visualization_locked(self) -> None:
        if self._current_urdf is None or not self._joint_order:
            return

        cfg = np.array(
            [
                self._joint_positions.get(joint_name, 0.0)
                for joint_name in self._joint_order
            ],
            dtype=float,
        )
        self._current_urdf.update_cfg(cfg)

    def _resolve_package_uris(self, urdf_text: str) -> str:
        def replace(match: re.Match[str]) -> str:
            package_name = match.group(1)
            relative_path = match.group(2)
            try:
                package_share = Path(get_package_share_directory(package_name))
            except PackageNotFoundError:
                self.get_logger().warning(
                    f"Could not resolve package:// URI for package '{package_name}'."
                )
                return match.group(0)

            return str(package_share / relative_path)

        return PACKAGE_URI_PATTERN.sub(replace, urdf_text)

    def _write_temp_urdf(self, urdf_text: str) -> Path:
        temp_file = tempfile.NamedTemporaryFile(
            mode="w",
            prefix="rnm_tools_",
            suffix=".urdf",
            delete=False,
        )
        with temp_file:
            temp_file.write(urdf_text)
        return Path(temp_file.name)

    def destroy_node(self) -> bool:
        with self._lock:
            if self._current_urdf is not None:
                self._current_urdf.remove()
                self._current_urdf = None
            if self._description_path is not None:
                self._description_path.unlink(missing_ok=True)
                self._description_path = None

        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UrdfVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
