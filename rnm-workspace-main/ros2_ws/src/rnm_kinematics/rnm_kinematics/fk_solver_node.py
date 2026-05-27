"""ROS 2 node for Franka Panda forward kinematics."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from rnm_kinematics.fk_solver import (
    ForwardKinematicsSolver,
    quaternion_from_rotation_matrix,
)

DEFAULT_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]


class ForwardKinematicsNode(Node):
    """Subscribe to joint states and publish the Panda flange pose."""

    def __init__(self) -> None:
        super().__init__("fk_solver_node")

        self.declare_parameter("dh_d", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("dh_a", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("dh_alpha", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("pose_topic", "/fk_pose")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("end_effector_frame", "panda_fk_flange")
        self.declare_parameter("publish_tf", True)

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._end_effector_frame = str(
            self.get_parameter("end_effector_frame").value
        )
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)

        self._fk_solver = ForwardKinematicsSolver(self._load_dh_params())
        if len(self._joint_names) > self._fk_solver.row_count:
            raise ValueError(
                "joint_names cannot contain more entries than the DH table rows."
            )

        self._missing_joints_logged = False
        self._pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_states_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"FK node listening on {joint_states_topic}, publishing poses on {pose_topic}."
        )

    def _load_dh_params(self) -> np.ndarray:
        d_values = list(self.get_parameter("dh_d").value or [])
        a_values = list(self.get_parameter("dh_a").value or [])
        alpha_values = list(self.get_parameter("dh_alpha").value or [])

        if not d_values or not a_values or not alpha_values:
            raise ValueError(
                "DH parameters are missing. Launch this node with panda_dh.yaml."
            )
        if not (len(d_values) == len(a_values) == len(alpha_values)):
            raise ValueError("DH parameter arrays dh_d, dh_a, and dh_alpha must match.")

        dh_matrix = np.array([d_values, a_values, alpha_values], dtype=float).T
        self.get_logger().info(f"Loaded DH table with {dh_matrix.shape[0]} rows.")
        return dh_matrix

    def _joint_states_callback(self, msg: JointState) -> None:
        joint_map = {
            name: position for name, position in zip(msg.name, msg.position)
        }
        missing_joints = [
            joint_name for joint_name in self._joint_names if joint_name not in joint_map
        ]
        if missing_joints:
            if not self._missing_joints_logged:
                self.get_logger().warning(
                    f"Waiting for joint states containing: {', '.join(missing_joints)}"
                )
                self._missing_joints_logged = True
            return

        self._missing_joints_logged = False
        joint_positions = [joint_map[joint_name] for joint_name in self._joint_names]
        transform = self._fk_solver.compute(joint_positions)
        stamp = self._stamp_from_joint_state(msg)

        self._publish_pose(transform, stamp)
        if self._tf_broadcaster is not None:
            self._publish_tf_transform(transform, stamp)

    def _stamp_from_joint_state(self, msg: JointState):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return self.get_clock().now().to_msg()
        return msg.header.stamp

    def _publish_pose(self, transform: np.ndarray, stamp) -> None:
        translation = transform[:3, 3]
        quaternion = quaternion_from_rotation_matrix(transform[:3, :3])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self._base_frame
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.x = float(quaternion[0])
        pose_msg.pose.orientation.y = float(quaternion[1])
        pose_msg.pose.orientation.z = float(quaternion[2])
        pose_msg.pose.orientation.w = float(quaternion[3])
        self._pose_publisher.publish(pose_msg)

    def _publish_tf_transform(self, transform: np.ndarray, stamp) -> None:
        translation = transform[:3, 3]
        quaternion = quaternion_from_rotation_matrix(transform[:3, :3])

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._base_frame
        tf_msg.child_frame_id = self._end_effector_frame
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(quaternion[0])
        tf_msg.transform.rotation.y = float(quaternion[1])
        tf_msg.transform.rotation.z = float(quaternion[2])
        tf_msg.transform.rotation.w = float(quaternion[3])
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
