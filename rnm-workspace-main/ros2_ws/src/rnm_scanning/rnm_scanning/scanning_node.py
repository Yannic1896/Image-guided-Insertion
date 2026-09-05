"""Synchronizes robot movement & image acquisition."""

import math
import os
import random

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, UInt64


class ScanningNode(Node):
    def __init__(self) -> None:
        super().__init__("scanning_node")

        self.declare_parameter(
            "scanning_mode", "hand_eye"
        )  # 'hand_eye' or 'model_registration'
        self.declare_parameter(
            "joint_states_topic", "/franka_state_controller/joint_states_desired"
        )
        self.declare_parameter("trajectory_finished_topic", "/trajectory_finished")
        self.declare_parameter(
            "pose_capture_done_topic", "/cloud_stitcher/pose_capture_done"
        )
        self.declare_parameter("scan_complete_topic", "/scanning/complete")
        self.declare_parameter(
            "joint_names",
            [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ],
        )
        self.declare_parameter("motion_start_threshold", 0.002)
        self.declare_parameter("use_capture_feedback", True)
        self.declare_parameter("capture_wait_timeout", 15.0)
        self.declare_parameter("publish_pointcloud_trigger", False)
        self.declare_parameter("max_samples", 30)
        self.declare_parameter("command_delay", 5.0)
        self.declare_parameter("max_retries", 5)
        self.declare_parameter("joint_feedback_timeout", 1.0)
        self.declare_parameter("trajectory_goal_tolerance", 0.01)

        # Bounding box of robot workspace
        self.declare_parameter("x_min", 0.40)
        self.declare_parameter("x_max", 0.85)
        self.declare_parameter("y_min", -0.30)
        self.declare_parameter("y_max", 0.30)
        self.declare_parameter("z_min", 0.40)
        self.declare_parameter("z_max", 0.70)

        self.declare_parameter("center_x", 0.2)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.6)

        self.declare_parameter("roll_deg", -90.0)
        self.declare_parameter("pitch_deg", 45.0)
        self.declare_parameter("yaw_deg", -90.0)

        _default_poses_file = os.path.join(
            get_package_share_directory("rnm_scanning"),
            "config",
            "stitching poses.yaml",
        )
        self.declare_parameter("stitching_poses_file", _default_poses_file)

        self.scanning_mode = self.get_parameter("scanning_mode").value
        if self.scanning_mode not in {"hand_eye", "model_registration"}:
            raise ValueError(
                "scanning_mode must be 'hand_eye' or 'model_registration', "
                f"got '{self.scanning_mode}'"
            )
        self.max_samples = self.get_parameter("max_samples").value
        self.command_delay = self.get_parameter("command_delay").value
        self.max_retries = self.get_parameter("max_retries").value
        self.joint_feedback_timeout = self.get_parameter(
            "joint_feedback_timeout"
        ).value
        self.trajectory_goal_tolerance = self.get_parameter(
            "trajectory_goal_tolerance"
        ).value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        trajectory_finished_topic = self.get_parameter(
            "trajectory_finished_topic"
        ).value
        pose_capture_done_topic = self.get_parameter("pose_capture_done_topic").value
        scan_complete_topic = self.get_parameter("scan_complete_topic").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.motion_start_threshold = self.get_parameter(
            "motion_start_threshold"
        ).value
        self.use_capture_feedback = self.get_parameter("use_capture_feedback").value
        self.capture_wait_timeout = self.get_parameter("capture_wait_timeout").value
        self.publish_pointcloud_trigger = self.get_parameter(
            "publish_pointcloud_trigger"
        ).value

        self.x_bounds = (
            self.get_parameter("x_min").value,
            self.get_parameter("x_max").value,
        )
        self.y_bounds = (
            self.get_parameter("y_min").value,
            self.get_parameter("y_max").value,
        )
        self.z_bounds = (
            self.get_parameter("z_min").value,
            self.get_parameter("z_max").value,
        )

        poses_file = self.get_parameter("stitching_poses_file").value
        self.model_reg_joints = self._load_stitching_poses(poses_file)

        self.start_roll = math.radians(self.get_parameter("roll_deg").value)
        self.start_pitch = math.radians(self.get_parameter("pitch_deg").value)
        self.start_yaw = math.radians(self.get_parameter("yaw_deg").value)
        self.max_orientation_tweak = math.radians(5.0)
        self.center_x = self.get_parameter("center_x").value
        self.center_y = self.get_parameter("center_y").value
        self.center_z = self.get_parameter("center_z").value

        self.chessboard_visible = False
        self.startPoseSend = False
        self.sample_count = 0
        self.model_reg_index = 0
        self.active_timer = None
        self.last_finished_count = -1
        self.count_at_last_send = -1
        self.number_retries = 0
        self.current_target_msg = None  # for hand-eye
        self.current_joint_msg = None  # for model-registration
        self.current_joint_positions = None
        self.current_goal_positions = None
        self.last_joint_feedback_time = None
        self.motion_start_reference = None
        self.capture_done_state = False
        self.capture_feedback_reset_seen = True
        self.stale_capture_warning_logged = False
        self.stale_joint_feedback_warning_logged = False
        self.finished_before_goal_warning_logged = False
        self.missing_joint_names_warning_logged = False
        self.workflow_state = "idle"
        self.scan_complete_published = False

        # Subscribers
        self.detect_sub = self.create_subscription(
            Bool, "/chessboard_detected", self.detect_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_states_callback, 10
        )
        self.trajectory_finished_sub = self.create_subscription(
            UInt64, trajectory_finished_topic, self.trajectory_finished_callback, 10
        )
        self.pose_capture_done_sub = self.create_subscription(
            Bool, pose_capture_done_topic, self.pose_capture_done_callback, 10
        )

        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.ik_joint_goal_pub = self.create_publisher(
            Float64MultiArray, "/ik_joint_goal", 10
        )
        self.trigger_image_pub = self.create_publisher(
            Bool, "/trigger_image_capture", 10
        )
        self.trigger_pointcloud_pub = self.create_publisher(
            Bool, "/trigger_pointcloud_capture", 10
        )
        scan_complete_qos = QoSProfile(depth=1)
        scan_complete_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        scan_complete_qos.reliability = ReliabilityPolicy.RELIABLE
        self.scan_complete_pub = self.create_publisher(
            Bool, scan_complete_topic, scan_complete_qos
        )

        self.get_logger().info(
            f"Scanning Node started in [{self.scanning_mode}] mode."
        )
        self.get_logger().info(
            "Waiting for first joint-state sample before sending scan target."
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def detect_callback(self, msg: Bool) -> None:
        self.chessboard_visible = msg.data

    def joint_states_callback(self, msg: JointState) -> None:
        positions = self._ordered_joint_positions(msg)
        if positions is None:
            return

        self.current_joint_positions = positions
        self.last_joint_feedback_time = self.get_clock().now()
        if self.workflow_state == "idle":
            self.send_next_target()
            return

        if self.workflow_state == "waiting_motion_start":
            if self.motion_start_reference is None:
                self.motion_start_reference = list(positions)
                return
            if self._motion_has_started():
                self._handle_motion_started()

        if (
            self.workflow_state in {"waiting_motion_start", "waiting_motion_finish"}
            and self._has_arrived()
        ):
            self._handle_arrival()

    def trajectory_finished_callback(self, msg: UInt64) -> None:
        # /trajectory_finished is a UInt64 counter that increments for every trajectory completion
        self.last_finished_count = msg.data
        if (
            self.workflow_state in {"waiting_motion_start", "waiting_motion_finish"}
            and self._has_arrived()
        ):
            self._handle_arrival()

    def pose_capture_done_callback(self, msg: Bool) -> None:
        done = bool(msg.data)
        self.capture_done_state = done

        if not done:
            self.capture_feedback_reset_seen = True
            return

        if (
            self.scanning_mode == "model_registration"
            and self.workflow_state == "waiting_capture"
        ):
            if self.capture_feedback_reset_seen:
                self._finish_model_registration_capture("cloud stitcher feedback")
            elif not self.stale_capture_warning_logged:
                self.stale_capture_warning_logged = True
                self.get_logger().warn(
                    "Ignoring capture-done=true because no false reset was seen "
                    "after commanding this scan point."
                )

    def _has_arrived(self) -> bool:
        if self.last_finished_count <= self.count_at_last_send:
            return False

        if self._goal_reached():
            return True

        if not self.finished_before_goal_warning_logged:
            self.finished_before_goal_warning_logged = True
            self.get_logger().warn(
                "trajectory_finished advanced before joint feedback reached "
                "the commanded scan goal; waiting for goal tolerance before "
                "continuing."
            )
        return False

    def _goal_reached(self) -> bool:
        if self.scanning_mode != "model_registration":
            return True

        if self.current_goal_positions is None or self.current_joint_positions is None:
            return False

        deltas = [
            abs(current - goal)
            for current, goal in zip(
                self.current_joint_positions, self.current_goal_positions
            )
        ]
        return max(deltas, default=float("inf")) <= self.trajectory_goal_tolerance

    def _motion_has_started(self) -> bool:
        if self.motion_start_reference is None or self.current_joint_positions is None:
            return False
        deltas = [
            abs(current - start)
            for current, start in zip(
                self.current_joint_positions, self.motion_start_reference
            )
        ]
        return max(deltas, default=0.0) >= self.motion_start_threshold

    def _joint_feedback_is_fresh(self) -> bool:
        if self.last_joint_feedback_time is None:
            return False
        age = (
            self.get_clock().now() - self.last_joint_feedback_time
        ).nanoseconds / 1e9
        return age <= self.joint_feedback_timeout

    def _cancel_active_timer(self) -> None:
        if self.active_timer is not None:
            self.active_timer.cancel()
            self.active_timer = None

    def _handle_motion_started(self) -> None:
        if self.workflow_state != "waiting_motion_start":
            return
        if self._has_arrived():
            self._handle_arrival()
            return
        self._cancel_active_timer()
        self.workflow_state = "waiting_motion_finish"
        if self.scanning_mode == "model_registration":
            self.get_logger().info(
                f"Robot started moving for scan point {self.model_reg_index + 1}/"
                f"{len(self.model_reg_joints)}; waiting for trajectory finish."
            )
        else:
            self.get_logger().info(
                "Robot started moving; waiting for trajectory finish."
            )

    def _handle_arrival(self) -> None:
        if self.workflow_state not in {
            "waiting_motion_start",
            "waiting_motion_finish",
        }:
            return
        self._cancel_active_timer()
        self.scanning_loop()
    
    def _resend_current_command(self) -> None:
        if self.workflow_state != "waiting_motion_start":
            self._cancel_active_timer()
            return

        if self._has_arrived():
            self._handle_arrival()
            return

        if self._motion_has_started():
            self._handle_motion_started()
            return

        if not self._joint_feedback_is_fresh():
            if not self.stale_joint_feedback_warning_logged:
                self.stale_joint_feedback_warning_logged = True
                self.get_logger().warn(
                    "Joint feedback is stale; not resending trajectory because "
                    "robot motion state cannot be confirmed."
                )
            return

        if self.number_retries >= self.max_retries:
            self._cancel_active_timer()
            self.workflow_state = "failed"
            self.get_logger().error(
                "Robot did not start moving after "
                f"{self.max_retries} retries; capture skipped."
            )
            return

        self.number_retries += 1

        if self.scanning_mode == "hand_eye":
            self.get_logger().info(
                "Resending target pose because robot has not started moving yet."
            )
            self.target_pub.publish(self.current_target_msg)
        elif self.scanning_mode == "model_registration":
            self.get_logger().info(
                f"Resending scan point {self.model_reg_index + 1}/"
                f"{len(self.model_reg_joints)} "
                f"(attempt {self.number_retries + 1}/{self.max_retries + 1})."
            )
            self.ik_joint_goal_pub.publish(self.current_joint_msg)


    def scanning_loop(self) -> None:
        if self.scanning_mode == "hand_eye":
            if self.chessboard_visible:
                self.get_logger().info(
                    f"Robot settled and chessboard visible — triggering image capture "
                    f"({self.sample_count + 1}/{self.max_samples})."
                )
                trigger_msg = Bool()
                trigger_msg.data = True
                self.trigger_image_pub.publish(trigger_msg)
                self.sample_count += 1

                if self.sample_count >= self.max_samples:
                    self.get_logger().info(
                        "Hand-eye calibration data collection complete."
                    )
                    self.workflow_state = "done"
                    self._publish_scan_complete()
                    return
            else:

                self.get_logger().warn("Chessboard not visible")

        elif self.scanning_mode == "model_registration":
            self._start_model_registration_capture_wait()
            return

        self.send_next_target()

    def _start_model_registration_capture_wait(self) -> None:
        self.workflow_state = "waiting_capture"
        self.get_logger().info(
            f"Arrived at scan point {self.model_reg_index + 1}/"
            f"{len(self.model_reg_joints)}; waiting for point cloud capture."
        )

        if self.publish_pointcloud_trigger:
            trigger_msg = Bool()
            trigger_msg.data = True
            self.trigger_pointcloud_pub.publish(trigger_msg)

        if self.use_capture_feedback:
            if not self.capture_feedback_reset_seen:
                self.get_logger().warn(
                    "Capture feedback is still true from the previous scan point; "
                    "waiting for cloud stitcher to publish false, then true."
                )
            self.get_logger().info(
                "Waiting for cloud stitcher capture feedback."
            )
        else:
            self.get_logger().info(
                f"Capture feedback disabled; using {self.capture_wait_timeout:.2f}s timer."
            )

        if self.capture_wait_timeout > 0.0:
            self._cancel_active_timer()
            self.active_timer = self.create_timer(
                self.capture_wait_timeout,
                self._capture_wait_timeout_callback,
            )
        elif not self.use_capture_feedback:
            self._finish_model_registration_capture("zero capture wait timeout")

    def _capture_wait_timeout_callback(self) -> None:
        if self.workflow_state != "waiting_capture":
            self._cancel_active_timer()
            return

        if self.use_capture_feedback:
            self.get_logger().warn(
                "Timed out waiting for cloud stitcher capture feedback; "
                "advancing by timer fallback."
            )
        self._finish_model_registration_capture("capture wait timer")

    def _finish_model_registration_capture(self, reason: str) -> None:
        if self.workflow_state != "waiting_capture":
            return

        self._cancel_active_timer()
        self.get_logger().info(
            f"Point cloud capture complete for scan point "
            f"{self.model_reg_index + 1}/{len(self.model_reg_joints)} "
            f"({reason})."
        )
        self.model_reg_index += 1

        if self.model_reg_index >= len(self.model_reg_joints):
            self.workflow_state = "done"
            self.get_logger().info("Scanning complete — all point clouds captured.")
            self._publish_scan_complete()
            return

        self.send_next_target()

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def send_next_target(self) -> None:
        
        self.count_at_last_send = self.last_finished_count
        self.number_retries = 0
        self.motion_start_reference = (
            list(self.current_joint_positions)
            if self.current_joint_positions is not None
            else None
        )
        self.capture_feedback_reset_seen = not self.capture_done_state
        self.stale_capture_warning_logged = False
        self.stale_joint_feedback_warning_logged = False
        self.finished_before_goal_warning_logged = False

        self._cancel_active_timer()

        if self.scanning_mode == "hand_eye":
            self.workflow_state = "waiting_motion_start"
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = "panda_link0"

            if not self.startPoseSend:
                x = self.center_x
                y = self.center_y
                z = self.center_z
                roll = self.start_roll
                pitch = self.start_pitch
                yaw = self.start_yaw
                self.startPoseSend = True
            else:
                x = random.uniform(*self.x_bounds)
                y = random.uniform(*self.y_bounds)
                z = random.uniform(*self.z_bounds)
                t = self.max_orientation_tweak
                roll = self.start_roll + random.uniform(-t, t)
                pitch = self.start_pitch + random.uniform(-t, t)
                yaw = self.start_yaw + random.uniform(-t, t)
                self.get_logger().info(
                    f"Moving to hand-eye pose: XYZ,RYP=[{x:.3f}, {y:.3f}, {z:.3f}, {roll:.3f}, {pitch:.3f}, {yaw:.3f}]"
                )

            target_msg.pose.position.x = x
            target_msg.pose.position.y = y
            target_msg.pose.position.z = z
            target_msg.pose.orientation = self.euler_to_quaternion(roll, pitch, yaw)

            self.current_target_msg = target_msg
            self.current_goal_positions = None
            self.target_pub.publish(target_msg)
            self.active_timer = self.create_timer(
                self.command_delay, self._resend_current_command
            )

        elif self.scanning_mode == "model_registration":
            if self.model_reg_index >= len(self.model_reg_joints):
                self.workflow_state = "done"
                self.get_logger().info("No more poses left.")
                self._publish_scan_complete()
                return
            self.workflow_state = "waiting_motion_start"
            joints = self.model_reg_joints[self.model_reg_index]
            self.get_logger().info(
                f"Moving to scan point {self.model_reg_index + 1}/"
                f"{len(self.model_reg_joints)} "
                f"(attempt 1/{self.max_retries + 1})."
            )
            joint_msg = Float64MultiArray()
            joint_msg.data = joints

            self.current_joint_msg = joint_msg
            self.current_goal_positions = list(joints)
            self.ik_joint_goal_pub.publish(joint_msg)
            self.active_timer = self.create_timer(
                self.command_delay, self._resend_current_command
            )

    def _ordered_joint_positions(self, msg: JointState) -> list[float] | None:
        if msg.name:
            joint_map = {
                name: position for name, position in zip(msg.name, msg.position)
            }
            missing = [name for name in self.joint_names if name not in joint_map]
            if missing:
                if not self.missing_joint_names_warning_logged:
                    self.missing_joint_names_warning_logged = True
                    self.get_logger().warn(
                        "Joint state message is missing expected joints: "
                        f"{missing}."
                    )
                return None
            return [float(joint_map[name]) for name in self.joint_names]

        if len(msg.position) < len(self.joint_names):
            if not self.missing_joint_names_warning_logged:
                self.missing_joint_names_warning_logged = True
                self.get_logger().warn(
                    f"Joint state message has {len(msg.position)} positions, "
                    f"expected at least {len(self.joint_names)}."
                )
            return None

        return [
            float(position) for position in msg.position[: len(self.joint_names)]
        ]

    def _publish_scan_complete(self) -> None:
        if self.scan_complete_published:
            return
        self.scan_complete_published = True
        msg = Bool()
        msg.data = True
        self.scan_complete_pub.publish(msg)

    def _load_stitching_poses(self, path: str) -> list[list[float]]:
        """Parse a multi-document JointState YAML and return a list of position arrays."""
        if not os.path.exists(path):
            raise FileNotFoundError(f"Stitching poses file does not exist: {path}")

        with open(path, "r") as f:
            docs = [d for d in yaml.safe_load_all(f) if d and "position" in d]

        poses = []
        for index, doc in enumerate(docs):
            positions = doc["position"]
            if not isinstance(positions, list) or len(positions) != 7:
                raise ValueError(
                    "Each stitching pose must contain exactly 7 joint positions; "
                    f"pose {index} has {len(positions) if isinstance(positions, list) else 'invalid'}."
                )
            poses.append([float(value) for value in positions])

        if not poses:
            raise ValueError(f"No stitching poses found in '{path}'")

        self.get_logger().info(f"Loaded {len(docs)} stitching poses from '{path}'")
        return poses

    def _publish_scan_complete(self) -> None:
        if self.scan_complete_published:
            return
        self.scan_complete_published = True
        msg = Bool()
        msg.data = True
        self.scan_complete_pub.publish(msg)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
