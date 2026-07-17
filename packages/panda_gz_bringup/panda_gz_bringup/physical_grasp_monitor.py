"""Observe real Gazebo contacts and pose drift for physics-only grasps."""

from __future__ import annotations

import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray
from panda_pick_place_interfaces.srv import GetPhysicalGraspState
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from panda_gz_bringup.physical_grasp_classifier import (
    PhysicalGraspClassifier,
    PhysicalGraspConfig,
    PoseSample,
)
from panda_gz_bringup.status import StatusCode, service_message


class PhysicalGraspMonitor(Node):
    """Classify bilateral contact, slip, and release without constraints."""

    def __init__(self) -> None:
        super().__init__("physical_grasp_monitor")
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._latest_cube_pose: Pose | None = None
        self._left_contact = False
        self._right_contact = False
        self._last_contact_time_sec: float | None = None

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("hand_frame", "panda_hand")
        self.declare_parameter("left_finger_collision_token", "panda_leftfinger")
        self.declare_parameter("right_finger_collision_token", "panda_rightfinger")
        self.declare_parameter("contact_freshness_sec", 0.10)
        self.declare_parameter("bilateral_contact_confirm_sec", 0.10)
        self.declare_parameter("release_confirm_sec", 0.10)
        self.declare_parameter("max_initial_hand_object_distance_m", 0.09)
        self.declare_parameter("max_relative_translation_drift_m", 0.015)
        self.declare_parameter("max_relative_rotation_drift_rad", 0.20)
        self.declare_parameter("stable_linear_speed_mps", 0.05)
        self.declare_parameter("update_rate_hz", 50.0)
        # The MoveIt resource model is intentionally shared with mock planning,
        # but it does not yet contain validated Franka mass / inertia data.
        # Refuse to emit usable physical-grasp truth until that model is audited.
        self.declare_parameter("physics_model_validated", False)

        self._classifier = PhysicalGraspClassifier(self._classifier_config())
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, "/panda_gz/physical_grasp_status", 10
        )
        self._holding_pub = self.create_publisher(
            Bool, "/panda_gz/physical_holding_object", 10
        )
        self.create_subscription(
            Contacts,
            "/panda_gz/cube_contacts",
            self._on_contacts,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            PoseArray,
            "/panda_gz/cube_pose",
            self._on_cube_pose,
            10,
            callback_group=self._group,
        )
        self.create_service(
            GetPhysicalGraspState,
            "/panda_gz/get_physical_grasp_state",
            self._get_state,
            callback_group=self._group,
        )
        update_rate = float(self.get_parameter("update_rate_hz").value)
        if update_rate <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        self.create_timer(
            1.0 / update_rate,
            self._update_classifier,
            callback_group=self._group,
        )
        self.create_timer(
            0.2,
            self._publish_diagnostic,
            callback_group=self._group,
        )

    def _classifier_config(self) -> PhysicalGraspConfig:
        def value(name: str) -> float:
            return float(self.get_parameter(name).value)

        return PhysicalGraspConfig(
            bilateral_contact_confirm_sec=value(
                "bilateral_contact_confirm_sec"
            ),
            release_confirm_sec=value("release_confirm_sec"),
            max_initial_hand_object_distance_m=value(
                "max_initial_hand_object_distance_m"
            ),
            max_relative_translation_drift_m=value(
                "max_relative_translation_drift_m"
            ),
            max_relative_rotation_drift_rad=value(
                "max_relative_rotation_drift_rad"
            ),
            stable_linear_speed_mps=value("stable_linear_speed_mps"),
        )

    def _on_cube_pose(self, message: PoseArray) -> None:
        if message.poses:
            with self._lock:
                self._latest_cube_pose = message.poses[0]

    def _on_contacts(self, message: Contacts) -> None:
        left_token = str(
            self.get_parameter("left_finger_collision_token").value
        )
        right_token = str(
            self.get_parameter("right_finger_collision_token").value
        )
        left = False
        right = False
        for contact in message.contacts:
            collision_names = (
                str(contact.collision1.name),
                str(contact.collision2.name),
            )
            left = left or any(left_token in name for name in collision_names)
            right = right or any(right_token in name for name in collision_names)
        with self._lock:
            self._left_contact = left
            self._right_contact = right
            self._last_contact_time_sec = self._now_sec()

    def _update_classifier(self) -> None:
        now = self._now_sec()
        with self._lock:
            cube_pose = self._latest_cube_pose
            contact_age = (
                now - self._last_contact_time_sec
                if self._last_contact_time_sec is not None
                else float("inf")
            )
            contacts_fresh = contact_age <= float(
                self.get_parameter("contact_freshness_sec").value
            )
            left_contact = self._left_contact if contacts_fresh else False
            right_contact = self._right_contact if contacts_fresh else False
        if cube_pose is None:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("world_frame").value),
                str(self.get_parameter("hand_frame").value),
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return

        hand_translation = transform.transform.translation
        hand_rotation = transform.transform.rotation
        hand = PoseSample.create(
            (hand_translation.x, hand_translation.y, hand_translation.z),
            (hand_rotation.x, hand_rotation.y, hand_rotation.z, hand_rotation.w),
        )
        cube = self._pose_sample(cube_pose)
        with self._lock:
            try:
                self._classifier.update(
                    timestamp_sec=now,
                    left_finger_contact=left_contact,
                    right_finger_contact=right_contact,
                    hand_pose=hand,
                    object_pose=cube,
                )
            except ValueError as exc:
                self.get_logger().error(f"physical grasp sample rejected: {exc}")

    def _get_state(
        self,
        request: GetPhysicalGraspState.Request,
        response: GetPhysicalGraspState.Response,
    ) -> GetPhysicalGraspState.Response:
        reset_command = GetPhysicalGraspState.Request.COMMAND_RESET
        query_command = GetPhysicalGraspState.Request.COMMAND_QUERY
        with self._lock:
            if request.command == reset_command:
                self._classifier.reset(episode_id=request.episode_id)
                # A reset starts a new observation generation.  Reusing a fresh
                # contact timestamp or pose from the previous episode can
                # otherwise confirm a grasp that never happened in this one.
                self._latest_cube_pose = None
                self._left_contact = False
                self._right_contact = False
                self._last_contact_time_sec = None
            elif request.command != query_command:
                code = StatusCode.INVALID_REQUEST
                response.success = False
                response.status_code = int(code)
                response.failure_type = code.name.lower()
                response.message = service_message(code, "unknown monitor command")
                return response
            snapshot = self._classifier.snapshot()
            cube_pose = self._latest_cube_pose

        if not bool(self.get_parameter("physics_model_validated").value):
            code = StatusCode.PHYSICS_MODEL_UNVALIDATED
            response.success = False
            response.failure_type = code.name.lower()
            detail = (
                "shared Panda Gazebo mass/inertia model has not been validated; "
                "physical-grasp results are disabled"
            )
        elif request.command == query_command and not snapshot.initialized:
            code = StatusCode.PHYSICS_STATE_UNAVAILABLE
            response.success = False
            response.failure_type = code.name.lower()
            detail = "waiting for cube pose, contacts, and world-to-hand TF"
        else:
            code = StatusCode.READY
            response.success = True
            response.failure_type = ""
            detail = "physical grasp state reset" if request.command else "state ready"
        response.status_code = int(code)
        response.message = service_message(code, detail)
        response.episode_id = snapshot.episode_id
        response.grasped = snapshot.grasped
        response.holding_object = snapshot.holding_object
        response.released = snapshot.released
        response.left_finger_contact = snapshot.left_finger_contact
        response.right_finger_contact = snapshot.right_finger_contact
        response.slipping = snapshot.slipping
        response.max_relative_translation_drift_m = (
            snapshot.max_relative_translation_drift_m
        )
        response.max_relative_rotation_drift_rad = (
            snapshot.max_relative_rotation_drift_rad
        )
        response.object_linear_speed_mps = snapshot.object_linear_speed_mps
        response.release_stable_time_sec = snapshot.release_stable_time_sec
        if cube_pose is not None:
            response.object_pose = cube_pose
        else:
            response.object_pose.orientation.w = 1.0
        return response

    def _publish_diagnostic(self) -> None:
        with self._lock:
            snapshot = self._classifier.snapshot()
        code = (
            StatusCode.READY
            if snapshot.initialized
            and bool(self.get_parameter("physics_model_validated").value)
            else StatusCode.PHYSICS_STATE_UNAVAILABLE
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "panda_gz/physical_grasp"
        status.hardware_id = "gazebo_fortress"
        status.level = (
            DiagnosticStatus.WARN
            if code == StatusCode.PHYSICS_STATE_UNAVAILABLE or snapshot.slipping
            else DiagnosticStatus.OK
        )
        status.message = service_message(code, "physics-only observation")
        values = (
            ("episode_id", snapshot.episode_id),
            ("grasped", snapshot.grasped),
            ("holding_object", snapshot.holding_object),
            ("released", snapshot.released),
            ("left_contact", snapshot.left_finger_contact),
            ("right_contact", snapshot.right_finger_contact),
            ("slipping", snapshot.slipping),
            ("translation_drift_m", snapshot.max_relative_translation_drift_m),
            ("rotation_drift_rad", snapshot.max_relative_rotation_drift_rad),
            ("object_speed_mps", snapshot.object_linear_speed_mps),
            ("release_stable_time_sec", snapshot.release_stable_time_sec),
        )
        status.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values
        ]
        array.status = [status]
        self._status_pub.publish(array)
        self._holding_pub.publish(Bool(data=snapshot.holding_object))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _pose_sample(pose: Pose) -> PoseSample:
        return PoseSample.create(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalGraspMonitor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
