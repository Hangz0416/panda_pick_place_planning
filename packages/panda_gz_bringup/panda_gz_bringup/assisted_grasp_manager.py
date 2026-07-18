"""Gate and command Gazebo Sim's DetachableJoint for assisted grasps."""

from __future__ import annotations

import threading
from typing import Iterable

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from panda_gz_bringup.grasp_gate import GateLimits, GateResult, evaluate_gate
from panda_gz_bringup.status import StatusCode, service_message
from panda_pick_place_interfaces.srv import SetAssistedObject


class AssistedGraspManager(Node):
    """Expose guarded attach/detach services without hiding physics mode."""

    def __init__(self) -> None:
        super().__init__("assisted_grasp_manager")
        self._group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._object_poses = {}
        self._joint_positions: dict[str, float] = {}
        self._attached_states: dict[str, bool | None] = {}
        self._state_generations: dict[str, int] = {}
        self._bootstrap_detach_in_progress: set[str] = set()

        self.declare_parameter("grasp_mode", "assisted")
        self.declare_parameter("workpiece_count", 1)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("hand_frame", "panda_hand")
        self.declare_parameter("finger_joints", ["panda_finger_joint1", "panda_finger_joint2"])
        self.declare_parameter("expected_hand_offset_xyz", [0.0, 0.0, 0.075])
        self.declare_parameter(
            "expected_hand_quaternion_xyzw", [0.9238795, -0.3826834, 0.0, 0.0]
        )
        self.declare_parameter("max_position_error_m", 0.03)
        self.declare_parameter("max_orientation_error_rad", 0.30)
        self.declare_parameter("max_hand_object_distance_m", 0.09)
        # This is the single finger-joint command used by MoveItBackend. The
        # observed total opening is the sum of both finger joint positions.
        self.declare_parameter("assisted_pre_attach_width_m", 0.038)
        self.declare_parameter("per_finger_width_tolerance_m", 0.004)
        self.declare_parameter("command_timeout_sec", 2.0)

        mode = str(self.get_parameter("grasp_mode").value)
        if mode not in {"assisted", "physics"}:
            raise ValueError("grasp_mode must be 'assisted' or 'physics'")
        count = int(self.get_parameter("workpiece_count").value)
        if count not in {1, 3}:
            raise ValueError("workpiece_count must be one or three")
        self._object_ids = (
            ("cube",)
            if count == 1
            else ("workpiece_0", "workpiece_1", "workpiece_2")
        )
        self._initial_detach_pending = set(self._object_ids) if mode == "assisted" else set()
        self._attached_states = {object_id: None for object_id in self._object_ids}
        self._state_generations = {object_id: 0 for object_id in self._object_ids}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        suffixes = {"cube": "", "workpiece_0": "", "workpiece_1": "_1", "workpiece_2": "_2"}
        self._attach_pubs = {
            object_id: self.create_publisher(
                Empty, f"/panda/detachable_joint{suffixes[object_id]}/attach", 10
            )
            for object_id in self._object_ids
        }
        self._detach_pubs = {
            object_id: self.create_publisher(
                Empty, f"/panda/detachable_joint{suffixes[object_id]}/detach", 10
            )
            for object_id in self._object_ids
        }
        self._status_pub = self.create_publisher(
            DiagnosticArray, "/panda_gz/grasp_status", 10
        )
        pose_topics = {
            "cube": "/panda_gz/cube_pose", "workpiece_0": "/panda_gz/cube_pose",
            "workpiece_1": "/panda_gz/cube_1_pose", "workpiece_2": "/panda_gz/cube_2_pose",
        }
        for object_id in self._object_ids:
            suffix = suffixes[object_id]
            self.create_subscription(
                String,
                f"/panda/detachable_joint{suffix}/state",
                lambda message, value=object_id: self._on_attachment_state(value, message),
                10,
                callback_group=self._group,
            )
            self.create_subscription(
                PoseArray,
                pose_topics[object_id],
                lambda message, value=object_id: self._on_object_pose(value, message),
                10,
                callback_group=self._group,
            )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
            callback_group=self._group,
        )
        self.create_service(
            Trigger,
            "/panda_gz/attach_object",
            self._attach,
            callback_group=self._group,
        )
        self.create_service(
            SetAssistedObject,
            "/panda_gz/set_assisted_object",
            self._set_assisted_object,
            callback_group=self._group,
        )
        self.create_service(
            Trigger,
            "/panda_gz/detach_object",
            self._detach,
            callback_group=self._group,
        )
        self._publish_status(StatusCode.READY, "grasp manager ready")

    def _limits(self) -> GateLimits:
        def values(name: str) -> tuple[float, ...]:
            return tuple(float(v) for v in self.get_parameter(name).value)

        finger_count = len(tuple(self.get_parameter("finger_joints").value))
        return GateLimits(
            expected_hand_offset_xyz=values("expected_hand_offset_xyz"),
            expected_hand_quaternion_xyzw=values(
                "expected_hand_quaternion_xyzw"
            ),
            max_position_error_m=float(
                self.get_parameter("max_position_error_m").value
            ),
            max_orientation_error_rad=float(
                self.get_parameter("max_orientation_error_rad").value
            ),
            max_hand_object_distance_m=float(
                self.get_parameter("max_hand_object_distance_m").value
            ),
            expected_gripper_width_m=finger_count
            * float(self.get_parameter("assisted_pre_attach_width_m").value),
            gripper_width_tolerance_m=finger_count
            * float(
                self.get_parameter("per_finger_width_tolerance_m").value
            ),
        )

    def _on_attachment_state(self, object_id: str, message: String) -> None:
        # Fortress publishes StringMsg("attached"|"detached"), not Boolean.
        state = message.data.strip().lower()
        if state not in {"attached", "detached"}:
            self.get_logger().error(
                f"unknown DetachableJoint state payload: {message.data!r}"
            )
            return
        request_initial_detach = False
        with self._condition:
            self._attached_states[object_id] = state == "attached"
            self._state_generations[object_id] += 1
            if object_id in self._initial_detach_pending:
                self._initial_detach_pending.discard(object_id)
                request_initial_detach = bool(self._attached_states[object_id])
                if request_initial_detach:
                    self._bootstrap_detach_in_progress.add(object_id)
            if not self._attached_states[object_id]:
                self._bootstrap_detach_in_progress.discard(object_id)
            self._condition.notify_all()
        if request_initial_detach:
            # DetachableJoint starts attached by design. Remove that bootstrap
            # joint before any episode even when auto_reset was disabled.
            self._detach_pubs[object_id].publish(Empty())
            self._publish_status(
                StatusCode.DETACH_REQUESTED,
                f"removing {object_id} bootstrap attachment",
            )
            return
        code = (
            StatusCode.ATTACH_CONFIRMED
            if self._attached_states[object_id]
            else StatusCode.DETACH_CONFIRMED
        )
        self._publish_status(code, "Gazebo acknowledged detachable-joint state")

    def _on_cube_pose(self, message: PoseArray) -> None:
        self._on_object_pose(self._object_ids[0], message)

    def _on_object_pose(self, object_id: str, message: PoseArray) -> None:
        if message.poses:
            self._object_poses[object_id] = message.poses[0]

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_positions.update(zip(message.name, message.position))

    def _current_gate(self, object_id: str) -> GateResult | None:
        object_pose = self._object_poses.get(object_id)
        if object_pose is None:
            return None
        finger_names = tuple(self.get_parameter("finger_joints").value)
        if not all(name in self._joint_positions for name in finger_names):
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("world_frame").value),
                str(self.get_parameter("hand_frame").value),
                Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        object_position = object_pose.position
        object_orientation = object_pose.orientation
        gripper_width = sum(
            abs(float(self._joint_positions[name])) for name in finger_names
        )
        return evaluate_gate(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            (object_position.x, object_position.y, object_position.z),
            gripper_width,
            self._limits(),
            (
                object_orientation.x,
                object_orientation.y,
                object_orientation.z,
                object_orientation.w,
            ),
        )

    def _attach(self, _request: Trigger.Request, response: Trigger.Response):
        response.success, _, response.message = self._command_object(
            self._object_ids[0], True
        )
        return response

    def _detach(self, _request: Trigger.Request, response: Trigger.Response):
        response.success, _, response.message = self._command_object(
            self._object_ids[0], False
        )
        return response

    def _set_assisted_object(self, request, response):
        success, failure_type, message = self._command_object(
            str(request.object_id), bool(request.attach)
        )
        response.success = success
        response.actual_object_id = str(request.object_id) if success else ""
        response.failure_type = failure_type
        response.message = message
        return response

    def _command_object(self, object_id: str, attach: bool):
        if object_id not in self._object_ids:
            return False, "instance_association_failed", "unknown workpiece identity"
        if str(self.get_parameter("grasp_mode").value) != "assisted":
            if not attach:
                return True, "", "physics mode has no assisted fixed joint"
            return False, "assisted_disabled", "fixed-joint attach disabled in physics mode"
        with self._condition:
            if object_id in self._bootstrap_detach_in_progress:
                return False, "selected_object_sync_failed", "bootstrap detach is in progress"
        gate = self._current_gate(object_id) if attach else None
        if attach and gate is None:
            return False, "selected_object_sync_failed", "object pose, joints, or TF unavailable"
        if gate is not None and not gate.accepted:
            detail = service_message(StatusCode.ATTACH_REJECTED_GATE, gate.reason)
            self._publish_status(
                StatusCode.ATTACH_REJECTED_GATE, detail, self._gate_values(gate)
            )
            return False, "assisted_grasp_gate_failed", detail
        with self._condition:
            self._initial_detach_pending.discard(object_id)
            desired = bool(attach)
            if self._attached_states[object_id] is desired:
                confirmed = True
            else:
                generation = self._state_generations[object_id]
                publisher = (
                    self._attach_pubs[object_id]
                    if attach
                    else self._detach_pubs[object_id]
                )
                publisher.publish(Empty())
                confirmed = self._condition.wait_for(
                    lambda: (
                        self._state_generations[object_id] > generation
                        and self._attached_states[object_id] is desired
                    ),
                    timeout=float(self.get_parameter("command_timeout_sec").value),
                )
        code = (
            StatusCode.ATTACH_CONFIRMED if attach and confirmed
            else StatusCode.DETACH_CONFIRMED if confirmed
            else StatusCode.TIMEOUT
        )
        message = service_message(
            code,
            f"{object_id} {'attached' if attach else 'detached'}"
            if confirmed
            else "no detachable-joint acknowledgement",
        )
        self._publish_status(
            code,
            message,
            (() if gate is None else self._gate_values(gate)),
        )
        return confirmed, "" if confirmed else "selected_attachment_mismatch", message

    @staticmethod
    def _gate_values(gate: GateResult) -> tuple[tuple[str, object], ...]:
        return (
            ("gate_reason", gate.reason),
            ("position_error_m", gate.position_error_m),
            ("orientation_error_rad", gate.orientation_error_rad),
            ("hand_object_distance_m", gate.hand_object_distance_m),
            ("gripper_width_m", gate.gripper_width_m),
        )

    def _publish_status(
        self,
        code: StatusCode,
        detail: str,
        extra: Iterable[tuple[str, object]] = (),
    ) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "panda_gz/assisted_grasp"
        status.hardware_id = "gazebo_fortress"
        status.level = (
            DiagnosticStatus.ERROR
            if int(code) >= int(StatusCode.GZ_SERVICE_UNAVAILABLE)
            else DiagnosticStatus.WARN
            if "REJECTED" in code.name or code == StatusCode.TIMEOUT
            else DiagnosticStatus.OK
        )
        status.message = detail
        values = [
            ("status_code", int(code)),
            ("status_name", code.name),
            ("grasp_mode", self.get_parameter("grasp_mode").value),
            ("attached", dict(self._attached_states)),
            *extra,
        ]
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values]
        array.status = [status]
        self._status_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AssistedGraspManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
