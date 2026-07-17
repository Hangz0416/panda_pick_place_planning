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


class AssistedGraspManager(Node):
    """Expose guarded attach/detach services without hiding physics mode."""

    def __init__(self) -> None:
        super().__init__("assisted_grasp_manager")
        self._group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._cube_pose = None
        self._joint_positions: dict[str, float] = {}
        self._attached: bool | None = None
        self._state_generation = 0
        self._bootstrap_detach_in_progress = False

        self.declare_parameter("grasp_mode", "assisted")
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
        self._initial_detach_pending = mode == "assisted"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._attach_pub = self.create_publisher(
            Empty, "/panda/detachable_joint/attach", 10
        )
        self._detach_pub = self.create_publisher(
            Empty, "/panda/detachable_joint/detach", 10
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, "/panda_gz/grasp_status", 10
        )
        self.create_subscription(
            String,
            "/panda/detachable_joint/state",
            self._on_attachment_state,
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

    def _on_attachment_state(self, message: String) -> None:
        # Fortress publishes StringMsg("attached"|"detached"), not Boolean.
        state = message.data.strip().lower()
        if state not in {"attached", "detached"}:
            self.get_logger().error(
                f"unknown DetachableJoint state payload: {message.data!r}"
            )
            return
        request_initial_detach = False
        with self._condition:
            self._attached = state == "attached"
            self._state_generation += 1
            if self._initial_detach_pending:
                self._initial_detach_pending = False
                request_initial_detach = self._attached
                self._bootstrap_detach_in_progress = request_initial_detach
            if not self._attached:
                self._bootstrap_detach_in_progress = False
            self._condition.notify_all()
        if request_initial_detach:
            # DetachableJoint starts attached by design. Remove that bootstrap
            # joint before any episode even when auto_reset was disabled.
            self._detach_pub.publish(Empty())
            self._publish_status(
                StatusCode.DETACH_REQUESTED,
                "removing DetachableJoint's initial bootstrap attachment",
            )
            return
        code = (
            StatusCode.ATTACH_CONFIRMED
            if self._attached
            else StatusCode.DETACH_CONFIRMED
        )
        self._publish_status(code, "Gazebo acknowledged detachable-joint state")

    def _on_cube_pose(self, message: PoseArray) -> None:
        if message.poses:
            self._cube_pose = message.poses[0]

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_positions.update(zip(message.name, message.position))

    def _current_gate(self) -> GateResult | None:
        if self._cube_pose is None:
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
        object_position = self._cube_pose.position
        gripper_width = sum(
            abs(float(self._joint_positions[name])) for name in finger_names
        )
        return evaluate_gate(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            (object_position.x, object_position.y, object_position.z),
            gripper_width,
            self._limits(),
        )

    def _attach(self, _request: Trigger.Request, response: Trigger.Response):
        if str(self.get_parameter("grasp_mode").value) != "assisted":
            code = StatusCode.ATTACH_REJECTED_MODE
            response.success = False
            response.message = service_message(code, "fixed-joint attach disabled in physics mode")
            self._publish_status(code, response.message)
            return response

        with self._condition:
            if self._bootstrap_detach_in_progress:
                code = StatusCode.ATTACH_REJECTED_STATE
                response.success = False
                response.message = service_message(
                    code, "initial DetachableJoint detach is still in progress"
                )
                self._publish_status(code, response.message)
                return response

        gate = self._current_gate()
        if gate is None:
            code = StatusCode.ATTACH_REJECTED_STATE
            response.success = False
            response.message = service_message(
                code, "cube pose, finger joints, or world-to-hand TF is unavailable"
            )
            self._publish_status(code, response.message)
            return response
        if not gate.accepted:
            code = StatusCode.ATTACH_REJECTED_GATE
            response.success = False
            response.message = service_message(code, gate.reason)
            self._publish_status(code, response.message, self._gate_values(gate))
            return response

        with self._condition:
            self._initial_detach_pending = False
            if self._attached is True:
                confirmed = True
            else:
                generation = self._state_generation
                self._attach_pub.publish(Empty())
                confirmed = self._condition.wait_for(
                    lambda: (
                        self._state_generation > generation
                        and self._attached is True
                    ),
                    timeout=float(
                        self.get_parameter("command_timeout_sec").value
                    ),
                )
        code = StatusCode.ATTACH_CONFIRMED if confirmed else StatusCode.TIMEOUT
        response.success = confirmed
        response.message = service_message(
            code, "object attached" if confirmed else "no attached-state acknowledgement"
        )
        self._publish_status(code, response.message, self._gate_values(gate))
        return response

    def _detach(self, _request: Trigger.Request, response: Trigger.Response):
        if str(self.get_parameter("grasp_mode").value) == "physics":
            response.success = True
            response.message = service_message(
                StatusCode.READY, "physics mode has no assisted fixed joint"
            )
            return response

        with self._condition:
            self._initial_detach_pending = False
            if self._attached is False:
                confirmed = True
            else:
                generation = self._state_generation
                self._detach_pub.publish(Empty())
                confirmed = self._condition.wait_for(
                    lambda: (
                        self._state_generation > generation
                        and self._attached is False
                    ),
                    timeout=float(
                        self.get_parameter("command_timeout_sec").value
                    ),
                )
        code = StatusCode.DETACH_CONFIRMED if confirmed else StatusCode.TIMEOUT
        response.success = confirmed
        response.message = service_message(
            code, "object detached" if confirmed else "no detached-state acknowledgement"
        )
        self._publish_status(code, response.message)
        return response

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
            ("attached", self._attached),
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
