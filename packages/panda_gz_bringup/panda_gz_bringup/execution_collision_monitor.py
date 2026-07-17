"""Typed execution-contact monitor for Gazebo Fortress."""

from __future__ import annotations

import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray
from panda_pick_place_interfaces.srv import GetExecutionCollisionState
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, String

from panda_gz_bringup.execution_collision_classifier import (
    CollisionContext,
    canonical_entity,
    classify_contact,
    point3,
)
from panda_gz_bringup.status import StatusCode, service_message


class ExecutionCollisionMonitor(Node):
    """Latch invalid robot or carried-object contacts for one episode."""

    def __init__(self) -> None:
        super().__init__("execution_collision_monitor")
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._episode_id = 0
        self._object_initial_position = (0.57, -0.17, 0.025)
        self._target_position = (0.57, 0.17, 0.025)
        self._object_position: tuple[float, float, float] | None = None
        self._assisted_attached = False
        self._physics_holding = False
        self._stream_last_seen: dict[str, float | None] = {
            "robot": None,
            "cube": None,
        }
        self._seen_invalid_pairs: set[tuple[str, str, str]] = set()
        self._collision_count = 0
        self._last_failure_type = ""
        self._last_collision1 = ""
        self._last_collision2 = ""

        self.declare_parameter("grasp_mode", "assisted")
        self.declare_parameter("contact_stream_timeout_sec", 1.0)
        self.declare_parameter("target_xy_tolerance_m", 0.04)
        self.declare_parameter("target_z_tolerance_m", 0.015)
        mode = str(self.get_parameter("grasp_mode").value)
        if mode not in {"assisted", "physics"}:
            raise ValueError("grasp_mode must be 'assisted' or 'physics'")

        self._status_pub = self.create_publisher(
            DiagnosticArray, "/panda_gz/execution_collision_status", 10
        )
        self.create_subscription(
            Contacts,
            "/panda_gz/robot_contacts",
            lambda message: self._on_contacts("robot", message),
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            Contacts,
            "/panda_gz/cube_contacts",
            lambda message: self._on_contacts("cube", message),
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
            String,
            "/panda/detachable_joint/state",
            self._on_assisted_attachment,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            Bool,
            "/panda_gz/physical_holding_object",
            self._on_physics_holding,
            10,
            callback_group=self._group,
        )
        self.create_service(
            GetExecutionCollisionState,
            "/panda_gz/get_execution_collision_state",
            self._get_state,
            callback_group=self._group,
        )
        self.create_timer(
            0.2,
            self._publish_diagnostic,
            callback_group=self._group,
        )

    def _on_cube_pose(self, message: PoseArray) -> None:
        if not message.poses:
            return
        position = message.poses[0].position
        with self._lock:
            self._object_position = (position.x, position.y, position.z)

    def _on_assisted_attachment(self, message: String) -> None:
        state = message.data.strip().lower()
        if state not in {"attached", "detached"}:
            self.get_logger().error(
                f"unknown DetachableJoint state payload: {message.data!r}"
            )
            return
        with self._lock:
            self._assisted_attached = state == "attached"

    def _on_physics_holding(self, message: Bool) -> None:
        with self._lock:
            self._physics_holding = bool(message.data)

    def _object_carried(self) -> bool:
        mode = str(self.get_parameter("grasp_mode").value)
        return self._assisted_attached if mode == "assisted" else self._physics_holding

    def _collision_context(self) -> CollisionContext:
        return CollisionContext(
            object_carried=self._object_carried(),
            object_position=self._object_position,
            object_initial_position=self._object_initial_position,
            target_position=self._target_position,
            target_xy_tolerance_m=float(
                self.get_parameter("target_xy_tolerance_m").value
            ),
            target_z_tolerance_m=float(
                self.get_parameter("target_z_tolerance_m").value
            ),
        )

    def _on_contacts(self, stream: str, message: Contacts) -> None:
        now = self._now_sec()
        with self._lock:
            context = self._collision_context()
            self._stream_last_seen[stream] = now

            for contact in message.contacts:
                first = str(contact.collision1.name)
                second = str(contact.collision2.name)
                classification = classify_contact(first, second, context)
                if classification.allowed:
                    continue
                semantic_pair = tuple(
                    sorted(
                        (
                            canonical_entity(first),
                            canonical_entity(second),
                        )
                    )
                )
                key = (
                    classification.failure_type,
                    semantic_pair[0],
                    semantic_pair[1],
                )
                if key not in self._seen_invalid_pairs:
                    self._seen_invalid_pairs.add(key)
                    self._collision_count += 1
                self._last_failure_type = classification.failure_type
                self._last_collision1 = first
                self._last_collision2 = second

    def _get_state(
        self,
        request: GetExecutionCollisionState.Request,
        response: GetExecutionCollisionState.Response,
    ) -> GetExecutionCollisionState.Response:
        reset_command = GetExecutionCollisionState.Request.COMMAND_RESET
        query_command = GetExecutionCollisionState.Request.COMMAND_QUERY
        with self._lock:
            if request.command == reset_command:
                try:
                    initial = point3(
                        (
                            request.object_initial_position.x,
                            request.object_initial_position.y,
                            request.object_initial_position.z,
                        )
                    )
                    target = point3(
                        (
                            request.target_position.x,
                            request.target_position.y,
                            request.target_position.z,
                        )
                    )
                except ValueError as exc:
                    return self._invalid_response(response, str(exc))
                self._episode_id = int(request.episode_id)
                self._object_initial_position = initial
                self._target_position = target
                self._seen_invalid_pairs.clear()
                self._collision_count = 0
                self._last_failure_type = ""
                self._last_collision1 = ""
                self._last_collision2 = ""
            elif request.command != query_command:
                return self._invalid_response(response, "unknown monitor command")
            return self._fill_response(response, reset=request.command == reset_command)

    def _fill_response(
        self,
        response: GetExecutionCollisionState.Response,
        *,
        reset: bool,
    ) -> GetExecutionCollisionState.Response:
        available = self._available()
        if reset or available:
            code = StatusCode.READY
            response.success = True
            response.failure_type = ""
            detail = "execution collision state reset" if reset else "state ready"
        else:
            code = StatusCode.COLLISION_STATE_UNAVAILABLE
            response.success = False
            response.failure_type = code.name.lower()
            detail = "waiting for robot contacts, cube contacts, and cube pose"
        response.status_code = int(code)
        response.message = service_message(code, detail)
        response.episode_id = self._episode_id
        response.available = available
        response.invalid_collision = self._collision_count > 0
        response.collision_count = self._collision_count
        response.last_failure_type = self._last_failure_type
        response.last_collision1 = self._last_collision1
        response.last_collision2 = self._last_collision2
        return response

    @staticmethod
    def _invalid_response(response, detail):
        code = StatusCode.INVALID_REQUEST
        response.success = False
        response.status_code = int(code)
        response.failure_type = code.name.lower()
        response.message = service_message(code, detail)
        return response

    def _available(self) -> bool:
        now = self._now_sec()
        timeout = float(self.get_parameter("contact_stream_timeout_sec").value)
        streams_ready = all(
            timestamp is not None and now - timestamp <= timeout
            for timestamp in self._stream_last_seen.values()
        )
        return streams_ready and self._object_position is not None

    def _publish_diagnostic(self) -> None:
        with self._lock:
            available = self._available()
            invalid = self._collision_count > 0
            values = (
                ("episode_id", self._episode_id),
                ("available", available),
                ("object_carried", self._object_carried()),
                ("invalid_collision", invalid),
                ("collision_count", self._collision_count),
                ("last_failure_type", self._last_failure_type),
                ("last_collision1", self._last_collision1),
                ("last_collision2", self._last_collision2),
            )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "panda_gz/execution_collision"
        status.hardware_id = "gazebo_fortress"
        status.level = (
            DiagnosticStatus.ERROR
            if invalid
            else DiagnosticStatus.OK
            if available
            else DiagnosticStatus.WARN
        )
        status.message = (
            self._last_failure_type
            if invalid
            else "collision streams ready"
            if available
            else "collision state unavailable"
        )
        status.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values
        ]
        array.status = [status]
        self._status_pub.publish(array)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutionCollisionMonitor()
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
