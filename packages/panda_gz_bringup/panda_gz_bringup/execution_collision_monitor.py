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
        self._object_positions = {}
        self._assisted_attached = {}
        self._selected_object_id = ""
        self._physics_holding = False
        self._stream_last_seen: dict[str, float | None] = {"robot": None}
        self._seen_invalid_pairs: set[tuple[str, str, str]] = set()
        self._collision_count = 0
        self._last_failure_type = ""
        self._last_collision1 = ""
        self._last_collision2 = ""

        self.declare_parameter("grasp_mode", "assisted")
        self.declare_parameter("workpiece_count", 1)
        self.declare_parameter("contact_stream_timeout_sec", 1.0)
        self.declare_parameter("target_xy_tolerance_m", 0.04)
        self.declare_parameter("target_z_tolerance_m", 0.015)
        mode = str(self.get_parameter("grasp_mode").value)
        if mode not in {"assisted", "physics"}:
            raise ValueError("grasp_mode must be 'assisted' or 'physics'")
        count = int(self.get_parameter("workpiece_count").value)
        if count not in {1, 3}:
            raise ValueError("workpiece_count must be one or three")
        self._object_ids = (
            ("cube",) if count == 1
            else ("workpiece_0", "workpiece_1", "workpiece_2")
        )
        self._assisted_attached = {value: False for value in self._object_ids}
        self._stream_last_seen.update({value: None for value in self._object_ids})

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
        suffixes = {"cube": "", "workpiece_0": "", "workpiece_1": "_1", "workpiece_2": "_2"}
        pose_topics = {
            "cube": "/panda_gz/cube_pose",
            "workpiece_0": "/panda_gz/cube_pose",
            "workpiece_1": "/panda_gz/cube_1_pose",
            "workpiece_2": "/panda_gz/cube_2_pose",
        }
        contact_topics = {
            "cube": "/panda_gz/cube_contacts",
            "workpiece_0": "/panda_gz/cube_contacts",
            "workpiece_1": "/panda_gz/cube_1_contacts",
            "workpiece_2": "/panda_gz/cube_2_contacts",
        }
        for object_id in self._object_ids:
            self.create_subscription(
                Contacts, contact_topics[object_id],
                lambda message, value=object_id: self._on_contacts(value, message),
                10, callback_group=self._group,
            )
            self.create_subscription(
                PoseArray, pose_topics[object_id],
                lambda message, value=object_id: self._on_object_pose(value, message),
                10, callback_group=self._group,
            )
            self.create_subscription(
                String, f"/panda/detachable_joint{suffixes[object_id]}/state",
                lambda message, value=object_id: self._on_assisted_attachment(value, message),
                10, callback_group=self._group,
            )
        self.create_subscription(
            String, "/panda_gz/selected_object_id", self._on_selected_object,
            10, callback_group=self._group,
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
        self._on_object_pose(self._object_ids[0], message)

    def _on_object_pose(self, object_id: str, message: PoseArray) -> None:
        if not message.poses:
            return
        position = message.poses[0].position
        with self._lock:
            value = (position.x, position.y, position.z)
            self._object_positions[object_id] = value
            if object_id == self._selected_object_id:
                self._object_position = value

    def _on_assisted_attachment(self, object_id: str, message: String) -> None:
        state = message.data.strip().lower()
        if state not in {"attached", "detached"}:
            self.get_logger().error(
                f"unknown DetachableJoint state payload: {message.data!r}"
            )
            return
        with self._lock:
            self._assisted_attached[object_id] = state == "attached"

    def _on_selected_object(self, message: String) -> None:
        object_id = message.data.strip()
        with self._lock:
            if object_id not in self._object_ids:
                self._selected_object_id = ""
                self._last_failure_type = "instance_association_failed"
                return
            self._selected_object_id = object_id
            self._object_position = self._object_positions.get(object_id)
            if self._object_position is not None:
                self._object_initial_position = self._object_position

    def _on_physics_holding(self, message: Bool) -> None:
        with self._lock:
            self._physics_holding = bool(message.data)

    def _object_carried(self) -> bool:
        mode = str(self.get_parameter("grasp_mode").value)
        return (
            bool(self._assisted_attached.get(self._selected_object_id, False))
            if mode == "assisted" else self._physics_holding
        )

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
            selected_object_entity={
                "workpiece_0": "cube", "workpiece_1": "cube_1", "workpiece_2": "cube_2",
            }.get(self._selected_object_id, self._selected_object_id),
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
        return (
            streams_ready
            and bool(self._selected_object_id)
            and self._object_position is not None
        )

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
