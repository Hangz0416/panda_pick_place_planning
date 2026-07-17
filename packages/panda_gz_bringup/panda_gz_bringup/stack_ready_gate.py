"""Exit successfully only when the MoveIt--Gazebo stack is actually ready."""

from __future__ import annotations

import threading
import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from panda_pick_place_interfaces.action import PlanExecuteMotion
from panda_pick_place_interfaces.srv import (
    GetExecutionCollisionState,
    GetPhysicalGraspState,
    RemoveWorldObject,
    ResetEpisode,
    SetAttachedObject,
    UpsertWorldObject,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


REQUIRED_CONTROLLERS = frozenset(
    {
        "joint_state_broadcaster",
        "panda_arm_controller",
        "panda_hand_controller",
    }
)

# ``qos_profile_clock`` is not exported by ROS 2 Humble's rclpy.  Match the
# volatile, best-effort clock publisher used by ros_gz_bridge explicitly so the
# readiness gate remains compatible with the distribution fixed by this repo.
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class StackReadyGate(Node):
    """Observe concrete readiness signals instead of sleeping for N seconds."""

    def __init__(self) -> None:
        super().__init__("panda_stack_ready_gate")
        self.declare_parameter("grasp_mode", "assisted")
        self.declare_parameter("world_name", "panda_pick_place")
        self.declare_parameter("status_period_sec", 2.0)
        self.declare_parameter("readiness_timeout_sec", 120.0)
        self.declare_parameter("controller_query_timeout_sec", 2.0)

        self.ready_event = threading.Event()
        self.terminal_event = threading.Event()
        self.timed_out = False
        self._started_at = time.monotonic()
        self._clock_seen = False
        self._joint_state_seen = False
        self._cube_odometry_seen = False
        self._controller_states: dict[str, str] = {}
        self._controller_future = None
        self._controller_requested_at: float | None = None
        self._last_status_time = 0.0

        self.create_subscription(
            Clock, "/clock", self._on_clock, CLOCK_QOS
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/panda_gz/cube_odometry",
            self._on_cube_odometry,
            qos_profile_sensor_data,
        )

        world = str(self.get_parameter("world_name").value)
        clients = {
            "world_control": self.create_client(
                ControlWorld, f"/world/{world}/control"
            ),
            "set_entity_pose": self.create_client(
                SetEntityPose, f"/world/{world}/set_pose"
            ),
            "reset_episode": self.create_client(
                ResetEpisode, "/panda_gz/reset_episode"
            ),
            "attach_object": self.create_client(
                Trigger, "/panda_gz/attach_object"
            ),
            "detach_object": self.create_client(
                Trigger, "/panda_gz/detach_object"
            ),
            "execution_collision_state": self.create_client(
                GetExecutionCollisionState,
                "/panda_gz/get_execution_collision_state",
            ),
            "upsert_world_object": self.create_client(
                UpsertWorldObject, "/upsert_world_object"
            ),
            "remove_world_object": self.create_client(
                RemoveWorldObject, "/remove_world_object"
            ),
            "set_attached_object": self.create_client(
                SetAttachedObject, "/set_attached_object"
            ),
        }
        if str(self.get_parameter("grasp_mode").value) == "physics":
            clients["physical_grasp_state"] = self.create_client(
                GetPhysicalGraspState, "/panda_gz/get_physical_grasp_state"
            )
        self._service_clients = clients
        self._controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._motion_action = ActionClient(
            self, PlanExecuteMotion, "/plan_execute_motion"
        )
        self._arm_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )
        self._hand_action = ActionClient(
            self,
            GripperCommand,
            "/panda_hand_controller/gripper_cmd",
        )
        self._timer = self.create_timer(0.20, self._poll)

    def _on_clock(self, _message: Clock) -> None:
        self._clock_seen = True

    def _on_joint_state(self, message: JointState) -> None:
        if message.name:
            self._joint_state_seen = True

    def _on_cube_odometry(self, _message: Odometry) -> None:
        self._cube_odometry_seen = True

    def _on_controllers(self, future) -> None:
        try:
            if future.cancelled():
                return
            response = future.result()
            self._controller_states = {
                controller.name: controller.state
                for controller in response.controller
            }
        except Exception as exc:
            self.get_logger().warning(f"list_controllers failed: {exc}")
        finally:
            if self._controller_future is future:
                self._controller_future = None
                self._controller_requested_at = None

    def _missing_requirements(self) -> list[str]:
        missing = []
        if not self._clock_seen:
            missing.append("clock")
        if not self._joint_state_seen:
            missing.append("joint_states")
        if not self._cube_odometry_seen:
            missing.append("cube_odometry")
        inactive = sorted(
            controller
            for controller in REQUIRED_CONTROLLERS
            if self._controller_states.get(controller) != "active"
        )
        missing.extend(f"controller:{name}" for name in inactive)
        missing.extend(
            f"service:{name}"
            for name, client in self._service_clients.items()
            if not client.service_is_ready()
        )
        if not self._motion_action.server_is_ready():
            missing.append("action:plan_execute_motion")
        if not self._arm_action.server_is_ready():
            missing.append("action:arm_trajectory")
        if not self._hand_action.server_is_ready():
            missing.append("action:gripper")
        return missing

    def _poll(self) -> None:
        timeout = float(self.get_parameter("readiness_timeout_sec").value)
        query_timeout = float(
            self.get_parameter("controller_query_timeout_sec").value
        )
        if timeout <= 0.0 or query_timeout <= 0.0:
            self.get_logger().error(
                "readiness_timeout_sec and controller_query_timeout_sec "
                "must be positive"
            )
            self.timed_out = True
            self._timer.cancel()
            self.terminal_event.set()
            return
        now = time.monotonic()
        elapsed = now - self._started_at
        if elapsed >= timeout:
            missing = self._missing_requirements()
            self.get_logger().error(
                f"stack readiness timed out after {timeout:.1f}s; missing: "
                + ", ".join(missing)
            )
            self.timed_out = True
            self._timer.cancel()
            self.terminal_event.set()
            return
        if (
            self._controller_future is not None
            and self._controller_requested_at is not None
            and not self._controller_future.done()
            and now - self._controller_requested_at >= query_timeout
        ):
            stale_future = self._controller_future
            self._controller_future = None
            self._controller_requested_at = None
            stale_future.cancel()
            self.get_logger().warning(
                "list_controllers query timed out; retrying readiness probe"
            )

        if (
            self._controller_future is None
            and self._controller_client.service_is_ready()
        ):
            self._controller_future = self._controller_client.call_async(
                ListControllers.Request()
            )
            self._controller_requested_at = now
            self._controller_future.add_done_callback(self._on_controllers)

        missing = self._missing_requirements()
        if not missing:
            self.get_logger().info(
                "Gazebo, controllers, scene bridge, and motion actions are ready"
            )
            self._timer.cancel()
            self.ready_event.set()
            self.terminal_event.set()
            return
        period = float(self.get_parameter("status_period_sec").value)
        if now - self._last_status_time >= period:
            self.get_logger().info("waiting for: " + ", ".join(missing))
            self._last_status_time = now


def main(args=None) -> None:
    """Spin until readiness is proved, then exit with success."""
    rclpy.init(args=args)
    node = StackReadyGate()
    ready = False
    try:
        while rclpy.ok() and not node.terminal_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.20)
        ready = node.ready_event.is_set()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.timed_out:
        raise TimeoutError("Panda stack readiness gate timed out")
    if not ready:
        raise RuntimeError("ROS context stopped before stack readiness was proved")
