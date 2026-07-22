"""ROS 2 / MoveIt implementation of the common manipulation backend.

The state machine deliberately stays synchronous.  Its ROS calls are safe when
the owning node is run in a ``MultiThreadedExecutor`` and the blocking task
callback is isolated from a serialized backend callback group (see
``manipulation_node.py``).  Waiting is done with events instead of trying to
spin a node that is already owned by an executor.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
import time
from typing import Any, Sequence

from panda_pick_place_planning.core.backends.base import (
    BackendMotionReport,
    BackendOperationError,
)
from panda_pick_place_planning.core.pybullet_env import SceneConfig
from panda_pick_place_planning.core.schemas import PlanResult, SceneEstimate, TaskSpec


def _pose_yaw(pose: Any) -> float:
    quaternion = pose.orientation
    x_value = float(quaternion.x)
    y_value = float(quaternion.y)
    z_value = float(quaternion.z)
    w_value = float(quaternion.w)
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


@dataclass(frozen=True)
class MoveItBackendConfig:
    action_name: str = "/plan_execute_motion"
    upsert_service_name: str = "/upsert_world_object"
    remove_service_name: str = "/remove_world_object"
    attached_service_name: str = "/set_attached_object"
    reset_service_name: str = "/panda_gz/reset_episode"
    gazebo_attach_service_name: str = "/panda_gz/attach_object"
    gazebo_detach_service_name: str = "/panda_gz/detach_object"
    set_assisted_object_service_name: str = "/panda_gz/set_assisted_object"
    physical_grasp_state_service_name: str = (
        "/panda_gz/get_physical_grasp_state"
    )
    execution_collision_state_service_name: str = (
        "/panda_gz/get_execution_collision_state"
    )
    gripper_action_name: str = "/panda_hand_controller/gripper_cmd"
    cube_pose_topic: str = "/panda_gz/cube_pose"
    workpiece_pose_topics: tuple[str, ...] = (
        "/panda_gz/cube_pose",
        "/panda_gz/cube_1_pose",
        "/panda_gz/cube_2_pose",
    )
    controller_state_topic: str = "/panda_arm_controller/controller_state"
    world_frame: str = "world"
    # Pose goals use the same grasp frame as the PyBullet baseline.  Object
    # attachment is deliberately kept on the physical hand link below: the
    # grasp-target frame is offset 105 mm from panda_hand and is not the rigid
    # body that owns the gripper collision geometry.
    end_effector_link: str = "panda_grasptarget"
    attach_link: str = "panda_hand"
    touch_links: tuple[str, ...] = (
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger",
    )
    home_target: str = "home"
    planner_id: str = "RRTConnectkConfigDefault"
    planning_time_sec: float = 3.0
    velocity_scaling: float = 0.2
    acceleration_scaling: float = 0.2
    service_timeout_sec: float = 5.0
    execution_timeout_sec: float = 30.0
    settle_timeout_sec: float = 3.0
    physical_grasp_confirm_timeout_sec: float = 2.0
    physical_grasp_poll_period_sec: float = 0.02
    physical_release_confirm_time_sec: float = 0.10
    reset_pose_tolerance_m: float = 0.01
    verify_gazebo_reset_pose: bool = True
    # Deprecated compatibility input.  Availability is always derived from
    # GetExecutionCollisionState; setting this flag cannot make an unobserved
    # Gazebo episode count as collision-free.
    collision_monitor_available: bool = False
    use_gazebo: bool = False
    execute_motion: bool = True
    command_gripper: bool = True
    assisted_pre_attach_width: float = 0.038
    max_gripper_effort: float = 20.0

    def __post_init__(self) -> None:
        if not 0.0 < self.planning_time_sec:
            raise ValueError("planning_time_sec must be positive")
        if not 0.0 <= self.velocity_scaling <= 1.0:
            raise ValueError("velocity_scaling must be in [0, 1]")
        if not 0.0 <= self.acceleration_scaling <= 1.0:
            raise ValueError("acceleration_scaling must be in [0, 1]")
        if self.use_gazebo and not self.execute_motion:
            raise ValueError("Gazebo backend cannot disable trajectory execution")


@dataclass
class MoveItClients:
    """Injectable ROS clients; injection keeps unit tests independent of servers."""

    motion: Any
    upsert: Any
    remove: Any
    attached: Any
    reset: Any = None
    gazebo_attach: Any = None
    gazebo_detach: Any = None
    gripper: Any = None
    physical_grasp: Any = None
    execution_collision: Any = None
    set_assisted_object: Any = None


class MoveItBackend:
    backend_name = "moveit"

    def __init__(
        self,
        node: Any,
        config: MoveItBackendConfig | None = None,
        *,
        scene_config: SceneConfig | None = None,
        clients: MoveItClients | None = None,
        callback_group: Any = None,
    ) -> None:
        self.node = node
        self.config = config or MoveItBackendConfig()
        self.scene_config = scene_config or SceneConfig()
        self.simulator_name = "gazebo" if self.config.use_gazebo else "moveit_mock"
        self.is_planning_only = not self.config.use_gazebo
        self.planner_id = self.config.planner_id
        self.simulation_steps = 0
        self.collision_count = 0
        self.invalid_collision_detected = False
        self.object_state_known = not self.config.use_gazebo
        self.collision_state_known = not self.config.use_gazebo
        self.execution_collision_metrics: dict[str, Any] = {}
        self.execution_collision_failure_type = ""
        self.planning_scene_sync_success = True
        self.last_failure_type = ""
        self.object_released_confirmed = False
        self.physical_grasp_metrics: dict[str, Any] = {}
        self.physical_grasp_failure_type = ""
        self.reset_motion_report: BackendMotionReport | None = None
        self.last_motion_report: BackendMotionReport | None = None
        self._task: TaskSpec | None = None
        self._selected_object_id = "cube"
        self._object_states: dict[
            str,
            tuple[tuple[float, float, float], tuple[float, float, float, float]],
        ] = {}
        self._initial_workpiece_positions: dict[str, tuple[float, float, float]] = {}
        self.max_nonselected_displacement_m = 0.0
        self._object_position = (0.0, 0.0, self.scene_config.cube_half_extent)
        self._object_orientation = (0.0, 0.0, 0.0, 1.0)
        self._object_velocity = (
            self._unknown_velocity()
            if self.config.use_gazebo
            else (0.0, 0.0, 0.0)
        )
        self._object_velocity_known = not self.config.use_gazebo
        self._last_object_sample: tuple[float, tuple[float, float, float]] | None = None
        self._pose_generation = 0
        self._release_pose_generation = 0
        self._last_target_position: tuple[float, float, float] | None = None
        self._attached = False
        self._released = False
        self._ever_attached = False
        self._ever_released = False
        self._physical_hold_confirmed = False
        self._physical_release_requested = False
        self._support_collision_pending = False
        self._gripper_width = self.scene_config.open_gripper_width
        self._pose_condition = threading.Condition()
        self._tracking_lock = threading.Lock()
        self._episode_tracking_max: float | None = None
        self._motion_tracking_max: float | None = None
        self._motion_tracking_active = False
        self._feedback_phase = ""
        self._callback_group = callback_group
        if clients is None:
            self.clients = self._create_ros_clients(callback_group)
            self._selected_object_publisher = self._create_selected_object_publisher()
            self._grasp_marker_publisher = self._create_grasp_marker_publisher()
            self._cube_subscription = self._create_cube_subscriptions(callback_group)
            self._controller_state_subscription = (
                self._create_controller_state_subscription(callback_group)
            )
        else:
            self.clients = clients
            self._selected_object_publisher = None
            self._grasp_marker_publisher = None
            self._cube_subscription = None
            self._controller_state_subscription = None

    @property
    def hover_height(self) -> float:
        return self.scene_config.hover_height

    @property
    def grasp_height(self) -> float:
        return self.scene_config.grasp_height

    @property
    def place_height(self) -> float:
        return self.scene_config.place_height

    @property
    def open_gripper_width(self) -> float:
        return self.scene_config.open_gripper_width

    @property
    def closed_gripper_width(self) -> float:
        return self.scene_config.closed_gripper_width

    @property
    def max_joint_tracking_error_rad(self) -> float | None:
        with self._tracking_lock:
            return self._episode_tracking_max

    def reset(self, task: TaskSpec) -> None:
        previous_workpiece_ids = (
            {item.object_id for item in self._task.workpieces}
            if self._task is not None
            else set()
        )
        if self._task is not None:
            previous_workpiece_ids.add(self._selected_object_id)
        self._task = task
        self._selected_object_id = task.workpieces[0].object_id
        self._object_states = {
            item.object_id: (item.position, item.orientation_xyzw)
            for item in task.workpieces
        }
        self._initial_workpiece_positions = {
            item.object_id: item.position for item in task.workpieces
        }
        self.max_nonselected_displacement_m = 0.0
        selected = task.workpieces[0]
        self._object_position = tuple(selected.position)
        self._object_orientation = tuple(selected.orientation_xyzw)
        self._object_velocity = (
            self._unknown_velocity()
            if self.config.use_gazebo
            else (0.0, 0.0, 0.0)
        )
        self._object_velocity_known = not self.config.use_gazebo
        self._last_object_sample = None
        self._last_target_position = None
        self._attached = False
        self._released = False
        self.simulation_steps = 0
        self.collision_count = 0
        self.invalid_collision_detected = False
        self.collision_state_known = not self.config.use_gazebo
        self.execution_collision_metrics = {}
        self.execution_collision_failure_type = ""
        self.object_state_known = not self.config.use_gazebo
        self.planning_scene_sync_success = True
        self.last_failure_type = ""
        self.object_released_confirmed = False
        self.physical_grasp_metrics = {}
        self.physical_grasp_failure_type = ""
        self.reset_motion_report = None
        self.last_motion_report = None
        self._ever_attached = False
        self._ever_released = False
        self._physical_hold_confirmed = False
        self._physical_release_requested = False
        self._support_collision_pending = False
        with self._tracking_lock:
            self._episode_tracking_max = None
            self._motion_tracking_max = None
            self._motion_tracking_active = False

        if self.config.use_gazebo:
            previous_generation = self._pose_generation
            reset_response = self._require_service_success(
                self.clients.reset,
                self._reset_request(task),
                "gazebo_reset_failed",
            )
            if not self._reset_response_matches(reset_response, task):
                self.planning_scene_sync_success = False
                raise BackendOperationError("gazebo_reset_response_mismatch")
            self._rebase_nonselected_displacement(reset_response, task)
            if self.config.verify_gazebo_reset_pose and not self._wait_for_reset_pose(
                selected.position,
                previous_generation,
            ):
                self.planning_scene_sync_success = False
                raise BackendOperationError("gazebo_reset_pose_mismatch")
            # Reset/teleport invalidates any finite-difference sample even if
            # a pre-reset pose arrived while the service was running.
            self._clear_object_motion_sample()
            self._reset_execution_collision_state(task)
            if task.grasp_mode == "physics":
                response = self._physical_grasp_state(command_reset=True)
                if response is None or not bool(response.success):
                    failure_type = str(
                        getattr(response, "failure_type", "")
                        or "physical_grasp_state_unavailable"
                    )
                    raise BackendOperationError(failure_type)

        # These services perform PlanningScene readback before returning
        # success.  Reset must fail closed if stale attachment/world state
        # cannot be proven absent.
        reset_workpiece_ids = previous_workpiece_ids | {
            item.object_id for item in task.workpieces
        }
        for object_id in sorted(reset_workpiece_ids):
            self._require_service_success(
                self.clients.attached,
                self._attached_request_for(object_id, False),
                "planning_scene_reset_detach_failed",
            )
        for object_id in (item.object_id for item in task.workpieces):
            self._require_world_object_removed(
                object_id,
                "planning_scene_reset_cube_remove_failed",
            )
        self._upsert_box(
            "table",
            self.scene_config.table_position,
            self.scene_config.table_size,
            allowed_collision_links=("panda_link0",),
        )
        cube_size = self._object_size(task.object_variant)
        cube_allowed_collisions = self.config.touch_links
        if self.config.use_gazebo:
            # A resting object is intentionally in contact with its support
            # surface.  Preserve this one ACM pair through attachment, then
            # clear it immediately after the first successful lift.
            cube_allowed_collisions += ("table",)
        for item in task.workpieces:
            self._upsert_box(
                item.object_id,
                item.position,
                cube_size,
                orientation=item.orientation_xyzw,
                allowed_collision_links=cube_allowed_collisions,
            )
        if task.scene == "obstacle":
            obstacle_size = tuple(
                2.0 * value for value in self.scene_config.obstacle_half_extents
            )
            self._upsert_box(
                "obstacle",
                self.scene_config.obstacle_position,
                obstacle_size,
            )
        else:
            self._require_world_object_removed(
                "obstacle",
                "planning_scene_reset_obstacle_remove_failed",
            )

        home = self.move_home()
        self.reset_motion_report = home
        if not home.success:
            raise BackendOperationError(home.failure_type or "moveit_home_failed")

    def move_to_pose(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        planner: str = "linear",
        seed: int = 0,
        timeout_sec: float = 1.0,
    ) -> BackendMotionReport:
        del seed  # OMPL seeding is controlled by MoveIt, not the task generator.
        self._last_target_position = tuple(float(value) for value in position)
        goal = self._new_motion_goal()
        goal.target_type = goal.TARGET_TYPE_POSE
        goal.pose_target.header.frame_id = self.config.world_frame
        goal.pose_target.pose.position.x = float(position[0])
        goal.pose_target.pose.position.y = float(position[1])
        goal.pose_target.pose.position.z = float(position[2])
        goal.pose_target.pose.orientation.x = float(orientation[0])
        goal.pose_target.pose.orientation.y = float(orientation[1])
        goal.pose_target.pose.orientation.z = float(orientation[2])
        goal.pose_target.pose.orientation.w = float(orientation[3])
        goal.end_effector_link = self.config.end_effector_link
        self._configure_motion_goal(goal, planner, timeout_sec)
        report = self._send_motion(goal)
        if (
            report.success
            and self._attached
            and self._support_collision_pending
        ):
            try:
                # RemoveWorldObject is deliberately idempotent: while cube is
                # attached it leaves the AttachedCollisionObject untouched and
                # removes only the temporary support-surface ACM entry.
                self._require_world_object_removed(
                    self._selected_object_id,
                    "planning_scene_support_cleanup_failed",
                )
            except BackendOperationError:
                failed = replace(
                    report,
                    success=False,
                    failure_type="planning_scene_support_cleanup_failed",
                    planning_scene_sync_success=False,
                )
                self.last_failure_type = failed.failure_type
                self.last_motion_report = failed
                return failed
            self._support_collision_pending = False
        return report

    def apply_scene_estimate(self, estimate: SceneEstimate, scene: str) -> bool:
        """Replace reset truth in PlanningScene with the visual estimate."""
        if not estimate.success or estimate.pose_source != "rgbd":
            self.planning_scene_sync_success = False
            return False
        try:
            cube_size = self._object_size(
                self._task.object_variant if self._task is not None else "cube"
            )
            allowed = self.config.touch_links
            if self.config.use_gazebo:
                allowed += ("table",)
            workpiece_ids = (
                [item.object_id for item in self._task.workpieces]
                if self._task is not None
                else ["cube"]
            )
            for object_id in workpiece_ids:
                workpiece = estimate.require(object_id)
                if not workpiece.position_valid:
                    raise ValueError("workpiece_position_invalid")
                self._upsert_box(
                    object_id,
                    workpiece.position,
                    cube_size,
                    orientation=workpiece.orientation_xyzw,
                    allowed_collision_links=allowed,
                )
            if scene == "obstacle":
                obstacle = estimate.require("obstacle")
                obstacle_size = tuple(
                    2.0 * value
                    for value in self.scene_config.obstacle_half_extents
                )
                self._upsert_box("obstacle", obstacle.position, obstacle_size)
            elif scene == "clear":
                self._require_world_object_removed(
                    "obstacle", "scene_estimate_sync_failed"
                )
            else:
                raise ValueError(f"unsupported scene: {scene}")
        except (BackendOperationError, ValueError):
            self.planning_scene_sync_success = False
            self.last_failure_type = "scene_estimate_sync_failed"
            return False
        self.planning_scene_sync_success = True
        return True

    def check_pose_feasibility(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        planner: str = "rrt_connect",
        seed: int = 0,
        timeout_sec: float = 3.0,
    ) -> BackendMotionReport:
        """Plan to a candidate pose without moving the robot."""
        del seed
        goal = self._new_motion_goal()
        goal.target_type = goal.TARGET_TYPE_POSE
        goal.pose_target.header.frame_id = self.config.world_frame
        goal.pose_target.pose.position.x = float(position[0])
        goal.pose_target.pose.position.y = float(position[1])
        goal.pose_target.pose.position.z = float(position[2])
        goal.pose_target.pose.orientation.x = float(orientation[0])
        goal.pose_target.pose.orientation.y = float(orientation[1])
        goal.pose_target.pose.orientation.z = float(orientation[2])
        goal.pose_target.pose.orientation.w = float(orientation[3])
        goal.end_effector_link = self.config.end_effector_link
        self._configure_motion_goal(goal, planner, timeout_sec)
        goal.execute = False
        return self._send_motion(goal)

    def select_manipulation_object(self, object_id: str) -> bool:
        """Lock Gazebo, PlanningScene and result accounting to one workpiece."""
        valid_ids = (
            {item.object_id for item in self._task.workpieces}
            if self._task is not None
            else {"cube"}
        )
        if object_id not in valid_ids:
            self.last_failure_type = "selected_object_sync_failed"
            return False
        self._selected_object_id = object_id
        state = self._object_states.get(object_id)
        if state is not None:
            self._object_position, self._object_orientation = state
            self.object_state_known = True
        elif self._task is not None:
            truth = next(item for item in self._task.workpieces if item.object_id == object_id)
            if self.config.use_gazebo:
                self.last_failure_type = "selected_object_sync_failed"
                return False
            self._object_position = truth.position
            self._object_orientation = truth.orientation_xyzw
        self.last_failure_type = ""
        if self._selected_object_publisher is not None:
            from std_msgs.msg import String

            self._selected_object_publisher.publish(String(data=object_id))
        return True

    def _create_selected_object_publisher(self):
        if not self.config.use_gazebo:
            return None
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        return self.node.create_publisher(String, "/panda_gz/selected_object_id", qos)

    def _create_grasp_marker_publisher(self):
        from visualization_msgs.msg import MarkerArray

        return self.node.create_publisher(
            MarkerArray, "/panda/grasp_candidate_markers", 10
        )

    def publish_grasp_candidates(self, candidates, selection) -> None:
        if self._grasp_marker_publisher is None:
            return
        from visualization_msgs.msg import Marker, MarkerArray

        array = MarkerArray()
        feasible = set(selection.feasible_candidate_keys)
        selected = (
            None
            if selection.candidate is None
            else (selection.candidate.object_id, selection.candidate.index)
        )
        for marker_id, candidate in enumerate(candidates):
            key = (candidate.object_id, candidate.index)
            marker = Marker()
            marker.header.frame_id = self.config.world_frame
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "grasp_candidates"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = candidate.pre_grasp_position[0]
            marker.pose.position.y = candidate.pre_grasp_position[1]
            marker.pose.position.z = candidate.pre_grasp_position[2]
            marker.pose.orientation.x = candidate.orientation_xyzw[0]
            marker.pose.orientation.y = candidate.orientation_xyzw[1]
            marker.pose.orientation.z = candidate.orientation_xyzw[2]
            marker.pose.orientation.w = candidate.orientation_xyzw[3]
            marker.scale.x = 0.10
            marker.scale.y = 0.018
            marker.scale.z = 0.018
            marker.color.a = 0.9
            if key == selected:
                marker.color.g = 1.0
            elif key in feasible:
                marker.color.r = 1.0
                marker.color.g = 0.75
            else:
                marker.color.r = 1.0
            array.markers.append(marker)
        self._grasp_marker_publisher.publish(array)

    def move_home(self) -> BackendMotionReport:
        goal = self._new_motion_goal()
        goal.target_type = goal.TARGET_TYPE_NAMED
        goal.named_target = self.config.home_target
        self._configure_motion_goal(goal, "rrt_connect", self.config.planning_time_sec)
        return self._send_motion(goal)

    def set_gripper(self, width: float) -> None:
        if not 0.0 <= width <= self.open_gripper_width:
            raise ValueError("gripper width is outside the configured range")
        physics_release = bool(
            self.config.use_gazebo
            and self._task is not None
            and self._task.grasp_mode == "physics"
            and self._physical_hold_confirmed
            and self._attached
            and math.isclose(width, self.open_gripper_width, abs_tol=1.0e-9)
        )
        if physics_release:
            failure = self._validate_physical_grasp_state(expect_holding=True)
            if failure:
                raise BackendOperationError(failure)
            self._physical_release_requested = True
        commanded_width = width
        assisted_pre_attach = bool(
            self.config.use_gazebo
            and self._task is not None
            and self._task.grasp_mode == "assisted"
            and math.isclose(width, self.closed_gripper_width, abs_tol=1.0e-9)
            and not self._attached
        )
        if assisted_pre_attach:
            # The hand is yawed 45 degrees over the box, so its in-plane
            # diagonal is about 70.7 mm.  Hold a commanded 76 mm total gap
            # until Gazebo confirms attach; measured joint state remains the
            # authority used by the guarded attach service.  The low-gain
            # position interface can report a stall for this 2 mm adjustment;
            # accept that status only here because the guarded attach service
            # immediately audits the measured width and grasp pose.
            commanded_width = self.config.assisted_pre_attach_width
        self._command_gripper(
            commanded_width,
            allow_stall=assisted_pre_attach,
        )
        self._gripper_width = commanded_width
        if physics_release:
            failure = self._validate_physical_grasp_state(expect_holding=False)
            if failure:
                raise BackendOperationError(failure)

    def attach_object(self) -> bool:
        if self._task is None:
            raise RuntimeError("backend_not_reset")
        self.last_failure_type = ""
        if self._task.grasp_mode == "physics":
            return self._attach_physical_object()

        # Cross-system transaction: Gazebo must first confirm that the exact
        # selected entity is constrained. Only then may MoveIt replace the
        # corresponding world object with an AttachedCollisionObject. If the
        # PlanningScene step fails, detach Gazebo immediately to avoid split
        # physical/planning truth.
        gazebo_attached = False
        if self.config.use_gazebo:
            if self.clients.set_assisted_object is not None:
                gazebo_attached = self._service_success(
                    self.clients.set_assisted_object,
                    self._assisted_object_request(True),
                )
            else:
                gazebo_attached = self._service_success(
                    self.clients.gazebo_attach,
                    self._trigger_request(),
                )
            if not gazebo_attached:
                self.last_failure_type = "assisted_attach_failed"
                return False

        if not self._set_attached(True):
            self.planning_scene_sync_success = False
            self.last_failure_type = "planning_scene_attach_failed"
            if gazebo_attached:
                client = self.clients.set_assisted_object or self.clients.gazebo_detach
                request = (
                    self._assisted_object_request(False)
                    if self.clients.set_assisted_object is not None
                    else self._trigger_request()
                )
                self._service_success(client, request)
            return False

        self._attached = True
        self._ever_attached = True
        self._support_collision_pending = self.config.use_gazebo
        if self.config.use_gazebo:
            # Contact with the constrained cube normally stalls the fingers
            # before the zero-width command.  Accept only an explicitly
            # reported stall here; pre-attach and release commands must reach
            # their requested positions.
            self._command_gripper(
                self.closed_gripper_width,
                allow_stall=True,
            )
            self._gripper_width = self.closed_gripper_width
        return True

    def detach_object(self) -> None:
        is_physics = bool(
            self.config.use_gazebo
            and self._task is not None
            and self._task.grasp_mode == "physics"
        )
        if is_physics:
            if self.physical_grasp_failure_type:
                raise BackendOperationError(self.physical_grasp_failure_type)
            self._physical_release_requested = True
            failure = self._validate_physical_grasp_state(expect_holding=False)
            if failure:
                raise BackendOperationError(failure)
        if self.config.use_gazebo and self._task is not None:
            if self._task.grasp_mode == "assisted":
                client = self.clients.set_assisted_object or self.clients.gazebo_detach
                request = (
                    self._assisted_object_request(False)
                    if self.clients.set_assisted_object is not None
                    else self._trigger_request()
                )
                if not self._service_success(client, request):
                    raise BackendOperationError("gazebo_detach_failed")
        if not self._set_attached(False):
            self.planning_scene_sync_success = False
            raise BackendOperationError("planning_scene_detach_failed")
        self._attached = False
        self._support_collision_pending = False
        self._released = not is_physics
        self.object_released_confirmed = not is_physics
        if not is_physics:
            self._ever_released = True
        self._release_pose_generation = self._pose_generation
        if self.config.use_gazebo:
            self.object_state_known = False
        if is_physics:
            failure = self._validate_physical_grasp_state(expect_holding=False)
            if failure:
                raise BackendOperationError(failure)
        # A MoveIt mock/planning-only run has no simulated object dynamics.
        # Never teleport the object to a commanded end-effector target.

    def grasp_metrics(self) -> dict[str, Any]:
        return dict(self.physical_grasp_metrics)

    def wait_for_object_settle(self) -> None:
        if (
            self.config.use_gazebo
            and self._task is not None
            and self._task.grasp_mode == "physics"
        ):
            self._wait_for_physical_release()
        physics_release_confirmed = bool(
            self._task is not None
            and self._task.grasp_mode == "physics"
            and self.object_released_confirmed
        )
        if self.config.use_gazebo and not physics_release_confirmed:
            deadline = time.monotonic() + self.config.settle_timeout_sec
            stable_since: float | None = None
            observed_generation = self._release_pose_generation
            with self._pose_condition:
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if self._pose_generation > observed_generation:
                        observed_generation = self._pose_generation
                        speed = self._object_speed()
                        if (
                            self._object_velocity_known
                            and speed
                            <= self.scene_config.success_max_linear_velocity_mps
                        ):
                            stable_since = stable_since or now
                        else:
                            stable_since = None
                    if stable_since is not None and now - stable_since >= 0.25:
                        self.object_state_known = True
                        break
                    self._pose_condition.wait(
                        timeout=max(0.0, min(0.05, deadline - now))
                    )
        cube_size = self._object_size(
            self._task.object_variant if self._task is not None else "cube"
        )
        self._upsert_box(
            self._selected_object_id,
            self._object_position,
            cube_size,
            orientation=self._object_orientation,
        )

    def object_state(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        return self._object_position, self._object_velocity

    def object_is_stable_in_target(self, target_position: Sequence[float]) -> bool:
        if self.config.use_gazebo and not self._object_velocity_known:
            return False
        position, velocity = self.object_state()
        xy_error = math.dist(position[:2], target_position[:2])
        z_error = abs(position[2] - target_position[2])
        speed = math.sqrt(sum(value * value for value in velocity))
        return (
            xy_error <= self.scene_config.success_xy_tolerance_m
            and z_error <= self.scene_config.success_z_tolerance_m
            and speed <= self.scene_config.success_max_linear_velocity_mps
        )

    @staticmethod
    def downward_orientation() -> Sequence[float]:
        return (0.9238795325, -0.3826834324, 0.0, 0.0)

    def shutdown(self) -> None:
        return None

    def _create_ros_clients(self, callback_group: Any) -> MoveItClients:
        from control_msgs.action import GripperCommand
        from panda_pick_place_interfaces.action import PlanExecuteMotion
        from panda_pick_place_interfaces.srv import (
            GetExecutionCollisionState,
            GetPhysicalGraspState,
            RemoveWorldObject,
            ResetEpisode,
            SetAssistedObject,
            SetAttachedObject,
            UpsertWorldObject,
        )
        from rclpy.action import ActionClient
        from std_srvs.srv import Trigger

        keyword = {"callback_group": callback_group} if callback_group is not None else {}
        reset = gazebo_attach = gazebo_detach = physical_grasp = None
        set_assisted_object = None
        execution_collision = None
        if self.config.use_gazebo:
            reset = self.node.create_client(
                ResetEpisode, self.config.reset_service_name, **keyword
            )
            gazebo_attach = self.node.create_client(
                Trigger, self.config.gazebo_attach_service_name, **keyword
            )
            gazebo_detach = self.node.create_client(
                Trigger, self.config.gazebo_detach_service_name, **keyword
            )
            set_assisted_object = self.node.create_client(
                SetAssistedObject,
                self.config.set_assisted_object_service_name,
                **keyword,
            )
            physical_grasp = self.node.create_client(
                GetPhysicalGraspState,
                self.config.physical_grasp_state_service_name,
                **keyword,
            )
            execution_collision = self.node.create_client(
                GetExecutionCollisionState,
                self.config.execution_collision_state_service_name,
                **keyword,
            )
        gripper = None
        if self.config.command_gripper:
            gripper = ActionClient(
                self.node,
                GripperCommand,
                self.config.gripper_action_name,
                **keyword,
            )
        return MoveItClients(
            motion=ActionClient(
                self.node,
                PlanExecuteMotion,
                self.config.action_name,
                **keyword,
            ),
            upsert=self.node.create_client(
                UpsertWorldObject, self.config.upsert_service_name, **keyword
            ),
            remove=self.node.create_client(
                RemoveWorldObject, self.config.remove_service_name, **keyword
            ),
            attached=self.node.create_client(
                SetAttachedObject, self.config.attached_service_name, **keyword
            ),
            reset=reset,
            gazebo_attach=gazebo_attach,
            gazebo_detach=gazebo_detach,
            gripper=gripper,
            physical_grasp=physical_grasp,
            execution_collision=execution_collision,
            set_assisted_object=set_assisted_object,
        )

    def _reset_execution_collision_state(self, task: TaskSpec) -> None:
        response = self._execution_collision_state(command_reset=True)
        if response is None:
            raise BackendOperationError("execution_collision_monitor_unavailable")
        if not bool(response.success):
            raise BackendOperationError(
                str(response.failure_type or "execution_collision_monitor_reset_failed")
            )
        if int(response.episode_id) != task.episode_id:
            raise BackendOperationError("execution_collision_monitor_episode_mismatch")
        self._apply_execution_collision_state(response)

    def refresh_execution_state(self) -> bool:
        """Refresh cumulative Gazebo collision truth for the active episode."""

        if not self.config.use_gazebo:
            self.collision_state_known = True
            return True
        response = self._execution_collision_state(command_reset=False)
        if response is None:
            self.collision_state_known = False
            return False
        if self._task is None or int(response.episode_id) != self._task.episode_id:
            self.collision_state_known = False
            self.execution_collision_failure_type = (
                "execution_collision_monitor_episode_mismatch"
            )
            return False
        self._apply_execution_collision_state(response)
        return bool(response.success and response.available)

    def _execution_collision_state(self, *, command_reset: bool) -> Any | None:
        from panda_pick_place_interfaces.srv import GetExecutionCollisionState

        client = self.clients.execution_collision
        if client is None or not client.wait_for_service(
            timeout_sec=self.config.service_timeout_sec
        ):
            return None
        request = GetExecutionCollisionState.Request()
        request.command = (
            request.COMMAND_RESET if command_reset else request.COMMAND_QUERY
        )
        if self._task is not None:
            request.episode_id = self._task.episode_id
            request.object_initial_position.x = self._task.object_position[0]
            request.object_initial_position.y = self._task.object_position[1]
            request.object_initial_position.z = self._task.object_position[2]
            request.target_position.x = self._task.target_position[0]
            request.target_position.y = self._task.target_position[1]
            request.target_position.z = self._task.target_position[2]
        try:
            return self._wait_future(
                client.call_async(request),
                self.config.service_timeout_sec,
            )
        except Exception:
            return None

    def _apply_execution_collision_state(self, response: Any) -> None:
        self.collision_state_known = bool(response.success and response.available)
        self.collision_count = int(response.collision_count)
        self.invalid_collision_detected = bool(response.invalid_collision)
        self.execution_collision_failure_type = str(response.last_failure_type)
        self.execution_collision_metrics = {
            "available": bool(response.available),
            "invalid_collision": bool(response.invalid_collision),
            "collision_count": int(response.collision_count),
            "failure_type": str(response.last_failure_type),
            "collision_pair": [
                str(response.last_collision1),
                str(response.last_collision2),
            ],
        }

    def _attach_physical_object(self) -> bool:
        if not self.config.use_gazebo:
            self.last_failure_type = "physical_grasp_state_unavailable"
            return False
        deadline = (
            time.monotonic()
            + self.config.physical_grasp_confirm_timeout_sec
        )
        self.last_failure_type = "physical_grasp_unconfirmed"
        while time.monotonic() < deadline:
            response = self._physical_grasp_state(command_reset=False)
            if response is None:
                self.last_failure_type = "physical_grasp_state_unavailable"
                time.sleep(self.config.physical_grasp_poll_period_sec)
                continue
            failure = self._physical_response_failure(
                response,
                expect_holding=False,
            )
            if failure:
                self.last_failure_type = failure
                if self.last_failure_type == "physics_state_unavailable":
                    time.sleep(self.config.physical_grasp_poll_period_sec)
                    continue
                return False
            if bool(response.slipping):
                self._latch_physical_grasp_failure("physical_grasp_slip")
                return False
            confirmed = (
                bool(response.grasped)
                and bool(response.holding_object)
                and bool(response.left_finger_contact)
                and bool(response.right_finger_contact)
            )
            if confirmed:
                if not self._set_attached(True):
                    self.planning_scene_sync_success = False
                    self.last_failure_type = "planning_scene_attach_failed"
                    return False
                self._attached = True
                self._ever_attached = True
                self._physical_hold_confirmed = True
                self._physical_release_requested = False
                self.last_failure_type = ""
                return True
            time.sleep(self.config.physical_grasp_poll_period_sec)
        return False

    def _wait_for_physical_release(self) -> None:
        if self.physical_grasp_failure_type:
            self.last_failure_type = self.physical_grasp_failure_type
            return
        deadline = time.monotonic() + self.config.settle_timeout_sec
        self.object_released_confirmed = False
        self.last_failure_type = "physical_release_unconfirmed"
        while time.monotonic() < deadline:
            response = self._physical_grasp_state(command_reset=False)
            if response is None:
                self.last_failure_type = "physical_grasp_state_unavailable"
                time.sleep(self.config.physical_grasp_poll_period_sec)
                continue
            failure = self._physical_response_failure(
                response,
                expect_holding=False,
            )
            if failure:
                self.last_failure_type = failure
                if self.last_failure_type == "physics_state_unavailable":
                    time.sleep(self.config.physical_grasp_poll_period_sec)
                    continue
                return
            stable_release = (
                bool(response.released)
                and not bool(response.holding_object)
                and float(response.release_stable_time_sec)
                >= self.config.physical_release_confirm_time_sec
                and float(response.object_linear_speed_mps)
                <= self.scene_config.success_max_linear_velocity_mps
            )
            if stable_release:
                self._released = True
                self._ever_released = True
                self._physical_hold_confirmed = False
                self.object_released_confirmed = True
                self.last_failure_type = self.physical_grasp_failure_type
                return
            time.sleep(self.config.physical_grasp_poll_period_sec)

    def _validate_physical_grasp_after_motion(self) -> str:
        if (
            not self.config.use_gazebo
            or self._task is None
            or self._task.grasp_mode != "physics"
        ):
            return ""
        expect_holding = bool(
            self._physical_hold_confirmed
            and not self._physical_release_requested
        )
        return self._validate_physical_grasp_state(
            expect_holding=expect_holding
        )

    def _validate_physical_grasp_state(self, *, expect_holding: bool) -> str:
        response = self._physical_grasp_state(command_reset=False)
        if response is None:
            self.last_failure_type = "physical_grasp_state_unavailable"
            return self.last_failure_type
        failure = self._physical_response_failure(
            response,
            expect_holding=expect_holding,
        )
        if failure:
            self.last_failure_type = failure
        return failure

    def _physical_response_failure(
        self,
        response: Any,
        *,
        expect_holding: bool,
    ) -> str:
        self._update_physical_metrics(response)
        if not bool(response.success):
            failure = str(
                response.failure_type or "physical_grasp_state_unavailable"
            )
            if failure in {"physical_grasp_slip", "grasp_lost"}:
                self._latch_physical_grasp_failure(failure)
            return self.physical_grasp_failure_type or failure
        if self._task is None or int(response.episode_id) != self._task.episode_id:
            return "physical_grasp_episode_mismatch"
        if self._physical_hold_confirmed and bool(response.slipping):
            self._latch_physical_grasp_failure("physical_grasp_slip")
        elif expect_holding and (
            bool(response.released) or not bool(response.holding_object)
        ):
            self._latch_physical_grasp_failure("grasp_lost")
        return self.physical_grasp_failure_type

    def _latch_physical_grasp_failure(self, failure_type: str) -> None:
        if not self.physical_grasp_failure_type:
            self.physical_grasp_failure_type = failure_type
        self.last_failure_type = self.physical_grasp_failure_type
        self.physical_grasp_metrics["latched_failure_type"] = (
            self.physical_grasp_failure_type
        )

    def _physical_grasp_state(self, *, command_reset: bool) -> Any | None:
        from panda_pick_place_interfaces.srv import GetPhysicalGraspState

        client = self.clients.physical_grasp
        wait_timeout = min(
            0.1,
            self.config.physical_grasp_confirm_timeout_sec,
        )
        if client is None or not client.wait_for_service(timeout_sec=wait_timeout):
            return None
        request = GetPhysicalGraspState.Request()
        request.command = (
            request.COMMAND_RESET if command_reset else request.COMMAND_QUERY
        )
        request.episode_id = 0 if self._task is None else self._task.episode_id
        try:
            return self._wait_future(
                client.call_async(request),
                min(self.config.service_timeout_sec, 1.0),
            )
        except Exception:
            return None

    def _update_physical_metrics(self, response: Any) -> None:
        self.physical_grasp_metrics = {
            "grasped": bool(response.grasped),
            "holding_object": bool(response.holding_object),
            "released": bool(response.released),
            "left_finger_contact": bool(response.left_finger_contact),
            "right_finger_contact": bool(response.right_finger_contact),
            "slipping": bool(response.slipping),
            "max_relative_translation_drift_m": float(
                response.max_relative_translation_drift_m
            ),
            "max_relative_rotation_drift_rad": float(
                response.max_relative_rotation_drift_rad
            ),
            "object_linear_speed_mps": float(
                response.object_linear_speed_mps
            ),
            "release_stable_time_sec": float(response.release_stable_time_sec),
            "failure_type": str(response.failure_type),
            "latched_failure_type": self.physical_grasp_failure_type,
        }
        pose = response.object_pose
        self._object_position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        # The service reports speed magnitude, which is sufficient for the
        # locked success criterion; direction is intentionally not invented.
        self._object_velocity = (
            float(response.object_linear_speed_mps),
            0.0,
            0.0,
        )
        self._object_velocity_known = bool(response.success)
        self.object_state_known = bool(response.success)

    def _create_cube_subscriptions(self, callback_group: Any):
        if not self.config.use_gazebo:
            return []
        from geometry_msgs.msg import PoseArray

        keyword = {"callback_group": callback_group} if callback_group is not None else {}
        subscriptions = []
        for index, topic in enumerate(self.config.workpiece_pose_topics):
            object_id = "cube" if index == 0 else f"workpiece_{index}"
            subscriptions.append(
                self.node.create_subscription(
                    PoseArray,
                    topic,
                    lambda message, value=object_id: self._on_workpiece_pose(value, message),
                    10,
                    **keyword,
                )
            )
        return subscriptions

    def _create_controller_state_subscription(self, callback_group: Any):
        from control_msgs.msg import JointTrajectoryControllerState

        keyword = {"callback_group": callback_group} if callback_group is not None else {}
        return self.node.create_subscription(
            JointTrajectoryControllerState,
            self.config.controller_state_topic,
            self._on_controller_state,
            10,
            **keyword,
        )

    def _on_controller_state(self, message: Any) -> None:
        sample = self._tracking_error(message.error.positions)
        if sample is None:
            return
        with self._tracking_lock:
            self._episode_tracking_max = max(
                self._episode_tracking_max or 0.0,
                sample,
            )
            if self._motion_tracking_active:
                self._motion_tracking_max = max(
                    self._motion_tracking_max or 0.0,
                    sample,
                )

    @staticmethod
    def _tracking_error(errors: Sequence[float]) -> float | None:
        finite = [abs(float(value)) for value in errors if math.isfinite(value)]
        return max(finite) if finite else None

    def _begin_motion_tracking(self, enabled: bool) -> None:
        with self._tracking_lock:
            self._motion_tracking_max = None
            self._motion_tracking_active = enabled

    def _finish_motion_tracking(
        self, report: BackendMotionReport
    ) -> BackendMotionReport:
        with self._tracking_lock:
            value = self._motion_tracking_max
            self._motion_tracking_active = False
        return replace(report, max_joint_tracking_error_rad=value)

    def _on_cube_pose(self, message: Any) -> None:
        self._on_workpiece_pose(self._selected_object_id, message)

    def _on_workpiece_pose(self, object_id: str, message: Any) -> None:
        if not message.poses:
            return
        pose = message.poses[0]
        position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        orientation = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        self._object_states[object_id] = (position, orientation)
        if object_id == "cube":
            self._object_states["workpiece_0"] = (position, orientation)
        semantic_id = (
            "workpiece_0"
            if object_id == "cube"
            and "workpiece_0" in self._initial_workpiece_positions
            else object_id
        )
        if semantic_id != self._selected_object_id:
            initial = self._initial_workpiece_positions.get(semantic_id)
            if initial is not None:
                self.max_nonselected_displacement_m = max(
                    self.max_nonselected_displacement_m,
                    math.dist(position, initial),
                )
        if object_id != self._selected_object_id and not (
            object_id == "cube" and self._selected_object_id == "workpiece_0"
        ):
            return
        sample_time = self._pose_stamp_sec(message)
        with self._pose_condition:
            velocity_known = False
            sample_is_monotonic = sample_time is not None
            if sample_time is not None and self._last_object_sample is not None:
                previous_time, previous = self._last_object_sample
                duration = sample_time - previous_time
                if duration > 1.0e-5:
                    self._object_velocity = tuple(
                        (current - old) / duration
                        for current, old in zip(position, previous)
                    )
                    velocity_known = all(
                        math.isfinite(value) for value in self._object_velocity
                    )
                else:
                    sample_is_monotonic = False
            if not velocity_known:
                self._object_velocity = self._unknown_velocity()
            self._object_velocity_known = velocity_known
            self._last_object_sample = (
                (sample_time, position) if sample_is_monotonic else None
            )
            self._object_position = position
            rotation = pose.orientation
            self._object_orientation = (
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            )
            self._pose_generation += 1
            self.object_state_known = velocity_known
            self._pose_condition.notify_all()

    @staticmethod
    def _pose_stamp_sec(message: Any) -> float | None:
        try:
            stamp = message.header.stamp
            nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return None
        if nanoseconds <= 0:
            return None
        value = nanoseconds / 1_000_000_000.0
        return value if math.isfinite(value) else None

    @staticmethod
    def _unknown_velocity() -> tuple[float, float, float]:
        return (float("nan"), float("nan"), float("nan"))

    def _clear_object_motion_sample(self) -> None:
        with self._pose_condition:
            self._last_object_sample = None
            self._object_velocity = self._unknown_velocity()
            self._object_velocity_known = False
            self.object_state_known = False

    def _object_speed(self) -> float:
        return math.sqrt(sum(value * value for value in self._object_velocity))

    def _wait_for_reset_pose(
        self,
        expected: Sequence[float],
        previous_generation: int,
    ) -> bool:
        deadline = time.monotonic() + self.config.service_timeout_sec
        with self._pose_condition:
            observed_generation = previous_generation
            while time.monotonic() < deadline:
                if self._pose_generation > observed_generation:
                    observed_generation = self._pose_generation
                    if (
                        math.dist(self._object_position, expected)
                        <= self.config.reset_pose_tolerance_m
                    ):
                        return True
                self._pose_condition.wait(
                    timeout=max(0.0, min(0.05, deadline - time.monotonic()))
                )
        return False

    @staticmethod
    def _new_motion_goal():
        from panda_pick_place_interfaces.action import PlanExecuteMotion

        return PlanExecuteMotion.Goal()

    def _configure_motion_goal(self, goal: Any, planner: str, timeout_sec: float) -> None:
        goal.planner_id = self._moveit_planner_id(planner)
        goal.allowed_planning_time_sec = max(
            float(timeout_sec), self.config.planning_time_sec
        )
        goal.max_velocity_scaling_factor = self.config.velocity_scaling
        goal.max_acceleration_scaling_factor = self.config.acceleration_scaling
        # Non-Gazebo MoveIt runs are independent planning regressions.  Even if
        # a caller accidentally enables execute_motion, do not turn them into a
        # synthetic pick/place execution.
        goal.execute = self.config.execute_motion and self.config.use_gazebo

    def _moveit_planner_id(self, planner: str) -> str:
        if planner in {"linear", "rrt_connect", "RRTConnectkConfigDefault"}:
            return self.config.planner_id
        return planner

    def _send_motion(self, goal: Any) -> BackendMotionReport:
        self._begin_motion_tracking(enabled=bool(goal.execute))
        collision_count_before = self.collision_count

        def finish(report: BackendMotionReport) -> BackendMotionReport:
            tracked = self._finish_motion_tracking(report)
            if not bool(goal.execute):
                self.last_motion_report = tracked
                return tracked
            self.refresh_execution_state()
            physical_failure = self._validate_physical_grasp_after_motion()
            collision_delta = max(0, self.collision_count - collision_count_before)
            if self.invalid_collision_detected:
                final_report = replace(
                    tracked,
                    success=False,
                    failure_type=(
                        self.execution_collision_failure_type
                        or "execution_collision"
                    ),
                    collision_count=collision_delta,
                )
            elif physical_failure:
                final_report = replace(
                    tracked,
                    success=False,
                    failure_type=physical_failure,
                    collision_count=collision_delta,
                )
            else:
                final_report = replace(
                    tracked,
                    collision_count=collision_delta,
                )
            self.last_motion_report = final_report
            return final_report

        client = self.clients.motion
        if not client.wait_for_server(timeout_sec=self.config.service_timeout_sec):
            return finish(self._motion_failure("moveit_action_unavailable"))
        try:
            goal_future = client.send_goal_async(
                goal,
                feedback_callback=self._on_motion_feedback,
            )
            goal_handle = self._wait_future(
                goal_future, self.config.service_timeout_sec
            )
            if goal_handle is None:
                return finish(self._motion_failure("moveit_goal_timeout"))
            if not goal_handle.accepted:
                return finish(self._motion_failure("moveit_goal_rejected"))
            wrapped = self._wait_future(
                goal_handle.get_result_async(),
                goal.allowed_planning_time_sec + self.config.execution_timeout_sec,
            )
            if wrapped is None:
                goal_handle.cancel_goal_async()
                return finish(self._motion_failure("moveit_result_timeout"))
            result = wrapped.result
        except Exception:
            return finish(self._motion_failure("moveit_client_error"))

        failure_type = str(result.failure_type)
        planning_success = bool(result.success) or failure_type.startswith("execution_")
        plan = PlanResult(
            success=planning_success,
            planning_time_sec=float(result.planning_time_sec),
            explored_nodes=0,
            failure_type="" if planning_success else failure_type,
        )
        return finish(
            BackendMotionReport(
                success=bool(result.success),
                planning_success=planning_success,
                failure_type=failure_type,
                plan_result=plan,
                execution_time_sec=float(result.execution_time_sec),
                path_length_rad=float(result.path_length_rad),
                trajectory_points=int(result.trajectory_points),
                moveit_error_code=int(result.moveit_error_code),
                planning_scene_sync_success=(
                    self.planning_scene_sync_success
                ),
            )
        )

    def _on_motion_feedback(self, wrapped: Any) -> None:
        feedback = getattr(wrapped, "feedback", wrapped)
        self._feedback_phase = str(getattr(feedback, "phase", ""))

    def _command_gripper(self, width: float, *, allow_stall: bool = False) -> None:
        if not self.config.command_gripper:
            return
        from control_msgs.action import GripperCommand

        client = self.clients.gripper
        if client is None or not client.wait_for_server(
            timeout_sec=self.config.service_timeout_sec
        ):
            raise BackendOperationError("gripper_action_unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = self.config.max_gripper_effort
        handle = self._wait_future(
            client.send_goal_async(goal), self.config.service_timeout_sec
        )
        if handle is None or not handle.accepted:
            raise BackendOperationError("gripper_goal_rejected")
        wrapped = self._wait_future(
            handle.get_result_async(), self.config.execution_timeout_sec
        )
        if wrapped is None:
            handle.cancel_goal_async()
            raise BackendOperationError("gripper_result_timeout")
        try:
            from action_msgs.msg import GoalStatus

            status_succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        except (ImportError, AttributeError):
            status_succeeded = True
        reached_goal = bool(getattr(wrapped.result, "reached_goal", False))
        stalled = bool(getattr(wrapped.result, "stalled", False))
        if not status_succeeded or not (reached_goal or (allow_stall and stalled)):
            raise BackendOperationError("gripper_execution_failed")

    def _upsert_box(
        self,
        object_id: str,
        position: Sequence[float],
        size: Sequence[float],
        *,
        allowed_collision_links: Sequence[str] = (),
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> None:
        from moveit_msgs.msg import CollisionObject
        from panda_pick_place_interfaces.srv import UpsertWorldObject
        from shape_msgs.msg import SolidPrimitive

        request = UpsertWorldObject.Request()
        if hasattr(request, "allowed_collision_links"):
            request.allowed_collision_links = list(allowed_collision_links)
        collision = CollisionObject()
        collision.header.frame_id = self.config.world_frame
        collision.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size]
        from geometry_msgs.msg import Pose

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        request.collision_object = collision
        self._require_service_success(
            self.clients.upsert,
            request,
            f"planning_scene_upsert_failed:{object_id}",
        )

    @staticmethod
    def _object_size(object_variant: str) -> tuple[float, float, float]:
        if object_variant == "cube":
            return (0.05, 0.05, 0.05)
        if object_variant == "rect_prism":
            return (0.06, 0.04, 0.05)
        raise ValueError(f"unsupported object variant: {object_variant}")

    def _require_world_object_removed(
        self,
        object_id: str,
        failure_type: str,
    ) -> None:
        from panda_pick_place_interfaces.srv import RemoveWorldObject

        request = RemoveWorldObject.Request()
        request.object_id = object_id
        self._require_service_success(
            self.clients.remove,
            request,
            failure_type,
        )

    def _require_attached_state(
        self,
        attach: bool,
        failure_type: str,
    ) -> None:
        self._require_service_success(
            self.clients.attached,
            self._attached_request(attach),
            failure_type,
        )

    def _set_attached(self, attach: bool) -> bool:
        return self._service_success(
            self.clients.attached,
            self._attached_request(attach),
        )

    def _attached_request(self, attach: bool):
        return self._attached_request_for(self._selected_object_id, attach)

    def _attached_request_for(self, object_id: str, attach: bool):
        from panda_pick_place_interfaces.srv import SetAttachedObject

        request = SetAttachedObject.Request()
        request.object_id = object_id
        request.link_name = self.config.attach_link
        request.touch_links = list(self.config.touch_links)
        request.attach = attach
        return request

    def _assisted_object_request(self, attach: bool):
        from panda_pick_place_interfaces.srv import SetAssistedObject

        request = SetAssistedObject.Request()
        request.object_id = self._selected_object_id
        request.attach = attach
        return request

    @staticmethod
    def _trigger_request():
        from std_srvs.srv import Trigger

        return Trigger.Request()

    def _reset_request(self, task: TaskSpec):
        from panda_pick_place_interfaces.msg import SceneObjectTask
        from panda_pick_place_interfaces.srv import ResetEpisode

        request = ResetEpisode.Request()
        request.header.frame_id = self.config.world_frame
        try:
            request.header.stamp = self.node.get_clock().now().to_msg()
        except AttributeError:
            pass
        request.episode_id = task.episode_id
        request.task_seed = task.task_seed
        request.scene = task.scene
        request.grasp_mode = task.grasp_mode
        request.object_variant = task.object_variant
        request.object_yaw_rad = task.object_yaw_rad
        request.object_position.x = task.object_position[0]
        request.object_position.y = task.object_position[1]
        request.object_position.z = task.object_position[2]
        request.target_position.x = task.target_position[0]
        request.target_position.y = task.target_position[1]
        request.target_position.z = task.target_position[2]
        for item in task.workpieces:
            workpiece = SceneObjectTask()
            workpiece.object_id = item.object_id
            workpiece.object_variant = item.object_variant
            workpiece.pose.position.x = item.position[0]
            workpiece.pose.position.y = item.position[1]
            workpiece.pose.position.z = item.position[2]
            workpiece.pose.orientation.x = item.orientation_xyzw[0]
            workpiece.pose.orientation.y = item.orientation_xyzw[1]
            workpiece.pose.orientation.z = item.orientation_xyzw[2]
            workpiece.pose.orientation.w = item.orientation_xyzw[3]
            request.workpieces.append(workpiece)
        return request

    def _reset_response_matches(self, response: Any, task: TaskSpec) -> bool:
        def values(point: Any) -> tuple[float, float, float]:
            return (float(point.x), float(point.y), float(point.z))

        try:
            applied_object = values(response.applied_object_position)
            applied_target = values(response.applied_target_position)
        except AttributeError:
            return False
        tolerance = self.config.reset_pose_tolerance_m
        applied_yaw = float(
            getattr(response, "applied_object_yaw_rad", task.object_yaw_rad)
        )
        yaw_error = abs(
            math.atan2(
                math.sin(applied_yaw - task.object_yaw_rad),
                math.cos(applied_yaw - task.object_yaw_rad),
            )
        )
        multi_matches = True
        if task.workpiece_count > 1:
            applied = {
                item.object_id: item for item in response.applied_workpieces
            }
            multi_matches = len(applied) == task.workpiece_count and all(
                item.object_id in applied
                and math.dist(values(applied[item.object_id].pose.position), item.position)
                <= tolerance
                and abs(
                    math.atan2(
                        math.sin(_pose_yaw(applied[item.object_id].pose) - item.yaw_rad),
                        math.cos(_pose_yaw(applied[item.object_id].pose) - item.yaw_rad),
                    )
                )
                <= 0.05
                for item in task.workpieces
            )
        return (
            math.dist(applied_object, task.object_position) <= tolerance
            and math.dist(applied_target, task.target_position) <= tolerance
            and yaw_error <= 0.05
            and multi_matches
        )

    def _rebase_nonselected_displacement(
        self, response: Any, task: TaskSpec
    ) -> None:
        """Start displacement accounting after Gazebo has applied the reset.

        Pose topics can publish the launch-time scene while the reset service
        is still moving entities.  Counting those samples against the pending
        task pose produces a large false displacement before an episode has
        begun.  The reset response contains verified Gazebo readback, so it is
        the correct baseline for post-reset non-selected motion.
        """
        if task.workpiece_count == 1:
            self._initial_workpiece_positions = {
                task.workpieces[0].object_id: task.workpieces[0].position
            }
            self.max_nonselected_displacement_m = 0.0
            return
        applied = {
            str(item.object_id): (
                float(item.pose.position.x),
                float(item.pose.position.y),
                float(item.pose.position.z),
            )
            for item in response.applied_workpieces
        }
        self._initial_workpiece_positions = applied
        self.max_nonselected_displacement_m = 0.0

    def _require_service_success(
        self,
        client: Any,
        request: Any,
        failure_type: str,
    ) -> Any:
        response = self._call_service(client, request)
        if response is None or not bool(response.success):
            self.planning_scene_sync_success = False
            detail = "" if response is None else str(
                getattr(response, "message", "")
                or getattr(response, "failure_type", "")
            )
            typed_failure = "" if response is None else str(
                getattr(response, "failure_type", "")
            )
            raise BackendOperationError(
                typed_failure or failure_type,
                detail,
            )
        return response

    def _service_success(self, client: Any, request: Any) -> bool:
        response = self._call_service(client, request)
        return response is not None and bool(response.success)

    def _call_service(self, client: Any, request: Any) -> Any | None:
        if client is None or not client.wait_for_service(
            timeout_sec=self.config.service_timeout_sec
        ):
            return None
        try:
            return self._wait_future(
                client.call_async(request), self.config.service_timeout_sec
            )
        except Exception:
            return None

    @staticmethod
    def _wait_future(future: Any, timeout_sec: float) -> Any | None:
        if future.done():
            return future.result()
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=max(0.0, timeout_sec)):
            try:
                future.cancel()
            except Exception:
                pass
            return None
        try:
            return future.result()
        except Exception:
            return None

    @staticmethod
    def _motion_failure(failure_type: str) -> BackendMotionReport:
        return BackendMotionReport(
            success=False,
            planning_success=False,
            failure_type=failure_type,
            plan_result=PlanResult(False, failure_type=failure_type),
        )
