"""Deterministic, task-typed Gazebo episode reset through ros2_control."""

from __future__ import annotations

import math
import threading
import time
from typing import Iterable

from control_msgs.action import FollowJointTrajectory, GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import Odometry
from panda_pick_place_interfaces.srv import (
    GetExecutionCollisionState,
    GetPhysicalGraspState,
    ResetEpisode,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from panda_gz_bringup.reset_contract import (
    ResetSpec,
    ScenePoseConfig,
    entity_pose_map,
    pose_error,
    require_world_frame,
    vector_norm,
)
from panda_gz_bringup.status import StatusCode, service_message


ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))
HAND_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")


class EpisodeResetManager(Node):
    """Reset one explicit task without relying on stale launch-time scene state."""

    def __init__(self) -> None:
        super().__init__("episode_reset_manager")
        self._group = ReentrantCallbackGroup()
        self._operation_lock = threading.Lock()
        self._joint_state_condition = threading.Condition()
        self._joint_state_generation = 0
        self._joint_positions: dict[str, float] = {}
        self._joint_velocities: dict[str, float] = {}
        self._entity_state_condition = threading.Condition()
        self._entity_poses: dict[str, Pose] = {}
        self._entity_generations = {role: 0 for role in ("cube", "target", "obstacle")}
        self._cube_linear_velocity = (math.inf, math.inf, math.inf)
        self._cube_angular_velocity = (math.inf, math.inf, math.inf)
        self._verified_entity_poses: dict[str, Pose] = {}
        self._active_spec: ResetSpec | None = None
        self._world_paused = False

        self.declare_parameter("world_name", "panda_pick_place")
        self.declare_parameter("scene", "clear")
        self.declare_parameter("grasp_mode", "assisted")
        self.declare_parameter("default_episode_id", 0)
        self.declare_parameter("default_task_seed", 42)
        self.declare_parameter("default_object_position", [0.57, -0.17, 0.025])
        self.declare_parameter("default_target_position", [0.57, 0.17, 0.025])
        self.declare_parameter("target_marker_z", 0.001)
        self.declare_parameter(
            "obstacle_pose", [0.54, 0.0, 0.14, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter(
            "clear_obstacle_pose", [0.54, 2.0, 0.14, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter("auto_reset", True)
        self.declare_parameter("auto_reset_delay_sec", 8.0)
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("home_timeout_sec", 8.0)
        self.declare_parameter("home_duration_sec", 3.0)
        self.declare_parameter("joint_position_tolerance_rad", 0.025)
        self.declare_parameter("joint_velocity_tolerance_rad_s", 0.025)
        self.declare_parameter("gripper_position_tolerance_m", 0.003)
        self.declare_parameter("joint_state_timeout_sec", 3.0)
        self.declare_parameter("home_stable_sec", 0.20)
        self.declare_parameter("open_gripper_width_m", 0.04)
        self.declare_parameter("gripper_max_effort", 20.0)
        self.declare_parameter("entity_state_timeout_sec", 5.0)
        self.declare_parameter("entity_state_stable_sec", 0.20)
        self.declare_parameter("entity_position_tolerance_m", 0.005)
        self.declare_parameter("entity_orientation_tolerance_rad", 0.05)
        self.declare_parameter("cube_linear_velocity_tolerance_mps", 0.010)
        self.declare_parameter("cube_angular_velocity_tolerance_rad_s", 0.050)
        self.declare_parameter(
            "home_joints", [0.0, -0.65, 0.0, -2.2, 0.0, 1.55, 0.78]
        )
        self.declare_parameter("cube_model", "cube")
        self.declare_parameter("target_model", "target")
        self.declare_parameter("obstacle_model", "obstacle")

        # Validate launch defaults before advertising a service.
        self._default_spec()
        world_name = str(self.get_parameter("world_name").value)
        self._control_client = self.create_client(
            ControlWorld,
            f"/world/{world_name}/control",
            callback_group=self._group,
        )
        self._set_pose_client = self.create_client(
            SetEntityPose,
            f"/world/{world_name}/set_pose",
            callback_group=self._group,
        )
        self._detach_client = self.create_client(
            Trigger, "/panda_gz/detach_object", callback_group=self._group
        )
        self._home_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
            callback_group=self._group,
        )
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            "/panda_hand_controller/gripper_cmd",
            callback_group=self._group,
        )
        self._physics_state_client = self.create_client(
            GetPhysicalGraspState,
            "/panda_gz/get_physical_grasp_state",
            callback_group=self._group,
        )
        self._collision_state_client = self.create_client(
            GetExecutionCollisionState,
            "/panda_gz/get_execution_collision_state",
            callback_group=self._group,
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, "/panda_gz/reset_status", 10
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            Odometry,
            "/panda_gz/cube_odometry",
            self._on_cube_odometry,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            PoseArray,
            "/panda_gz/target_pose",
            self._on_target_pose,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            PoseArray,
            "/panda_gz/obstacle_pose",
            self._on_obstacle_pose,
            10,
            callback_group=self._group,
        )
        self.create_service(
            ResetEpisode,
            "/panda_gz/reset_episode",
            self._reset_service,
            callback_group=self._group,
        )

        if bool(self.get_parameter("auto_reset").value):
            delay = float(self.get_parameter("auto_reset_delay_sec").value)
            self._auto_reset_timer = self.create_timer(
                delay, self._auto_reset_once, callback_group=self._group
            )
        self._publish_status(StatusCode.READY, "typed reset manager ready")

    def _default_spec(self) -> ResetSpec:
        return ResetSpec.create(
            episode_id=int(self.get_parameter("default_episode_id").value),
            task_seed=int(self.get_parameter("default_task_seed").value),
            scene=str(self.get_parameter("scene").value),
            grasp_mode=str(self.get_parameter("grasp_mode").value),
            object_position=self.get_parameter("default_object_position").value,
            target_position=self.get_parameter("default_target_position").value,
        )

    def _request_spec(self, request: ResetEpisode.Request) -> ResetSpec:
        require_world_frame(request.header.frame_id)
        spec = ResetSpec.create(
            episode_id=request.episode_id,
            task_seed=request.task_seed,
            scene=request.scene,
            grasp_mode=request.grasp_mode,
            object_position=(
                request.object_position.x,
                request.object_position.y,
                request.object_position.z,
            ),
            target_position=(
                request.target_position.x,
                request.target_position.y,
                request.target_position.z,
            ),
        )
        configured_mode = str(self.get_parameter("grasp_mode").value)
        if spec.grasp_mode != configured_mode:
            raise ValueError(
                "request grasp_mode does not match launch grasp_mode "
                f"({spec.grasp_mode} != {configured_mode})"
            )
        return spec

    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        with self._joint_state_condition:
            self._joint_positions.update(positions)
            self._joint_velocities.update(velocities)
            self._joint_state_generation += 1
            self._joint_state_condition.notify_all()

    def _record_entity_pose(self, role: str, pose: Pose) -> None:
        with self._entity_state_condition:
            self._entity_poses[role] = pose
            self._entity_generations[role] += 1
            self._entity_state_condition.notify_all()

    def _on_cube_odometry(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        with self._entity_state_condition:
            self._entity_poses["cube"] = message.pose.pose
            self._cube_linear_velocity = (linear.x, linear.y, linear.z)
            self._cube_angular_velocity = (angular.x, angular.y, angular.z)
            self._entity_generations["cube"] += 1
            self._entity_state_condition.notify_all()

    def _on_target_pose(self, message: PoseArray) -> None:
        if message.poses:
            self._record_entity_pose("target", message.poses[0])

    def _on_obstacle_pose(self, message: PoseArray) -> None:
        if message.poses:
            self._record_entity_pose("obstacle", message.poses[0])

    def _auto_reset_once(self) -> None:
        self._auto_reset_timer.cancel()
        success, code, detail = self._perform_reset(self._default_spec())
        if not success:
            self.get_logger().error(service_message(code, detail))

    def _reset_service(
        self, request: ResetEpisode.Request, response: ResetEpisode.Response
    ) -> ResetEpisode.Response:
        try:
            spec = self._request_spec(request)
        except ValueError as exc:
            code = StatusCode.INVALID_REQUEST
            self._publish_status(code, str(exc))
            return self._fill_response(response, False, code, str(exc), None)
        success, code, detail = self._perform_reset(spec)
        return self._fill_response(response, success, code, detail, spec)

    def _fill_response(
        self,
        response: ResetEpisode.Response,
        success: bool,
        code: StatusCode,
        detail: str,
        spec: ResetSpec | None,
    ) -> ResetEpisode.Response:
        response.success = success
        response.status_code = int(code)
        response.failure_type = "" if success else code.name.lower()
        response.message = service_message(code, detail)
        # Applied coordinates are populated only after fresh Gazebo readback
        # has verified all three scene entities.  A failed SetEntityPose call
        # must never be reported by echoing the request.
        if success and spec is not None:
            with self._entity_state_condition:
                cube_pose = self._verified_entity_poses.get("cube")
                target_pose = self._verified_entity_poses.get("target")
            if cube_pose is None or target_pose is None:
                raise RuntimeError("successful reset has no verified entity state")
            response.applied_object_position = Point(
                x=cube_pose.position.x,
                y=cube_pose.position.y,
                z=cube_pose.position.z,
            )
            # The target model is a thin marker at target_marker_z, whereas
            # this service returns the semantic desired cube-centre position.
            marker_offset = spec.target_position[2] - float(
                self.get_parameter("target_marker_z").value
            )
            response.applied_target_position = Point(
                x=target_pose.position.x,
                y=target_pose.position.y,
                z=target_pose.position.z + marker_offset,
            )
        return response

    def _perform_reset(
        self, spec: ResetSpec
    ) -> tuple[bool, StatusCode, str]:
        if not self._operation_lock.acquire(blocking=False):
            return False, StatusCode.RESETTING, "a reset is already in progress"
        self._active_spec = spec
        with self._entity_state_condition:
            self._verified_entity_poses.clear()
        try:
            self._publish_status(StatusCode.RESETTING, "task reset started")
            timeout = float(self.get_parameter("service_timeout_sec").value)
            if not self._control_client.wait_for_service(timeout_sec=timeout):
                return self._fail(
                    StatusCode.GZ_SERVICE_UNAVAILABLE,
                    "world control service unavailable",
                )
            if not self._set_pose_client.wait_for_service(timeout_sec=timeout):
                return self._fail(
                    StatusCode.GZ_SERVICE_UNAVAILABLE,
                    "set_pose service unavailable",
                )
            if not self._detach_client.wait_for_service(timeout_sec=timeout):
                return self._fail(
                    StatusCode.GZ_SERVICE_UNAVAILABLE,
                    "detach service unavailable",
                )

            # Fortress explicitly does not support model-only reset or changing
            # the physics seed at runtime.  Keep /clock monotonic and reset the
            # controlled state explicitly instead: detach while physics is
            # running, pause, place all task entities, resume, command the arm
            # and hand, then prove convergence from fresh Gazebo readback.
            detached = self._wait_future(
                self._detach_client.call_async(Trigger.Request()), timeout
            )
            if detached is None or not detached.success:
                return self._fail(
                    StatusCode.GZ_SERVICE_FAILED,
                    "DetachableJoint did not confirm reset-time detach",
                )
            if not self._control_world(pause=True):
                return self._fail(StatusCode.GZ_SERVICE_FAILED, "world pause failed")
            generations = self._entity_generation_snapshot()
            if not self._set_scene_poses(spec):
                return self._fail(
                    StatusCode.GZ_SERVICE_FAILED, "task entity pose reset failed"
                )
            if not self._control_world(pause=False):
                return self._fail(StatusCode.GZ_SERVICE_FAILED, "world resume failed")

            if not self._move_home() or not self._open_gripper():
                return self._fail(
                    StatusCode.HOME_FAILED,
                    "arm home or open-gripper command failed",
                )
            home_ok, home_detail = self._wait_for_home_settled()
            if not home_ok:
                return self._fail(
                    StatusCode.HOME_FAILED,
                    home_detail,
                )

            # SetEntityPose does not promise to clear twist.  Do not report a
            # successful reset until fresh odometry proves that cube pose and
            # both velocity channels remain within tolerance for the configured
            # stable interval.
            state_ok, state_detail = self._wait_for_scene_readback(
                spec, generations
            )
            if not state_ok:
                return self._fail(StatusCode.RESET_STATE_INVALID, state_detail)
            if not self._reset_collision_monitor(spec):
                return self._fail(
                    StatusCode.GZ_SERVICE_FAILED,
                    "execution collision monitor reset failed",
                )
            if not self._reset_physics_monitor(spec):
                return self._fail(
                    StatusCode.GZ_SERVICE_FAILED,
                    "physical grasp monitor reset failed",
                )

            detail = (
                f"episode={spec.episode_id} seed={spec.task_seed} "
                f"scene={spec.scene} reset at configured home"
            )
            self._publish_status(StatusCode.RESET_SUCCEEDED, detail)
            return True, StatusCode.RESET_SUCCEEDED, detail
        except Exception as exc:  # return stable typed failures to orchestration
            self.get_logger().exception("unexpected reset exception")
            return self._fail(StatusCode.INTERNAL_ERROR, str(exc))
        finally:
            if self._world_paused:
                self._control_world(pause=False)
            self._operation_lock.release()

    def _control_world(self, pause: bool) -> bool:
        request = ControlWorld.Request()
        request.world_control.pause = pause
        response = self._wait_future(
            self._control_client.call_async(request),
            float(self.get_parameter("service_timeout_sec").value),
        )
        success = bool(response is not None and response.success)
        if success:
            self._world_paused = pause
        return success

    def _scene_pose_config(self) -> ScenePoseConfig:
        return ScenePoseConfig(
            target_marker_z=float(self.get_parameter("target_marker_z").value),
            obstacle_pose=tuple(self.get_parameter("obstacle_pose").value),
            clear_obstacle_pose=tuple(
                self.get_parameter("clear_obstacle_pose").value
            ),
        )

    def _set_scene_poses(self, spec: ResetSpec) -> bool:
        model_names = {
            "cube": str(self.get_parameter("cube_model").value),
            "target": str(self.get_parameter("target_model").value),
            "obstacle": str(self.get_parameter("obstacle_model").value),
        }
        poses = entity_pose_map(spec, self._scene_pose_config())
        return all(
            self._set_entity_pose(model_names[role], pose)
            for role, pose in poses.items()
        )

    def _set_entity_pose(self, entity_name: str, values) -> bool:
        if len(values) != 7:
            raise ValueError(f"pose for {entity_name} must contain xyz + xyzw")
        request = SetEntityPose.Request()
        request.entity = Entity(name=entity_name, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = float(values[0])
        request.pose.position.y = float(values[1])
        request.pose.position.z = float(values[2])
        request.pose.orientation.x = float(values[3])
        request.pose.orientation.y = float(values[4])
        request.pose.orientation.z = float(values[5])
        request.pose.orientation.w = float(values[6])
        response = self._wait_future(
            self._set_pose_client.call_async(request),
            float(self.get_parameter("service_timeout_sec").value),
        )
        return bool(response is not None and response.success)

    def _entity_generation_snapshot(self) -> dict[str, int]:
        with self._entity_state_condition:
            return dict(self._entity_generations)

    @staticmethod
    def _pose_components(pose: Pose) -> tuple[tuple[float, ...], tuple[float, ...]]:
        return (
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _wait_for_scene_readback(
        self, spec: ResetSpec, previous_generations: dict[str, int]
    ) -> tuple[bool, str]:
        expected = entity_pose_map(spec, self._scene_pose_config())
        timeout = float(self.get_parameter("entity_state_timeout_sec").value)
        stable_required = float(
            self.get_parameter("entity_state_stable_sec").value
        )
        position_tolerance = float(
            self.get_parameter("entity_position_tolerance_m").value
        )
        orientation_tolerance = float(
            self.get_parameter("entity_orientation_tolerance_rad").value
        )
        linear_tolerance = float(
            self.get_parameter("cube_linear_velocity_tolerance_mps").value
        )
        angular_tolerance = float(
            self.get_parameter("cube_angular_velocity_tolerance_rad_s").value
        )
        if min(
            timeout,
            stable_required,
            position_tolerance,
            orientation_tolerance,
            linear_tolerance,
            angular_tolerance,
        ) < 0.0:
            raise ValueError("entity reset tolerances must be non-negative")

        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        stable_cube_generation = -1
        last_detail = "no Gazebo entity-state message received"
        with self._entity_state_condition:
            while True:
                now = time.monotonic()
                if now >= deadline:
                    return False, f"Gazebo reset readback timed out: {last_detail}"

                stale_roles = [
                    role
                    for role in expected
                    if self._entity_generations.get(role, 0)
                    <= previous_generations.get(role, 0)
                ]
                missing_roles = [
                    role for role in expected if role not in self._entity_poses
                ]
                pose_failures = []
                if not stale_roles and not missing_roles:
                    for role, expected_pose in expected.items():
                        xyz, xyzw = self._pose_components(
                            self._entity_poses[role]
                        )
                        translation, rotation = pose_error(
                            xyz, xyzw, expected_pose
                        )
                        if (
                            translation > position_tolerance
                            or rotation > orientation_tolerance
                        ):
                            pose_failures.append(
                                f"{role}(position={translation:.6f}m, "
                                f"rotation={rotation:.6f}rad)"
                            )

                linear_speed = vector_norm(self._cube_linear_velocity)
                angular_speed = vector_norm(self._cube_angular_velocity)
                velocities_ok = (
                    linear_speed <= linear_tolerance
                    and angular_speed <= angular_tolerance
                )
                state_ok = (
                    not stale_roles
                    and not missing_roles
                    and not pose_failures
                    and velocities_ok
                )
                if state_ok:
                    if stable_since is None:
                        stable_since = now
                        stable_cube_generation = self._entity_generations["cube"]
                    stable_elapsed = now - stable_since
                    cube_advanced = (
                        self._entity_generations["cube"]
                        > stable_cube_generation
                    )
                    if stable_elapsed >= stable_required and cube_advanced:
                        self._verified_entity_poses = {
                            role: self._entity_poses[role] for role in expected
                        }
                        return (
                            True,
                            "fresh Gazebo pose and zero-twist readback verified",
                        )
                    last_detail = (
                        f"stable for {stable_elapsed:.3f}/{stable_required:.3f}s; "
                        f"cube odometry advanced={cube_advanced}"
                    )
                else:
                    stable_since = None
                    stable_cube_generation = -1
                    parts = []
                    if stale_roles:
                        parts.append(f"stale={','.join(stale_roles)}")
                    if missing_roles:
                        parts.append(f"missing={','.join(missing_roles)}")
                    if pose_failures:
                        parts.append("pose=" + ";".join(pose_failures))
                    if not velocities_ok:
                        parts.append(
                            f"cube_speed=({linear_speed:.6f}m/s,"
                            f"{angular_speed:.6f}rad/s)"
                        )
                    last_detail = "; ".join(parts)
                self._entity_state_condition.wait(
                    timeout=min(0.05, max(0.0, deadline - now))
                )

    def _move_home(self) -> bool:
        timeout = float(self.get_parameter("home_timeout_sec").value)
        if not self._home_client.wait_for_server(timeout_sec=timeout):
            self._publish_status(
                StatusCode.CONTROLLER_UNAVAILABLE,
                "panda arm FollowJointTrajectory server unavailable",
            )
            return False
        home = tuple(float(v) for v in self.get_parameter("home_joints").value)
        if len(home) != len(ARM_JOINTS):
            raise ValueError("home_joints must contain seven positions")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(home)
        duration = float(self.get_parameter("home_duration_sec").value)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        goal.trajectory.points = [point]
        goal_handle = self._wait_future(
            self._home_client.send_goal_async(goal), timeout
        )
        if goal_handle is None or not goal_handle.accepted:
            return False
        wrapped_result = self._wait_future(goal_handle.get_result_async(), timeout)
        return bool(
            wrapped_result is not None
            and wrapped_result.result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )

    def _open_gripper(self) -> bool:
        timeout = float(self.get_parameter("home_timeout_sec").value)
        if not self._gripper_client.wait_for_server(timeout_sec=timeout):
            self._publish_status(
                StatusCode.CONTROLLER_UNAVAILABLE,
                "Panda hand GripperCommand server unavailable",
            )
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(
            self.get_parameter("open_gripper_width_m").value
        )
        goal.command.max_effort = float(
            self.get_parameter("gripper_max_effort").value
        )
        goal_handle = self._wait_future(
            self._gripper_client.send_goal_async(goal), timeout
        )
        if goal_handle is None or not goal_handle.accepted:
            return False
        wrapped_result = self._wait_future(goal_handle.get_result_async(), timeout)
        return bool(
            wrapped_result is not None and wrapped_result.result.reached_goal
        )

    def _reset_physics_monitor(self, spec: ResetSpec) -> bool:
        if spec.grasp_mode != "physics":
            return True
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not self._physics_state_client.wait_for_service(timeout_sec=timeout):
            return False
        request = GetPhysicalGraspState.Request()
        request.command = GetPhysicalGraspState.Request.COMMAND_RESET
        request.episode_id = spec.episode_id
        response = self._wait_future(
            self._physics_state_client.call_async(request), timeout
        )
        return bool(response is not None and response.success)

    def _reset_collision_monitor(self, spec: ResetSpec) -> bool:
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not self._collision_state_client.wait_for_service(timeout_sec=timeout):
            return False
        request = GetExecutionCollisionState.Request()
        request.command = GetExecutionCollisionState.Request.COMMAND_RESET
        request.episode_id = spec.episode_id
        request.object_initial_position = Point(
            x=spec.object_position[0],
            y=spec.object_position[1],
            z=spec.object_position[2],
        )
        request.target_position = Point(
            x=spec.target_position[0],
            y=spec.target_position[1],
            z=spec.target_position[2],
        )
        response = self._wait_future(
            self._collision_state_client.call_async(request), timeout
        )
        return bool(response is not None and response.success)

    def _wait_for_home_settled(self) -> tuple[bool, str]:
        home = tuple(float(v) for v in self.get_parameter("home_joints").value)
        position_tolerance = float(
            self.get_parameter("joint_position_tolerance_rad").value
        )
        velocity_tolerance = float(
            self.get_parameter("joint_velocity_tolerance_rad_s").value
        )
        targets = dict(zip(ARM_JOINTS, home))
        gripper_width = float(self.get_parameter("open_gripper_width_m").value)
        targets.update({name: gripper_width for name in HAND_JOINTS})
        gripper_tolerance = float(
            self.get_parameter("gripper_position_tolerance_m").value
        )
        timeout = float(self.get_parameter("joint_state_timeout_sec").value)
        stable_required = float(self.get_parameter("home_stable_sec").value)
        if min(
            position_tolerance,
            velocity_tolerance,
            gripper_tolerance,
            timeout,
            stable_required,
        ) < 0.0:
            raise ValueError("home-state tolerances must be non-negative")

        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        stable_generation = -1
        last_detail = "no fresh joint state received"
        with self._joint_state_condition:
            initial_generation = self._joint_state_generation
            while True:
                now = time.monotonic()
                if now >= deadline:
                    return False, f"home-state readback timed out: {last_detail}"

                missing = [
                    name
                    for name in targets
                    if name not in self._joint_positions
                    or name not in self._joint_velocities
                ]
                fresh = self._joint_state_generation > initial_generation
                if missing:
                    max_arm_error = math.inf
                    max_hand_error = math.inf
                    max_speed = math.inf
                else:
                    arm_errors = tuple(
                        abs(self._joint_positions[name] - targets[name])
                        for name in ARM_JOINTS
                    )
                    hand_errors = tuple(
                        abs(self._joint_positions[name] - targets[name])
                        for name in HAND_JOINTS
                    )
                    speeds = tuple(
                        abs(self._joint_velocities[name]) for name in targets
                    )
                    max_arm_error = max(arm_errors)
                    max_hand_error = max(hand_errors)
                    max_speed = max(speeds)

                state_ok = (
                    fresh
                    and not missing
                    and max_arm_error <= position_tolerance
                    and max_hand_error <= gripper_tolerance
                    and max_speed <= velocity_tolerance
                )
                if state_ok:
                    if stable_since is None:
                        stable_since = now
                        stable_generation = self._joint_state_generation
                    stable_elapsed = now - stable_since
                    state_advanced = (
                        self._joint_state_generation > stable_generation
                    )
                    if stable_elapsed >= stable_required and state_advanced:
                        return (
                            True,
                            "fresh home position and zero-velocity readback verified",
                        )
                    last_detail = (
                        f"stable for {stable_elapsed:.3f}/{stable_required:.3f}s; "
                        f"joint state advanced={state_advanced}"
                    )
                else:
                    stable_since = None
                    stable_generation = -1
                    parts = []
                    if not fresh:
                        parts.append("joint state is stale")
                    if missing:
                        parts.append("missing=" + ",".join(missing))
                    if not missing:
                        parts.append(
                            f"arm_error={max_arm_error:.6f}rad/"
                            f"{position_tolerance:.6f}rad"
                        )
                        parts.append(
                            f"gripper_error={max_hand_error:.6f}m/"
                            f"{gripper_tolerance:.6f}m"
                        )
                        parts.append(
                            f"joint_speed={max_speed:.6f}rad/s/"
                            f"{velocity_tolerance:.6f}rad/s"
                        )
                    last_detail = "; ".join(parts)
                self._joint_state_condition.wait(
                    timeout=min(0.05, max(0.0, deadline - now))
                )

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout=max(0.0, timeout_sec)):
            return None
        try:
            return future.result()
        except Exception:
            return None

    def _fail(self, code: StatusCode, detail: str):
        self._publish_status(code, detail)
        return False, code, detail

    def _publish_status(
        self,
        code: StatusCode,
        detail: str,
        extra: Iterable[tuple[str, object]] = (),
    ) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "panda_gz/episode_reset"
        status.hardware_id = "gazebo_fortress"
        status.level = (
            DiagnosticStatus.ERROR
            if int(code) >= int(StatusCode.GZ_SERVICE_UNAVAILABLE)
            else DiagnosticStatus.OK
        )
        status.message = detail
        values = [
            ("status_code", int(code)),
            ("status_name", code.name),
            *extra,
        ]
        if self._active_spec is not None:
            values.extend(
                [
                    ("episode_id", self._active_spec.episode_id),
                    ("task_seed", self._active_spec.task_seed),
                    ("scene", self._active_spec.scene),
                    ("grasp_mode", self._active_spec.grasp_mode),
                ]
            )
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values]
        array.status = [status]
        self._status_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EpisodeResetManager()
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
