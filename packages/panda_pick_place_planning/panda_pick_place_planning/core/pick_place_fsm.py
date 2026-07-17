from __future__ import annotations

from enum import Enum
import math
from typing import Any

from panda_pick_place_planning.core.backends import (
    BackendMotionReport,
    ManipulationBackend,
    PyBulletBackend,
)
from panda_pick_place_planning.core.planners.base import path_length
from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv
from panda_pick_place_planning.core.schemas import EpisodeResult, PlanResult, TaskSpec


class PickPlaceState(str, Enum):
    RESET = "reset"
    PRE_GRASP = "pre_grasp"
    GRASP = "grasp"
    LIFT = "lift"
    TRANSFER = "transfer"
    PLACE = "place"
    RETREAT = "retreat"
    DONE = "done"
    FAILED = "failed"


class PickPlaceRunner:
    """Backend-agnostic pick/place state machine.

    Passing a raw ``PyBulletPandaEnv`` remains supported for V1 callers; it is
    wrapped in ``PyBulletBackend`` rather than creating a second state machine.
    """

    def __init__(self, backend: ManipulationBackend | PyBulletPandaEnv) -> None:
        if isinstance(backend, PyBulletPandaEnv):
            backend = PyBulletBackend(backend)
        if not isinstance(backend, ManipulationBackend):
            raise TypeError("backend does not implement ManipulationBackend")
        self.backend = backend
        self.state = PickPlaceState.RESET
        self._object_was_attached = False
        self._object_was_released = False
        self._stage_metrics: dict[str, dict[str, Any]] = {}

    def run(self, task: TaskSpec) -> EpisodeResult:
        self.state = PickPlaceState.RESET
        self._object_was_attached = False
        self._object_was_released = False
        self._stage_metrics = {}
        self.backend.reset(task)
        episode_start_steps = self.backend.simulation_steps
        episode_start_collisions = self.backend.collision_count
        object_initial_position, _ = self.backend.object_state()
        orientation = self.backend.downward_orientation()

        self.state = PickPlaceState.PRE_GRASP
        pre_grasp = (
            task.object_position[0],
            task.object_position[1],
            self.backend.hover_height,
        )
        report = self._move_to_pose(pre_grasp, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
            )

        self.backend.set_gripper(self.backend.open_gripper_width)
        grasp_pose = (
            task.object_position[0],
            task.object_position[1],
            self.backend.grasp_height,
        )
        report = self._move_to_pose(grasp_pose, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
            )

        self.state = PickPlaceState.GRASP
        self.backend.set_gripper(self.backend.closed_gripper_width)
        if not self.backend.attach_object():
            return self._failure(
                task,
                object_initial_position,
                str(
                    getattr(
                        self.backend,
                        "last_failure_type",
                        "",
                    )
                    or "grasp_failed"
                ),
                episode_start_steps,
                episode_start_collisions,
            )
        self._object_was_attached = True
        self._capture_grasp_metrics(PickPlaceState.GRASP)

        self.state = PickPlaceState.LIFT
        lift_pose = (
            task.object_position[0],
            task.object_position[1],
            self.backend.hover_height,
        )
        report = self._move_to_pose(lift_pose, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
            )

        self.state = PickPlaceState.TRANSFER
        pre_place = (
            task.target_position[0],
            task.target_position[1],
            self.backend.hover_height,
        )
        report = self._move_to_pose(
            pre_place,
            orientation,
            planner=task.planner,
            seed=task.planner_seed,
            timeout_sec=3.0,
        )
        transfer_report = report
        plan_result = report.plan_result
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
                plan_result=plan_result,
            )

        self.state = PickPlaceState.PLACE
        place_pose = (
            task.target_position[0],
            task.target_position[1],
            self.backend.place_height,
        )
        report = self._move_to_pose(place_pose, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
                plan_result=plan_result,
            )
        self.backend.set_gripper(self.backend.open_gripper_width)
        self.backend.detach_object()
        self._object_was_released = True
        self.backend.wait_for_object_settle()
        self._capture_grasp_metrics(PickPlaceState.PLACE)

        self.state = PickPlaceState.RETREAT
        report = self._move_to_pose(pre_place, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
                plan_result=plan_result,
            )

        self._refresh_execution_state()
        placement_stable = self.backend.object_is_stable_in_target(
            task.target_position
        )
        collision_count = (
            self.backend.collision_count - episode_start_collisions
        )
        collision_state_known = bool(
            getattr(self.backend, "collision_state_known", True)
        )
        object_state_known = bool(
            getattr(self.backend, "object_state_known", True)
        )
        invalid_collision = bool(
            getattr(
                self.backend,
                "invalid_collision_detected",
                collision_count > 0,
            )
        )
        release_confirmed = bool(
            self._object_was_released
            and getattr(self.backend, "object_released_confirmed", True)
        )
        task_success = (
            release_confirmed
            and object_state_known
            and placement_stable
            and collision_state_known
            and not invalid_collision
            and collision_count == 0
        )
        final_failure_type = self._final_failure_type(
            released=release_confirmed,
            release_failure_type=str(
                getattr(self.backend, "last_failure_type", "")
            ),
            object_state_known=object_state_known,
            placement_stable=placement_stable,
            collision_state_known=collision_state_known,
            invalid_collision=invalid_collision or collision_count > 0,
            collision_failure_type=str(
                getattr(
                    self.backend,
                    "execution_collision_failure_type",
                    "",
                )
            ),
        )
        self.state = PickPlaceState.DONE if task_success else PickPlaceState.FAILED
        object_final_position, object_final_velocity = self.backend.object_state()
        xy_error, z_error = self._placement_errors(object_final_position, task.target_position)
        final_speed = self._speed(object_final_velocity)
        backend_name = str(getattr(self.backend, "backend_name", "pybullet"))
        simulator_name = str(
            getattr(self.backend, "simulator_name", backend_name)
        )
        planner_id = str(getattr(self.backend, "planner_id", task.planner))
        return EpisodeResult(
            episode_id=task.episode_id,
            task_seed=task.task_seed,
            planner=task.planner,
            planner_seed=task.planner_seed,
            scene=task.scene,
            planning_success=True,
            planning_time_sec=plan_result.planning_time_sec,
            explored_nodes=plan_result.explored_nodes,
            path_waypoints=self._trajectory_points(transfer_report),
            path_length_rad=self._path_length(transfer_report),
            execution_sim_steps=self.backend.simulation_steps - episode_start_steps,
            task_success=task_success,
            collision_count=collision_count,
            failure_type=final_failure_type,
            object_initial_position=object_initial_position,
            object_final_position=object_final_position,
            target_position=task.target_position,
            backend=backend_name,
            simulator=simulator_name,
            planner_id=planner_id,
            grasp_mode=task.grasp_mode,
            execution_time_sec=self._total_execution_time(),
            execution_success=not invalid_collision and collision_count == 0,
            trajectory_points=self._trajectory_points(transfer_report),
            max_joint_tracking_error_rad=self._max_tracking_error(),
            final_xy_error_m=xy_error,
            final_z_error_m=z_error,
            final_object_speed_mps=final_speed,
            planning_collision=False,
            execution_collision=invalid_collision or collision_count > 0,
            object_attached=self._object_was_attached,
            object_released=release_confirmed,
            planning_scene_sync_success=bool(
                getattr(self.backend, "planning_scene_sync_success", True)
            ),
            failure_state="" if task_success else PickPlaceState.PLACE.value,
            stage_metrics=self._stage_metrics,
        )

    def _move_to_pose(
        self,
        position,
        orientation,
        *,
        planner: str = "linear",
        seed: int = 0,
        timeout_sec: float = 1.0,
    ) -> BackendMotionReport:
        report = self.backend.move_to_pose(
            position,
            orientation,
            planner=planner,
            seed=seed,
            timeout_sec=timeout_sec,
        )
        plan = report.plan_result
        metrics = self._stage_metrics.setdefault(
            self.state.value,
            {
                "planning_success": True,
                "planning_time_sec": 0.0,
                "path_waypoints": 0,
                "path_length_rad": 0.0,
                "execution_time_sec": 0.0,
                "execution_sim_steps": 0,
                "collision_count": 0,
                "failure_type": "",
                "moveit_error_code": 0,
                "max_joint_tracking_error_rad": None,
                "planning_scene_sync_success": True,
                "motion_count": 0,
            },
        )
        metrics["planning_success"] = bool(
            metrics["planning_success"] and report.planning_success
        )
        metrics["planning_time_sec"] += plan.planning_time_sec
        metrics["path_waypoints"] += self._trajectory_points(report)
        metrics["path_length_rad"] += self._path_length(report)
        metrics["execution_time_sec"] += report.execution_time_sec
        metrics["execution_sim_steps"] += report.simulation_steps
        metrics["collision_count"] += report.collision_count
        metrics["failure_type"] = report.failure_type
        metrics["moveit_error_code"] = report.moveit_error_code
        tracking_error = report.max_joint_tracking_error_rad
        if tracking_error is not None:
            previous_error = metrics["max_joint_tracking_error_rad"]
            metrics["max_joint_tracking_error_rad"] = max(
                float(previous_error or 0.0), float(tracking_error)
            )
        metrics["planning_scene_sync_success"] = bool(
            metrics["planning_scene_sync_success"]
            and report.planning_scene_sync_success
        )
        metrics["motion_count"] += 1
        return report

    def _failure(
        self,
        task: TaskSpec,
        object_initial_position: tuple[float, float, float],
        failure_type: str,
        episode_start_steps: int,
        episode_start_collisions: int,
        plan_result: PlanResult | None = None,
    ) -> EpisodeResult:
        failure_state = self.state.value
        self.state = PickPlaceState.FAILED
        self._refresh_execution_state()
        object_final_position, object_final_velocity = self.backend.object_state()
        xy_error, z_error = self._placement_errors(object_final_position, task.target_position)
        final_speed = self._speed(object_final_velocity)
        plan = plan_result or PlanResult(False, failure_type=failure_type)
        stage_metrics = self._stage_metrics.get(failure_state, {})
        backend_name = str(getattr(self.backend, "backend_name", "pybullet"))
        simulator_name = str(
            getattr(self.backend, "simulator_name", backend_name)
        )
        planner_id = str(getattr(self.backend, "planner_id", task.planner))
        return EpisodeResult(
            episode_id=task.episode_id,
            task_seed=task.task_seed,
            planner=task.planner,
            planner_seed=task.planner_seed,
            scene=task.scene,
            planning_success=bool(
                stage_metrics.get("planning_success", plan.success)
            ),
            planning_time_sec=float(
                stage_metrics.get("planning_time_sec", plan.planning_time_sec)
            ),
            explored_nodes=plan.explored_nodes,
            path_waypoints=int(
                stage_metrics.get("path_waypoints", len(plan.path))
            ),
            path_length_rad=float(
                stage_metrics.get("path_length_rad", path_length(plan.path))
            ),
            execution_sim_steps=self.backend.simulation_steps - episode_start_steps,
            task_success=False,
            collision_count=self.backend.collision_count - episode_start_collisions,
            failure_type=failure_type,
            object_initial_position=object_initial_position,
            object_final_position=object_final_position,
            target_position=task.target_position,
            backend=backend_name,
            simulator=simulator_name,
            planner_id=planner_id,
            grasp_mode=task.grasp_mode,
            execution_time_sec=self._total_execution_time(),
            execution_success=self._execution_success(failure_type),
            trajectory_points=int(
                stage_metrics.get("path_waypoints", len(plan.path))
            ),
            max_joint_tracking_error_rad=self._max_tracking_error(),
            final_xy_error_m=xy_error,
            final_z_error_m=z_error,
            final_object_speed_mps=final_speed,
            planning_collision=self._is_planning_collision(failure_type),
            execution_collision=self._is_execution_collision(failure_type),
            object_attached=self._object_was_attached,
            object_released=self._object_was_released,
            planning_scene_sync_success=bool(
                getattr(self.backend, "planning_scene_sync_success", True)
            ),
            failure_state=failure_state,
            stage_metrics=self._stage_metrics,
        )

    @staticmethod
    def _path_length(report: BackendMotionReport) -> float:
        if report.path_length_rad is not None:
            return float(report.path_length_rad)
        return path_length(report.plan_result.path)

    @staticmethod
    def _trajectory_points(report: BackendMotionReport) -> int:
        if report.trajectory_points is not None:
            return int(report.trajectory_points)
        return len(report.plan_result.path)

    def _total_execution_time(self) -> float:
        return sum(
            float(metrics.get("execution_time_sec", 0.0))
            for metrics in self._stage_metrics.values()
        )

    def _max_tracking_error(self) -> float | None:
        values = [
            metrics.get("max_joint_tracking_error_rad")
            for metrics in self._stage_metrics.values()
        ]
        backend_value = getattr(
            self.backend,
            "max_joint_tracking_error_rad",
            None,
        )
        if backend_value is not None:
            values.append(backend_value)
        defined = [float(value) for value in values if value is not None]
        return max(defined) if defined else None

    def _capture_grasp_metrics(self, state: PickPlaceState) -> None:
        provider = getattr(self.backend, "grasp_metrics", None)
        if not callable(provider):
            return
        values = provider()
        if not values:
            return
        metrics = self._stage_metrics.setdefault(state.value, {})
        metrics.update(values)

    def _refresh_execution_state(self) -> None:
        provider = getattr(self.backend, "refresh_execution_state", None)
        if callable(provider):
            provider()

    @staticmethod
    def _speed(velocity) -> float:
        return math.sqrt(sum(float(value) ** 2 for value in velocity))

    @staticmethod
    def _is_planning_collision(failure_type: str) -> bool:
        return failure_type in {
            "planning_collision",
            "planning_carried_object_collision",
            "start_state_collision",
            "goal_collision",
        }

    @staticmethod
    def _is_execution_collision(failure_type: str) -> bool:
        return failure_type in {
            "execution_collision",
            "execution_carried_object_collision",
            "robot_self_collision",
            "robot_object_collision",
            "carried_object_environment_collision",
            "object_environment_collision",
            "robot_environment_collision",
            "unknown_monitored_collision",
        }

    @classmethod
    def _execution_success(cls, failure_type: str) -> bool:
        # Motions may all execute correctly and still fail the task-level place
        # or observation criterion.  Keep those concepts separate in results.
        return failure_type in {
            "place_failed",
            "collision_state_unavailable",
            "object_state_unavailable",
        }

    @staticmethod
    def _final_failure_type(
        *,
        released: bool,
        release_failure_type: str,
        object_state_known: bool,
        placement_stable: bool,
        collision_state_known: bool,
        invalid_collision: bool,
        collision_failure_type: str = "",
    ) -> str:
        if not released:
            return release_failure_type or "release_failed"
        if not object_state_known:
            return "object_state_unavailable"
        if not collision_state_known:
            return "collision_state_unavailable"
        if invalid_collision:
            return collision_failure_type or "execution_collision"
        if not placement_stable:
            return "place_failed"
        return ""

    @staticmethod
    def _placement_errors(
        object_position: tuple[float, float, float],
        target_position: tuple[float, float, float],
    ) -> tuple[float, float]:
        return (
            math.dist(object_position[:2], target_position[:2]),
            abs(object_position[2] - target_position[2]),
        )
