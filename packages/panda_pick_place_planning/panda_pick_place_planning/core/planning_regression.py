"""Honest MoveIt planning-only regression without synthetic task execution."""

from __future__ import annotations

from panda_pick_place_planning.core.backends import (
    BackendMotionReport,
    ManipulationBackend,
)
from panda_pick_place_planning.core.planners.base import path_length
from panda_pick_place_planning.core.schemas import EpisodeResult, TaskSpec


class PlanningRegressionRunner:
    """Plan independent reachability requests from the current mock state.

    No trajectory, gripper, attachment, release, or object motion is implied.
    The resulting episode therefore reports planning success separately while
    keeping ``task_success`` and ``execution_success`` false.
    """

    def __init__(self, backend: ManipulationBackend) -> None:
        self.backend = backend

    def run(self, task: TaskSpec) -> EpisodeResult:
        self.backend.reset(task)
        object_initial, _ = self.backend.object_state()
        orientation = self.backend.downward_orientation()
        stages: list[tuple[str, BackendMotionReport]] = []
        home = getattr(self.backend, "reset_motion_report", None)
        if home is not None:
            stages.append(("home_from_current", home))

        targets = (
            (
                "pre_grasp_from_current",
                (
                    task.object_position[0],
                    task.object_position[1],
                    self.backend.hover_height,
                ),
            ),
            (
                "transfer_from_current",
                (
                    task.target_position[0],
                    task.target_position[1],
                    self.backend.hover_height,
                ),
            ),
            (
                "place_from_current",
                (
                    task.target_position[0],
                    task.target_position[1],
                    self.backend.place_height,
                ),
            ),
        )
        for state, position in targets:
            report = self.backend.move_to_pose(
                position,
                orientation,
                planner=task.planner,
                seed=task.planner_seed,
                timeout_sec=3.0,
            )
            stages.append((state, report))
            if not report.success:
                return self._result(
                    task,
                    object_initial,
                    stages,
                    planning_success=False,
                    failure_type=report.failure_type,
                    failure_state=state,
                )

        return self._result(
            task,
            object_initial,
            stages,
            planning_success=True,
            failure_type="not_executed",
            failure_state="",
        )

    def _result(
        self,
        task: TaskSpec,
        object_initial: tuple[float, float, float],
        stages: list[tuple[str, BackendMotionReport]],
        *,
        planning_success: bool,
        failure_type: str,
        failure_state: str,
    ) -> EpisodeResult:
        metrics = {
            state: self._stage_metrics(report)
            for state, report in stages
        }
        return EpisodeResult(
            episode_id=task.episode_id,
            task_seed=task.task_seed,
            planner=task.planner,
            planner_seed=task.planner_seed,
            scene=task.scene,
            planning_success=planning_success,
            planning_time_sec=sum(
                report.plan_result.planning_time_sec for _, report in stages
            ),
            explored_nodes=sum(
                report.plan_result.explored_nodes for _, report in stages
            ),
            path_waypoints=sum(
                self._trajectory_points(report) for _, report in stages
            ),
            path_length_rad=sum(
                self._path_length(report) for _, report in stages
            ),
            execution_sim_steps=0,
            task_success=False,
            collision_count=0,
            failure_type=failure_type,
            object_initial_position=object_initial,
            object_final_position=object_initial,
            target_position=task.target_position,
            backend=str(getattr(self.backend, "backend_name", "moveit")),
            simulator=str(
                getattr(self.backend, "simulator_name", "moveit_mock")
            ),
            planner_id=str(
                getattr(self.backend, "planner_id", task.planner)
            ),
            grasp_mode=task.grasp_mode,
            execution_time_sec=0.0,
            execution_success=False,
            trajectory_points=sum(
                self._trajectory_points(report) for _, report in stages
            ),
            final_xy_error_m=None,
            final_z_error_m=None,
            final_object_speed_mps=None,
            planning_collision=self._is_planning_collision(failure_type),
            execution_collision=False,
            object_attached=False,
            object_released=False,
            planning_scene_sync_success=bool(
                getattr(
                    self.backend,
                    "planning_scene_sync_success",
                    True,
                )
            ),
            failure_state=failure_state,
            stage_metrics=metrics,
        )

    @classmethod
    def _stage_metrics(
        cls, report: BackendMotionReport
    ) -> dict[str, object]:
        return {
            "evaluation_mode": "independent_planning_only",
            "planning_success": report.planning_success,
            "planning_time_sec": report.plan_result.planning_time_sec,
            "path_waypoints": cls._trajectory_points(report),
            "path_length_rad": cls._path_length(report),
            "execution_time_sec": 0.0,
            "failure_type": report.failure_type,
            "moveit_error_code": report.moveit_error_code,
        }

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

    @staticmethod
    def _is_planning_collision(failure_type: str) -> bool:
        return failure_type in {
            "planning_collision",
            "planning_carried_object_collision",
            "start_state_collision",
            "goal_collision",
        }
