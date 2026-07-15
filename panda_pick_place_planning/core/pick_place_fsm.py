from __future__ import annotations

from enum import Enum
from typing import Sequence

from panda_pick_place_planning.core.planners import LinearPlanner, RRTConnectPlanner
from panda_pick_place_planning.core.planners.base import PlanningRequest, path_length
from panda_pick_place_planning.core.pybullet_env import ExecutionReport, PyBulletPandaEnv
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
    def __init__(self, env: PyBulletPandaEnv) -> None:
        self.env = env
        self.state = PickPlaceState.RESET

    def run(self, task: TaskSpec) -> EpisodeResult:
        self.state = PickPlaceState.RESET
        self.env.reset(task)
        episode_start_steps = self.env.simulation_steps
        episode_start_collisions = self.env.collision_count
        object_initial_position, _ = self.env.object_state()
        orientation = self.env.downward_orientation()

        self.state = PickPlaceState.PRE_GRASP
        pre_grasp = (
            task.object_position[0],
            task.object_position[1],
            self.env.config.hover_height,
        )
        report = self._move_to_pose_linear(pre_grasp, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
            )

        self.env.open_gripper()
        grasp_pose = (
            task.object_position[0],
            task.object_position[1],
            self.env.config.grasp_height,
        )
        report = self._move_to_pose_linear(grasp_pose, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
            )

        self.state = PickPlaceState.GRASP
        if not self.env.close_gripper_and_attach():
            return self._failure(
                task,
                object_initial_position,
                "grasp_failed",
                episode_start_steps,
                episode_start_collisions,
            )

        self.state = PickPlaceState.LIFT
        lift_pose = (
            task.object_position[0],
            task.object_position[1],
            self.env.config.hover_height,
        )
        report = self._move_to_pose_linear(lift_pose, orientation)
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
            self.env.config.hover_height,
        )
        transfer_goal = self.env.solve_ik(pre_place, orientation)
        if not transfer_goal.success:
            return self._failure(
                task,
                object_initial_position,
                "ik_failed",
                episode_start_steps,
                episode_start_collisions,
            )

        plan_result = self._plan_transfer(task, transfer_goal.joints)
        if not plan_result.success:
            return self._failure(
                task,
                object_initial_position,
                plan_result.failure_type,
                episode_start_steps,
                episode_start_collisions,
                plan_result=plan_result,
            )
        execution = self.env.execute_joint_path(plan_result.path)
        if not execution.success:
            return self._failure(
                task,
                object_initial_position,
                execution.failure_type,
                episode_start_steps,
                episode_start_collisions,
                plan_result=plan_result,
            )

        self.state = PickPlaceState.PLACE
        place_pose = (task.target_position[0], task.target_position[1], self.env.config.place_height)
        report = self._move_to_pose_linear(place_pose, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
                plan_result=plan_result,
            )
        self.env.open_gripper()
        self.env.step(self.env.config.settle_steps, check_collisions=False)

        self.state = PickPlaceState.RETREAT
        report = self._move_to_pose_linear(pre_place, orientation)
        if not report.success:
            return self._failure(
                task,
                object_initial_position,
                report.failure_type,
                episode_start_steps,
                episode_start_collisions,
                plan_result=plan_result,
            )

        task_success = self.env.object_is_stable_in_target(task.target_position)
        self.state = PickPlaceState.DONE if task_success else PickPlaceState.FAILED
        object_final_position, _ = self.env.object_state()
        return EpisodeResult(
            episode_id=task.episode_id,
            task_seed=task.task_seed,
            planner=task.planner,
            planner_seed=task.planner_seed,
            scene=task.scene,
            planning_success=True,
            planning_time_sec=plan_result.planning_time_sec,
            explored_nodes=plan_result.explored_nodes,
            path_waypoints=len(plan_result.path),
            path_length_rad=path_length(plan_result.path),
            execution_sim_steps=self.env.simulation_steps - episode_start_steps,
            task_success=task_success,
            collision_count=self.env.collision_count - episode_start_collisions,
            failure_type="" if task_success else "place_failed",
            object_initial_position=object_initial_position,
            object_final_position=object_final_position,
            target_position=task.target_position,
        )

    def _move_to_pose_linear(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> ExecutionReport:
        solution = self.env.solve_ik(position, orientation)
        if not solution.success:
            return ExecutionReport(False, 0, 0, "ik_failed")
        request = self._planning_request(
            self.env.current_arm_joints(),
            solution.joints,
            seed=0,
            timeout_sec=1.0,
        )
        plan = LinearPlanner().plan(request)
        if not plan.success:
            return ExecutionReport(False, 0, 0, plan.failure_type)
        return self.env.execute_joint_path(plan.path)

    def _plan_transfer(self, task: TaskSpec, goal: list[float]) -> PlanResult:
        request = self._planning_request(
            self.env.current_arm_joints(),
            goal,
            seed=task.planner_seed,
            timeout_sec=3.0,
        )
        planner = LinearPlanner() if task.planner == "linear" else RRTConnectPlanner()
        return planner.plan(request)

    def _planning_request(
        self,
        start: list[float],
        goal: list[float],
        seed: int,
        timeout_sec: float,
    ) -> PlanningRequest:
        if self.env.collision_checker is None:
            raise RuntimeError("environment_not_reset")
        return PlanningRequest(
            start=start,
            goal=goal,
            joint_limits=self.env.joint_limits,
            state_validator=self.env.collision_checker.is_valid,
            collision_resolution=0.045,
            step_size=0.24,
            timeout_sec=timeout_sec,
            max_iterations=5000,
            goal_bias=0.12,
            seed=seed,
        )

    def _failure(
        self,
        task: TaskSpec,
        object_initial_position: tuple[float, float, float],
        failure_type: str,
        episode_start_steps: int,
        episode_start_collisions: int,
        plan_result: PlanResult | None = None,
    ) -> EpisodeResult:
        self.state = PickPlaceState.FAILED
        object_final_position, _ = self.env.object_state()
        plan = plan_result or PlanResult(False, failure_type=failure_type)
        return EpisodeResult(
            episode_id=task.episode_id,
            task_seed=task.task_seed,
            planner=task.planner,
            planner_seed=task.planner_seed,
            scene=task.scene,
            planning_success=plan.success,
            planning_time_sec=plan.planning_time_sec,
            explored_nodes=plan.explored_nodes,
            path_waypoints=len(plan.path),
            path_length_rad=path_length(plan.path),
            execution_sim_steps=self.env.simulation_steps - episode_start_steps,
            task_success=False,
            collision_count=self.env.collision_count - episode_start_collisions,
            failure_type=failure_type,
            object_initial_position=object_initial_position,
            object_final_position=object_final_position,
            target_position=task.target_position,
        )
