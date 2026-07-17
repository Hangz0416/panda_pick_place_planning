from __future__ import annotations

import time
from typing import Sequence

from panda_pick_place_planning.core.backends.base import BackendMotionReport
from panda_pick_place_planning.core.planners import LinearPlanner, RRTConnectPlanner
from panda_pick_place_planning.core.planners.base import PlanningRequest
from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv, SceneConfig
from panda_pick_place_planning.core.schemas import PlanResult, TaskSpec


class PyBulletBackend:
    """Adapter exposing :class:`PyBulletPandaEnv` through the common contract."""

    backend_name = "pybullet"
    simulator_name = "pybullet"
    planner_id = ""
    collision_state_known = True

    def __init__(
        self,
        env: PyBulletPandaEnv | None = None,
        *,
        use_gui: bool = False,
        config: SceneConfig | None = None,
    ) -> None:
        if env is not None and config is not None:
            raise ValueError("pass either env or config, not both")
        self.env = env or PyBulletPandaEnv(use_gui=use_gui, config=config)

    @property
    def hover_height(self) -> float:
        return self.env.config.hover_height

    @property
    def grasp_height(self) -> float:
        return self.env.config.grasp_height

    @property
    def place_height(self) -> float:
        return self.env.config.place_height

    @property
    def open_gripper_width(self) -> float:
        return self.env.config.open_gripper_width

    @property
    def closed_gripper_width(self) -> float:
        return self.env.config.closed_gripper_width

    @property
    def simulation_steps(self) -> int:
        return self.env.simulation_steps

    @property
    def collision_count(self) -> int:
        return self.env.collision_count

    @property
    def invalid_collision_detected(self) -> bool:
        return self.env.collision_count > 0

    def reset(self, task: TaskSpec) -> None:
        self.planner_id = task.planner
        self.env.reset(task)

    def move_to_pose(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        planner: str = "linear",
        seed: int = 0,
        timeout_sec: float = 1.0,
    ) -> BackendMotionReport:
        solution = self.env.solve_ik(position, orientation)
        if not solution.success:
            return self._failure("ik_failed")
        return self._plan_and_execute(
            solution.joints,
            planner=planner,
            seed=seed,
            timeout_sec=timeout_sec,
        )

    def move_home(self) -> BackendMotionReport:
        return self._plan_and_execute(
            list(self.env.config.home_joints),
            planner="linear",
            seed=0,
            timeout_sec=1.0,
        )

    def set_gripper(self, width: float) -> None:
        self.env.set_gripper(width)

    def attach_object(self) -> bool:
        return self.env.attach_object()

    def detach_object(self) -> None:
        self.env.detach_object()

    def wait_for_object_settle(self) -> None:
        self.env.step(self.env.config.settle_steps, check_collisions=False)

    def object_state(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        return self.env.object_state()

    def object_is_stable_in_target(self, target_position: Sequence[float]) -> bool:
        return self.env.object_is_stable_in_target(target_position)

    def downward_orientation(self) -> Sequence[float]:
        return self.env.downward_orientation()

    def shutdown(self) -> None:
        self.env.shutdown()

    def _plan_and_execute(
        self,
        goal: list[float],
        *,
        planner: str,
        seed: int,
        timeout_sec: float,
    ) -> BackendMotionReport:
        request = self._planning_request(goal, seed=seed, timeout_sec=timeout_sec)
        if planner == "linear":
            planner_impl = LinearPlanner()
        elif planner == "rrt_connect":
            planner_impl = RRTConnectPlanner()
        else:
            return self._failure("unsupported_planner")
        plan = planner_impl.plan(request)
        if not plan.success:
            return self._failure(self._classify_planning_failure(plan), plan)

        execution_start = time.perf_counter()
        execution = self.env.execute_joint_path(plan.path)
        execution_time_sec = time.perf_counter() - execution_start
        return BackendMotionReport(
            success=execution.success,
            planning_success=True,
            execution_time_sec=execution_time_sec,
            simulation_steps=execution.simulation_steps,
            collision_count=execution.collision_count,
            failure_type=execution.failure_type,
            plan_result=plan,
        )

    def _planning_request(
        self,
        goal: list[float],
        *,
        seed: int,
        timeout_sec: float,
    ) -> PlanningRequest:
        if self.env.collision_checker is None:
            raise RuntimeError("environment_not_reset")
        return PlanningRequest(
            start=self.env.current_arm_joints(),
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

    def _classify_planning_failure(self, plan: PlanResult) -> str:
        checker = self.env.collision_checker
        carried_failure = checker is not None and (
            checker.last_validation_failure == "carried_object_collision"
        )
        if carried_failure and plan.failure_type in {
            "planning_collision",
            "invalid_start",
            "invalid_goal",
        }:
            return "planning_carried_object_collision"
        return plan.failure_type

    @staticmethod
    def _failure(failure_type: str, plan: PlanResult | None = None) -> BackendMotionReport:
        plan_result = plan or PlanResult(False, failure_type=failure_type)
        return BackendMotionReport(
            success=False,
            planning_success=plan_result.success,
            failure_type=failure_type,
            plan_result=plan_result,
        )
