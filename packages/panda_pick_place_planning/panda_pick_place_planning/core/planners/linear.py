from __future__ import annotations

import time

from panda_pick_place_planning.core.planners.base import (
    PlanningRequest,
    interpolate_segment,
    within_joint_limits,
)
from panda_pick_place_planning.core.schemas import PlanResult


class LinearPlanner:
    name = "linear"

    def plan(self, request: PlanningRequest) -> PlanResult:
        request.validate()
        started_at = time.perf_counter()

        if not within_joint_limits(request.start, request.joint_limits):
            return self._failure(started_at, "invalid_start")
        if not within_joint_limits(request.goal, request.joint_limits):
            return self._failure(started_at, "invalid_goal")

        path = interpolate_segment(request.start, request.goal, request.collision_resolution)
        for config in path:
            if not request.state_validator(config):
                return self._failure(started_at, "planning_collision")

        return PlanResult(
            success=True,
            path=path,
            planning_time_sec=time.perf_counter() - started_at,
            explored_nodes=len(path),
        )

    @staticmethod
    def _failure(started_at: float, failure_type: str) -> PlanResult:
        return PlanResult(
            success=False,
            planning_time_sec=time.perf_counter() - started_at,
            failure_type=failure_type,
        )
