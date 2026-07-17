from __future__ import annotations

from dataclasses import dataclass
import random
import time
from typing import Sequence

from panda_pick_place_planning.core.planners.base import (
    JointConfig,
    PlanningRequest,
    distance,
    edge_is_valid,
    within_joint_limits,
)
from panda_pick_place_planning.core.schemas import PlanResult


TRAPPED = "trapped"
ADVANCED = "advanced"
REACHED = "reached"


@dataclass
class _Node:
    config: JointConfig
    parent: int | None


@dataclass
class _Tree:
    root_kind: str
    nodes: list[_Node]


class RRTConnectPlanner:
    name = "rrt_connect"

    def plan(self, request: PlanningRequest) -> PlanResult:
        request.validate()
        started_at = time.perf_counter()
        if not within_joint_limits(request.start, request.joint_limits):
            return self._failure(started_at, "invalid_start")
        if not within_joint_limits(request.goal, request.joint_limits):
            return self._failure(started_at, "invalid_goal")
        if not request.state_validator(request.start):
            return self._failure(started_at, "invalid_start")
        if not request.state_validator(request.goal):
            return self._failure(started_at, "invalid_goal")
        if edge_is_valid(
            request.start,
            request.goal,
            request.collision_resolution,
            request.state_validator,
        ):
            return PlanResult(
                success=True,
                path=[list(request.start), list(request.goal)],
                planning_time_sec=time.perf_counter() - started_at,
                explored_nodes=2,
            )

        rng = random.Random(request.seed)
        first = _Tree("start", [_Node(list(request.start), None)])
        second = _Tree("goal", [_Node(list(request.goal), None)])

        for _ in range(request.max_iterations):
            if time.perf_counter() - started_at >= request.timeout_sec:
                return self._failure(
                    started_at,
                    "planning_timeout",
                    len(first.nodes) + len(second.nodes),
                )

            target = self._sample(request, rng, second.nodes[0].config)
            status, first_index = self._extend(first, target, request)
            if status != TRAPPED and first_index is not None:
                reached, second_index = self._connect(
                    second,
                    first.nodes[first_index].config,
                    request,
                )
                if reached == REACHED and second_index is not None:
                    path = self._join_paths(first, first_index, second, second_index)
                    path = self._shortcut(path, request, rng)
                    return PlanResult(
                        success=True,
                        path=path,
                        planning_time_sec=time.perf_counter() - started_at,
                        explored_nodes=len(first.nodes) + len(second.nodes),
                    )
            first, second = second, first

        return self._failure(
            started_at,
            "planning_timeout",
            len(first.nodes) + len(second.nodes),
        )

    @staticmethod
    def _sample(
        request: PlanningRequest,
        rng: random.Random,
        biased_target: Sequence[float],
    ) -> JointConfig:
        if rng.random() < request.goal_bias:
            return list(biased_target)
        return [rng.uniform(lower, upper) for lower, upper in request.joint_limits]

    def _extend(
        self,
        tree: _Tree,
        target: Sequence[float],
        request: PlanningRequest,
    ) -> tuple[str, int | None]:
        nearest_index = min(
            range(len(tree.nodes)),
            key=lambda index: distance(tree.nodes[index].config, target),
        )
        nearest = tree.nodes[nearest_index].config
        target_distance = distance(nearest, target)
        if target_distance <= request.step_size:
            candidate = list(target)
            status = REACHED
        else:
            ratio = request.step_size / target_distance
            candidate = [left + (right - left) * ratio for left, right in zip(nearest, target)]
            status = ADVANCED

        if not within_joint_limits(candidate, request.joint_limits):
            return TRAPPED, None
        if not edge_is_valid(
            nearest,
            candidate,
            request.collision_resolution,
            request.state_validator,
        ):
            return TRAPPED, None

        tree.nodes.append(_Node(candidate, nearest_index))
        return status, len(tree.nodes) - 1

    def _connect(
        self,
        tree: _Tree,
        target: Sequence[float],
        request: PlanningRequest,
    ) -> tuple[str, int | None]:
        while True:
            status, node_index = self._extend(tree, target, request)
            if status != ADVANCED:
                return status, node_index

    @staticmethod
    def _trace(tree: _Tree, node_index: int) -> list[JointConfig]:
        path = []
        current: int | None = node_index
        while current is not None:
            node = tree.nodes[current]
            path.append(node.config)
            current = node.parent
        path.reverse()
        return path

    def _join_paths(
        self,
        first: _Tree,
        first_index: int,
        second: _Tree,
        second_index: int,
    ) -> list[JointConfig]:
        first_path = self._trace(first, first_index)
        second_path = self._trace(second, second_index)
        if first.root_kind == "start":
            start_path, goal_path = first_path, second_path
        else:
            start_path, goal_path = second_path, first_path
        return start_path + list(reversed(goal_path[:-1]))

    def _shortcut(
        self,
        path: list[JointConfig],
        request: PlanningRequest,
        rng: random.Random,
        attempts: int = 80,
    ) -> list[JointConfig]:
        simplified = [list(config) for config in path]
        for _ in range(attempts):
            if len(simplified) < 3:
                break
            left, right = sorted(rng.sample(range(len(simplified)), 2))
            if right - left < 2:
                continue
            if edge_is_valid(
                simplified[left],
                simplified[right],
                request.collision_resolution,
                request.state_validator,
            ):
                simplified = simplified[: left + 1] + simplified[right:]
        return simplified

    @staticmethod
    def _failure(started_at: float, failure_type: str, explored_nodes: int = 0) -> PlanResult:
        return PlanResult(
            success=False,
            planning_time_sec=time.perf_counter() - started_at,
            explored_nodes=explored_nodes,
            failure_type=failure_type,
        )
