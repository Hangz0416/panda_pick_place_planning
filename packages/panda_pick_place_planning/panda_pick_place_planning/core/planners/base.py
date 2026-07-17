from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Protocol, Sequence

from panda_pick_place_planning.core.schemas import PlanResult


JointConfig = list[float]
StateValidator = Callable[[Sequence[float]], bool]


@dataclass(frozen=True)
class PlanningRequest:
    start: JointConfig
    goal: JointConfig
    joint_limits: list[tuple[float, float]]
    state_validator: StateValidator
    collision_resolution: float = 0.05
    step_size: float = 0.25
    timeout_sec: float = 2.0
    max_iterations: int = 4000
    goal_bias: float = 0.1
    seed: int = 0

    def validate(self) -> None:
        dimensions = len(self.start)
        if dimensions == 0 or len(self.goal) != dimensions or len(self.joint_limits) != dimensions:
            raise ValueError("start, goal and joint_limits must have the same non-zero dimension")
        if self.collision_resolution <= 0 or self.step_size <= 0:
            raise ValueError("planning resolutions must be positive")
        if self.timeout_sec <= 0 or self.max_iterations <= 0:
            raise ValueError("planning budget must be positive")


class Planner(Protocol):
    def plan(self, request: PlanningRequest) -> PlanResult: ...


def distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(first, second)))


def interpolate_segment(
    start: Sequence[float],
    goal: Sequence[float],
    resolution: float,
) -> list[JointConfig]:
    segment_length = distance(start, goal)
    steps = max(1, math.ceil(segment_length / resolution))
    return [
        [left + (right - left) * index / steps for left, right in zip(start, goal)]
        for index in range(steps + 1)
    ]


def edge_is_valid(
    start: Sequence[float],
    goal: Sequence[float],
    resolution: float,
    state_validator: StateValidator,
) -> bool:
    return all(state_validator(config) for config in interpolate_segment(start, goal, resolution))


def within_joint_limits(config: Sequence[float], limits: Sequence[tuple[float, float]]) -> bool:
    return len(config) == len(limits) and all(
        lower <= value <= upper for value, (lower, upper) in zip(config, limits)
    )


def path_length(path: Sequence[Sequence[float]]) -> float:
    return sum(distance(path[index - 1], path[index]) for index in range(1, len(path)))
