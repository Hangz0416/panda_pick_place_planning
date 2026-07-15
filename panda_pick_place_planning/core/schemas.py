from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from typing import Any


SUPPORTED_PLANNERS = {"linear", "rrt_connect"}
SUPPORTED_SCENES = {"clear", "obstacle"}


@dataclass(frozen=True)
class TaskSpec:
    episode_id: int
    task_seed: int
    planner: str
    planner_seed: int
    scene: str
    object_position: tuple[float, float, float]
    target_position: tuple[float, float, float]

    def __post_init__(self) -> None:
        if self.planner not in SUPPORTED_PLANNERS:
            raise ValueError(f"unsupported planner: {self.planner}")
        if self.scene not in SUPPORTED_SCENES:
            raise ValueError(f"unsupported scene: {self.scene}")
        if len(self.object_position) != 3 or len(self.target_position) != 3:
            raise ValueError("object_position and target_position must be 3D")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskSpec":
        return cls(
            episode_id=int(data["episode_id"]),
            task_seed=int(data["task_seed"]),
            planner=str(data["planner"]),
            planner_seed=int(data["planner_seed"]),
            scene=str(data["scene"]),
            object_position=tuple(float(value) for value in data["object_position"]),
            target_position=tuple(float(value) for value in data["target_position"]),
        )

    @classmethod
    def from_json(cls, payload: str) -> "TaskSpec":
        return cls.from_dict(json.loads(payload))


@dataclass(frozen=True)
class PlanResult:
    success: bool
    path: list[list[float]] = field(default_factory=list)
    planning_time_sec: float = 0.0
    explored_nodes: int = 0
    failure_type: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class EpisodeResult:
    episode_id: int
    task_seed: int
    planner: str
    planner_seed: int
    scene: str
    planning_success: bool
    planning_time_sec: float
    explored_nodes: int
    path_waypoints: int
    path_length_rad: float
    execution_sim_steps: int
    task_success: bool
    collision_count: int
    failure_type: str
    object_initial_position: tuple[float, float, float]
    object_final_position: tuple[float, float, float]
    target_position: tuple[float, float, float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "EpisodeResult":
        return cls(
            episode_id=int(data["episode_id"]),
            task_seed=int(data["task_seed"]),
            planner=str(data["planner"]),
            planner_seed=int(data["planner_seed"]),
            scene=str(data["scene"]),
            planning_success=bool(data["planning_success"]),
            planning_time_sec=float(data["planning_time_sec"]),
            explored_nodes=int(data["explored_nodes"]),
            path_waypoints=int(data["path_waypoints"]),
            path_length_rad=float(data["path_length_rad"]),
            execution_sim_steps=int(data["execution_sim_steps"]),
            task_success=bool(data["task_success"]),
            collision_count=int(data["collision_count"]),
            failure_type=str(data["failure_type"]),
            object_initial_position=tuple(float(value) for value in data["object_initial_position"]),
            object_final_position=tuple(float(value) for value in data["object_final_position"]),
            target_position=tuple(float(value) for value in data["target_position"]),
        )

    @classmethod
    def from_json(cls, payload: str) -> "EpisodeResult":
        return cls.from_dict(json.loads(payload))
