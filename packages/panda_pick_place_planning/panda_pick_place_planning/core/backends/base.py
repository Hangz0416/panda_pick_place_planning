from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, Sequence, runtime_checkable

from panda_pick_place_planning.core.schemas import PlanResult, TaskSpec


class BackendOperationError(RuntimeError):
    """Typed failure raised at a backend boundary outside a motion report."""

    def __init__(self, failure_type: str, detail: str = "") -> None:
        self.failure_type = failure_type
        self.detail = detail
        super().__init__(f"{failure_type}: {detail}" if detail else failure_type)


@dataclass(frozen=True)
class BackendMotionReport:
    """Backend-neutral outcome of planning and optionally executing one motion."""

    success: bool
    planning_success: bool
    simulation_steps: int = 0
    collision_count: int = 0
    failure_type: str = ""
    plan_result: PlanResult = field(default_factory=lambda: PlanResult(False))
    execution_time_sec: float = 0.0
    path_length_rad: float | None = None
    trajectory_points: int | None = None
    moveit_error_code: int = 0
    max_joint_tracking_error_rad: float | None = None
    planning_scene_sync_success: bool = True


@runtime_checkable
class ManipulationBackend(Protocol):
    """Runtime contract shared by simulation and future MoveIt backends."""

    @property
    def hover_height(self) -> float: ...

    @property
    def grasp_height(self) -> float: ...

    @property
    def place_height(self) -> float: ...

    @property
    def open_gripper_width(self) -> float: ...

    @property
    def closed_gripper_width(self) -> float: ...

    @property
    def simulation_steps(self) -> int: ...

    @property
    def collision_count(self) -> int: ...

    def reset(self, task: TaskSpec) -> None: ...

    def move_to_pose(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        planner: str = "linear",
        seed: int = 0,
        timeout_sec: float = 1.0,
    ) -> BackendMotionReport: ...

    def check_pose_feasibility(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        planner: str = "rrt_connect",
        seed: int = 0,
        timeout_sec: float = 3.0,
    ) -> BackendMotionReport: ...

    def move_home(self) -> BackendMotionReport: ...

    def set_gripper(self, width: float) -> None: ...

    def attach_object(self) -> bool: ...

    def detach_object(self) -> None: ...

    def wait_for_object_settle(self) -> None: ...

    def object_state(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]: ...

    def object_is_stable_in_target(self, target_position: Sequence[float]) -> bool: ...

    def downward_orientation(self) -> Sequence[float]: ...

    def shutdown(self) -> None: ...
