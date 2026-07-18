"""ROS-independent sequential batch bookkeeping."""

from __future__ import annotations

from dataclasses import dataclass, field

from panda_pick_place_planning.core.schemas import EpisodeResult, TaskSpec


def batch_exit_code(status: str) -> int:
    """Map the persisted terminal status to a shell-visible exit code."""

    if status == "complete":
        return 0
    if status == "timeout":
        return 2
    raise ValueError(f"unknown batch status: {status}")


@dataclass
class BatchSequencer:
    tasks: list[TaskSpec]
    results: list[EpisodeResult] = field(default_factory=list)
    index: int = 0
    inflight: TaskSpec | None = None

    @property
    def done(self) -> bool:
        return self.index >= len(self.tasks) and self.inflight is None

    @property
    def remaining(self) -> int:
        return len(self.tasks) - self.index

    def next_task(self) -> TaskSpec | None:
        if self.inflight is not None:
            raise RuntimeError("an episode is already in flight")
        if self.index >= len(self.tasks):
            return None
        self.inflight = self.tasks[self.index]
        return self.inflight

    def accept(self, result: EpisodeResult) -> bool:
        expected = self.inflight
        if expected is None or not self._matches(expected, result):
            return False
        self.results.append(result)
        self.index += 1
        self.inflight = None
        return True

    @staticmethod
    def _matches(task: TaskSpec, result: EpisodeResult) -> bool:
        return (
            result.episode_id == task.episode_id
            and result.task_seed == task.task_seed
            and result.planner_seed == task.planner_seed
            and result.scene == task.scene
            and result.backend == task.backend
            and result.simulator == task.simulator
            and result.object_variant == task.object_variant
            and result.perception_profile == task.perception_profile
        )
