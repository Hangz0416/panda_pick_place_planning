"""Pose-source boundary between task truth and manipulation control targets."""

from __future__ import annotations

import threading
import time
from typing import Any, Protocol, Sequence, runtime_checkable

from panda_pick_place_planning.core.schemas import (
    SceneEstimate,
    SceneObjectEstimate,
    TaskSpec,
)


@runtime_checkable
class ScenePoseProvider(Protocol):
    """Return world-frame positions after the backend has reset an episode."""

    pose_source: str

    def estimate(self, task: TaskSpec) -> SceneEstimate: ...


class GroundTruthPoseProvider:
    """Oracle provider used by existing regression and PyBullet workflows."""

    pose_source = "ground_truth"

    def __init__(self, obstacle_position: Sequence[float]) -> None:
        self.obstacle_position = tuple(float(value) for value in obstacle_position)

    def estimate(self, task: TaskSpec) -> SceneEstimate:
        estimates = {
            "cube": SceneObjectEstimate("cube", task.object_position),
            "target": SceneObjectEstimate("target", task.target_position),
        }
        if task.scene == "obstacle":
            estimates["obstacle"] = SceneObjectEstimate(
                "obstacle", self.obstacle_position
            )
        return SceneEstimate(
            episode_id=task.episode_id,
            pose_source=self.pose_source,
            success=True,
            estimates=estimates,
        )


class RGBDScenePoseProvider:
    """Synchronous FSM adapter for the typed RGB-D scene-estimation service."""

    pose_source = "rgbd"

    def __init__(self, client: Any, timeout_sec: float = 3.0) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("perception timeout must be positive")
        self.client = client
        self.timeout_sec = float(timeout_sec)

    def estimate(self, task: TaskSpec) -> SceneEstimate:
        start = time.perf_counter()
        if self.client is None or not self.client.wait_for_service(
            timeout_sec=self.timeout_sec
        ):
            return self._failure(task, "perception_timeout", start)

        from panda_pick_place_interfaces.srv import EstimateScene

        request = EstimateScene.Request()
        request.episode_id = task.episode_id
        request.scene = task.scene
        request.required_object_ids = ["cube", "target"]
        if task.scene == "obstacle":
            request.required_object_ids.append("obstacle")
        request.timeout_sec = self.timeout_sec
        future = self.client.call_async(request)
        response = self._wait_future(future, self.timeout_sec + 0.25)
        if response is None:
            return self._failure(task, "perception_timeout", start)
        if not bool(response.success):
            return SceneEstimate(
                episode_id=task.episode_id,
                pose_source=self.pose_source,
                success=False,
                failure_type=str(response.failure_type or "low_confidence"),
                latency_sec=max(0.0, float(response.latency_sec)),
            )
        try:
            estimates = {
                str(item.object_id): self._from_ros(item)
                for item in response.estimates
            }
            missing = set(request.required_object_ids) - estimates.keys()
            if missing:
                failure = {
                    "cube": "cube_not_detected",
                    "target": "target_not_detected",
                    "obstacle": "obstacle_not_detected",
                }.get(sorted(missing)[0], "low_confidence")
                return self._failure(task, failure, start)
            return SceneEstimate(
                episode_id=task.episode_id,
                pose_source=self.pose_source,
                success=True,
                estimates=estimates,
                latency_sec=max(0.0, float(response.latency_sec)),
            )
        except (AttributeError, TypeError, ValueError):
            return self._failure(task, "low_confidence", start)

    @staticmethod
    def _from_ros(message: Any) -> SceneObjectEstimate:
        position = message.pose.pose.position
        return SceneObjectEstimate(
            object_id=str(message.object_id),
            position=(float(position.x), float(position.y), float(position.z)),
            confidence=float(message.confidence),
            valid_pixel_count=int(message.valid_pixel_count),
            covariance=tuple(float(value) for value in message.pose.covariance),
        )

    def _failure(
        self, task: TaskSpec, failure_type: str, started_at: float
    ) -> SceneEstimate:
        return SceneEstimate(
            episode_id=task.episode_id,
            pose_source=self.pose_source,
            success=False,
            failure_type=failure_type,
            latency_sec=time.perf_counter() - started_at,
        )

    @staticmethod
    def _wait_future(future: Any, timeout_sec: float) -> Any | None:
        if future.done():
            try:
                return future.result()
            except Exception:
                return None
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=max(0.0, timeout_sec)):
            try:
                future.cancel()
            except Exception:
                pass
            return None
        try:
            return future.result()
        except Exception:
            return None
