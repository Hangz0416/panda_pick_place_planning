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
        estimates = {"target": SceneObjectEstimate("target", task.target_position)}
        for workpiece in task.workpieces:
            estimates[workpiece.object_id] = SceneObjectEstimate(
                workpiece.object_id,
                workpiece.position,
                object_class="workpiece",
                orientation_xyzw=workpiece.orientation_xyzw,
            )
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
    """Synchronous adapter whose estimates are the RGB-D control authority.

    ``TaskSpec`` is passed only to identify the episode and required objects;
    this provider never falls back to task-truth poses after a perception
    failure.
    """

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
        request.task_seed = task.task_seed
        request.scene = task.scene
        request.object_variant = task.object_variant
        request.expected_workpiece_count = task.workpiece_count
        request.required_object_ids = ["target"]
        if task.workpiece_count == 1:
            request.required_object_ids.insert(0, task.workpieces[0].object_id)
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
            required = set(request.required_object_ids)
            if task.workpiece_count > 1:
                required.update(item.object_id for item in task.workpieces)
            missing = required - estimates.keys()
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
        orientation = message.pose.pose.orientation
        return SceneObjectEstimate(
            object_id=str(message.object_id),
            position=(float(position.x), float(position.y), float(position.z)),
            object_class=str(message.object_class or message.object_id),
            position_valid=bool(message.position_valid),
            orientation_valid=bool(message.orientation_valid),
            failure_type=str(message.failure_type),
            confidence=float(message.confidence),
            valid_pixel_count=int(message.valid_pixel_count),
            covariance=tuple(float(value) for value in message.pose.covariance),
            orientation_xyzw=(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
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
