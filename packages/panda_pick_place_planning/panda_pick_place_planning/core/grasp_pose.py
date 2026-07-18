"""Deterministic top-grasp candidates driven by a perceived planar pose."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from panda_pick_place_planning.core.planners.base import path_length
from panda_pick_place_planning.core.schemas import ObjectSelectionMetric, SceneObjectEstimate


@dataclass(frozen=True)
class GraspPoseCandidate:
    index: int
    source: str
    yaw_rad: float
    orientation_xyzw: tuple[float, float, float, float]
    pre_grasp_position: tuple[float, float, float]
    grasp_position: tuple[float, float, float]
    retreat_position: tuple[float, float, float]
    object_id: str = "cube"
    perception_confidence: float = 1.0


@dataclass(frozen=True)
class GraspSelection:
    success: bool
    candidate: GraspPoseCandidate | None = None
    candidate_count: int = 0
    path_length_rad: float | None = None
    failure_type: str = ""
    feasible_candidate_count: int = 0
    selected_object_id: str = ""
    selection_score: float | None = None
    object_metrics: tuple[ObjectSelectionMetric, ...] = ()
    feasible_candidate_keys: tuple[tuple[str, int], ...] = ()


class GraspPoseGenerator:
    """Generate top grasps while preserving the legacy zero-yaw orientation."""

    def generate(
        self,
        estimate: SceneObjectEstimate,
        object_variant: str,
        base_downward_xyzw: Sequence[float],
        *,
        hover_height: float,
        grasp_height: float,
    ) -> list[GraspPoseCandidate]:
        raw_base = tuple(float(value) for value in base_downward_xyzw)
        base = _normalized_quaternion(raw_base)
        if object_variant == "cube":
            yaws = (0.0,)
        elif object_variant == "rect_prism":
            yaw = normalize_symmetric_yaw(estimate.yaw_rad)
            yaws = (yaw, yaw + math.pi)
        else:
            raise ValueError(f"unsupported object variant: {object_variant}")
        x_value, y_value, _ = estimate.position
        candidates = []
        for index, yaw in enumerate(yaws):
            orientation = (
                raw_base
                if yaw == 0.0
                else quaternion_multiply(yaw_quaternion(yaw), base)
            )
            candidates.append(
                GraspPoseCandidate(
                    index=index,
                    source=(
                        "legacy_cube_top_grasp"
                        if object_variant == "cube"
                        else (
                            "rgbd_rect_prism_yaw"
                            if index == 0
                            else "rgbd_rect_prism_yaw_pi_symmetric"
                        )
                    ),
                    yaw_rad=yaw,
                    orientation_xyzw=orientation,
                    pre_grasp_position=(x_value, y_value, float(hover_height)),
                    grasp_position=(x_value, y_value, float(grasp_height)),
                    retreat_position=(x_value, y_value, float(hover_height)),
                    object_id=estimate.object_id,
                    perception_confidence=estimate.confidence,
                )
            )
        return candidates


def select_feasible_grasp(
    candidates: Sequence[GraspPoseCandidate],
    backend,
    *,
    planner: str,
    seed: int,
    timeout_sec: float,
) -> GraspSelection:
    """Check pre-grasp and grasp goals without execution, then rank candidates."""
    feasible = []
    collision_failures = 0
    per_object: dict[str, dict[str, object]] = {}
    for candidate_number, candidate in enumerate(candidates):
        metric = per_object.setdefault(
            candidate.object_id,
            {
                "confidence": candidate.perception_confidence,
                "candidate_count": 0,
                "feasible_count": 0,
                "best_path": None,
                "best_score": None,
            },
        )
        metric["candidate_count"] = int(metric["candidate_count"]) + 1
        report = backend.check_pose_feasibility(
            candidate.pre_grasp_position,
            candidate.orientation_xyzw,
            planner=planner,
            seed=seed + candidate_number,
            timeout_sec=timeout_sec,
        )
        if not report.success:
            if "collision" in str(report.failure_type):
                collision_failures += 1
            continue
        grasp_report = backend.check_pose_feasibility(
            candidate.grasp_position,
            candidate.orientation_xyzw,
            planner=planner,
            seed=seed + len(candidates) + candidate_number,
            timeout_sec=timeout_sec,
        )
        if not grasp_report.success:
            if "collision" in str(grasp_report.failure_type):
                collision_failures += 1
            continue
        length = report.path_length_rad
        if length is None:
            length = path_length(report.plan_result.path)
        score = float(length) + 1.0 * (1.0 - candidate.perception_confidence)
        metric["feasible_count"] = int(metric["feasible_count"]) + 1
        if metric["best_score"] is None or score < float(metric["best_score"]):
            metric["best_path"] = float(length)
            metric["best_score"] = score
        feasible.append(
            (score, candidate.object_id, candidate.index, float(length), candidate)
        )
    object_metrics = tuple(
        ObjectSelectionMetric(
            object_id=object_id,
            confidence=float(values["confidence"]),
            candidate_count=int(values["candidate_count"]),
            feasible_candidate_count=int(values["feasible_count"]),
            best_path_length_rad=values["best_path"],
            selection_score=values["best_score"],
            selected=False,
        )
        for object_id, values in sorted(per_object.items())
    )
    if feasible:
        score, object_id, _, length, candidate = min(
            feasible, key=lambda item: (item[0], item[1], item[2])
        )
        object_metrics = tuple(
            ObjectSelectionMetric(
                **{**metric.__dict__, "selected": metric.object_id == object_id}
            )
            for metric in object_metrics
        )
        return GraspSelection(
            True,
            candidate=candidate,
            candidate_count=len(candidates),
            path_length_rad=length,
            feasible_candidate_count=len(feasible),
            selected_object_id=object_id,
            selection_score=score,
            object_metrics=object_metrics,
            feasible_candidate_keys=tuple(
                (item[4].object_id, item[4].index) for item in feasible
            ),
        )
    failure = (
        "all_grasp_candidates_in_collision"
        if candidates and collision_failures == len(candidates)
        else "no_feasible_grasp_candidate"
    )
    return GraspSelection(
        False,
        candidate_count=len(candidates),
        failure_type=failure,
        feasible_candidate_count=0,
        object_metrics=object_metrics,
    )


def normalize_symmetric_yaw(yaw_rad: float) -> float:
    return (float(yaw_rad) + math.pi / 2.0) % math.pi - math.pi / 2.0


def yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def quaternion_multiply(
    left_xyzw: Sequence[float], right_xyzw: Sequence[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = (float(value) for value in left_xyzw)
    rx, ry, rz, rw = (float(value) for value in right_xyzw)
    return _normalized_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def _normalized_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    quaternion = tuple(float(value) for value in values)
    if len(quaternion) != 4:
        raise ValueError("quaternion must contain four values")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)
