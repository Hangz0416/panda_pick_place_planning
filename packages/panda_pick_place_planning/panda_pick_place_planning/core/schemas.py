from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from typing import Any


SUPPORTED_PLANNERS = {"linear", "rrt_connect"}
SUPPORTED_SCENES = {"clear", "obstacle"}
SUPPORTED_BACKENDS = {"pybullet", "moveit"}
SUPPORTED_GRASP_MODES = {"assisted", "physics"}
SUPPORTED_POSE_SOURCES = {"ground_truth", "rgbd"}
SUPPORTED_OBJECT_VARIANTS = {"cube", "rect_prism"}
SUPPORTED_PERCEPTION_PROFILES = {
    "nominal", "low_light", "rgb_noise", "depth_dropout", "combined"
}


@dataclass(frozen=True)
class WorkpieceSpec:
    """One reset-time workpiece pose; truth is never used for online selection."""

    object_id: str
    object_variant: str
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        if not self.object_id:
            raise ValueError("workpiece object_id must not be empty")
        if self.object_variant not in SUPPORTED_OBJECT_VARIANTS:
            raise ValueError(f"unsupported object variant: {self.object_variant}")
        _validate_pose_values(self.position, self.orientation_xyzw)

    @property
    def yaw_rad(self) -> float:
        return _quaternion_yaw(self.orientation_xyzw)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "WorkpieceSpec":
        orientation = data.get("orientation_xyzw", (0.0, 0.0, 0.0, 1.0))
        return cls(
            object_id=str(data["object_id"]),
            object_variant=str(data.get("object_variant", "rect_prism")),
            position=tuple(float(value) for value in data["position"]),
            orientation_xyzw=tuple(float(value) for value in orientation),
        )


@dataclass(frozen=True)
class TaskSpec:
    episode_id: int
    task_seed: int
    planner: str
    planner_seed: int
    scene: str
    object_position: tuple[float, float, float]
    target_position: tuple[float, float, float]
    backend: str = "pybullet"
    grasp_mode: str = "assisted"
    simulator: str = ""
    object_variant: str = "cube"
    object_yaw_rad: float = 0.0
    perception_profile: str = "nominal"
    workpieces: tuple[WorkpieceSpec, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.planner not in SUPPORTED_PLANNERS:
            raise ValueError(f"unsupported planner: {self.planner}")
        if self.scene not in SUPPORTED_SCENES:
            raise ValueError(f"unsupported scene: {self.scene}")
        if self.backend not in SUPPORTED_BACKENDS:
            raise ValueError(f"unsupported backend: {self.backend}")
        if self.grasp_mode not in SUPPORTED_GRASP_MODES:
            raise ValueError(f"unsupported grasp mode: {self.grasp_mode}")
        if self.object_variant not in SUPPORTED_OBJECT_VARIANTS:
            raise ValueError(f"unsupported object variant: {self.object_variant}")
        if not math.isfinite(float(self.object_yaw_rad)):
            raise ValueError("object_yaw_rad must be finite")
        if self.perception_profile not in SUPPORTED_PERCEPTION_PROFILES:
            raise ValueError(
                f"unsupported perception profile: {self.perception_profile}"
            )
        if len(self.object_position) != 3 or len(self.target_position) != 3:
            raise ValueError("object_position and target_position must be 3D")
        if not self.workpieces:
            half_yaw = 0.5 * float(self.object_yaw_rad)
            object.__setattr__(
                self,
                "workpieces",
                (
                    WorkpieceSpec(
                        object_id="cube",
                        object_variant=self.object_variant,
                        position=tuple(float(value) for value in self.object_position),
                        orientation_xyzw=(
                            0.0,
                            0.0,
                            math.sin(half_yaw),
                            math.cos(half_yaw),
                        ),
                    ),
                ),
            )
        identifiers = [item.object_id for item in self.workpieces]
        if len(identifiers) not in {1, 3}:
            raise ValueError("workpieces must contain one or three objects")
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("workpiece object_id values must be unique")
        if len(self.workpieces) == 3 and self.object_variant != "rect_prism":
            raise ValueError("three-workpiece tasks require rect_prism objects")
        if any(item.object_variant != self.object_variant for item in self.workpieces):
            raise ValueError("all workpieces must match task object_variant")

    @property
    def workpiece_count(self) -> int:
        return len(self.workpieces)

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
            backend=str(data.get("backend", "pybullet")),
            grasp_mode=str(data.get("grasp_mode", "assisted")),
            simulator=str(data.get("simulator", "")),
            object_variant=str(data.get("object_variant", "cube")),
            object_yaw_rad=float(data.get("object_yaw_rad", 0.0)),
            perception_profile=str(data.get("perception_profile", "nominal")),
            workpieces=tuple(
                WorkpieceSpec.from_dict(item) for item in data.get("workpieces", [])
            ),
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
class SceneObjectEstimate:
    """One world-frame object pose returned by a pose provider."""

    object_id: str
    position: tuple[float, float, float]
    confidence: float = 1.0
    valid_pixel_count: int = 0
    covariance: tuple[float, ...] = (0.0,) * 36
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    object_class: str = ""
    position_valid: bool = True
    orientation_valid: bool = True
    failure_type: str = ""

    def __post_init__(self) -> None:
        if not self.object_id:
            raise ValueError("object_id must not be empty")
        if len(self.position) != 3:
            raise ValueError("position must contain three values")
        if self.position_valid and not all(
            math.isfinite(float(value)) for value in self.position
        ):
            raise ValueError("valid position must contain three finite values")
        if len(self.orientation_xyzw) != 4 or not all(
            math.isfinite(float(value)) for value in self.orientation_xyzw
        ):
            raise ValueError("orientation_xyzw must contain four finite values")
        norm = math.sqrt(sum(float(value) ** 2 for value in self.orientation_xyzw))
        if norm <= 1.0e-9:
            raise ValueError("orientation quaternion must be non-zero")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if int(self.valid_pixel_count) < 0:
            raise ValueError("valid_pixel_count must be non-negative")
        if len(self.covariance) != 36:
            raise ValueError("covariance must contain 36 values")
        if self.failure_type and self.position_valid and self.orientation_valid:
            raise ValueError("fully valid estimate cannot have a failure_type")

    @property
    def yaw_rad(self) -> float:
        return _quaternion_yaw(self.orientation_xyzw)


@dataclass(frozen=True)
class ObjectSelectionMetric:
    object_id: str
    position_valid: bool = True
    orientation_valid: bool = True
    confidence: float = 0.0
    position_error_m: float | None = None
    yaw_error_rad: float | None = None
    candidate_count: int = 0
    feasible_candidate_count: int = 0
    best_path_length_rad: float | None = None
    selection_score: float | None = None
    selected: bool = False
    failure_type: str = ""

    def __post_init__(self) -> None:
        if not self.object_id:
            raise ValueError("selection metric object_id must not be empty")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise ValueError("selection metric confidence must be in [0, 1]")
        for name in (
            "position_error_m",
            "yaw_error_rad",
            "best_path_length_rad",
            "selection_score",
        ):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite, non-negative, or null")
        if self.candidate_count < 0 or self.feasible_candidate_count < 0:
            raise ValueError("candidate counts must be non-negative")
        if self.feasible_candidate_count > self.candidate_count:
            raise ValueError("feasible candidates cannot exceed candidates")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ObjectSelectionMetric":
        return cls(
            object_id=str(data["object_id"]),
            position_valid=bool(data.get("position_valid", True)),
            orientation_valid=bool(data.get("orientation_valid", True)),
            confidence=float(data.get("confidence", 0.0)),
            position_error_m=_optional_float(data.get("position_error_m")),
            yaw_error_rad=_optional_float(data.get("yaw_error_rad")),
            candidate_count=int(data.get("candidate_count", 0)),
            feasible_candidate_count=int(data.get("feasible_candidate_count", 0)),
            best_path_length_rad=_optional_float(data.get("best_path_length_rad")),
            selection_score=_optional_float(data.get("selection_score")),
            selected=bool(data.get("selected", False)),
            failure_type=str(data.get("failure_type", "")),
        )


@dataclass(frozen=True)
class SceneEstimate:
    """Scene localization result consumed by the manipulation state machine."""

    episode_id: int
    pose_source: str
    success: bool
    estimates: dict[str, SceneObjectEstimate] = field(default_factory=dict)
    failure_type: str = ""
    latency_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.pose_source not in SUPPORTED_POSE_SOURCES:
            raise ValueError(f"unsupported pose source: {self.pose_source}")
        if self.success and self.failure_type:
            raise ValueError("successful scene estimate cannot have a failure_type")
        if not self.success and not self.failure_type:
            raise ValueError("failed scene estimate requires a failure_type")
        if not math.isfinite(float(self.latency_sec)) or self.latency_sec < 0.0:
            raise ValueError("latency_sec must be finite and non-negative")
        for key, estimate in self.estimates.items():
            if key != estimate.object_id:
                raise ValueError("estimate dictionary key must match object_id")

    def require(self, object_id: str) -> SceneObjectEstimate:
        try:
            return self.estimates[object_id]
        except KeyError as exc:
            raise ValueError(f"missing required estimate: {object_id}") from exc


@dataclass(frozen=True)
class EpisodeResult:
    """One evaluation result.

    For full pick/place episodes, the legacy top-level planning/path fields keep
    the critical TRANSFER-motion scope; ``stage_metrics`` contains every FSM
    state. Planning-only regression results instead aggregate their explicitly
    independent named/pose requests and identify that scope in batch summaries.
    """

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
    schema_version: int = 5
    backend: str = "pybullet"
    simulator: str = "pybullet"
    planner_id: str = ""
    grasp_mode: str = "assisted"
    pose_source: str = "ground_truth"
    perception_success: bool = True
    perception_latency_sec: float | None = None
    object_position_error_m: float | None = None
    target_position_error_m: float | None = None
    obstacle_position_error_m: float | None = None
    minimum_perception_confidence: float | None = None
    object_variant: str = "cube"
    perception_profile: str = "nominal"
    object_yaw_error_rad: float | None = None
    grasp_selection_success: bool = False
    grasp_selection_latency_sec: float | None = None
    grasp_candidate_count: int = 0
    selected_grasp_yaw_rad: float | None = None
    selected_grasp_path_length_rad: float | None = None
    detected_workpiece_count: int = 1
    selected_object_id: str = "cube"
    target_selection_success: bool = False
    target_selection_latency_sec: float | None = None
    selected_object_score: float | None = None
    total_candidate_count: int = 0
    feasible_candidate_count: int = 0
    selected_object_position_error_m: float | None = None
    selected_object_yaw_error_rad: float | None = None
    max_nonselected_displacement_m: float | None = None
    object_selection_metrics: tuple[ObjectSelectionMetric, ...] = field(
        default_factory=tuple
    )
    execution_time_sec: float = 0.0
    execution_success: bool = False
    trajectory_points: int = 0
    max_joint_tracking_error_rad: float | None = None
    final_xy_error_m: float | None = None
    final_z_error_m: float | None = None
    final_object_speed_mps: float | None = None
    planning_collision: bool = False
    execution_collision: bool = False
    object_attached: bool = False
    object_released: bool = False
    planning_scene_sync_success: bool = True
    failure_state: str = ""
    stage_metrics: dict[str, dict[str, Any]] = field(default_factory=dict)
    physical_left_finger_contact: bool | None = None
    physical_right_finger_contact: bool | None = None
    physical_bilateral_contact: bool | None = None
    physical_slip: bool | None = None
    physical_translation_drift_m: float | None = None
    physical_rotation_drift_rad: float | None = None
    physical_release_stable_time_sec: float | None = None
    last_collision_pair: tuple[str, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        """Promote typed episode metrics already present in stage data."""

        if self.pose_source not in SUPPORTED_POSE_SOURCES:
            raise ValueError(f"unsupported pose source: {self.pose_source}")
        if self.object_variant not in SUPPORTED_OBJECT_VARIANTS:
            raise ValueError(f"unsupported object variant: {self.object_variant}")
        if self.perception_profile not in SUPPORTED_PERCEPTION_PROFILES:
            raise ValueError(
                f"unsupported perception profile: {self.perception_profile}"
            )
        for name in (
            "perception_latency_sec",
            "object_position_error_m",
            "target_position_error_m",
            "obstacle_position_error_m",
            "object_yaw_error_rad",
            "grasp_selection_latency_sec",
            "selected_grasp_path_length_rad",
            "target_selection_latency_sec",
            "selected_object_score",
            "selected_object_position_error_m",
            "selected_object_yaw_error_rad",
            "max_nonselected_displacement_m",
        ):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite, non-negative, or null")
        if self.grasp_candidate_count < 0:
            raise ValueError("grasp_candidate_count must be non-negative")
        if self.detected_workpiece_count < 0:
            raise ValueError("detected_workpiece_count must be non-negative")
        if self.total_candidate_count < 0 or self.feasible_candidate_count < 0:
            raise ValueError("selection candidate counts must be non-negative")
        if self.feasible_candidate_count > self.total_candidate_count:
            raise ValueError("feasible candidates cannot exceed total candidates")
        if self.selected_grasp_yaw_rad is not None and not math.isfinite(
            self.selected_grasp_yaw_rad
        ):
            raise ValueError("selected_grasp_yaw_rad must be finite or null")
        confidence = self.minimum_perception_confidence
        if confidence is not None and not 0.0 <= confidence <= 1.0:
            raise ValueError("minimum_perception_confidence must be in [0, 1]")

        stage_values = list(self.stage_metrics.values())
        contact_fields = {
            "physical_left_finger_contact": "left_finger_contact",
            "physical_right_finger_contact": "right_finger_contact",
        }
        for field_name, metric_name in contact_fields.items():
            if getattr(self, field_name) is not None:
                continue
            values = [
                bool(metrics[metric_name])
                for metrics in stage_values
                if metric_name in metrics
            ]
            if values:
                object.__setattr__(self, field_name, any(values))
        if self.physical_bilateral_contact is None:
            contacts = [
                bool(metrics["left_finger_contact"])
                and bool(metrics["right_finger_contact"])
                for metrics in stage_values
                if "left_finger_contact" in metrics
                and "right_finger_contact" in metrics
            ]
            if contacts:
                object.__setattr__(
                    self, "physical_bilateral_contact", any(contacts)
                )
            elif (
                self.physical_left_finger_contact is not None
                and self.physical_right_finger_contact is not None
            ):
                object.__setattr__(
                    self,
                    "physical_bilateral_contact",
                    self.physical_left_finger_contact
                    and self.physical_right_finger_contact,
                )
        if self.physical_slip is None:
            slip_values = [
                bool(metrics["slipping"])
                for metrics in stage_values
                if "slipping" in metrics
            ]
            if slip_values:
                object.__setattr__(self, "physical_slip", any(slip_values))

        promoted_floats = {
            "physical_translation_drift_m": "max_relative_translation_drift_m",
            "physical_rotation_drift_rad": "max_relative_rotation_drift_rad",
            "physical_release_stable_time_sec": "release_stable_time_sec",
        }
        for field_name, metric_name in promoted_floats.items():
            if getattr(self, field_name) is not None:
                continue
            values = [
                _optional_float(metrics.get(metric_name))
                for metrics in stage_values
                if metric_name in metrics
            ]
            defined = [value for value in values if value is not None]
            if defined:
                object.__setattr__(self, field_name, max(defined))

        if not self.last_collision_pair:
            for metrics in stage_values:
                pair = metrics.get("collision_pair")
                if not isinstance(pair, (list, tuple)):
                    continue
                normalized = tuple(str(value) for value in pair if str(value))
                if normalized:
                    object.__setattr__(self, "last_collision_pair", normalized)

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
            object_initial_position=tuple(
                float(value) for value in data["object_initial_position"]
            ),
            object_final_position=tuple(float(value) for value in data["object_final_position"]),
            target_position=tuple(float(value) for value in data["target_position"]),
            schema_version=int(data.get("schema_version", 1)),
            backend=str(data.get("backend", "pybullet")),
            simulator=str(data.get("simulator", data.get("backend", "pybullet"))),
            planner_id=str(data.get("planner_id", data.get("planner", ""))),
            grasp_mode=str(data.get("grasp_mode", "assisted")),
            pose_source=str(data.get("pose_source", "ground_truth")),
            perception_success=bool(data.get("perception_success", True)),
            perception_latency_sec=_optional_float(
                data.get("perception_latency_sec")
            ),
            object_position_error_m=_optional_float(
                data.get("object_position_error_m")
            ),
            target_position_error_m=_optional_float(
                data.get("target_position_error_m")
            ),
            obstacle_position_error_m=_optional_float(
                data.get("obstacle_position_error_m")
            ),
            minimum_perception_confidence=_optional_float(
                data.get("minimum_perception_confidence")
            ),
            object_variant=str(data.get("object_variant", "cube")),
            perception_profile=str(data.get("perception_profile", "nominal")),
            object_yaw_error_rad=_optional_float(
                data.get("object_yaw_error_rad")
            ),
            grasp_selection_success=bool(
                data.get("grasp_selection_success", False)
            ),
            grasp_selection_latency_sec=_optional_float(
                data.get("grasp_selection_latency_sec")
            ),
            grasp_candidate_count=int(data.get("grasp_candidate_count", 0)),
            selected_grasp_yaw_rad=_optional_float(
                data.get("selected_grasp_yaw_rad")
            ),
            selected_grasp_path_length_rad=_optional_float(
                data.get("selected_grasp_path_length_rad")
            ),
            detected_workpiece_count=int(data.get("detected_workpiece_count", 1)),
            selected_object_id=str(data.get("selected_object_id", "cube")),
            target_selection_success=bool(
                data.get("target_selection_success", False)
            ),
            target_selection_latency_sec=_optional_float(
                data.get("target_selection_latency_sec")
            ),
            selected_object_score=_optional_float(
                data.get("selected_object_score")
            ),
            total_candidate_count=int(data.get("total_candidate_count", 0)),
            feasible_candidate_count=int(
                data.get("feasible_candidate_count", 0)
            ),
            selected_object_position_error_m=_optional_float(
                data.get("selected_object_position_error_m")
            ),
            selected_object_yaw_error_rad=_optional_float(
                data.get("selected_object_yaw_error_rad")
            ),
            max_nonselected_displacement_m=_optional_float(
                data.get("max_nonselected_displacement_m")
            ),
            object_selection_metrics=tuple(
                ObjectSelectionMetric.from_dict(item)
                for item in data.get("object_selection_metrics", [])
            ),
            execution_time_sec=float(data.get("execution_time_sec", 0.0)),
            execution_success=bool(
                data.get("execution_success", data.get("task_success", False))
            ),
            trajectory_points=int(
                data.get("trajectory_points", data.get("path_waypoints", 0))
            ),
            max_joint_tracking_error_rad=(
                _optional_float(data.get("max_joint_tracking_error_rad"))
            ),
            final_xy_error_m=_optional_float(data.get("final_xy_error_m")),
            final_z_error_m=_optional_float(data.get("final_z_error_m")),
            final_object_speed_mps=_optional_float(
                data.get("final_object_speed_mps")
            ),
            planning_collision=bool(data.get("planning_collision", False)),
            execution_collision=bool(data.get("execution_collision", False)),
            object_attached=bool(data.get("object_attached", False)),
            object_released=bool(data.get("object_released", False)),
            planning_scene_sync_success=bool(
                data.get("planning_scene_sync_success", True)
            ),
            failure_state=str(data.get("failure_state", "")),
            stage_metrics={
                str(stage): dict(metrics)
                for stage, metrics in data.get("stage_metrics", {}).items()
            },
            physical_left_finger_contact=_optional_bool(
                data.get("physical_left_finger_contact")
            ),
            physical_right_finger_contact=_optional_bool(
                data.get("physical_right_finger_contact")
            ),
            physical_bilateral_contact=_optional_bool(
                data.get("physical_bilateral_contact")
            ),
            physical_slip=_optional_bool(data.get("physical_slip")),
            physical_translation_drift_m=_optional_float(
                data.get("physical_translation_drift_m")
            ),
            physical_rotation_drift_rad=_optional_float(
                data.get("physical_rotation_drift_rad")
            ),
            physical_release_stable_time_sec=_optional_float(
                data.get("physical_release_stable_time_sec")
            ),
            last_collision_pair=tuple(
                str(value) for value in data.get("last_collision_pair", [])
            ),
        )

    @classmethod
    def from_json(cls, payload: str) -> "EpisodeResult":
        return cls.from_dict(json.loads(payload))


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    parsed = float(value)
    if math.isnan(parsed):
        return None
    if not math.isfinite(parsed):
        raise ValueError("metric must be finite, null, or NaN at the ROS boundary")
    return parsed


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if not isinstance(value, bool):
        raise ValueError("optional boolean metric must be true, false, or null")
    return value


def _quaternion_yaw(values: tuple[float, float, float, float]) -> float:
    x, y, z, w = (float(value) for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _validate_pose_values(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> None:
    if len(position) != 3 or not all(math.isfinite(float(v)) for v in position):
        raise ValueError("position must contain three finite values")
    if len(orientation) != 4 or not all(
        math.isfinite(float(v)) for v in orientation
    ):
        raise ValueError("orientation must contain four finite values")
    if math.sqrt(sum(float(v) ** 2 for v in orientation)) <= 1.0e-9:
        raise ValueError("orientation quaternion must be non-zero")
