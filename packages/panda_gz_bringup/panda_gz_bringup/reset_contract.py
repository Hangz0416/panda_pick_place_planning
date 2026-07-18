"""ROS-independent typed reset request validation and scene mapping."""

from dataclasses import dataclass
import math
from typing import Sequence


SUPPORTED_SCENES = frozenset({"clear", "obstacle"})
SUPPORTED_GRASP_MODES = frozenset({"assisted", "physics"})
SUPPORTED_OBJECT_VARIANTS = frozenset({"cube", "rect_prism"})
SUPPORTED_RESET_FRAMES = frozenset({"", "world"})
RESET_VALIDATION_THRESHOLDS = {
    "joint_position_error_rad": 0.025,
    "joint_velocity_rad_s": 0.025,
    "gripper_position_error_m": 0.003,
    "cube_position_error_m": 0.005,
    "cube_orientation_error_rad": 0.050,
    "target_position_error_m": 0.005,
    "obstacle_position_error_m": 0.005,
    "cube_linear_speed_mps": 0.010,
    "cube_angular_speed_rad_s": 0.050,
}


@dataclass(frozen=True)
class ResetValidationMetrics:
    """Maximum error channels observed after one successful reset."""

    joint_position_error_rad: float
    joint_velocity_rad_s: float
    gripper_position_error_m: float
    cube_position_error_m: float
    cube_orientation_error_rad: float
    target_position_error_m: float
    obstacle_position_error_m: float
    cube_linear_speed_mps: float
    cube_angular_speed_rad_s: float

    def passed(self) -> bool:
        return all(
            getattr(self, field) <= limit
            for field, limit in RESET_VALIDATION_THRESHOLDS.items()
        )


def require_world_frame(frame_id: str) -> None:
    """Reject reset coordinates whose frame cannot be interpreted safely."""
    if frame_id not in SUPPORTED_RESET_FRAMES:
        raise ValueError(
            "reset header.frame_id must be empty or 'world', "
            f"got {frame_id!r}"
        )


def vector_norm(values: Sequence[float]) -> float:
    """Return the Euclidean norm after rejecting non-finite components."""
    vector = tuple(float(value) for value in values)
    if not vector or not all(math.isfinite(value) for value in vector):
        return math.inf
    return math.sqrt(sum(value * value for value in vector))


def quaternion_angle_error(
    actual_xyzw: Sequence[float], expected_xyzw: Sequence[float]
) -> float:
    """Return the shortest angular distance between two quaternions."""
    actual = tuple(float(value) for value in actual_xyzw)
    expected = tuple(float(value) for value in expected_xyzw)
    if len(actual) != 4 or len(expected) != 4:
        return math.inf
    actual_norm = vector_norm(actual)
    expected_norm = vector_norm(expected)
    if actual_norm <= 1.0e-12 or expected_norm <= 1.0e-12:
        return math.inf
    dot = sum(a * b for a, b in zip(actual, expected))
    dot /= actual_norm * expected_norm
    return 2.0 * math.acos(min(1.0, max(-1.0, abs(dot))))


def pose_error(
    actual_xyz: Sequence[float],
    actual_xyzw: Sequence[float],
    expected_pose: Sequence[float],
) -> tuple[float, float]:
    """Return translation and rotation error for an xyz + xyzw pose."""
    if len(actual_xyz) != 3 or len(expected_pose) != 7:
        return math.inf, math.inf
    translation = vector_norm(
        tuple(float(a) - float(b) for a, b in zip(actual_xyz, expected_pose[:3]))
    )
    rotation = quaternion_angle_error(actual_xyzw, expected_pose[3:])
    return translation, rotation


def _point(values: Sequence[float], field: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{field} must contain exactly three coordinates")
    point = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in point):
        raise ValueError(f"{field} must contain finite coordinates")
    return point


@dataclass(frozen=True)
class ResetWorkpieceSpec:
    object_id: str
    object_variant: str
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    @classmethod
    def create(cls, object_id, object_variant, position, orientation_xyzw):
        orientation = tuple(float(value) for value in orientation_xyzw)
        if not object_id or object_variant not in SUPPORTED_OBJECT_VARIANTS:
            raise ValueError("invalid workpiece identity or variant")
        if len(orientation) != 4 or not all(math.isfinite(v) for v in orientation):
            raise ValueError("workpiece orientation must be a finite quaternion")
        if vector_norm(orientation) <= 1.0e-9:
            raise ValueError("workpiece orientation quaternion is zero")
        return cls(
            str(object_id),
            str(object_variant),
            _point(position, "workpiece_position"),
            orientation,
        )


@dataclass(frozen=True)
class ResetSpec:
    episode_id: int
    task_seed: int
    scene: str
    grasp_mode: str
    object_position: tuple[float, float, float]
    target_position: tuple[float, float, float]
    object_variant: str = "cube"
    object_yaw_rad: float = 0.0
    workpieces: tuple[ResetWorkpieceSpec, ...] = ()

    @classmethod
    def create(
        cls,
        *,
        episode_id: int,
        task_seed: int,
        scene: str,
        grasp_mode: str,
        object_position: Sequence[float],
        target_position: Sequence[float],
        object_variant: str = "cube",
        object_yaw_rad: float = 0.0,
        workpieces: Sequence[ResetWorkpieceSpec] = (),
    ) -> "ResetSpec":
        if scene not in SUPPORTED_SCENES:
            raise ValueError(f"unsupported scene: {scene}")
        if grasp_mode not in SUPPORTED_GRASP_MODES:
            raise ValueError(f"unsupported grasp_mode: {grasp_mode}")
        if int(episode_id) < 0 or int(task_seed) < 0:
            raise ValueError("episode_id and task_seed must be non-negative")
        if object_variant not in SUPPORTED_OBJECT_VARIANTS:
            raise ValueError(f"unsupported object_variant: {object_variant}")
        if not math.isfinite(float(object_yaw_rad)):
            raise ValueError("object_yaw_rad must be finite")
        values = tuple(workpieces)
        if not values:
            half_yaw = 0.5 * float(object_yaw_rad)
            values = (
                ResetWorkpieceSpec.create(
                    "cube",
                    object_variant,
                    object_position,
                    (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
                ),
            )
        if len(values) not in {1, 3} or len({item.object_id for item in values}) != len(values):
            raise ValueError("reset requires one or three uniquely named workpieces")
        if any(item.object_variant != object_variant for item in values):
            raise ValueError("workpiece variants must match reset object_variant")
        return cls(
            episode_id=int(episode_id),
            task_seed=int(task_seed),
            scene=scene,
            grasp_mode=grasp_mode,
            object_position=_point(object_position, "object_position"),
            target_position=_point(target_position, "target_position"),
            object_variant=object_variant,
            object_yaw_rad=float(object_yaw_rad),
            workpieces=values,
        )


@dataclass(frozen=True)
class ScenePoseConfig:
    target_marker_z: float
    obstacle_pose: tuple[float, ...]
    clear_obstacle_pose: tuple[float, ...]


def entity_pose_map(
    spec: ResetSpec, config: ScenePoseConfig
) -> dict[str, tuple[float, ...]]:
    """Map one task request to deterministic Gazebo model poses (xyz + xyzw)."""
    obstacle = (
        config.obstacle_pose
        if spec.scene == "obstacle"
        else config.clear_obstacle_pose
    )
    values = {
        "target": (
            spec.target_position[0],
            spec.target_position[1],
            float(config.target_marker_z),
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        "obstacle": tuple(float(value) for value in obstacle),
    }
    values.update(
        {
            item.object_id: (*item.position, *item.orientation_xyzw)
            for item in spec.workpieces
        }
    )
    return values
