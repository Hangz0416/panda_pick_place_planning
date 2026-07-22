"""
Experimental, fail-closed physical-grasp classifier without ROS dependencies.

The classifier deliberately needs bilateral finger contact before declaring a
grasp. After that event it measures the cube pose relative to the tool frame;
translation or rotation drift is reported as slip instead of being hidden by a
fixed constraint. It is not part of the current assisted workflow or resume
claims.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


def _finite_tuple(values: Sequence[float], length: int, field: str) -> tuple:
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field} must contain {length} finite values")
    return result


def _norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _quaternion_normalize(values: Sequence[float]) -> tuple[float, ...]:
    quaternion = _finite_tuple(values, 4, "quaternion")
    magnitude = _norm(quaternion)
    if magnitude <= 1.0e-12:
        raise ValueError("quaternion must have non-zero norm")
    return tuple(value / magnitude for value in quaternion)


def _quaternion_conjugate(values: Sequence[float]) -> tuple[float, ...]:
    x, y, z, w = _quaternion_normalize(values)
    return (-x, -y, -z, w)


def _quaternion_multiply(
    first: Sequence[float], second: Sequence[float]
) -> tuple[float, ...]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    quaternion_xyzw: Sequence[float], vector_xyz: Sequence[float]
) -> tuple[float, ...]:
    quaternion = _quaternion_normalize(quaternion_xyzw)
    vector = (*_finite_tuple(vector_xyz, 3, "vector"), 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector),
        _quaternion_conjugate(quaternion),
    )
    return rotated[:3]


def quaternion_angle_error(
    first_xyzw: Sequence[float], second_xyzw: Sequence[float]
) -> float:
    first = _quaternion_normalize(first_xyzw)
    second = _quaternion_normalize(second_xyzw)
    dot = abs(sum(a * b for a, b in zip(first, second)))
    return 2.0 * math.acos(min(1.0, max(0.0, dot)))


@dataclass(frozen=True)
class PoseSample:
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    @classmethod
    def create(
        cls, position_xyz: Sequence[float], orientation_xyzw: Sequence[float]
    ) -> "PoseSample":
        return cls(
            position_xyz=_finite_tuple(position_xyz, 3, "position"),
            orientation_xyzw=_quaternion_normalize(orientation_xyzw),
        )


@dataclass(frozen=True)
class PhysicalGraspConfig:
    bilateral_contact_confirm_sec: float = 0.10
    release_confirm_sec: float = 0.10
    max_initial_hand_object_distance_m: float = 0.09
    max_relative_translation_drift_m: float = 0.015
    max_relative_rotation_drift_rad: float = 0.20
    stable_linear_speed_mps: float = 0.05

    def __post_init__(self) -> None:
        values = (
            self.bilateral_contact_confirm_sec,
            self.release_confirm_sec,
            self.max_initial_hand_object_distance_m,
            self.max_relative_translation_drift_m,
            self.max_relative_rotation_drift_rad,
            self.stable_linear_speed_mps,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("physical grasp thresholds must be finite and non-negative")


@dataclass(frozen=True)
class PhysicalGraspSnapshot:
    episode_id: int
    initialized: bool
    grasped: bool
    holding_object: bool
    released: bool
    left_finger_contact: bool
    right_finger_contact: bool
    slipping: bool
    max_relative_translation_drift_m: float
    max_relative_rotation_drift_rad: float
    object_linear_speed_mps: float
    release_stable_time_sec: float


def relative_pose(hand_pose: PoseSample, object_pose: PoseSample) -> PoseSample:
    inverse_hand = _quaternion_conjugate(hand_pose.orientation_xyzw)
    world_delta = tuple(
        object_value - hand_value
        for object_value, hand_value in zip(
            object_pose.position_xyz, hand_pose.position_xyz
        )
    )
    return PoseSample.create(
        _rotate_vector(inverse_hand, world_delta),
        _quaternion_multiply(inverse_hand, object_pose.orientation_xyzw),
    )


class PhysicalGraspClassifier:
    """Stateful, deterministic physical-grasp event classifier."""

    def __init__(self, config: PhysicalGraspConfig | None = None) -> None:
        self.config = config or PhysicalGraspConfig()
        self.reset(episode_id=0)

    def reset(self, episode_id: int) -> None:
        if int(episode_id) < 0:
            raise ValueError("episode_id must be non-negative")
        self.episode_id = int(episode_id)
        self.initialized = False
        self.grasped = False
        self.released = False
        self.left_contact = False
        self.right_contact = False
        self.slipping = False
        self.max_translation_drift = 0.0
        self.max_rotation_drift = 0.0
        self.object_speed = 0.0
        self._both_contact_since: float | None = None
        self._no_contact_since: float | None = None
        self._stable_since: float | None = None
        self._reference_relative_pose: PoseSample | None = None
        self._last_cube_pose: PoseSample | None = None
        self._last_timestamp: float | None = None

    def update(
        self,
        *,
        timestamp_sec: float,
        left_finger_contact: bool,
        right_finger_contact: bool,
        hand_pose: PoseSample,
        object_pose: PoseSample,
    ) -> PhysicalGraspSnapshot:
        timestamp = float(timestamp_sec)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp_sec must be finite")
        if self._last_timestamp is not None and timestamp < self._last_timestamp:
            raise ValueError("timestamp_sec must be monotonic")

        delta_time = (
            timestamp - self._last_timestamp
            if self._last_timestamp is not None
            else 0.0
        )
        if self._last_cube_pose is not None and delta_time > 1.0e-9:
            displacement = tuple(
                current - previous
                for current, previous in zip(
                    object_pose.position_xyz,
                    self._last_cube_pose.position_xyz,
                )
            )
            self.object_speed = _norm(displacement) / delta_time
        else:
            self.object_speed = 0.0

        self.initialized = True
        self.left_contact = bool(left_finger_contact)
        self.right_contact = bool(right_finger_contact)
        both_contacts = self.left_contact and self.right_contact
        no_finger_contact = not self.left_contact and not self.right_contact
        current_relative = relative_pose(hand_pose, object_pose)

        if not self.grasped:
            close_enough = (
                _norm(current_relative.position_xyz)
                <= self.config.max_initial_hand_object_distance_m
            )
            if both_contacts and close_enough:
                if self._both_contact_since is None:
                    self._both_contact_since = timestamp
                if (
                    timestamp - self._both_contact_since
                    >= self.config.bilateral_contact_confirm_sec
                ):
                    self.grasped = True
                    self._reference_relative_pose = current_relative
            else:
                self._both_contact_since = None

        if self.grasped and not self.released:
            reference = self._reference_relative_pose
            if reference is not None:
                translation_drift = _norm(
                    tuple(
                        current - initial
                        for current, initial in zip(
                            current_relative.position_xyz,
                            reference.position_xyz,
                        )
                    )
                )
                rotation_drift = quaternion_angle_error(
                    current_relative.orientation_xyzw,
                    reference.orientation_xyzw,
                )
                self.max_translation_drift = max(
                    self.max_translation_drift, translation_drift
                )
                self.max_rotation_drift = max(
                    self.max_rotation_drift, rotation_drift
                )
                self.slipping = self.slipping or (
                    translation_drift
                    > self.config.max_relative_translation_drift_m
                    or rotation_drift
                    > self.config.max_relative_rotation_drift_rad
                )

            if no_finger_contact:
                if self._no_contact_since is None:
                    self._no_contact_since = timestamp
                if (
                    timestamp - self._no_contact_since
                    >= self.config.release_confirm_sec
                ):
                    self.released = True
                    if self.object_speed <= self.config.stable_linear_speed_mps:
                        self._stable_since = timestamp
            else:
                self._no_contact_since = None

        if self.released:
            if (
                no_finger_contact
                and self.object_speed <= self.config.stable_linear_speed_mps
            ):
                if self._stable_since is None:
                    self._stable_since = timestamp
            else:
                self._stable_since = None

        self._last_cube_pose = object_pose
        self._last_timestamp = timestamp
        return self.snapshot()

    def snapshot(self) -> PhysicalGraspSnapshot:
        stable_time = (
            max(0.0, self._last_timestamp - self._stable_since)
            if self._last_timestamp is not None and self._stable_since is not None
            else 0.0
        )
        return PhysicalGraspSnapshot(
            episode_id=self.episode_id,
            initialized=self.initialized,
            grasped=self.grasped,
            holding_object=self.grasped and not self.released,
            released=self.released,
            left_finger_contact=self.left_contact,
            right_finger_contact=self.right_contact,
            slipping=self.slipping,
            max_relative_translation_drift_m=self.max_translation_drift,
            max_relative_rotation_drift_rad=self.max_rotation_drift,
            object_linear_speed_mps=self.object_speed,
            release_stable_time_sec=stable_time,
        )
