"""ROS-independent assisted-grasp gate calculations."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class GateLimits:
    expected_hand_offset_xyz: tuple[float, float, float]
    expected_hand_quaternion_xyzw: tuple[float, float, float, float]
    max_position_error_m: float
    max_orientation_error_rad: float
    max_hand_object_distance_m: float
    expected_gripper_width_m: float
    gripper_width_tolerance_m: float
    allow_pi_symmetry: bool = True


@dataclass(frozen=True)
class GateResult:
    accepted: bool
    reason: str
    position_error_m: float
    orientation_error_rad: float
    hand_object_distance_m: float
    gripper_width_m: float


def _norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def quaternion_angle_error(
    actual_xyzw: Sequence[float], expected_xyzw: Sequence[float]
) -> float:
    """Return shortest angular distance for two unit or non-unit quaternions."""
    actual_norm = _norm(actual_xyzw)
    expected_norm = _norm(expected_xyzw)
    if actual_norm <= 1.0e-12 or expected_norm <= 1.0e-12:
        return math.inf
    dot = sum(
        float(actual) * float(expected)
        for actual, expected in zip(actual_xyzw, expected_xyzw)
    ) / (actual_norm * expected_norm)
    return 2.0 * math.acos(min(1.0, max(0.0, abs(dot))))


def _quaternion_multiply(left: Sequence[float], right: Sequence[float]):
    lx, ly, lz, lw = (float(value) for value in left)
    rx, ry, rz, rw = (float(value) for value in right)
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def evaluate_gate(
    hand_xyz: Sequence[float],
    hand_quaternion_xyzw: Sequence[float],
    object_xyz: Sequence[float],
    gripper_width_m: float,
    limits: GateLimits,
    object_quaternion_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
) -> GateResult:
    """Evaluate proximity, pose, and gripper-clearance requirements."""
    hand_object = tuple(float(h) - float(o) for h, o in zip(hand_xyz, object_xyz))
    position_error = _norm(
        tuple(
            actual - expected
            for actual, expected in zip(
                hand_object, limits.expected_hand_offset_xyz
            )
        )
    )
    distance = _norm(hand_object)
    ox, oy, oz, ow = (float(value) for value in object_quaternion_xyzw)
    object_inverse = (-ox, -oy, -oz, ow)
    relative_hand = _quaternion_multiply(object_inverse, hand_quaternion_xyzw)
    expected_orientations = [limits.expected_hand_quaternion_xyzw]
    if limits.allow_pi_symmetry:
        expected_orientations.append(
            _quaternion_multiply(
                (0.0, 0.0, 1.0, 0.0),
                limits.expected_hand_quaternion_xyzw,
            )
        )
    orientation_error = min(
        quaternion_angle_error(relative_hand, expected)
        for expected in expected_orientations
    )

    checks = (
        (distance <= limits.max_hand_object_distance_m, "object_too_far"),
        (position_error <= limits.max_position_error_m, "hand_pose_position_error"),
        (
            orientation_error <= limits.max_orientation_error_rad,
            "hand_pose_orientation_error",
        ),
        (
            gripper_width_m
            >= limits.expected_gripper_width_m
            - limits.gripper_width_tolerance_m,
            "gripper_too_narrow",
        ),
        (
            gripper_width_m
            <= limits.expected_gripper_width_m
            + limits.gripper_width_tolerance_m,
            "gripper_too_wide",
        ),
    )
    reason = next((reason for passed, reason in checks if not passed), "accepted")
    return GateResult(
        accepted=reason == "accepted",
        reason=reason,
        position_error_m=position_error,
        orientation_error_rad=orientation_error,
        hand_object_distance_m=distance,
        gripper_width_m=float(gripper_width_m),
    )
