from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from panda_pick_place_planning.core.planners.base import within_joint_limits


@dataclass(frozen=True)
class IKSolution:
    success: bool
    joints: list[float]
    position_error: float
    orientation_error: float
    failure_type: str = ""


class PandaIKSolver:
    def __init__(
        self,
        bullet,
        physics_client_id: int,
        robot_id: int,
        arm_joint_indices: Sequence[int],
        end_effector_link: int,
        joint_limits: Sequence[tuple[float, float]],
        position_tolerance: float = 0.015,
        orientation_tolerance: float = 0.12,
    ) -> None:
        self.bullet = bullet
        self.physics_client_id = physics_client_id
        self.robot_id = robot_id
        self.arm_joint_indices = list(arm_joint_indices)
        self.end_effector_link = end_effector_link
        self.joint_limits = list(joint_limits)
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance

    def solve(
        self,
        target_position: Sequence[float],
        target_orientation: Sequence[float],
        rest_poses: Sequence[float] | None = None,
    ) -> IKSolution:
        current = self.current_joints()
        rest = list(rest_poses) if rest_poses is not None else current
        lower = [limits[0] for limits in self.joint_limits]
        upper = [limits[1] for limits in self.joint_limits]
        ranges = [upper_value - lower_value for lower_value, upper_value in self.joint_limits]

        raw_solution = self.bullet.calculateInverseKinematics(
            self.robot_id,
            self.end_effector_link,
            targetPosition=list(target_position),
            targetOrientation=list(target_orientation),
            lowerLimits=lower,
            upperLimits=upper,
            jointRanges=ranges,
            restPoses=rest,
            jointDamping=[0.08] * len(self.arm_joint_indices),
            maxNumIterations=200,
            residualThreshold=1e-5,
            physicsClientId=self.physics_client_id,
        )
        candidate = [float(raw_solution[index]) for index in range(len(self.arm_joint_indices))]
        if not within_joint_limits(candidate, self.joint_limits):
            return IKSolution(False, candidate, math.inf, math.inf, "ik_joint_limit")

        self._reset_joints(candidate)
        state = self.bullet.getLinkState(
            self.robot_id,
            self.end_effector_link,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client_id,
        )
        actual_position = state[4]
        actual_orientation = state[5]
        self._reset_joints(current)

        position_error = math.dist(actual_position, target_position)
        orientation_error = quaternion_angle(actual_orientation, target_orientation)
        if position_error > self.position_tolerance or orientation_error > self.orientation_tolerance:
            return IKSolution(
                False,
                candidate,
                position_error,
                orientation_error,
                "ik_pose_error",
            )
        return IKSolution(True, candidate, position_error, orientation_error)

    def current_joints(self) -> list[float]:
        return [
            float(
                self.bullet.getJointState(
                    self.robot_id,
                    joint_index,
                    physicsClientId=self.physics_client_id,
                )[0]
            )
            for joint_index in self.arm_joint_indices
        ]

    def _reset_joints(self, joints: Sequence[float]) -> None:
        for joint_index, value in zip(self.arm_joint_indices, joints):
            self.bullet.resetJointState(
                self.robot_id,
                joint_index,
                float(value),
                physicsClientId=self.physics_client_id,
            )


def quaternion_angle(first: Sequence[float], second: Sequence[float]) -> float:
    dot_product = abs(sum(left * right for left, right in zip(first, second)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot_product)))
