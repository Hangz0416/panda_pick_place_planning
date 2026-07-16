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
        self.movable_joint_indices = self._movable_joint_indices()
        self.movable_joint_limits = [
            self._joint_limits(joint_index)
            for joint_index in self.movable_joint_indices
        ]
        self.solution_index_by_joint = {
            joint_index: solution_index
            for solution_index, joint_index in enumerate(self.movable_joint_indices)
        }
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance

    def solve(
        self,
        target_position: Sequence[float],
        target_orientation: Sequence[float],
        rest_poses: Sequence[float] | None = None,
    ) -> IKSolution:
        current = self.current_joints()
        current_movable = self._current_movable_joints()
        rest = self._movable_rest_poses(current_movable, rest_poses)
        lower = [limits[0] for limits in self.movable_joint_limits]
        upper = [limits[1] for limits in self.movable_joint_limits]
        ranges = [
            upper_value - lower_value
            for lower_value, upper_value in self.movable_joint_limits
        ]

        raw_solution = self.bullet.calculateInverseKinematics(
            self.robot_id,
            self.end_effector_link,
            targetPosition=list(target_position),
            targetOrientation=list(target_orientation),
            lowerLimits=lower,
            upperLimits=upper,
            jointRanges=ranges,
            restPoses=rest,
            jointDamping=[0.08] * len(self.movable_joint_indices),
            maxNumIterations=200,
            residualThreshold=1e-5,
            physicsClientId=self.physics_client_id,
        )
        if len(raw_solution) != len(self.movable_joint_indices):
            return IKSolution(False, [], math.inf, math.inf, "ik_solution_size")
        candidate = [
            float(raw_solution[self.solution_index_by_joint[joint_index]])
            for joint_index in self.arm_joint_indices
        ]
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

    def _movable_joint_indices(self) -> list[int]:
        return [
            joint_index
            for joint_index in range(
                self.bullet.getNumJoints(
                    self.robot_id,
                    physicsClientId=self.physics_client_id,
                )
            )
            if self.bullet.getJointInfo(
                self.robot_id,
                joint_index,
                physicsClientId=self.physics_client_id,
            )[2]
            != self.bullet.JOINT_FIXED
        ]

    def _joint_limits(self, joint_index: int) -> tuple[float, float]:
        info = self.bullet.getJointInfo(
            self.robot_id,
            joint_index,
            physicsClientId=self.physics_client_id,
        )
        lower, upper = float(info[8]), float(info[9])
        if lower > upper:
            return (-math.pi, math.pi)
        return (lower, upper)

    def _current_movable_joints(self) -> list[float]:
        return [
            float(
                self.bullet.getJointState(
                    self.robot_id,
                    joint_index,
                    physicsClientId=self.physics_client_id,
                )[0]
            )
            for joint_index in self.movable_joint_indices
        ]

    def _movable_rest_poses(
        self,
        current_movable: Sequence[float],
        rest_poses: Sequence[float] | None,
    ) -> list[float]:
        rest = list(current_movable)
        if rest_poses is None:
            return rest
        if len(rest_poses) == len(self.movable_joint_indices):
            return [float(value) for value in rest_poses]
        if len(rest_poses) != len(self.arm_joint_indices):
            raise ValueError("rest_poses must describe arm joints or all movable joints")
        for joint_index, value in zip(self.arm_joint_indices, rest_poses):
            rest[self.solution_index_by_joint[joint_index]] = float(value)
        return rest


def quaternion_angle(first: Sequence[float], second: Sequence[float]) -> float:
    dot_product = abs(sum(left * right for left, right in zip(first, second)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot_product)))
