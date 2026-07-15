from __future__ import annotations

from typing import Sequence

from panda_pick_place_planning.core.planners.base import within_joint_limits


class PandaCollisionChecker:
    def __init__(
        self,
        bullet,
        physics_client_id: int,
        robot_id: int,
        arm_joint_indices: Sequence[int],
        joint_limits: Sequence[tuple[float, float]],
        world_body_ids: Sequence[int],
    ) -> None:
        self.bullet = bullet
        self.physics_client_id = physics_client_id
        self.robot_id = robot_id
        self.arm_joint_indices = list(arm_joint_indices)
        self.joint_limits = list(joint_limits)
        self.world_body_ids = list(world_body_ids)

    def is_valid(self, joints: Sequence[float]) -> bool:
        if not within_joint_limits(joints, self.joint_limits):
            return False
        previous = self.current_joints()
        self._reset_joints(joints)
        self.bullet.performCollisionDetection(physicsClientId=self.physics_client_id)
        collision_count = self.current_collision_count()
        self._reset_joints(previous)
        self.bullet.performCollisionDetection(physicsClientId=self.physics_client_id)
        return collision_count == 0

    def current_collision_count(self) -> int:
        collisions: set[tuple[int, int, int]] = set()
        for contact in self.bullet.getContactPoints(
            bodyA=self.robot_id,
            bodyB=self.robot_id,
            physicsClientId=self.physics_client_id,
        ):
            first_link, second_link = int(contact[3]), int(contact[4])
            if first_link == second_link or abs(first_link - second_link) <= 1:
                continue
            collisions.add((self.robot_id, first_link, second_link))

        for body_id in self.world_body_ids:
            for contact in self.bullet.getContactPoints(
                bodyA=self.robot_id,
                bodyB=body_id,
                physicsClientId=self.physics_client_id,
            ):
                robot_link = int(contact[3])
                if robot_link < 0:
                    continue
                collisions.add((body_id, robot_link, int(contact[4])))
        return len(collisions)

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
