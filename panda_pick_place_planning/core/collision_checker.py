from __future__ import annotations

from itertools import combinations
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
        allowed_world_contacts: dict[int, set[int]] | None = None,
        world_clearance_margins: dict[int, float] | None = None,
    ) -> None:
        self.bullet = bullet
        self.physics_client_id = physics_client_id
        self.robot_id = robot_id
        self.arm_joint_indices = list(arm_joint_indices)
        self.joint_limits = list(joint_limits)
        self.world_body_ids = list(world_body_ids)
        self.allowed_world_contacts = allowed_world_contacts or {}
        self.world_clearance_margins = world_clearance_margins or {}
        self.allowed_self_pairs = self._adjacent_link_pairs()
        self.allowed_self_pairs.update(self._rigid_link_pairs())
        hand_links = {7, 8, 9, 10, 11}
        self.allowed_self_pairs.update(
            frozenset((first_link, second_link))
            for first_link in hand_links
            for second_link in hand_links
            if first_link != second_link
        )

    def is_valid(self, joints: Sequence[float]) -> bool:
        if not within_joint_limits(joints, self.joint_limits):
            return False
        previous = self.current_joints()
        self._reset_joints(joints)
        self.bullet.performCollisionDetection(physicsClientId=self.physics_client_id)
        collisions = self.current_collisions()
        clearance_violations = self._world_clearance_violations()
        self._reset_joints(previous)
        self.bullet.performCollisionDetection(physicsClientId=self.physics_client_id)
        return not collisions and not clearance_violations

    def current_collision_count(self) -> int:
        return len(self.current_collisions())

    def current_collisions(self) -> list[tuple[str, int, int, int]]:
        collisions: set[tuple[str, int, int, int]] = set()
        for contact in self.bullet.getContactPoints(
            bodyA=self.robot_id,
            bodyB=self.robot_id,
            physicsClientId=self.physics_client_id,
        ):
            first_link, second_link = int(contact[3]), int(contact[4])
            if first_link == second_link:
                continue
            if frozenset((first_link, second_link)) in self.allowed_self_pairs:
                continue
            collisions.add(("self", self.robot_id, first_link, second_link))

        for body_id in self.world_body_ids:
            for contact in self.bullet.getContactPoints(
                bodyA=self.robot_id,
                bodyB=body_id,
                physicsClientId=self.physics_client_id,
            ):
                robot_link = int(contact[3])
                if robot_link in self.allowed_world_contacts.get(body_id, set()):
                    continue
                collisions.add(("world", body_id, robot_link, int(contact[4])))
        return sorted(collisions)

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

    def _world_clearance_violations(self) -> list[tuple[int, int]]:
        violations: set[tuple[int, int]] = set()
        for body_id, margin in self.world_clearance_margins.items():
            if margin <= 0.0:
                continue
            for point in self.bullet.getClosestPoints(
                bodyA=self.robot_id,
                bodyB=body_id,
                distance=margin,
                physicsClientId=self.physics_client_id,
            ):
                robot_link = int(point[3])
                if robot_link in self.allowed_world_contacts.get(body_id, set()):
                    continue
                violations.add((body_id, robot_link))
        return sorted(violations)

    def _reset_joints(self, joints: Sequence[float]) -> None:
        for joint_index, value in zip(self.arm_joint_indices, joints):
            self.bullet.resetJointState(
                self.robot_id,
                joint_index,
                float(value),
                physicsClientId=self.physics_client_id,
            )

    def _adjacent_link_pairs(self) -> set[frozenset[int]]:
        pairs: set[frozenset[int]] = set()
        joint_count = self.bullet.getNumJoints(
            self.robot_id,
            physicsClientId=self.physics_client_id,
        )
        for child_link in range(joint_count):
            info = self.bullet.getJointInfo(
                self.robot_id,
                child_link,
                physicsClientId=self.physics_client_id,
            )
            parent_link = int(info[16])
            pairs.add(frozenset((parent_link, child_link)))
        return pairs

    def _rigid_link_pairs(self) -> set[frozenset[int]]:
        fixed_graph: dict[int, set[int]] = {}
        joint_count = self.bullet.getNumJoints(
            self.robot_id,
            physicsClientId=self.physics_client_id,
        )
        for child_link in range(joint_count):
            info = self.bullet.getJointInfo(
                self.robot_id,
                child_link,
                physicsClientId=self.physics_client_id,
            )
            if info[2] != self.bullet.JOINT_FIXED:
                continue
            parent_link = int(info[16])
            fixed_graph.setdefault(parent_link, set()).add(child_link)
            fixed_graph.setdefault(child_link, set()).add(parent_link)

        pairs: set[frozenset[int]] = set()
        visited: set[int] = set()
        for start_link in fixed_graph:
            if start_link in visited:
                continue
            component: set[int] = set()
            pending = [start_link]
            while pending:
                link = pending.pop()
                if link in component:
                    continue
                component.add(link)
                pending.extend(fixed_graph[link] - component)
            visited.update(component)
            pairs.update(frozenset(pair) for pair in combinations(component, 2))
        return pairs
