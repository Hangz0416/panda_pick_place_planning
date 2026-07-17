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
        self.carried_body_id: int | None = None
        self.carried_parent_link: int | None = None
        self.carried_relative_position: tuple[float, float, float] | None = None
        self.carried_relative_orientation: tuple[float, float, float, float] | None = None
        self.carried_world_body_ids: list[int] = []
        self.carried_clearance_margins: dict[int, float] = {}
        self.carried_contact_tolerances: dict[int, float] = {}
        self.last_validation_failure = ""
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
            self.last_validation_failure = "joint_limits"
            return False
        previous = self.current_joints()
        previous_carried_pose = self._current_carried_pose()
        try:
            self._reset_joints(joints)
            self._update_carried_body_pose()
            self.bullet.performCollisionDetection(physicsClientId=self.physics_client_id)
            collisions = self.current_collisions()
            clearance_violations = self._world_clearance_violations()
            carried_clearance_violations = self._carried_clearance_violations()
            if any(item[0] == "carried" for item in collisions) or carried_clearance_violations:
                self.last_validation_failure = "carried_object_collision"
            elif collisions or clearance_violations:
                self.last_validation_failure = "robot_collision"
            else:
                self.last_validation_failure = ""
        finally:
            self._reset_joints(previous)
            self._restore_carried_pose(previous_carried_pose)
            self.bullet.performCollisionDetection(physicsClientId=self.physics_client_id)
        return not collisions and not clearance_violations and not carried_clearance_violations

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
        collisions.update(self.current_carried_collisions())
        return sorted(collisions)

    def current_carried_collisions(self) -> list[tuple[str, int, int, int]]:
        """Return contacts between the attached object and configured world bodies."""

        if self.carried_body_id is None:
            return []
        collisions: set[tuple[str, int, int, int]] = set()
        for body_id in self.carried_world_body_ids:
            tolerance = self.carried_contact_tolerances.get(body_id, 0.0)
            for contact in self.bullet.getContactPoints(
                bodyA=self.carried_body_id,
                bodyB=body_id,
                physicsClientId=self.physics_client_id,
            ):
                # PyBullet contactDistance is index 8. A small penetration with
                # the table is expected at the instant a resting cube becomes
                # attached; deeper penetration still remains a collision.
                contact_distance = float(contact[8]) if len(contact) > 8 else -float("inf")
                if tolerance > 0.0 and contact_distance >= -tolerance:
                    continue
                collisions.add(("carried", body_id, -1, int(contact[4])))
        return sorted(collisions)

    def attach_carried_body(
        self,
        body_id: int,
        parent_link: int,
        relative_position: Sequence[float],
        relative_orientation: Sequence[float],
        world_body_ids: Sequence[int],
        *,
        clearance_margins: dict[int, float] | None = None,
        contact_tolerances: dict[int, float] | None = None,
    ) -> None:
        self.carried_body_id = body_id
        self.carried_parent_link = parent_link
        self.carried_relative_position = tuple(float(value) for value in relative_position)
        self.carried_relative_orientation = tuple(float(value) for value in relative_orientation)
        self.carried_world_body_ids = list(world_body_ids)
        self.carried_clearance_margins = clearance_margins or {}
        self.carried_contact_tolerances = contact_tolerances or {}

    def detach_carried_body(self) -> None:
        self.carried_body_id = None
        self.carried_parent_link = None
        self.carried_relative_position = None
        self.carried_relative_orientation = None
        self.carried_world_body_ids = []
        self.carried_clearance_margins = {}
        self.carried_contact_tolerances = {}
        self.last_validation_failure = ""

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

    def _carried_clearance_violations(self) -> list[int]:
        if self.carried_body_id is None:
            return []
        violations: set[int] = set()
        for body_id, margin in self.carried_clearance_margins.items():
            if margin <= 0.0:
                continue
            points = self.bullet.getClosestPoints(
                bodyA=self.carried_body_id,
                bodyB=body_id,
                distance=margin,
                physicsClientId=self.physics_client_id,
            )
            if points:
                violations.add(body_id)
        return sorted(violations)

    def _current_carried_pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
        if self.carried_body_id is None:
            return None
        position, orientation = self.bullet.getBasePositionAndOrientation(
            self.carried_body_id,
            physicsClientId=self.physics_client_id,
        )
        return tuple(position), tuple(orientation)

    def _update_carried_body_pose(self) -> None:
        if (
            self.carried_body_id is None
            or self.carried_parent_link is None
            or self.carried_relative_position is None
            or self.carried_relative_orientation is None
        ):
            return
        link_state = self.bullet.getLinkState(
            self.robot_id,
            self.carried_parent_link,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client_id,
        )
        position, orientation = self.bullet.multiplyTransforms(
            link_state[4],
            link_state[5],
            self.carried_relative_position,
            self.carried_relative_orientation,
        )
        self.bullet.resetBasePositionAndOrientation(
            self.carried_body_id,
            position,
            orientation,
            physicsClientId=self.physics_client_id,
        )

    def _restore_carried_pose(
        self,
        pose: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None,
    ) -> None:
        if self.carried_body_id is None or pose is None:
            return
        self.bullet.resetBasePositionAndOrientation(
            self.carried_body_id,
            pose[0],
            pose[1],
            physicsClientId=self.physics_client_id,
        )

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
