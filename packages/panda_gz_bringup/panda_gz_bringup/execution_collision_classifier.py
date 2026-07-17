"""Pure Gazebo contact classification shared by tests and the ROS monitor."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


ROBOT_LINKS = frozenset(
    {
        *(f"panda_link{index}" for index in range(9)),
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger",
    }
)

# This list mirrors the SRDF Allowed Collision Matrix. Contacts not present in
# this set are real self-collisions and remain invalid.
DEFAULT_ALLOWED_ROBOT_CONTACTS = frozenset(
    {
        *(frozenset((f"panda_link{index}", f"panda_link{index + 1}"))
          for index in range(8)),
        frozenset(("panda_link8", "panda_hand")),
        frozenset(("panda_hand", "panda_leftfinger")),
        frozenset(("panda_hand", "panda_rightfinger")),
        *(frozenset((f"panda_link{index}", f"panda_link{index + 2}"))
          for index in range(7)),
        frozenset(("panda_link6", "panda_hand")),
        frozenset(("panda_link7", "panda_hand")),
        frozenset(("panda_leftfinger", "panda_rightfinger")),
    }
)

TOUCH_LINKS = frozenset(
    {"panda_hand", "panda_leftfinger", "panda_rightfinger"}
)
ENVIRONMENT = frozenset({"table", "obstacle", "ground"})


@dataclass(frozen=True)
class CollisionContext:
    object_carried: bool
    object_position: tuple[float, float, float] | None
    object_initial_position: tuple[float, float, float]
    target_position: tuple[float, float, float]
    target_xy_tolerance_m: float = 0.04
    target_z_tolerance_m: float = 0.015


@dataclass(frozen=True)
class ContactClassification:
    allowed: bool
    failure_type: str
    entity1: str
    entity2: str


def canonical_entity(collision_name: str) -> str:
    """Map a scoped Gazebo collision name to a stable semantic entity."""
    name = str(collision_name)
    for link in sorted(ROBOT_LINKS, key=len, reverse=True):
        if link in name:
            return link
    for entity in ("cube", "obstacle", "table", "ground", "target"):
        if entity in name:
            return entity
    return "unknown"


def _object_is_at_rest_pose(
    context: CollisionContext, expected: tuple[float, float, float]
) -> bool:
    if context.object_position is None:
        return False
    object_position = context.object_position
    xy_error = math.dist(object_position[:2], expected[:2])
    z_error = abs(object_position[2] - expected[2])
    return (
        xy_error <= context.target_xy_tolerance_m
        and z_error <= context.target_z_tolerance_m
    )


def classify_contact(
    collision1: str,
    collision2: str,
    context: CollisionContext,
) -> ContactClassification:
    """Apply conservative robot/object/environment contact semantics."""
    first = canonical_entity(collision1)
    second = canonical_entity(collision2)
    entities = frozenset((first, second))

    def result(allowed, reason):
        return ContactClassification(
            allowed=allowed,
            failure_type=reason,
            entity1=first,
            entity2=second,
        )

    if first == second:
        return result(True, "same_entity_internal_contact")

    robot_entities = entities & ROBOT_LINKS
    if len(robot_entities) == 2:
        if entities in DEFAULT_ALLOWED_ROBOT_CONTACTS:
            return result(True, "allowed_robot_internal_contact")
        return result(False, "robot_self_collision")

    if "cube" in entities and robot_entities:
        robot_link = next(iter(robot_entities))
        if robot_link in TOUCH_LINKS:
            return result(True, "allowed_gripper_object_contact")
        return result(False, "robot_object_collision")

    if "cube" in entities and "table" in entities:
        if not context.object_carried:
            return result(True, "resting_or_released_object_table_contact")
        if _object_is_at_rest_pose(context, context.object_initial_position):
            return result(True, "allowed_source_pickup_contact")
        if _object_is_at_rest_pose(context, context.target_position):
            return result(True, "allowed_target_placement_contact")
        return result(False, "carried_object_environment_collision")

    if "cube" in entities and entities & {"obstacle", "ground"}:
        failure = (
            "carried_object_environment_collision"
            if context.object_carried
            else "object_environment_collision"
        )
        return result(False, failure)

    if robot_entities == {"panda_link0"} and "table" in entities:
        return result(True, "allowed_base_table_support_contact")

    if robot_entities and entities & ENVIRONMENT:
        return result(False, "robot_environment_collision")

    if robot_entities or "cube" in entities:
        # A monitored robot or object collision with an unknown model must not
        # silently become allowed when a model is renamed or added.
        return result(False, "unknown_monitored_collision")

    return result(True, "unmonitored_world_contact")


def point3(values: Sequence[float]) -> tuple[float, float, float]:
    point = tuple(float(value) for value in values)
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        raise ValueError("point must contain three finite coordinates")
    return point
