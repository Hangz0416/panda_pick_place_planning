import xml.etree.ElementTree as ET
from pathlib import Path

from panda_gz_bringup.execution_collision_classifier import (
    CollisionContext,
    DEFAULT_ALLOWED_ROBOT_CONTACTS,
    canonical_entity,
    classify_contact,
)


MOVEIT = Path(__file__).parents[2] / "panda_moveit_config"


def context(*, carried=False, object_position=(0.57, -0.17, 0.025)):
    return CollisionContext(
        object_carried=carried,
        object_position=object_position,
        object_initial_position=(0.57, -0.17, 0.025),
        target_position=(0.57, 0.17, 0.025),
    )


def test_allowed_robot_pairs_match_the_srdf_acm():
    srdf = ET.parse(MOVEIT / "config" / "panda.srdf").getroot()
    srdf_pairs = frozenset(
        frozenset((item.attrib["link1"], item.attrib["link2"]))
        for item in srdf.findall("disable_collisions")
    )
    assert DEFAULT_ALLOWED_ROBOT_CONTACTS == srdf_pairs


def test_scoped_collision_names_are_canonicalized():
    assert canonical_entity("panda::panda_link4::panda_link4_collision") == (
        "panda_link4"
    )
    assert canonical_entity("cube::cube_link::cube_collision") == "cube"
    assert canonical_entity("table::table_link::top_collision") == "table"


def test_adjacent_links_and_gripper_cube_contact_are_allowed():
    adjacent = classify_contact(
        "panda::panda_link3::collision",
        "panda::panda_link4::collision",
        context(),
    )
    touch = classify_contact(
        "panda::panda_leftfinger::collision",
        "cube::cube_link::cube_collision",
        context(),
    )
    assert adjacent.allowed
    assert touch.allowed


def test_real_self_and_robot_environment_collisions_are_invalid():
    self_collision = classify_contact(
        "panda::panda_link1::collision",
        "panda::panda_link7::collision",
        context(),
    )
    table_collision = classify_contact(
        "panda::panda_link4::collision",
        "table::table_link::top_collision",
        context(),
    )
    assert not self_collision.allowed
    assert self_collision.failure_type == "robot_self_collision"
    assert not table_collision.allowed
    assert table_collision.failure_type == "robot_environment_collision"


def test_only_base_table_support_contact_is_allowed_for_robot_environment():
    base_table = classify_contact(
        "panda::panda_link0::collision",
        "table::table_link::top_collision",
        context(),
    )
    base_obstacle = classify_contact(
        "panda::panda_link0::collision",
        "obstacle::obstacle_link::collision",
        context(),
    )
    assert base_table.allowed
    assert base_table.failure_type == "allowed_base_table_support_contact"
    assert not base_obstacle.allowed


def test_carried_cube_contact_is_only_allowed_at_target_placement():
    away = classify_contact(
        "cube::cube_link::cube_collision",
        "table::table_link::top_collision",
        context(carried=True, object_position=(0.40, 0.0, 0.025)),
    )
    at_source = classify_contact(
        "cube::cube_link::cube_collision",
        "table::table_link::top_collision",
        context(carried=True),
    )
    at_target = classify_contact(
        "cube::cube_link::cube_collision",
        "table::table_link::top_collision",
        context(carried=True, object_position=(0.57, 0.17, 0.025)),
    )
    assert not away.allowed
    assert away.failure_type == "carried_object_environment_collision"
    assert at_source.allowed
    assert at_source.failure_type == "allowed_source_pickup_contact"
    assert at_target.allowed
    assert at_target.failure_type == "allowed_target_placement_contact"


def test_unknown_collision_with_robot_is_conservatively_invalid():
    result = classify_contact(
        "panda::panda_link5::collision",
        "new_model::new_link::collision",
        context(),
    )
    assert not result.allowed
    assert result.failure_type == "unknown_monitored_collision"
