from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
from shape_msgs.msg import SolidPrimitive


MODULE_PATH = Path(__file__).with_name("mock_bridge_smoke.py")
SPEC = spec_from_file_location("mock_bridge_smoke", MODULE_PATH)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_collision_box_uses_world_frame_and_exact_geometry():
    collision_object = MODULE.box(
        "obstacle", (0.11, 0.10, 0.28), (0.54, 0.0, 0.14)
    )
    assert collision_object.id == "obstacle"
    assert collision_object.header.frame_id == "world"
    assert collision_object.primitives[0].type == SolidPrimitive.BOX
    assert list(collision_object.primitives[0].dimensions) == [0.11, 0.10, 0.28]
    pose = collision_object.primitive_poses[0]
    assert (pose.position.x, pose.position.y, pose.position.z) == (0.54, 0.0, 0.14)
    assert pose.orientation.w == 1.0


def test_controller_contract_matches_moveit_and_gazebo():
    assert MODULE.REQUIRED_CONTROLLERS == {
        "joint_state_broadcaster",
        "panda_arm_controller",
        "panda_hand_controller",
    }
    assert MODULE.POSE_END_EFFECTOR_LINK == "panda_grasptarget"
    assert MODULE.CUBE_TOUCH_LINKS == (
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger",
    )


def test_table_geometry_matches_v1_collision_scene():
    table = MODULE.box("table", MODULE.TABLE_DIMENSIONS, MODULE.TABLE_POSITION)
    assert list(table.primitives[0].dimensions) == [1.5, 1.0, 0.05]
    pose = table.primitive_poses[0]
    assert (pose.position.x, pose.position.y, pose.position.z) == (
        0.5,
        0.0,
        -0.025,
    )


def test_acm_pair_requires_explicit_symmetric_entries():
    matrix = AllowedCollisionMatrix()
    matrix.entry_names = ["panda_link0", "table"]
    first_row = AllowedCollisionEntry()
    first_row.enabled = [False, True]
    second_row = AllowedCollisionEntry()
    second_row.enabled = [True, False]
    matrix.entry_values = [first_row, second_row]

    assert MODULE.acm_pair_allowed(matrix, "table", "panda_link0")
    second_row.enabled[0] = False
    assert not MODULE.acm_pair_allowed(matrix, "table", "panda_link0")
    assert not MODULE.acm_pair_allowed(matrix, "table", "missing_link")


def test_planning_only_targets_match_shared_task_geometry():
    assert MODULE.PLANNING_ONLY_TARGETS == {
        "pre_grasp": (0.59, 0.22, 0.24),
        "grasp": (0.59, 0.22, 0.075),
        "transfer_hover": (0.59, -0.225, 0.24),
        "place": (0.59, -0.225, 0.09),
    }
    for position in MODULE.PLANNING_ONLY_TARGETS.values():
        target = MODULE.pose_target(position, MODULE.DOWNWARD_ORIENTATION)
        assert target.header.frame_id == "world"
        assert (
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ) == position
        assert (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) == MODULE.DOWNWARD_ORIENTATION


def test_bridge_reads_back_the_exact_allowed_collision_matrix_state():
    source = (MODULE_PATH.parents[1] / "src" / "moveit_bridge_node.cpp").read_text()
    assert "wait_for_allowed_collisions(" in source
    assert "allowed_collisions_match(" in source
    assert "allowed_collision_links_for_object(" in source
    assert "getAllEntryNames" in source
    assert "observed == expected" in source
    assert "temporary support collision allowed" in source


def test_world_object_removal_is_idempotent_but_still_cleans_acm():
    source = (MODULE_PATH.parents[1] / "src" / "moveit_bridge_node.cpp").read_text()
    remove_body = source.split("void remove_world_object(", 1)[1].split(
        "void set_attached_object(", 1
    )[0]
    assert "currently_present" in remove_body
    assert "world object already absent and ACM synchronized" in remove_body
    assert "update_allowed_collisions(request->object_id, {}, true)" in remove_body


def test_transient_planning_retries_share_one_budget_and_accumulate_time():
    source = (MODULE_PATH.parents[1] / "src" / "moveit_bridge_node.cpp").read_text()
    defaults = (MODULE_PATH.parents[1] / "config" / "bridge_defaults.yaml").read_text()
    assert 'parameter<int>("max_planning_attempts", 3)' in source
    assert "max_planning_attempts: 3" in defaults
    assert "const double remaining_sec = planning_budget_sec - elapsed_sec" in source
    assert "move_group_->setPlanningTime(remaining_sec)" in source
    assert "std::chrono::steady_clock::now() - attempt_started" in source
    assert "result->planning_time_sec = accumulated_planning_time_sec" in source
    retry_body = source.split("static bool retryable_planning_failure(", 1)[1].split(
        "static void publish_feedback(", 1
    )[0]
    assert "Codes::PLANNING_FAILED" in retry_body
    assert "Codes::INVALID_MOTION_PLAN" in retry_body
    assert "Codes::GOAL_IN_COLLISION" not in retry_body
    assert "Codes::NO_IK_SOLUTION" not in retry_body


def test_executed_pose_uses_single_ik_but_planning_only_keeps_pose_sampling():
    source = (MODULE_PATH.parents[1] / "src" / "moveit_bridge_node.cpp").read_text()
    target_body = source.split("bool target_set = false;", 1)[1].split(
        "publish_feedback(goal_handle, \"planning\")", 1
    )[0]
    assert "if (goal->execute)" in target_body
    assert "setJointValueTarget(" in target_body
    assert "pose_target, end_effector_link)" in target_body
    assert "setPoseTarget(pose_target, end_effector_link)" in target_body
    assert "result->failure_type = pose_uses_single_ik ?" in target_body
    assert '"no_ik_solution" : "target_rejected"' in target_body
