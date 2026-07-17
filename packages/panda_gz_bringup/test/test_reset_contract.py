import pytest

from panda_gz_bringup.reset_contract import (
    ResetSpec,
    ScenePoseConfig,
    entity_pose_map,
    pose_error,
    require_world_frame,
    vector_norm,
)


def spec(scene="obstacle", grasp_mode="assisted"):
    return ResetSpec.create(
        episode_id=7,
        task_seed=42,
        scene=scene,
        grasp_mode=grasp_mode,
        object_position=(0.59, 0.22, 0.025),
        target_position=(0.59, -0.225, 0.025),
    )


CONFIG = ScenePoseConfig(
    target_marker_z=0.001,
    obstacle_pose=(0.54, 0.0, 0.14, 0.0, 0.0, 0.0, 1.0),
    clear_obstacle_pose=(0.54, 2.0, 0.14, 0.0, 0.0, 0.0, 1.0),
)


def test_obstacle_task_maps_all_models_deterministically():
    poses = entity_pose_map(spec(), CONFIG)
    assert poses["cube"][:3] == (0.59, 0.22, 0.025)
    assert poses["target"][:3] == (0.59, -0.225, 0.001)
    assert poses["obstacle"] == CONFIG.obstacle_pose


def test_clear_task_parks_obstacle_outside_workspace():
    poses = entity_pose_map(spec(scene="clear"), CONFIG)
    assert poses["obstacle"] == CONFIG.clear_obstacle_pose


@pytest.mark.parametrize(
    ("scene", "grasp_mode"),
    [("unknown", "assisted"), ("clear", "learned")],
)
def test_invalid_task_enums_are_rejected(scene, grasp_mode):
    with pytest.raises(ValueError):
        spec(scene=scene, grasp_mode=grasp_mode)


def test_non_finite_positions_are_rejected():
    with pytest.raises(ValueError):
        ResetSpec.create(
            episode_id=0,
            task_seed=0,
            scene="clear",
            grasp_mode="physics",
            object_position=(float("nan"), 0.0, 0.025),
            target_position=(0.5, 0.2, 0.025),
        )


@pytest.mark.parametrize("frame_id", ["", "world"])
def test_reset_coordinates_accept_only_the_world_frame(frame_id):
    require_world_frame(frame_id)


@pytest.mark.parametrize("frame_id", ["map", "panda_link0", "/world"])
def test_reset_coordinates_reject_ambiguous_frames(frame_id):
    with pytest.raises(ValueError, match="header.frame_id"):
        require_world_frame(frame_id)


def test_pose_and_twist_metrics_are_finite_and_quaternion_sign_invariant():
    translation, rotation = pose_error(
        (0.501, -0.2, 0.025),
        (0.0, 0.0, 0.0, -1.0),
        (0.5, -0.2, 0.025, 0.0, 0.0, 0.0, 1.0),
    )
    assert translation == pytest.approx(0.001)
    assert rotation == pytest.approx(0.0)
    assert vector_norm((0.003, 0.004, 0.0)) == pytest.approx(0.005)
    assert vector_norm((float("nan"), 0.0, 0.0)) == float("inf")
