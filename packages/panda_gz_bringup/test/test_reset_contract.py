import pytest
import math
from types import SimpleNamespace

from control_msgs.action import FollowJointTrajectory

from panda_gz_bringup.reset_contract import (
    ResetSpec,
    ResetWorkpieceSpec,
    ScenePoseConfig,
    entity_pose_map,
    pose_error,
    require_world_frame,
    vector_norm,
)
from panda_gz_bringup.episode_reset_manager import (
    EpisodeResetManager,
    reset_failure_type,
)
from panda_gz_bringup.status import StatusCode


def test_multi_object_reset_has_explicit_failure_classification():
    assert (
        reset_failure_type(False, StatusCode.RESET_STATE_INVALID, 3)
        == "multi_object_reset_failed"
    )
    assert (
        reset_failure_type(False, StatusCode.RESET_STATE_INVALID, 1)
        == "reset_state_invalid"
    )
    assert reset_failure_type(True, StatusCode.RESET_SUCCEEDED, 3) == ""


class _HomeGoalHandle:
    accepted = True

    def __init__(self, error_code):
        self.error_code = error_code

    def get_result_async(self):
        return SimpleNamespace(
            result=SimpleNamespace(
                error_code=self.error_code,
                error_string="path tolerance violated",
            )
        )


class _HomeActionClient:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.goals = []

    def wait_for_server(self, timeout_sec):
        return timeout_sec > 0.0

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _HomeGoalHandle(self.outcomes.pop(0))


class _Logger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class _HomeManager:
    def __init__(self, outcomes, attempts=3):
        self._home_client = _HomeActionClient(outcomes)
        self._logger = _Logger()
        self._parameters = {
            "home_timeout_sec": 8.0,
            "home_joints": [0.0, -0.65, 0.0, -2.2, 0.0, 1.55, 0.78],
            "home_duration_sec": 3.0,
            "home_max_attempts": attempts,
        }

    def get_parameter(self, name):
        return SimpleNamespace(value=self._parameters[name])

    def get_logger(self):
        return self._logger

    @staticmethod
    def _wait_future(value, timeout):
        return value if timeout > 0.0 else None

    @staticmethod
    def _publish_status(*_args):
        return None


def test_home_reset_retries_controller_path_tolerance_abort_within_episode():
    manager = _HomeManager(
        [
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            FollowJointTrajectory.Result.SUCCESSFUL,
        ]
    )

    assert EpisodeResetManager._move_home(manager) is True
    assert len(manager._home_client.goals) == 3
    assert len(manager._logger.warnings) == 2
    assert manager._logger.errors == []


def test_home_reset_still_fails_closed_after_bounded_retries():
    manager = _HomeManager(
        [FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED] * 3
    )

    assert EpisodeResetManager._move_home(manager) is False
    assert len(manager._home_client.goals) == 3
    assert len(manager._logger.errors) == 1


def test_three_workpiece_reset_pose_map_preserves_identity_and_yaw():
    workpieces = tuple(
        ResetWorkpieceSpec.create(
            f"workpiece_{index}",
            "rect_prism",
            (0.46 + 0.11 * index, -0.17, 0.025),
            (0.0, 0.0, math.sin(0.1 * index), math.cos(0.1 * index)),
        )
        for index in range(3)
    )
    value = ResetSpec.create(
        episode_id=7,
        task_seed=42,
        scene="clear",
        grasp_mode="assisted",
        object_position=workpieces[0].position,
        target_position=(0.57, 0.17, 0.025),
        object_variant="rect_prism",
        workpieces=workpieces,
    )

    poses = entity_pose_map(value, CONFIG)
    assert set(poses) == {
        "workpiece_0", "workpiece_1", "workpiece_2", "target", "obstacle"
    }
    assert poses["workpiece_2"] == (
        *workpieces[2].position,
        *workpieces[2].orientation_xyzw,
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


def test_rect_prism_reset_maps_requested_yaw_to_quaternion():
    rotated = ResetSpec.create(
        episode_id=7,
        task_seed=42,
        scene="clear",
        grasp_mode="assisted",
        object_position=(0.59, 0.22, 0.025),
        target_position=(0.59, -0.225, 0.025),
        object_variant="rect_prism",
        object_yaw_rad=0.6,
    )

    quaternion = entity_pose_map(rotated, CONFIG)["cube"][3:]

    assert quaternion == pytest.approx(
        (0.0, 0.0, math.sin(0.3), math.cos(0.3))
    )


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
