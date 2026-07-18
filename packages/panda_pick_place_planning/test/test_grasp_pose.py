import math

import pytest

from panda_pick_place_planning.core.backends import BackendMotionReport
from panda_pick_place_planning.core.grasp_pose import (
    GraspPoseGenerator,
    normalize_symmetric_yaw,
    select_feasible_grasp,
)
from panda_pick_place_planning.core.schemas import PlanResult, SceneObjectEstimate


BASE_DOWN = (0.9238795325, -0.3826834324, 0.0, 0.0)


class FeasibilityBackend:
    def __init__(self, lengths):
        self.lengths = iter(lengths)

    def check_pose_feasibility(self, *args, **kwargs):
        length = next(self.lengths)
        if length is None:
            return BackendMotionReport(
                False,
                False,
                failure_type="planning_collision",
                plan_result=PlanResult(False, failure_type="planning_collision"),
            )
        return BackendMotionReport(
            True,
            True,
            path_length_rad=length,
            plan_result=PlanResult(True, path=[[0.0], [length]]),
        )


def estimate(yaw):
    return SceneObjectEstimate(
        "cube",
        (0.57, -0.17, 0.025),
        orientation_xyzw=(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
    )


def test_cube_zero_yaw_preserves_legacy_orientation():
    candidates = GraspPoseGenerator().generate(
        estimate(0.0),
        "cube",
        BASE_DOWN,
        hover_height=0.24,
        grasp_height=0.075,
    )

    assert len(candidates) == 1
    assert candidates[0].source == "legacy_cube_top_grasp"
    assert candidates[0].orientation_xyzw == BASE_DOWN


def test_rect_prism_generates_pi_symmetric_candidates():
    candidates = GraspPoseGenerator().generate(
        estimate(math.radians(35.0)),
        "rect_prism",
        BASE_DOWN,
        hover_height=0.24,
        grasp_height=0.075,
    )

    assert len(candidates) == 2
    assert {candidate.source for candidate in candidates} == {
        "rgbd_rect_prism_yaw",
        "rgbd_rect_prism_yaw_pi_symmetric",
    }
    assert candidates[1].yaw_rad - candidates[0].yaw_rad == pytest.approx(math.pi)


def test_selection_uses_shortest_feasible_path_and_stable_tie_break():
    candidates = GraspPoseGenerator().generate(
        estimate(0.2),
        "rect_prism",
        BASE_DOWN,
        hover_height=0.24,
        grasp_height=0.075,
    )

    selected = select_feasible_grasp(
        candidates,
        FeasibilityBackend([2.0, 1.0]),
        planner="rrt_connect",
        seed=42,
        timeout_sec=3.0,
    )

    assert selected.success
    assert selected.candidate.index == 1
    assert selected.path_length_rad == 1.0


def test_all_collision_failures_are_typed():
    candidates = GraspPoseGenerator().generate(
        estimate(0.2),
        "rect_prism",
        BASE_DOWN,
        hover_height=0.24,
        grasp_height=0.075,
    )

    selected = select_feasible_grasp(
        candidates,
        FeasibilityBackend([None, None]),
        planner="rrt_connect",
        seed=42,
        timeout_sec=3.0,
    )

    assert not selected.success
    assert selected.failure_type == "all_grasp_candidates_in_collision"


@pytest.mark.parametrize("angle", [-4.0, -1.2, 0.0, 2.4, 5.0])
def test_symmetric_yaw_normalization(angle):
    normalized = normalize_symmetric_yaw(angle)
    assert -math.pi / 2.0 <= normalized < math.pi / 2.0
