import math

import pytest

from panda_gz_bringup.physical_grasp_classifier import (
    PhysicalGraspClassifier,
    PhysicalGraspConfig,
    PoseSample,
    relative_pose,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def pose(x=0.0, y=0.0, z=0.0, quaternion=IDENTITY):
    return PoseSample.create((x, y, z), quaternion)


def test_bilateral_contact_is_required_and_debounced():
    classifier = PhysicalGraspClassifier()
    hand = pose(z=0.075)
    cube = pose(z=0.025)

    first = classifier.update(
        timestamp_sec=0.0,
        left_finger_contact=True,
        right_finger_contact=False,
        hand_pose=hand,
        object_pose=cube,
    )
    assert not first.grasped
    classifier.update(
        timestamp_sec=0.01,
        left_finger_contact=True,
        right_finger_contact=True,
        hand_pose=hand,
        object_pose=cube,
    )
    confirmed = classifier.update(
        timestamp_sec=0.12,
        left_finger_contact=True,
        right_finger_contact=True,
        hand_pose=hand,
        object_pose=cube,
    )
    assert confirmed.grasped
    assert confirmed.holding_object


def test_rigid_motion_does_not_look_like_slip_but_relative_motion_does():
    classifier = PhysicalGraspClassifier(
        PhysicalGraspConfig(bilateral_contact_confirm_sec=0.0)
    )
    classifier.update(
        timestamp_sec=0.0,
        left_finger_contact=True,
        right_finger_contact=True,
        hand_pose=pose(z=0.075),
        object_pose=pose(z=0.025),
    )
    rigid = classifier.update(
        timestamp_sec=1.0,
        left_finger_contact=True,
        right_finger_contact=True,
        hand_pose=pose(x=0.2, z=0.20),
        object_pose=pose(x=0.2, z=0.15),
    )
    assert not rigid.slipping
    assert rigid.max_relative_translation_drift_m == pytest.approx(0.0)

    slipped = classifier.update(
        timestamp_sec=2.0,
        left_finger_contact=True,
        right_finger_contact=True,
        hand_pose=pose(x=0.2, z=0.20),
        object_pose=pose(x=0.23, z=0.15),
    )
    assert slipped.slipping
    assert slipped.max_relative_translation_drift_m == pytest.approx(0.03)


def test_release_requires_contact_gap_and_reports_stable_duration():
    classifier = PhysicalGraspClassifier(
        PhysicalGraspConfig(
            bilateral_contact_confirm_sec=0.0,
            release_confirm_sec=0.1,
        )
    )
    hand = pose(z=0.075)
    cube = pose(z=0.025)
    classifier.update(
        timestamp_sec=0.0,
        left_finger_contact=True,
        right_finger_contact=True,
        hand_pose=hand,
        object_pose=cube,
    )
    pending = classifier.update(
        timestamp_sec=0.1,
        left_finger_contact=False,
        right_finger_contact=False,
        hand_pose=hand,
        object_pose=cube,
    )
    assert not pending.released
    released = classifier.update(
        timestamp_sec=0.21,
        left_finger_contact=False,
        right_finger_contact=False,
        hand_pose=hand,
        object_pose=cube,
    )
    assert released.released
    assert not released.holding_object
    stable = classifier.update(
        timestamp_sec=0.71,
        left_finger_contact=False,
        right_finger_contact=False,
        hand_pose=hand,
        object_pose=cube,
    )
    assert stable.release_stable_time_sec == pytest.approx(0.5)


def test_relative_pose_and_rotation_are_measured_in_hand_frame():
    half_turn_z = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    relative = relative_pose(
        pose(quaternion=half_turn_z),
        pose(x=1.0, quaternion=half_turn_z),
    )
    assert relative.position_xyz[0] == pytest.approx(0.0, abs=1.0e-9)
    assert relative.position_xyz[1] == pytest.approx(-1.0, abs=1.0e-9)
    assert relative.orientation_xyzw == pytest.approx(IDENTITY)


def test_time_must_be_monotonic_and_reset_changes_episode():
    classifier = PhysicalGraspClassifier()
    classifier.reset(episode_id=7)
    classifier.update(
        timestamp_sec=1.0,
        left_finger_contact=False,
        right_finger_contact=False,
        hand_pose=pose(),
        object_pose=pose(),
    )
    with pytest.raises(ValueError, match="monotonic"):
        classifier.update(
            timestamp_sec=0.5,
            left_finger_contact=False,
            right_finger_contact=False,
            hand_pose=pose(),
            object_pose=pose(),
        )
    classifier.reset(episode_id=8)
    assert classifier.snapshot().episode_id == 8
    assert not classifier.snapshot().initialized
