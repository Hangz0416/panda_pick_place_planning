import math

from panda_gz_bringup.grasp_gate import GateLimits, evaluate_gate, quaternion_angle_error


LIMITS = GateLimits(
    expected_hand_offset_xyz=(0.0, 0.0, 0.075),
    expected_hand_quaternion_xyzw=(0.9238795, -0.3826834, 0.0, 0.0),
    max_position_error_m=0.03,
    max_orientation_error_rad=0.30,
    max_hand_object_distance_m=0.09,
    expected_gripper_width_m=0.060,
    gripper_width_tolerance_m=0.008,
)


def test_gate_accepts_clearance_pose():
    result = evaluate_gate(
        (0.6, 0.2, 0.10),
        LIMITS.expected_hand_quaternion_xyzw,
        (0.6, 0.2, 0.025),
        0.06,
        LIMITS,
    )
    assert result.accepted
    assert result.reason == "accepted"


def test_gate_rejects_contact_width_before_detachable_attach():
    result = evaluate_gate(
        (0.6, 0.2, 0.10),
        LIMITS.expected_hand_quaternion_xyzw,
        (0.6, 0.2, 0.025),
        0.05,
        LIMITS,
    )
    assert not result.accepted
    assert result.reason == "gripper_too_narrow"


def test_gate_rejects_wrong_hand_orientation():
    result = evaluate_gate(
        (0.6, 0.2, 0.10),
        (0.0, 0.0, 0.0, 1.0),
        (0.6, 0.2, 0.025),
        0.06,
        LIMITS,
    )
    assert not result.accepted
    assert result.reason == "hand_pose_orientation_error"


def test_quaternion_error_is_sign_invariant():
    quaternion = (0.5, 0.5, 0.5, 0.5)
    opposite = tuple(-value for value in quaternion)
    assert math.isclose(quaternion_angle_error(quaternion, opposite), 0.0)
