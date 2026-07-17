from sensor_msgs.msg import JointState

from panda_gz_bringup.joint_state_canonicalizer import (
    canonical_joint_name,
    canonicalize_joint_state,
)


def test_only_fortress_panda_mimic_name_is_rewritten():
    assert canonical_joint_name("panda_finger_joint2_mimic") == (
        "panda_finger_joint2"
    )
    assert canonical_joint_name("panda_joint1") == "panda_joint1"
    assert canonical_joint_name("unrelated_mimic") == "unrelated_mimic"


def test_joint_state_values_and_order_are_preserved():
    raw = JointState()
    raw.name = ["panda_joint1", "panda_finger_joint2_mimic"]
    raw.position = [0.1, 0.04]
    raw.velocity = [0.2, 0.0]
    raw.effort = [0.3, 0.0]

    canonical = canonicalize_joint_state(raw)

    assert canonical.name == ["panda_joint1", "panda_finger_joint2"]
    assert canonical.position == raw.position
    assert canonical.velocity == raw.velocity
    assert canonical.effort == raw.effort


def test_duplicate_canonical_name_fails_closed():
    raw = JointState()
    raw.name = ["panda_finger_joint2", "panda_finger_joint2_mimic"]
    raw.position = [0.04, 0.04]

    try:
        canonicalize_joint_state(raw)
    except ValueError as exc:
        assert "not unique" in str(exc)
    else:
        raise AssertionError("duplicate canonical joint state was accepted")
