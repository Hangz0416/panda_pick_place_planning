import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro

from panda_gz_bringup.model_validation import (
    ARM_JOINTS,
    DYNAMIC_LINKS,
    FINGER_JOINTS,
    audit_model,
    convert_urdf_to_sdf,
    validate_sdf,
    validate_urdf,
)


ROOT = Path(__file__).parents[1]
MOVEIT = ROOT.parent / "panda_moveit_config"


def _urdf() -> str:
    robot = ET.Element("robot", name="panda")
    for link_name in (*DYNAMIC_LINKS, "panda_grasptarget"):
        link = ET.SubElement(robot, "link", name=link_name)
        if link_name in DYNAMIC_LINKS:
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass", value="1.0")
            ET.SubElement(
                inertial,
                "inertia",
                ixx="1.0",
                ixy="0.0",
                ixz="0.0",
                iyy="1.0",
                iyz="0.0",
                izz="1.0",
            )
    for joint_name in ARM_JOINTS + FINGER_JOINTS:
        ET.SubElement(robot, "joint", name=joint_name, type="revolute")
    return ET.tostring(robot, encoding="unicode")


def _sdf() -> str:
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name="panda")
    for link_name in ("panda_hand", "panda_grasptarget"):
        ET.SubElement(model, "link", name=link_name)
    for joint_name in ARM_JOINTS + FINGER_JOINTS:
        ET.SubElement(model, "joint", name=joint_name, type="revolute")
    return ET.tostring(sdf, encoding="unicode")


def test_valid_urdf_and_sdf_pass():
    assert validate_urdf(_urdf()).tag == "robot"
    assert validate_sdf(_sdf()).tag == "sdf"


def test_non_positive_mass_fails_closed():
    root = ET.fromstring(_urdf())
    root.find("./link[@name='panda_link3']/inertial/mass").set("value", "0")
    with pytest.raises(ValueError, match="non-positive mass: panda_link3"):
        validate_urdf(ET.tostring(root, encoding="unicode"))


def test_non_positive_definite_inertia_fails_closed():
    root = ET.fromstring(_urdf())
    inertia = root.find("./link[@name='panda_link4']/inertial/inertia")
    inertia.set("ixy", "2.0")
    with pytest.raises(ValueError, match="not positive definite: panda_link4"):
        validate_urdf(ET.tostring(root, encoding="unicode"))


def test_missing_sdf_task_frame_fails_closed():
    root = ET.fromstring(_sdf())
    model = root.find("model")
    model.remove(model.find("./link[@name='panda_grasptarget']"))
    with pytest.raises(ValueError, match="panda_grasptarget"):
        validate_sdf(ET.tostring(root, encoding="unicode"))


def test_shared_gazebo_xacro_converts_to_valid_sdf():
    document = xacro.process_file(
        str(MOVEIT / "config" / "panda.urdf.xacro"),
        mappings={
            "hardware_type": "gz",
            "initial_positions_file": str(
                MOVEIT / "config" / "initial_positions.yaml"
            ),
            "controllers_file": str(ROOT / "config" / "controllers.yaml"),
            "assisted_grasp": "true",
        },
    )
    urdf_xml = document.toxml()
    audit = audit_model(urdf_xml, convert_urdf_to_sdf(urdf_xml))
    assert audit.arm_joint_count == 7
    assert audit.finger_joint_count == 2
    assert audit.dynamic_link_count == 11
    assert "panda_hand" in audit.sdf_frame_names
    assert "panda_grasptarget" in audit.sdf_frame_names
