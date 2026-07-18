import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
import xacro

from panda_gz_bringup.model_validation import (
    ARM_JOINTS,
    DYNAMIC_LINKS,
    FINGER_JOINTS,
    audit_model,
    compose_world_with_deferred_control,
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
    for link_name in DYNAMIC_LINKS:
        link = ET.SubElement(model, "link", name=link_name)
        sensor = ET.SubElement(
            link,
            "sensor",
            name=f"{link_name}_contact_sensor",
            type="contact",
        )
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "topic").text = "/panda/robot_contacts"
    ET.SubElement(model, "link", name="panda_grasptarget")
    for joint_name in ARM_JOINTS + FINGER_JOINTS:
        ET.SubElement(model, "joint", name=joint_name, type="revolute")
    ET.SubElement(
        model,
        "plugin",
        filename="gz_ros2_control-system",
        name="gz_ros2_control::GazeboSimROS2ControlPlugin",
    )
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


def test_sensor_level_contact_topic_fails_closed():
    root = ET.fromstring(_sdf())
    sensor = root.find(".//sensor[@name='panda_link3_contact_sensor']")
    contact = sensor.find("contact")
    topic = contact.findtext("topic")
    sensor.remove(contact)
    ET.SubElement(sensor, "topic").text = topic
    with pytest.raises(ValueError, match="type-specific topic"):
        validate_sdf(ET.tostring(root, encoding="unicode"))


def test_initial_world_keeps_contacts_and_defers_control_plugin(tmp_path):
    base_world = tmp_path / "base.sdf"
    base_world.write_text(
        '<sdf version="1.8"><world name="test">'
        '<plugin filename="libignition-gazebo-contact-system.so" '
        'name="ignition::gazebo::systems::Contact" />'
        "</world></sdf>"
    )
    output = compose_world_with_deferred_control(
        base_world,
        _sdf(),
        output_directory=tmp_path / "generated",
    )
    root = ET.parse(output).getroot()
    panda = root.find("./world/model[@name='panda']")
    assert panda is not None
    assert len(panda.findall(".//sensor[@type='contact']")) == len(
        DYNAMIC_LINKS
    )
    assert panda.find(
        "./plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"
    ) is None
    assert root.find(
        "./world/plugin[@name='ignition::gazebo::systems::Contact']"
    ) is not None
    assert compose_world_with_deferred_control(
        base_world,
        _sdf(),
        output_directory=tmp_path / "generated",
    ) == output


def test_generated_world_camera_matches_shared_yaml(tmp_path):
    base_world = tmp_path / "base.sdf"
    base_world.write_text('<sdf version="1.8"><world name="test" /></sdf>')
    perception = ROOT.parent / "panda_perception" / "config" / "perception.yaml"
    camera = yaml.safe_load(perception.read_text())["camera"]

    output = compose_world_with_deferred_control(
        base_world,
        _sdf(),
        output_directory=tmp_path / "generated",
        camera_config=camera,
    )

    root = ET.parse(output).getroot()
    model = root.find("./world/model[@name='overhead_camera']")
    sensor = model.find("./link/sensor[@type='rgbd_camera']")
    assert model.findtext("pose") == " ".join(
        str(float(value)) for value in (*camera["position"], *camera["rpy"])
    )
    assert sensor.findtext("update_rate") == str(camera["update_rate_hz"])
    assert sensor.findtext("camera/image/width") == str(camera["width"])
    assert sensor.findtext("camera/image/height") == str(camera["height"])


def test_rect_prism_world_preserves_entity_contract_and_updates_inertia(tmp_path):
    output = compose_world_with_deferred_control(
        ROOT / "worlds" / "panda_pick_place.sdf",
        _sdf(),
        output_directory=tmp_path,
        object_variant="rect_prism",
    )

    root = ET.parse(output).getroot()
    model = root.find("./world/model[@name='cube']")
    assert model.find("./link[@name='cube_link']") is not None
    sizes = {node.text for node in model.findall(".//geometry/box/size")}
    assert sizes == {"0.06 0.04 0.05"}
    assert float(model.findtext("./link/inertial/inertia/ixx")) > 0.0
    assert float(model.findtext("./link/inertial/inertia/iyy")) > 0.0
    assert float(model.findtext("./link/inertial/inertia/izz")) > 0.0


def test_three_rect_prism_worlds_have_independent_transport_contracts(tmp_path):
    output = compose_world_with_deferred_control(
        ROOT / "worlds" / "panda_pick_place.sdf",
        _sdf(),
        output_directory=tmp_path,
        object_variant="rect_prism",
        workpiece_count=3,
    )
    world = ET.parse(output).getroot().find("world")
    for index, model_name in enumerate(("cube", "cube_1", "cube_2")):
        model = world.find(f"./model[@name='{model_name}']")
        assert model is not None
        suffix = "" if index == 0 else f"_{index}"
        assert model.find(f"./link[@name='cube{suffix}_link']") is not None
        assert model.findtext(".//contact/topic") == f"/cube{suffix}/contacts"
        assert model.findtext(
            "./plugin[@name='ignition::gazebo::systems::OdometryPublisher']/odom_topic"
        ) == f"/model/cube{suffix}/odometry"


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
