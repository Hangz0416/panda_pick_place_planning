"""Validate the shared Panda Xacro before Gazebo runtime is attempted."""

from __future__ import annotations

from dataclasses import dataclass
import copy
import hashlib
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))
FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
DYNAMIC_LINKS = (
    "panda_link0",
    "panda_link1",
    "panda_link2",
    "panda_link3",
    "panda_link4",
    "panda_link5",
    "panda_link6",
    "panda_link7",
    "panda_hand",
    "panda_leftfinger",
    "panda_rightfinger",
)
REQUIRED_FRAMES = ("panda_hand", "panda_grasptarget")
ROBOT_CONTACT_TOPIC = "/panda/robot_contacts"
GZ_CONTROL_PLUGIN_NAME = "gz_ros2_control::GazeboSimROS2ControlPlugin"
GZ_CONTROL_PLUGIN_FILENAME = "gz_ros2_control-system"


@dataclass(frozen=True)
class ModelAudit:
    """Machine-readable facts produced by a successful validation."""

    arm_joint_count: int
    finger_joint_count: int
    dynamic_link_count: int
    sdf_joint_count: int
    sdf_frame_names: tuple[str, ...]


def _positive_definite(inertia: ET.Element) -> bool:
    """Check a symmetric 3x3 inertia using Sylvester's criterion."""
    values = {
        key: float(inertia.attrib[key])
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    }
    xx = values["ixx"]
    xy = values["ixy"]
    xz = values["ixz"]
    yy = values["iyy"]
    yz = values["iyz"]
    zz = values["izz"]
    leading_two = xx * yy - xy * xy
    determinant = (
        xx * (yy * zz - yz * yz)
        - xy * (xy * zz - yz * xz)
        + xz * (xy * yz - yy * xz)
    )
    return xx > 0.0 and leading_two > 0.0 and determinant > 0.0


def validate_urdf(urdf_xml: str) -> ET.Element:
    """Validate joints, frames, mass, and inertia in expanded Xacro XML."""
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"expected URDF robot root, got {root.tag!r}")
    joints = {joint.attrib.get("name"): joint for joint in root.findall("joint")}
    links = {link.attrib.get("name"): link for link in root.findall("link")}
    missing_joints = sorted(set(ARM_JOINTS + FINGER_JOINTS) - joints.keys())
    if missing_joints:
        raise ValueError(f"URDF missing required joints: {missing_joints}")
    missing_frames = sorted(set(REQUIRED_FRAMES) - links.keys())
    if missing_frames:
        raise ValueError(f"URDF missing required links: {missing_frames}")
    for link_name in DYNAMIC_LINKS:
        link = links.get(link_name)
        if link is None:
            raise ValueError(f"URDF missing dynamic link: {link_name}")
        inertial = link.find("inertial")
        if inertial is None:
            raise ValueError(f"URDF link has no inertial: {link_name}")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is None or float(mass.attrib.get("value", "0")) <= 0.0:
            raise ValueError(f"URDF link has non-positive mass: {link_name}")
        if inertia is None or not _positive_definite(inertia):
            raise ValueError(
                f"URDF link inertia is not positive definite: {link_name}"
            )
    return root


def sdf_command() -> list[str]:
    """Return the installed sdformat pretty-print command for Fortress/Jammy."""
    ignition = shutil.which("ign")
    if ignition:
        return [ignition, "sdf", "-p"]
    gz = shutil.which("gz")
    if gz:
        return [gz, "sdf", "-p"]
    raise RuntimeError("neither 'ign sdf' nor 'gz sdf' is installed")


def convert_urdf_to_sdf(urdf_xml: str) -> str:
    """Convert an expanded URDF through the system libsdformat CLI."""
    with tempfile.TemporaryDirectory(prefix="panda_model_audit_") as directory:
        urdf_path = Path(directory) / "panda.urdf"
        urdf_path.write_text(urdf_xml, encoding="utf-8")
        completed = subprocess.run(
            [*sdf_command(), str(urdf_path)],
            check=False,
            capture_output=True,
            text=True,
        )
    if completed.returncode != 0:
        raise RuntimeError(
            "URDF to SDF conversion failed: " + completed.stderr.strip()
        )
    if not completed.stdout.strip():
        raise RuntimeError("URDF to SDF conversion produced empty output")
    return completed.stdout


def compose_world_with_deferred_control(
    world_path: Path,
    robot_sdf: str,
    *,
    output_directory: Path | None = None,
    camera_config: dict | None = None,
    object_variant: str = "cube",
    workpiece_count: int = 1,
) -> Path:
    """Put Panda in the initial world but defer its control system plugin."""
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"Gazebo world has no <world>: {world_path}")
    robot_root = ET.fromstring(robot_sdf)
    robot_model = (
        robot_root
        if robot_root.tag == "model"
        else robot_root.find("model")
    )
    if robot_model is None or robot_model.attrib.get("name") != "panda":
        raise ValueError("derived Panda SDF has no model named 'panda'")
    plugins = [
        item
        for item in robot_model.findall("plugin")
        if item.attrib.get("name") == GZ_CONTROL_PLUGIN_NAME
    ]
    if len(plugins) != 1:
        raise ValueError("Panda SDF must have exactly one gz_ros2_control plugin")
    if plugins[0].attrib.get("filename") != GZ_CONTROL_PLUGIN_FILENAME:
        raise ValueError("Panda gz_ros2_control plugin filename mismatch")
    robot_model.remove(plugins[0])
    if world.find("./model[@name='panda']") is not None:
        raise ValueError("base Gazebo world already contains a Panda model")
    _configure_object_variant(world, object_variant)
    _configure_workpiece_instances(world, workpiece_count)
    if camera_config is not None:
        _append_rgbd_camera(world, camera_config)
    world.append(robot_model)
    xml = ET.tostring(root, encoding="unicode", xml_declaration=True)
    digest = hashlib.sha256(xml.encode("utf-8")).hexdigest()[:16]
    directory = output_directory or (
        Path(tempfile.gettempdir()) / "panda_gz_bringup"
    )
    directory.mkdir(parents=True, exist_ok=True)
    output_path = directory / f"panda_pick_place_{digest}.sdf"
    if not output_path.exists() or output_path.read_text() != xml:
        output_path.write_text(xml, encoding="utf-8")
    return output_path


def _configure_object_variant(world: ET.Element, object_variant: str) -> None:
    """Select audited object geometry while preserving cube/cube_link names."""
    sizes = {
        "cube": (0.05, 0.05, 0.05),
        "rect_prism": (0.06, 0.04, 0.05),
    }
    if object_variant not in sizes:
        raise ValueError(f"unsupported object variant: {object_variant}")
    model = world.find("./model[@name='cube']")
    if model is None:
        if object_variant == "cube":
            return
        raise ValueError("rect_prism world requires a cube model contract")
    size = sizes[object_variant]
    for size_node in model.findall(".//geometry/box/size"):
        size_node.text = " ".join(str(value) for value in size)
    mass = float(model.findtext("./link/inertial/mass", "0.05"))
    x_size, y_size, z_size = size
    inertia = model.find("./link/inertial/inertia")
    if inertia is None:
        raise ValueError("grasp object has no inertia")
    values = {
        "ixx": mass * (y_size**2 + z_size**2) / 12.0,
        "iyy": mass * (x_size**2 + z_size**2) / 12.0,
        "izz": mass * (x_size**2 + y_size**2) / 12.0,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }
    for name, value in values.items():
        node = inertia.find(name)
        if node is None:
            raise ValueError(f"grasp object inertia has no {name}")
        node.text = f"{value:.12g}"


def _configure_workpiece_instances(world: ET.Element, workpiece_count: int) -> None:
    if workpiece_count not in {1, 3}:
        raise ValueError("workpiece_count must be one or three")
    if workpiece_count == 1:
        return
    base = world.find("./model[@name='cube']")
    if base is None:
        raise ValueError("multi-workpiece world requires cube model")
    base.find("pose").text = "0.46 -0.17 0.025 0 0 0"
    for index, x_value in ((1, 0.57), (2, 0.68)):
        model = copy.deepcopy(base)
        old = "cube"
        new = f"cube_{index}"
        model.find("pose").text = f"{x_value} -0.17 0.025 0 0 0"
        for element in model.iter():
            if "name" in element.attrib:
                element.attrib["name"] = element.attrib["name"].replace(old, new)
            if element.text:
                if f"/{old}/" in element.text:
                    element.text = element.text.replace(
                        f"/model/{old}/", f"/model/{new}/"
                    ).replace(f"/{old}/", f"/{new}/")
                else:
                    element.text = element.text.replace(f"{old}_", f"{new}_")
        model.attrib["name"] = new
        world.append(model)


def _append_rgbd_camera(world: ET.Element, config: dict) -> None:
    """Append the fixed RGB-D model generated from perception.yaml."""
    name = str(config["model_name"])
    if world.find(f"./model[@name='{name}']") is not None:
        raise ValueError(f"base Gazebo world already contains camera {name!r}")
    position = tuple(float(value) for value in config["position"])
    rpy = tuple(float(value) for value in config["rpy"])
    if len(position) != 3 or len(rpy) != 3:
        raise ValueError("camera position and rpy must contain three values")
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(
        str(value) for value in (*position, *rpy)
    )
    link = ET.SubElement(model, "link", {"name": str(config["link_name"])})
    visual = ET.SubElement(link, "visual", {"name": "camera_visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = "0.06 0.04 0.04"
    sensor = ET.SubElement(
        link,
        "sensor",
        {"name": str(config["sensor_name"]), "type": "rgbd_camera"},
    )
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(config["update_rate_hz"])
    ET.SubElement(sensor, "visualize").text = "false"
    ET.SubElement(sensor, "topic").text = f"/{name}"
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = str(
        config["horizontal_fov_rad"]
    )
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = str(config["width"])
    ET.SubElement(image, "height").text = str(config["height"])
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = str(config["near_clip_m"])
    ET.SubElement(clip, "far").text = str(config["far_clip_m"])


def validate_sdf(sdf_xml: str) -> ET.Element:
    """Verify required joints and task frames survived URDF-to-SDF conversion."""
    root = ET.fromstring(sdf_xml)
    model = root if root.tag == "model" else root.find(".//model")
    if model is None:
        raise ValueError("converted SDF has no model")
    joint_names = {item.attrib.get("name") for item in model.findall(".//joint")}
    required_joints = set(ARM_JOINTS + FINGER_JOINTS)
    missing_joints = sorted(required_joints - joint_names)
    if missing_joints:
        raise ValueError(f"SDF missing required joints: {missing_joints}")
    frame_names = {
        item.attrib.get("name")
        for tag in ("link", "frame")
        for item in model.findall(f".//{tag}")
    }
    missing_frames = sorted(set(REQUIRED_FRAMES) - frame_names)
    if missing_frames:
        raise ValueError(f"SDF missing required links/frames: {missing_frames}")
    contact_sensors = model.findall(".//sensor[@type='contact']")
    if len(contact_sensors) != len(DYNAMIC_LINKS):
        raise ValueError(
            "SDF contact sensor count mismatch: "
            f"expected {len(DYNAMIC_LINKS)}, got {len(contact_sensors)}"
        )
    for sensor in contact_sensors:
        topic = sensor.findtext("./contact/topic")
        if topic != ROBOT_CONTACT_TOPIC:
            raise ValueError(
                "SDF contact sensor has invalid type-specific topic: "
                f"{sensor.attrib.get('name')}: {topic!r}"
            )
    control_plugins = [
        plugin
        for plugin in model.findall(".//plugin")
        if plugin.attrib.get("name") == GZ_CONTROL_PLUGIN_NAME
    ]
    if len(control_plugins) != 1:
        raise ValueError("SDF Panda must own exactly one gz_ros2_control plugin")
    if control_plugins[0].attrib.get("filename") != GZ_CONTROL_PLUGIN_FILENAME:
        raise ValueError("SDF Panda gz_ros2_control plugin filename mismatch")
    return root


def audit_model(urdf_xml: str, sdf_xml: str) -> ModelAudit:
    """Validate both representations and return stable audit counts."""
    validate_urdf(urdf_xml)
    sdf_root = validate_sdf(sdf_xml)
    model = sdf_root if sdf_root.tag == "model" else sdf_root.find(".//model")
    assert model is not None
    joint_names = {
        item.attrib.get("name") for item in model.findall(".//joint")
    }
    frame_names = tuple(
        sorted(
            {
                item.attrib.get("name", "")
                for tag in ("link", "frame")
                for item in model.findall(f".//{tag}")
            }
        )
    )
    return ModelAudit(
        arm_joint_count=sum(name in joint_names for name in ARM_JOINTS),
        finger_joint_count=sum(name in joint_names for name in FINGER_JOINTS),
        dynamic_link_count=len(DYNAMIC_LINKS),
        sdf_joint_count=len(joint_names),
        sdf_frame_names=frame_names,
    )
