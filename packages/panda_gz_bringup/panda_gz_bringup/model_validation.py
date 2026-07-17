"""Validate the shared Panda Xacro before Gazebo runtime is attempted."""

from __future__ import annotations

from dataclasses import dataclass
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
