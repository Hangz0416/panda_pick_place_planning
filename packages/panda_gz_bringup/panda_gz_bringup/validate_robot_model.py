"""CLI for the audited shared Panda Xacro-to-SDF validation."""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import xacro

from panda_gz_bringup.model_validation import audit_model, convert_urdf_to_sdf


def main(args=None) -> int:
    """Expand the installed Gazebo Xacro, convert it, and print audit JSON."""
    del args
    moveit_share = Path(get_package_share_directory("panda_moveit_config"))
    bringup_share = Path(get_package_share_directory("panda_gz_bringup"))
    xacro_path = moveit_share / "config" / "panda.urdf.xacro"
    document = xacro.process_file(
        str(xacro_path),
        mappings={
            "hardware_type": "gz",
            "controllers_file": str(
                bringup_share / "config" / "controllers.yaml"
            ),
            "assisted_grasp": "true",
        },
    )
    urdf_xml = document.toxml()
    audit = audit_model(urdf_xml, convert_urdf_to_sdf(urdf_xml))
    print(
        json.dumps(
            {
                "status": "passed",
                "arm_joint_count": audit.arm_joint_count,
                "finger_joint_count": audit.finger_joint_count,
                "dynamic_link_count": audit.dynamic_link_count,
                "sdf_joint_count": audit.sdf_joint_count,
                "required_frames": ["panda_hand", "panda_grasptarget"],
                "physics_model_validated": False,
            },
            indent=2,
        )
    )
    return 0
