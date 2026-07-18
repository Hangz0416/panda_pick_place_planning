"""Capture reproducible software and configuration provenance as JSON."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess


APT_PACKAGES = (
    "ros-humble-moveit-resources-panda-description",
    "ros-humble-moveit-ros-move-group",
    "ros-humble-moveit-planners-ompl",
    "ros-humble-ros-gz-interfaces",
    "ros-humble-ros-gz-bridge",
    "ros-humble-ros-gz-sim",
    "ros-humble-gz-ros2-control",
    "libignition-msgs8-dev",
    "libignition-transport11-dev",
    "ros-humble-cv-bridge",
    "ros-humble-message-filters",
    "ros-humble-tf2-ros",
    "python3-numpy",
    "python3-opencv",
)


def _command(args: list[str], cwd: Path | None = None) -> str | None:
    completed = subprocess.run(
        args,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def package_versions() -> dict[str, str | None]:
    """Return exact dpkg versions; missing packages remain explicit nulls."""
    versions = {}
    for package in APT_PACKAGES:
        versions[package] = _command(
            ["dpkg-query", "-W", "-f=${Version}", package]
        )
    return versions


def git_state(repo: Path) -> dict[str, object]:
    """Record commit metadata plus a content hash of the evaluated sources."""
    commit = _command(["git", "rev-parse", "HEAD"], repo)
    branch = _command(["git", "branch", "--show-current"], repo)
    status = _command(["git", "status", "--porcelain"], repo)
    if commit is None:
        raise ValueError(f"not a git repository: {repo}")
    source_hash, source_file_count = source_tree_hash(repo)
    return {
        "commit": commit,
        "branch": branch,
        "dirty": bool(status),
        "source_tree_sha256": source_hash,
        "source_file_count": source_file_count,
    }


def source_tree_hash(repo: Path) -> tuple[str, int]:
    """Hash tracked and untracked source files, excluding result/media data."""
    completed = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "--cached",
            "--others",
            "--exclude-standard",
        ],
        cwd=repo,
        check=False,
        capture_output=True,
    )
    if completed.returncode != 0:
        raise ValueError(f"not a git repository: {repo}")
    relative_paths = sorted(
        path.decode("utf-8")
        for path in completed.stdout.split(b"\0")
        if path
        and not path.startswith(b"results/")
        and not path.startswith(b"docs/media/")
    )
    digest = hashlib.sha256()
    included = 0
    for relative in relative_paths:
        path = repo / relative
        if not path.is_file():
            continue
        data = path.read_bytes()
        executable = bool(path.stat().st_mode & 0o111)
        digest.update(relative.encode("utf-8") + b"\0")
        digest.update(b"x" if executable else b"-")
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
        included += 1
    return digest.hexdigest(), included


def capture_environment(
    repo: Path,
    simulator: str,
    *,
    object_variant: str = "cube",
    pose_source: str = "rgbd",
    perception_profile: str = "nominal",
) -> dict:
    """Build the environment document shared by final mock/Gazebo outputs."""
    if simulator not in {"moveit_mock", "gazebo"}:
        raise ValueError(f"unsupported simulator: {simulator}")
    if object_variant not in {"cube", "rect_prism"}:
        raise ValueError(f"unsupported object variant: {object_variant}")
    if pose_source not in {"ground_truth", "rgbd"}:
        raise ValueError(f"unsupported pose source: {pose_source}")
    if perception_profile not in {
        "nominal",
        "low_light",
        "rgb_noise",
        "depth_dropout",
        "combined",
    }:
        raise ValueError(f"unsupported perception profile: {perception_profile}")
    os_release = {}
    for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            os_release[key] = value.strip('"')
    return {
        "schema_version": 1,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "platform": {
            "os": os_release.get("PRETTY_NAME", platform.platform()),
            "kernel": platform.release(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "simulator": simulator,
        "git": git_state(repo),
        "packages": package_versions(),
        "experiment": {
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time_sec": 3.0,
            "velocity_scaling": 0.2,
            "acceleration_scaling": 0.2,
            "robot_collision_padding_m": 0.010,
            "task_seed_base": 42,
            "grasp_mode": "assisted",
            "object_variant": object_variant,
            "pose_source": pose_source,
            "perception_profile": perception_profile,
            "physics_model_validated": False,
            "camera": {
                "resolution": [640, 480],
                "update_rate_hz": 20.0,
                "depth_noise_stddev_m": 0.002,
                "optical_frame": "overhead_camera_optical_frame",
            },
        },
        "inertial_source": {
            "repository": "https://github.com/frankarobotics/franka_ros",
            "tag": "0.10.2",
            "commit": "c11f000c2737acc22ed8dfeff40555b2644a4294",
            "official_properties": False,
        },
    }


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument(
        "--simulator", choices=("moveit_mock", "gazebo"), required=True
    )
    parser.add_argument(
        "--object-variant", choices=("cube", "rect_prism"), default="cube"
    )
    parser.add_argument(
        "--pose-source", choices=("ground_truth", "rgbd"), default="rgbd"
    )
    parser.add_argument(
        "--perception-profile",
        choices=(
            "nominal",
            "low_light",
            "rgb_noise",
            "depth_dropout",
            "combined",
        ),
        default="nominal",
    )
    parser.add_argument("--output", type=Path, required=True)
    parsed = parser.parse_args(args)
    document = capture_environment(
        parsed.repo.resolve(),
        parsed.simulator,
        object_variant=parsed.object_variant,
        pose_source=parsed.pose_source,
        perception_profile=parsed.perception_profile,
    )
    parsed.output.parent.mkdir(parents=True, exist_ok=True)
    parsed.output.write_text(
        json.dumps(document, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(parsed.output.resolve())
    return 0
