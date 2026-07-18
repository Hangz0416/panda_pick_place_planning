"""Recommended one-command RGB-D grasp-pose demonstration."""

from pathlib import Path
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_output_path() -> str:
    return str(
        Path.home()
        / ".ros"
        / "panda_pick_place"
        / "runs"
        / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        / "episodes.jsonl"
    )


def _include_stack(context):
    bringup = Path(get_package_share_directory("panda_gz_bringup"))
    output_path = LaunchConfiguration("output_path").perform(context).strip()
    if not output_path:
        output_path = _default_output_path()
    arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "scene",
            "seed",
            "object_variant",
            "workpiece_count",
            "pose_source",
            "perception_profile",
            "headless",
            "use_rviz",
        )
    }
    arguments.update(
        {
            "grasp_mode": "assisted",
            "start_application": "true",
            "use_perception_debug": "true",
            "output_path": output_path,
            "keep_open_after_result": LaunchConfiguration("keep_open_after_result"),
        }
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup / "launch" / "panda_moveit_gz.launch.py")
            ),
            launch_arguments=arguments.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene", default_value="clear", choices=["clear", "obstacle"]
            ),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument(
                "object_variant",
                default_value="rect_prism",
                choices=["cube", "rect_prism"],
            ),
            DeclareLaunchArgument(
                "workpiece_count", default_value="3", choices=["1", "3"]
            ),
            DeclareLaunchArgument(
                "pose_source",
                default_value="rgbd",
                choices=["ground_truth", "rgbd"],
            ),
            DeclareLaunchArgument(
                "perception_profile",
                default_value="nominal",
                choices=[
                    "nominal",
                    "low_light",
                    "rgb_noise",
                    "depth_dropout",
                    "combined",
                ],
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "keep_open_after_result",
                default_value="true",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=_include_stack),
        ]
    )
