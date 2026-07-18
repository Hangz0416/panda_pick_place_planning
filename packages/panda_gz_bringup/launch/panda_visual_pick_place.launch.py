"""Recommended one-command RGB-D grasp-pose demonstration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("panda_gz_bringup"))
    arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "scene",
            "seed",
            "object_variant",
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
            "output_path": "results/runs/gazebo_rgbd_grasp_pose/episodes.jsonl",
        }
    )
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup / "launch" / "panda_moveit_gz.launch.py")
                ),
                launch_arguments=arguments.items(),
            ),
        ]
    )
