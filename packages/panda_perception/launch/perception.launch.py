"""Launch the standalone RGB-D scene-estimation service."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("panda_perception"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_debug = LaunchConfiguration("use_debug_overlay")
    profile = LaunchConfiguration("perception_profile")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_debug_overlay", default_value="true"),
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
            Node(
                package="panda_perception",
                executable="scene_estimator_node",
                name="panda_scene_estimator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "use_debug_overlay": ParameterValue(use_debug, value_type=bool),
                        "config_path": str(share / "config" / "perception.yaml"),
                        "perception_profile": profile,
                    }
                ],
            ),
        ]
    )
