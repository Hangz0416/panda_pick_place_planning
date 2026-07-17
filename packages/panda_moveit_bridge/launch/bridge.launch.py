from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("panda_moveit_bridge"))
    return LaunchDescription(
        [
            Node(
                package="panda_moveit_bridge",
                executable="moveit_bridge_node",
                name="panda_moveit_bridge",
                output="screen",
                parameters=[str(package_share / "config" / "bridge_defaults.yaml")],
            )
        ]
    )
