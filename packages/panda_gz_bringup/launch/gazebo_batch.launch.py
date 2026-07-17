"""Own one Gazebo/MoveIt stack and run an assisted benchmark after readiness."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(_context):
    bringup_share = Path(get_package_share_directory("panda_gz_bringup"))
    planning_share = Path(
        get_package_share_directory("panda_pick_place_planning")
    )
    episodes = LaunchConfiguration("episodes_per_scene")
    seed = LaunchConfiguration("seed")
    scenes = LaunchConfiguration("scenes_csv")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    output_path = LaunchConfiguration("output_path")
    summary_path = LaunchConfiguration("summary_path")
    readiness_timeout = LaunchConfiguration("readiness_timeout_sec")
    pose_source = LaunchConfiguration("pose_source")
    perception_timeout = LaunchConfiguration("perception_timeout_sec")
    use_perception_debug = LaunchConfiguration("use_perception_debug")

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "panda_moveit_gz.launch.py")
        ),
        launch_arguments={
            "scene": "clear",
            "seed": seed,
            "grasp_mode": "assisted",
            "headless": headless,
            "use_rviz": use_rviz,
            "start_application": "false",
            "physics_model_validated": "false",
            "readiness_timeout_sec": readiness_timeout,
            "pose_source": pose_source,
            "perception_timeout_sec": perception_timeout,
            "use_perception_debug": use_perception_debug,
        }.items(),
    )
    gate = Node(
        package="panda_gz_bringup",
        executable="stack_ready_gate",
        name="panda_batch_stack_ready_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "grasp_mode": "assisted",
                "pose_source": pose_source,
            },
            {
                "readiness_timeout_sec": ParameterValue(
                    readiness_timeout, value_type=float
                )
            },
        ],
    )
    batch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(planning_share / "launch" / "batch_benchmark.launch.py")
        ),
        launch_arguments={
            "episodes_per_scene": episodes,
            "seed": seed,
            "scenes_csv": scenes,
            "planner": "rrt_connect",
            "backend": "moveit",
            "simulator": "gazebo",
            "grasp_mode": "assisted",
            "use_gazebo": "true",
            "moveit_execute": "true",
            "moveit_command_gripper": "true",
            "output_path": output_path,
            "summary_path": summary_path,
            "pose_source": pose_source,
            "perception_timeout_sec": perception_timeout,
        }.items(),
    )

    def start_batch_if_ready(event, _launch_context):
        if event.returncode == 0:
            return [LogInfo(msg="Panda stack ready; starting assisted batch"), batch]
        reason = f"Panda batch readiness gate failed with code {event.returncode}"
        return [
            LogInfo(msg=f"ERROR: {reason}; batch will not start"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return [
        stack,
        gate,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gate,
                on_exit=start_batch_if_ready,
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("episodes_per_scene", default_value="5"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("scenes_csv", default_value="clear,obstacle"),
            DeclareLaunchArgument(
                "headless", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value="results/runs/gazebo_rgbd/episodes.jsonl",
            ),
            DeclareLaunchArgument(
                "summary_path",
                default_value="results/runs/gazebo_rgbd/summary.json",
            ),
            DeclareLaunchArgument(
                "readiness_timeout_sec", default_value="120.0"
            ),
            DeclareLaunchArgument(
                "pose_source",
                default_value="rgbd",
                choices=["ground_truth", "rgbd"],
            ),
            DeclareLaunchArgument("perception_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument(
                "use_perception_debug",
                default_value="false",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
