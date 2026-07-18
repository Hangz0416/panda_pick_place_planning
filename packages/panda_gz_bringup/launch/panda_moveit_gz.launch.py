"""Single-entry MoveIt 2 + Gazebo Fortress Panda stack."""

from pathlib import Path
import signal
import json
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    bringup_share = Path(get_package_share_directory("panda_gz_bringup"))
    moveit_share = Path(get_package_share_directory("panda_moveit_config"))
    bridge_share = Path(get_package_share_directory("panda_moveit_bridge"))
    planning_share = Path(
        get_package_share_directory("panda_pick_place_planning")
    )

    scene = LaunchConfiguration("scene")
    object_variant = LaunchConfiguration("object_variant")
    workpiece_count = LaunchConfiguration("workpiece_count")
    seed = LaunchConfiguration("seed")
    backend = LaunchConfiguration("backend")
    grasp_mode = LaunchConfiguration("grasp_mode")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    start_application = LaunchConfiguration("start_application")
    output_path = LaunchConfiguration("output_path")
    keep_open_after_result = LaunchConfiguration("keep_open_after_result")
    readiness_timeout = LaunchConfiguration("readiness_timeout_sec")
    physics_model_validated = LaunchConfiguration("physics_model_validated")
    pose_source = LaunchConfiguration("pose_source")
    perception_timeout = LaunchConfiguration("perception_timeout_sec")
    use_perception_debug = LaunchConfiguration("use_perception_debug")
    perception_profile = LaunchConfiguration("perception_profile")
    grasp_mode_value = grasp_mode.perform(context)
    controllers = bringup_share / "config" / "controllers.yaml"

    xacro_mappings = {
        "hardware_type": "gz",
        "controllers_file": str(controllers),
        "assisted_grasp": str(grasp_mode_value == "assisted").lower(),
        "workpiece_count": workpiece_count.perform(context),
    }
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings=xacro_mappings,
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    common_parameters = moveit_config.to_dict()
    sim_time = {"use_sim_time": True}

    gazebo_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "panda_gz.launch.py")
        ),
        launch_arguments={
            "scene": scene,
            "object_variant": object_variant,
            "workpiece_count": workpiece_count,
            "grasp_mode": grasp_mode,
            "headless": headless,
            "use_sim_time": "true",
            # The typed task reset is issued by the application or manually;
            # avoid a second launch-time reset racing the first episode.
            "auto_reset": "false",
            "physics_model_validated": physics_model_validated,
        }.items(),
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            common_parameters,
            sim_time,
            {
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )
    moveit_bridge = Node(
        package="panda_moveit_bridge",
        executable="moveit_bridge_node",
        name="panda_moveit_bridge",
        output="screen",
        parameters=[
            common_parameters,
            sim_time,
            str(bridge_share / "config" / "bridge_defaults.yaml"),
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="panda_moveit_rviz",
        output="log",
        arguments=["-d", str(moveit_share / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(use_rviz),
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("panda_perception"))
                / "launch"
                / "perception.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_debug_overlay": use_perception_debug,
            "perception_profile": perception_profile,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", pose_source, "' == 'rgbd'"])
        ),
    )

    application = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(planning_share / "launch" / "demo.launch.py")
        ),
        launch_arguments={
            "planner": "rrt_connect",
            "scene": scene,
            "object_variant": object_variant,
            "workpiece_count": workpiece_count,
            "seed": seed,
            "backend": backend,
            "grasp_mode": grasp_mode,
            "use_gui": "false",
            "use_gazebo": "true",
            "moveit_execute": "true",
            "moveit_command_gripper": "true",
            "moveit_collision_monitor_available": "false",
            "output_path": output_path,
            "pose_source": pose_source,
            "perception_timeout_sec": perception_timeout,
            "perception_profile": perception_profile,
        }.items(),
    )
    ready_gate = Node(
        package="panda_gz_bringup",
        executable="stack_ready_gate",
        name="panda_stack_ready_gate",
        output="screen",
        parameters=[
            sim_time,
            {
                "grasp_mode": grasp_mode,
                "pose_source": pose_source,
                "readiness_timeout_sec": ParameterValue(
                    readiness_timeout, value_type=float
                ),
            },
        ],
        condition=IfCondition(start_application),
    )
    completion_gate = Node(
        package="panda_gz_bringup",
        executable="demo_completion_gate",
        name="panda_demo_completion_gate",
        output="screen",
        parameters=[
            sim_time,
            {
                "output_path": output_path,
                "keep_open_after_result": keep_open_after_result,
            },
        ],
        condition=IfCondition(start_application),
    )

    def start_application_if_ready(event, _context):
        if event.returncode == 0:
            return [application]
        reason = f"Panda stack readiness gate failed with code {event.returncode}"
        result_path = Path(output_path.perform(_context)).expanduser()
        status_path = result_path.parent / "run_status.json"
        status_path.parent.mkdir(parents=True, exist_ok=True)
        status_path.write_text(
            json.dumps(
                {
                    "readiness": "failed",
                    "readiness_exit_code": int(event.returncode or 1),
                    "episode_status": "not_started",
                    "result_saved": False,
                    "teardown": "requested",
                    "updated_unix_sec": time.time(),
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return [
            LogInfo(msg=f"ERROR: {reason}"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    start_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=ready_gate,
            on_exit=start_application_if_ready,
        ),
        condition=IfCondition(start_application),
    )

    def staged_shutdown_if_complete(event, _context):
        if event.returncode != 0:
            return []
        return [
                LogInfo(msg="Result saved; stopping MoveIt bridge"),
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=lambda action: action == moveit_bridge,
                    )
                ),
                TimerAction(
                    period=1.0,
                    actions=[
                        LogInfo(msg="Stopping move_group"),
                        EmitEvent(
                            event=SignalProcess(
                                signal_number=signal.SIGINT,
                                process_matcher=lambda action: action == move_group,
                            )
                        ),
                    ],
                ),
                TimerAction(
                    period=2.0,
                    actions=[
                        EmitEvent(event=Shutdown(reason="demo result saved"))
                    ],
                ),
        ]

    staged_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=completion_gate,
            on_exit=staged_shutdown_if_complete,
        ),
        condition=IfCondition(start_application),
    )

    return [
        gazebo_stack,
        move_group,
        moveit_bridge,
        rviz,
        perception,
        ready_gate,
        completion_gate,
        start_when_ready,
        staged_shutdown,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend", default_value="moveit", choices=["moveit"]
            ),
            DeclareLaunchArgument(
                "scene",
                default_value="clear",
                choices=["clear", "obstacle"],
            ),
            DeclareLaunchArgument(
                "object_variant",
                default_value="cube",
                choices=["cube", "rect_prism"],
            ),
            DeclareLaunchArgument(
                "workpiece_count", default_value="1", choices=["1", "3"]
            ),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument(
                "grasp_mode",
                default_value="assisted",
                choices=["assisted", "physics"],
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "start_application",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value="results/moveit_gazebo_episode_results.jsonl",
            ),
            DeclareLaunchArgument(
                "keep_open_after_result",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "readiness_timeout_sec", default_value="120.0"
            ),
            DeclareLaunchArgument(
                "physics_model_validated",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "pose_source",
                default_value="ground_truth",
                choices=["ground_truth", "rgbd"],
            ),
            DeclareLaunchArgument("perception_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument(
                "use_perception_debug",
                default_value="true",
                choices=["true", "false"],
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
