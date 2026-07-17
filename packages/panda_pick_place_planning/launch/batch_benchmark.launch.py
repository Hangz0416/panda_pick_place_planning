"""Sequential ROS batch orchestration for an already-running backend stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("panda_pick_place_planning")
    )
    backend_defaults = str(
        package_share / "config" / "moveit_backend_defaults.yaml"
    )
    episodes = LaunchConfiguration("episodes_per_scene")
    seed = LaunchConfiguration("seed")
    scenes_csv = LaunchConfiguration("scenes_csv")
    planner = LaunchConfiguration("planner")
    backend = LaunchConfiguration("backend")
    simulator = LaunchConfiguration("simulator")
    grasp_mode = LaunchConfiguration("grasp_mode")
    use_gazebo = LaunchConfiguration("use_gazebo")
    moveit_execute = LaunchConfiguration("moveit_execute")
    moveit_command_gripper = LaunchConfiguration("moveit_command_gripper")
    collision_monitor = LaunchConfiguration("collision_monitor_available")
    result_timeout = LaunchConfiguration("result_timeout_sec")
    output_path = LaunchConfiguration("output_path")
    summary_path = LaunchConfiguration("summary_path")

    common_time = {
        "use_sim_time": ParameterValue(use_gazebo, value_type=bool)
    }
    batch_generator = Node(
        package="panda_pick_place_planning",
        executable="batch_task_generator_node",
        name="batch_task_generator",
        output="screen",
        parameters=[
            common_time,
            {
                "episodes_per_scene": ParameterValue(
                    episodes, value_type=int
                ),
                "seed": ParameterValue(seed, value_type=int),
                "scenes_csv": scenes_csv,
                "planners_csv": planner,
                "backend": backend,
                "simulator": simulator,
                "grasp_mode": grasp_mode,
                "result_timeout_sec": ParameterValue(
                    result_timeout, value_type=float
                ),
                "summary_path": summary_path,
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("episodes_per_scene", default_value="5"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("scenes_csv", default_value="clear,obstacle"),
            DeclareLaunchArgument("planner", default_value="rrt_connect"),
            DeclareLaunchArgument("backend", default_value="moveit"),
            DeclareLaunchArgument("simulator", default_value="moveit_mock"),
            DeclareLaunchArgument("grasp_mode", default_value="assisted"),
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("moveit_execute", default_value="false"),
            DeclareLaunchArgument("moveit_command_gripper", default_value="false"),
            DeclareLaunchArgument(
                "collision_monitor_available",
                default_value="false",
                description=(
                    "Deprecated and ignored for truth: the typed Gazebo "
                    "collision-state service determines availability."
                ),
            ),
            DeclareLaunchArgument("result_timeout_sec", default_value="120.0"),
            DeclareLaunchArgument(
                "output_path",
                default_value="results/moveit_batch_results.jsonl",
            ),
            DeclareLaunchArgument(
                "summary_path",
                default_value="results/moveit_batch_summary.json",
            ),
            Node(
                package="panda_pick_place_planning",
                executable="evaluator_node",
                name="batch_evaluator",
                output="screen",
                parameters=[
                    common_time,
                    {"output_path": output_path, "truncate_output": True},
                ],
            ),
            Node(
                package="panda_pick_place_planning",
                executable="manipulation_node",
                name="manipulation",
                output="screen",
                parameters=[
                    common_time,
                    backend_defaults,
                    {
                        "backend": backend,
                        "use_gui": False,
                        "use_gazebo": ParameterValue(
                            use_gazebo, value_type=bool
                        ),
                        "moveit_execute": ParameterValue(
                            moveit_execute, value_type=bool
                        ),
                        "moveit_command_gripper": ParameterValue(
                            moveit_command_gripper, value_type=bool
                        ),
                        "moveit_collision_monitor_available": ParameterValue(
                            collision_monitor, value_type=bool
                        ),
                    },
                ],
            ),
            batch_generator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=batch_generator,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="batch task generator reached terminal status"
                            )
                        )
                    ],
                )
            ),
        ]
    )
