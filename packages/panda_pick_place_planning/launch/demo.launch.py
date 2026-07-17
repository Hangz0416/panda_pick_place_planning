from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    planner = LaunchConfiguration("planner")
    scene = LaunchConfiguration("scene")
    seed = LaunchConfiguration("seed")
    backend = LaunchConfiguration("backend")
    grasp_mode = LaunchConfiguration("grasp_mode")
    use_gui = LaunchConfiguration("use_gui")
    use_gazebo = LaunchConfiguration("use_gazebo")
    moveit_execute = LaunchConfiguration("moveit_execute")
    moveit_command_gripper = LaunchConfiguration("moveit_command_gripper")
    moveit_collision_monitor_available = LaunchConfiguration(
        "moveit_collision_monitor_available"
    )
    output_path = LaunchConfiguration("output_path")
    pose_source = LaunchConfiguration("pose_source")
    perception_timeout = LaunchConfiguration("perception_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("planner", default_value="rrt_connect"),
            DeclareLaunchArgument("scene", default_value="obstacle"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("backend", default_value="pybullet"),
            DeclareLaunchArgument("grasp_mode", default_value="assisted"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("moveit_execute", default_value="false"),
            DeclareLaunchArgument("moveit_command_gripper", default_value="false"),
            DeclareLaunchArgument(
                "pose_source",
                default_value="ground_truth",
                choices=["ground_truth", "rgbd"],
            ),
            DeclareLaunchArgument("perception_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument(
                "moveit_collision_monitor_available",
                default_value="false",
                description=(
                    "Deprecated and ignored for truth: the typed Gazebo "
                    "collision-state service determines availability."
                ),
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value="results/ros2_episode_results.jsonl",
            ),
            Node(
                package="panda_pick_place_planning",
                executable="evaluator_node",
                name="evaluator",
                output="screen",
                parameters=[{"output_path": output_path}],
            ),
            Node(
                package="panda_pick_place_planning",
                executable="manipulation_node",
                name="manipulation",
                output="screen",
                parameters=[
                    backend_defaults,
                    {
                        "backend": backend,
                        "use_gui": ParameterValue(use_gui, value_type=bool),
                        "use_gazebo": ParameterValue(use_gazebo, value_type=bool),
                        "moveit_execute": ParameterValue(
                            moveit_execute, value_type=bool
                        ),
                        "moveit_command_gripper": ParameterValue(
                            moveit_command_gripper, value_type=bool
                        ),
                        "moveit_collision_monitor_available": ParameterValue(
                            moveit_collision_monitor_available,
                            value_type=bool,
                        ),
                        "pose_source": pose_source,
                        "perception_timeout_sec": ParameterValue(
                            perception_timeout, value_type=float
                        ),
                        "use_sim_time": ParameterValue(use_gazebo, value_type=bool),
                    }
                ],
            ),
            Node(
                package="panda_pick_place_planning",
                executable="task_generator_node",
                name="task_generator",
                output="screen",
                parameters=[
                    {
                        "planner": planner,
                        "scene": scene,
                        "seed": ParameterValue(seed, value_type=int),
                        "backend": backend,
                        "grasp_mode": grasp_mode,
                        "use_gazebo": ParameterValue(
                            use_gazebo, value_type=bool
                        ),
                        "use_sim_time": ParameterValue(use_gazebo, value_type=bool),
                    }
                ],
            ),
        ]
    )
