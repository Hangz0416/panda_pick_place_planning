from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    planner = LaunchConfiguration("planner")
    scene = LaunchConfiguration("scene")
    seed = LaunchConfiguration("seed")
    use_gui = LaunchConfiguration("use_gui")
    output_path = LaunchConfiguration("output_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("planner", default_value="rrt_connect"),
            DeclareLaunchArgument("scene", default_value="obstacle"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("use_gui", default_value="true"),
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
                parameters=[{"use_gui": ParameterValue(use_gui, value_type=bool)}],
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
                    }
                ],
            ),
        ]
    )
