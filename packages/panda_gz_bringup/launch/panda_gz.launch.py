"""Bring up Panda in modern Gazebo Fortress with ros2_control."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    package_share = Path(get_package_share_directory("panda_gz_bringup"))
    scene = LaunchConfiguration("scene").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    grasp_mode = LaunchConfiguration("grasp_mode")
    grasp_mode_value = grasp_mode.perform(context)
    assisted_grasp = grasp_mode_value == "assisted"
    physics_model_validated = LaunchConfiguration(
        "physics_model_validated"
    ).perform(context).lower() == "true"
    if grasp_mode_value == "physics" and not physics_model_validated:
        raise RuntimeError(
            "grasp_mode:=physics is disabled: the shared Panda Gazebo "
            "mass/inertia model has not been validated. Use assisted mode "
            "until an audited inertial model and runtime validation exist."
        )
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_reset = LaunchConfiguration("auto_reset")

    world = package_share / "worlds" / "panda_pick_place.sdf"
    controllers = package_share / "config" / "controllers.yaml"
    bridge = package_share / "config" / "bridge.yaml"
    grasp = package_share / "config" / "grasp.yaml"
    scene_config = package_share / "config" / f"scene_{scene}.yaml"
    robot_xacro = (
        Path(get_package_share_directory("panda_moveit_config"))
        / "config"
        / "panda.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(robot_xacro),
                " hardware_type:=gz",
                " controllers_file:=",
                str(controllers),
                " assisted_grasp:=",
                str(assisted_grasp).lower(),
            ]
        ),
        value_type=str,
    )
    gz_args = f"{'-s ' if headless else ''}-r -v 3 {world}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_panda",
        output="screen",
        arguments=[
            "-name",
            "panda",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )
    bridge_topics = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="panda_gz_topic_bridge",
        output="screen",
        arguments=["--ros-args", "-p", f"config_file:={bridge}"],
    )
    # Service entries are positional for compatibility with every Humble
    # ros_gz patch release; topic mappings remain in bridge.yaml.
    bridge_control = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="panda_gz_control_bridge",
        output="screen",
        arguments=[
            "/world/panda_pick_place/control@ros_gz_interfaces/srv/ControlWorld"
        ],
    )
    # Humble's standard ros_gz_bridge exposes ControlWorld but does not bridge
    # SetEntityPose. This small adapter calls Fortress' native Transport
    # service while preserving the standard ROS SetEntityPose type.
    entity_pose_bridge = Node(
        package="panda_gz_bringup",
        executable="entity_pose_bridge",
        name="panda_gz_entity_pose_bridge",
        output="screen",
        parameters=[{"world_name": "panda_pick_place"}],
    )

    spawners = []
    for controller in (
        "joint_state_broadcaster",
        "panda_arm_controller",
        "panda_hand_controller",
    ):
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        )
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=spawners)
    )

    grasp_manager = Node(
        package="panda_gz_bringup",
        executable="assisted_grasp_manager",
        output="screen",
        parameters=[
            str(grasp),
            {"grasp_mode": grasp_mode, "use_sim_time": use_sim_time},
        ],
    )
    reset_manager = Node(
        package="panda_gz_bringup",
        executable="episode_reset_manager",
        output="screen",
        parameters=[
            str(grasp),
            str(scene_config),
            {
                "scene": scene,
                "grasp_mode": grasp_mode,
                "auto_reset": auto_reset,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    collision_monitor = Node(
        package="panda_gz_bringup",
        executable="execution_collision_monitor",
        name="execution_collision_monitor",
        output="screen",
        parameters=[
            str(grasp),
            {"grasp_mode": grasp_mode, "use_sim_time": use_sim_time},
        ],
    )
    managers = [grasp_manager, collision_monitor]
    if grasp_mode_value == "physics":
        managers.append(
            Node(
                package="panda_gz_bringup",
                executable="physical_grasp_monitor",
                name="physical_grasp_monitor",
                output="screen",
                parameters=[
                    str(grasp),
                    {
                        "use_sim_time": use_sim_time,
                        "physics_model_validated": physics_model_validated,
                    },
                ],
            )
        )
    managers.append(reset_manager)

    return [
        gazebo,
        state_publisher,
        bridge_topics,
        bridge_control,
        entity_pose_bridge,
        spawn_robot,
        start_controllers,
        *managers,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene", default_value="clear", choices=["clear", "obstacle"]
            ),
            DeclareLaunchArgument(
                "grasp_mode",
                default_value="assisted",
                choices=["assisted", "physics"],
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "auto_reset", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "physics_model_validated",
                default_value="false",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
