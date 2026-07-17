"""Bring up Panda in modern Gazebo Fortress with ros2_control."""

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
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
import xacro

from panda_gz_bringup.model_validation import (
    audit_model,
    compose_world_with_deferred_control,
    convert_urdf_to_sdf,
)


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

    base_world = package_share / "worlds" / "panda_pick_place.sdf"
    controllers = package_share / "config" / "controllers.yaml"
    bridge = package_share / "config" / "bridge.yaml"
    grasp = package_share / "config" / "grasp.yaml"
    scene_config = package_share / "config" / f"scene_{scene}.yaml"
    robot_xacro = (
        Path(get_package_share_directory("panda_moveit_config"))
        / "config"
        / "panda.urdf.xacro"
    )
    initial_positions = robot_xacro.parent / "initial_positions.yaml"
    panda_description_share = Path(
        get_package_share_directory("moveit_resources_panda_description")
    )
    gazebo_resource_root = str(panda_description_share.parent)
    gazebo_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            gazebo_resource_root,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    # Fortress must see contact sensors during world initialization, while the
    # Humble gz_ros2_control plugin crashes if it is initialized in that same
    # phase. Derive both from one audited Xacro, put Panda geometry in the
    # initial world, then load only the control plugin after simulation starts.
    robot_description = xacro.process_file(
        str(robot_xacro),
        mappings={
            "hardware_type": "gz",
            "initial_positions_file": str(initial_positions),
            "controllers_file": str(controllers),
            "assisted_grasp": str(assisted_grasp).lower(),
        },
    ).toxml()
    robot_sdf = convert_urdf_to_sdf(robot_description)
    audit_model(robot_description, robot_sdf)
    world = compose_world_with_deferred_control(base_world, robot_sdf)
    gz_args = f"{'-s ' if headless else ''}-r -v 3 {world}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("ros_gz_sim"))
                / "launch"
                / "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    joint_state_canonicalizer = Node(
        package="panda_gz_bringup",
        executable="joint_state_canonicalizer",
        name="panda_joint_state_canonicalizer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    control_loader = Node(
        package="panda_gz_bringup",
        executable="control_system_loader",
        name="panda_control_system_loader",
        output="screen",
        parameters=[
            {
                "world_name": "panda_pick_place",
                "entity_name": "panda",
                "controllers_file": str(controllers),
                "timeout_sec": 60.0,
            }
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

    def start_controllers_if_loaded(event, _context):
        if event.returncode == 0:
            return spawners
        reason = (
            "deferred gz_ros2_control loader failed with code "
            f"{event.returncode}"
        )
        return [
            LogInfo(msg=f"ERROR: {reason}"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=control_loader,
            on_exit=start_controllers_if_loaded,
        )
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
        gazebo_resource_path,
        gazebo,
        state_publisher,
        joint_state_canonicalizer,
        bridge_topics,
        bridge_control,
        entity_pose_bridge,
        control_loader,
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
