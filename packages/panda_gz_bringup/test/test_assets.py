from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
PACKAGES = ROOT.parent
MOVEIT = PACKAGES / "panda_moveit_config"
INTERFACES = PACKAGES / "panda_pick_place_interfaces"
PERCEPTION = PACKAGES / "panda_perception"


def test_package_and_world_xml_are_well_formed():
    ET.parse(ROOT / "package.xml")
    world = ET.parse(ROOT / "worlds" / "panda_pick_place.sdf").getroot()
    physics = world.find("./world/physics")
    assert physics is not None
    assert physics.findtext("max_step_size") == "0.001"
    assert physics.findtext("real_time_update_rate") == "1000"
    systems = {
        (plugin.attrib["filename"], plugin.attrib["name"])
        for plugin in world.findall("./world/plugin")
    }
    assert (
        "libignition-gazebo-contact-system.so",
        "ignition::gazebo::systems::Contact",
    ) in systems
    assert (
        "libignition-gazebo-sensors-system.so",
        "ignition::gazebo::systems::Sensors",
    ) in systems
    assert {model.attrib["name"] for model in world.findall("./world/model")} >= {
        "table",
        "cube",
        "target",
        "obstacle",
    }
    table = world.find("./world/model[@name='table']")
    assert table is not None
    assert [item.attrib["name"] for item in table.findall("./link/collision")] == [
        "top_collision"
    ]
    table_size = table.findtext("./link/collision/geometry/box/size")
    table_pose = table.findtext("./link/collision/pose")
    assert table_size == "1.5 1.0 0.05"
    assert table_pose is not None
    assert abs(float(table_pose.split()[2]) - 0.35 + 0.025) < 1.0e-9
    contact_topic = world.findtext(
        "./world/model[@name='cube']/link/sensor[@type='contact']/contact/topic"
    )
    assert contact_topic == "/cube/contacts"
    cube = world.find("./world/model[@name='cube']")
    assert cube is not None
    assert cube.find(
        "./plugin[@name='ignition::gazebo::systems::OdometryPublisher']"
    ) is not None
    for model_name in ("target", "obstacle"):
        assert world.find(f"./world/model[@name='{model_name}']") is not None


def test_one_shared_xacro_selects_mock_or_modern_gazebo():
    shared = MOVEIT / "config" / "panda.urdf.xacro"
    control = MOVEIT / "config" / "panda.ros2_control.xacro"
    ET.parse(shared)
    ET.parse(control)
    shared_text = shared.read_text()
    dynamics = (MOVEIT / "config" / "panda_description.urdf.xacro").read_text()
    control_text = control.read_text()
    launch_text = (ROOT / "launch" / "panda_gz.launch.py").read_text()

    assert not list((ROOT / "urdf").glob("*.xacro"))
    assert 'name="hardware_type"' in shared_text
    assert "selected_hardware == 'mock'" in shared_text
    assert "selected_hardware == 'gz'" in shared_text
    assert 'filename="panda_description.urdf.xacro"' in shared_text
    assert dynamics.count("<inertial>") == 11
    assert "c11f000c2737acc22ed8dfeff40555b2644a4294" in dynamics
    assert shared_text.count('name="world_to_panda_link0"') == 1
    assert "mock_components/GenericSystem" in control_text
    assert "gz_ros2_control/GazeboSimSystem" in control_text
    assert "gz_ros2_control-system" in control_text
    assert "DetachableJoint" in control_text
    assert '<gazebo reference="panda_hand_joint">' in control_text
    assert "<preserveFixedJoint>true</preserveFixedJoint>" in control_text
    assert "<child_link>cube_link</child_link>" in control_text
    assert "child_model_link" not in control_text
    assert "panda_moveit_config" in launch_text
    assert '"hardware_type": "gz"' in launch_text
    assert "robot_sdf = convert_urdf_to_sdf(robot_description)" in launch_text
    assert "audit_model(robot_description, robot_sdf)" in launch_text
    assert "compose_world_with_deferred_control(" in launch_text
    assert "camera_config=camera_config" in launch_text
    assert 'executable="control_system_loader"' in launch_text
    loader = (ROOT / "src" / "control_system_loader.cpp").read_text()
    assert "std::chrono::duration<double>(timeout_sec)" in loader
    assert "wait_for_service(std::chrono::seconds(10))" not in loader
    assert "start_controllers_if_loaded" in launch_text
    assert "event.returncode == 0" in launch_text
    assert 'executable="create"' not in launch_text
    assert "libgazebo_ros2_control.so" not in control_text
    assert "gazebo_ros2_control/GazeboSystem" not in control_text
    assert "<contact>" in control_text
    assert "<topic>/panda/robot_contacts</topic>" in control_text
    assert control_text.count("<xacro:panda_contact_sensor") == 11
    for link_name in (
        "panda_link0",
        "panda_link1",
        "panda_link2",
        "panda_link3",
        "panda_link4",
        "panda_link5",
        "panda_link6",
        "panda_link7",
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger",
    ):
        assert f'collision_name="{link_name}_collision"' in control_text


def test_world_frame_has_one_authoritative_transform():
    shared = (MOVEIT / "config" / "panda.urdf.xacro").read_text()
    srdf = (MOVEIT / "config" / "panda.srdf").read_text()
    mock_launch = (MOVEIT / "launch" / "mock_demo.launch.py").read_text()
    assert '<parent link="world"' in shared
    assert '<child link="panda_link0"' in shared
    assert "virtual_joint" not in srdf
    assert "static_transform_publisher" not in mock_launch


def test_v1_home_is_identical_in_moveit_gazebo_and_srdf():
    expected = [0.0, -0.65, 0.0, -2.2, 0.0, 1.55, 0.78]
    initial = yaml.safe_load(
        (MOVEIT / "config" / "initial_positions.yaml").read_text()
    )["initial_positions"]
    assert [initial[f"panda_joint{i}"] for i in range(1, 8)] == expected

    srdf = ET.parse(MOVEIT / "config" / "panda.srdf").getroot()
    home = srdf.find("./group_state[@name='home']")
    assert home is not None
    assert [float(joint.attrib["value"]) for joint in home.findall("joint")] == expected

    grasp = yaml.safe_load((ROOT / "config" / "grasp.yaml").read_text())
    reset = grasp["episode_reset_manager"]["ros__parameters"]
    assert reset["home_joints"] == expected


def test_controller_and_bridge_contracts():
    controllers = yaml.safe_load(
        (ROOT / "config" / "controllers.yaml").read_text()
    )
    manager = controllers["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 100
    assert manager["panda_arm_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert "panda_hand_controller" in manager
    assert "panda_gripper_controller" not in manager
    broadcaster = controllers["joint_state_broadcaster"]["ros__parameters"]
    assert broadcaster["use_local_topics"] is True
    arm = controllers["panda_arm_controller"]["ros__parameters"]
    assert arm["joints"] == [f"panda_joint{index}" for index in range(1, 8)]

    moveit_controllers = yaml.safe_load(
        (MOVEIT / "config" / "moveit_controllers.yaml").read_text()
    )
    names = moveit_controllers["moveit_simple_controller_manager"][
        "controller_names"
    ]
    assert names == ["panda_arm_controller", "panda_hand_controller"]

    bridges = yaml.safe_load((ROOT / "config" / "bridge.yaml").read_text())
    topics = {entry["ros_topic_name"] for entry in bridges}
    assert "/clock" in topics
    assert "/panda/detachable_joint/attach" in topics
    assert "/panda/detachable_joint/detach" in topics
    assert "/panda/detachable_joint/state" in topics
    assert "/panda_gz/cube_odometry" in topics
    assert "/panda_gz/target_pose" not in topics
    assert "/panda_gz/obstacle_pose" not in topics
    detachable_state = next(
        item
        for item in bridges
        if item["ros_topic_name"] == "/panda/detachable_joint/state"
    )
    assert detachable_state["ros_type_name"] == "std_msgs/msg/String"
    assert detachable_state["gz_type_name"] == "gz.msgs.StringMsg"
    contact_bridge = next(
        item
        for item in bridges
        if item["ros_topic_name"] == "/panda_gz/cube_contacts"
    )
    assert contact_bridge["gz_topic_name"] == "/cube/contacts"
    assert contact_bridge["direction"] == "GZ_TO_ROS"
    robot_contact_bridge = next(
        item
        for item in bridges
        if item["ros_topic_name"] == "/panda_gz/robot_contacts"
    )
    assert robot_contact_bridge["gz_topic_name"] == "/panda/robot_contacts"
    assert robot_contact_bridge["direction"] == "GZ_TO_ROS"
    cube_odometry = next(
        item
        for item in bridges
        if item["ros_topic_name"] == "/panda_gz/cube_odometry"
    )
    assert cube_odometry["ros_type_name"] == "nav_msgs/msg/Odometry"
    assert cube_odometry["gz_type_name"] == "gz.msgs.Odometry"
    camera_bridges = {
        item["ros_topic_name"]: item for item in bridges
        if item["ros_topic_name"].startswith("/panda_perception/")
    }
    assert set(camera_bridges) == {
        "/panda_perception/color/image_raw",
        "/panda_perception/depth/image_raw",
        "/panda_perception/camera_info",
    }
    assert camera_bridges["/panda_perception/color/image_raw"][
        "gz_topic_name"
    ] == "/overhead_camera/image"


def test_typed_reset_and_model_mapping_contract():
    service = (INTERFACES / "srv" / "ResetEpisode.srv").read_text()
    for field in (
        "uint64 episode_id",
        "uint64 task_seed",
        "string scene",
        "string grasp_mode",
        "string object_variant",
        "float64 object_yaw_rad",
        "geometry_msgs/Point object_position",
        "geometry_msgs/Point target_position",
        "uint16 status_code",
        "string failure_type",
    ):
        assert field in service

    cmake = (INTERFACES / "CMakeLists.txt").read_text()
    reset_manager = (
        ROOT / "panda_gz_bringup" / "episode_reset_manager.py"
    ).read_text()
    assert '"srv/ResetEpisode.srv"' in cmake
    assert "ResetEpisode," in reset_manager
    assert '"/panda_gz/reset_episode"' in reset_manager
    assert '"cube": str(self.get_parameter("cube_model").value)' in reset_manager
    assert '"target": str(self.get_parameter("target_model").value)' in reset_manager
    assert '"obstacle": str(self.get_parameter("obstacle_model").value)' in reset_manager
    assert "request.world_control.reset.model_only" not in reset_manager
    assert "request.world_control.seed" not in reset_manager
    assert "Fortress explicitly does not support model-only reset" in reset_manager
    assert 'require_world_frame(request.header.frame_id)' in reset_manager
    assert '"/panda_gz/cube_odometry"' in reset_manager
    assert '"/panda_gz/target_pose"' in reset_manager
    assert '"/panda_gz/obstacle_pose"' in reset_manager
    assert "_wait_for_scene_readback" in reset_manager
    assert "_wait_for_home_settled" in reset_manager
    assert "self._joint_state_condition.wait(" in reset_manager
    assert 'self.declare_parameter("home_stable_sec", 0.20)' in reset_manager
    assert "if success and spec is not None" in reset_manager
    assert '"/panda_hand_controller/gripper_cmd"' in reset_manager
    assert "GetPhysicalGraspState.Request.COMMAND_RESET" in reset_manager


def test_typed_rgbd_interfaces_and_single_camera_config():
    message = (INTERFACES / "msg" / "SceneObjectEstimate.msg").read_text()
    service = (INTERFACES / "srv" / "EstimateScene.srv").read_text()
    result = (INTERFACES / "msg" / "EpisodeResult.msg").read_text()
    camera = yaml.safe_load(
        (PERCEPTION / "config" / "perception.yaml").read_text()
    )["camera"]
    launch = (ROOT / "launch" / "panda_gz.launch.py").read_text()
    for field in (
        "string object_id",
        "geometry_msgs/PoseWithCovariance pose",
        "float64 confidence",
        "uint32 valid_pixel_count",
    ):
        assert field in message
    assert "string[] required_object_ids" in service
    assert "string object_variant" in service
    assert "SceneObjectEstimate[] estimates" in service
    for field in (
        "string pose_source",
        "string object_variant",
        "string perception_profile",
        "bool perception_success",
        "float64 perception_latency_sec",
        "float64 object_position_error_m",
        "float64 object_yaw_error_rad",
        "bool grasp_selection_success",
        "uint32 grasp_candidate_count",
    ):
        assert field in result
    assert camera["width"] == 640
    assert camera["height"] == 480
    assert camera["update_rate_hz"] == 20.0
    assert "perception.yaml" in launch
    assert "camera_config=camera_config" in launch


def test_scene_defaults_are_task_semantic_points():
    for scene in ("clear", "obstacle"):
        data = yaml.safe_load(
            (ROOT / "config" / f"scene_{scene}.yaml").read_text()
        )
        params = data["episode_reset_manager"]["ros__parameters"]
        assert params["scene"] == scene
        assert len(params["default_object_position"]) == 3
        assert len(params["default_target_position"]) == 3


def test_grasp_and_reset_service_names_are_stable():
    grasp = (
        ROOT / "panda_gz_bringup" / "assisted_grasp_manager.py"
    ).read_text()
    reset = (ROOT / "panda_gz_bringup" / "episode_reset_manager.py").read_text()
    assert '"/panda_gz/attach_object"' in grasp
    assert '"/panda_gz/detach_object"' in grasp
    assert '"/panda_gz/reset_episode"' in reset


def test_humble_entity_pose_adapter_is_built_and_launched():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    source = (ROOT / "src" / "entity_pose_bridge.cpp").read_text()
    launch = (ROOT / "launch" / "panda_gz.launch.py").read_text()
    package = ET.parse(ROOT / "package.xml").getroot()
    build_type = package.findtext("./export/build_type")

    assert build_type == "ament_cmake"
    assert "add_executable(entity_pose_bridge" in cmake
    assert "ignition-transport11::core" in cmake
    assert 'service_name_ = "/world/" + world + "/set_pose"' in source
    assert 'pose_info_topic_ = "/world/" + world + "/pose/info"' in source
    assert '"/panda_gz/target_pose"' in source
    assert '"/panda_gz/obstacle_pose"' in source
    assert "pose.name() == \"target\"" in source
    assert "pose.name() == \"obstacle\"" in source
    assert 'executable="entity_pose_bridge"' in launch
    assert "panda_gz_set_pose_bridge" not in launch


def test_control_system_is_loaded_after_world_iterations_and_fails_closed():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    source = (ROOT / "src" / "control_system_loader.cpp").read_text()
    launch = (ROOT / "launch" / "panda_gz.launch.py").read_text()

    assert "add_executable(control_system_loader" in cmake
    assert '"/entity/system/add"' in source
    assert "model_entity_id.store(pose.id())" in source
    assert "set_id(model_entity_id.load())" in source
    assert "iterations.load() < 100" in source
    assert "gz_ros2_control::GazeboSimROS2ControlPlugin" in source
    assert '"/controller_manager/list_controllers"' in source
    assert "return 7;" in source
    assert "deferred gz_ros2_control loader failed" in launch


def test_python_quality_checks_are_registered_with_colcon_test():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    package = (ROOT / "package.xml").read_text()
    assert "find_package(ament_cmake_flake8 REQUIRED)" in cmake
    assert "find_package(ament_cmake_pep257 REQUIRED)" in cmake
    assert "ament_flake8(MAX_LINE_LENGTH 100" in cmake
    assert "ament_pep257(${panda_gz_python_sources})" in cmake
    assert "<test_depend>ament_cmake_flake8</test_depend>" in package
    assert "<test_depend>ament_cmake_pep257</test_depend>" in package
    assert "scripts/stack_ready_gate" in cmake
    assert "scripts/joint_state_canonicalizer" in cmake
    assert "scripts/validate_episode_reset" in cmake
    assert "scripts/validate_robot_model" in cmake


def test_readiness_gate_is_installed_and_fail_closed():
    gate = ROOT / "panda_gz_bringup" / "stack_ready_gate.py"
    wrapper = ROOT / "scripts" / "stack_ready_gate"
    source = gate.read_text()
    cmake = (ROOT / "CMakeLists.txt").read_text()
    standalone = (ROOT / "launch" / "panda_gz.launch.py").read_text()

    assert wrapper.stat().st_mode & 0o111
    assert "scripts/stack_ready_gate" in cmake
    assert "time.sleep" not in source
    assert "sleep(" not in source
    assert "if not ready:" in source
    assert "ROS context stopped before stack readiness was proved" in source
    assert 'declare_parameter("readiness_timeout_sec", 120.0)' in source
    assert "Panda stack readiness gate timed out" in source
    assert 'if grasp_mode_value == "physics" and not physics_model_validated' in standalone
    assert "mass/inertia model has not been validated" in standalone


def test_installed_python_entrypoint_scripts_are_executable():
    scripts = ROOT / "scripts"
    for script in scripts.iterdir():
        assert script.stat().st_mode & 0o111, f"{script.name} is not executable"


def test_physics_monitor_contract_is_typed_and_physics_only():
    service = (INTERFACES / "srv" / "GetPhysicalGraspState.srv").read_text()
    cmake = (INTERFACES / "CMakeLists.txt").read_text()
    launch = (ROOT / "launch" / "panda_gz.launch.py").read_text()
    monitor = (
        ROOT / "panda_gz_bringup" / "physical_grasp_monitor.py"
    ).read_text()
    for field in (
        "bool grasped",
        "bool holding_object",
        "bool released",
        "bool left_finger_contact",
        "bool right_finger_contact",
        "bool slipping",
        "float64 max_relative_translation_drift_m",
        "float64 release_stable_time_sec",
    ):
        assert field in service
    assert '"srv/GetPhysicalGraspState.srv"' in cmake
    assert 'if grasp_mode_value == "physics"' in launch
    assert "physics_model_validated" in launch
    assert 'executable="physical_grasp_monitor"' in launch
    assert '"/panda_gz/cube_contacts"' in monitor
    assert '"/panda_gz/get_physical_grasp_state"' in monitor
    for assignment in (
        "self._latest_cube_pose = None",
        "self._left_contact = False",
        "self._right_contact = False",
        "self._last_contact_time_sec = None",
    ):
        assert assignment in monitor

    grasp = yaml.safe_load((ROOT / "config" / "grasp.yaml").read_text())
    assert (
        grasp["assisted_grasp_manager"]["ros__parameters"]["hand_frame"]
        == "panda_grasptarget"
    )
    assert (
        grasp["physical_grasp_monitor"]["ros__parameters"]["hand_frame"]
        == "panda_grasptarget"
    )
    shared = (MOVEIT / "config" / "panda.urdf.xacro").read_text()
    assert 'name="panda_grasptarget"' in shared
    assert '<origin xyz="0 0 0.105"' in shared


def test_execution_collision_monitor_is_typed_and_not_a_manual_boolean():
    service = (
        INTERFACES / "srv" / "GetExecutionCollisionState.srv"
    ).read_text()
    cmake = (INTERFACES / "CMakeLists.txt").read_text()
    launch = (ROOT / "launch" / "panda_gz.launch.py").read_text()
    monitor = (
        ROOT / "panda_gz_bringup" / "execution_collision_monitor.py"
    ).read_text()
    reset = (ROOT / "panda_gz_bringup" / "episode_reset_manager.py").read_text()
    for field in (
        "geometry_msgs/Point object_initial_position",
        "geometry_msgs/Point target_position",
        "bool available",
        "bool invalid_collision",
        "uint32 collision_count",
        "string last_failure_type",
    ):
        assert field in service
    assert '"srv/GetExecutionCollisionState.srv"' in cmake
    assert 'executable="execution_collision_monitor"' in launch
    assert '"/panda_gz/robot_contacts"' in monitor
    assert '"/panda_gz/cube_contacts"' in monitor
    assert '"/panda_gz/get_execution_collision_state"' in monitor
    assert "GetExecutionCollisionState.Request.COMMAND_RESET" in reset


def test_assisted_width_matches_the_fsm_backend_command():
    grasp = yaml.safe_load((ROOT / "config" / "grasp.yaml").read_text())
    params = grasp["assisted_grasp_manager"]["ros__parameters"]
    planning_backend = (
        PACKAGES
        / "panda_pick_place_planning"
        / "panda_pick_place_planning"
        / "core"
        / "backends"
        / "moveit.py"
    ).read_text()
    assert params["assisted_pre_attach_width_m"] == 0.038
    assert "assisted_pre_attach_width: float = 0.038" in planning_backend
