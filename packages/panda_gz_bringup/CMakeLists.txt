cmake_minimum_required(VERSION 3.8)
project(panda_gz_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

add_executable(entity_pose_bridge src/entity_pose_bridge.cpp)
ament_target_dependencies(entity_pose_bridge geometry_msgs rclcpp ros_gz_interfaces)
target_link_libraries(entity_pose_bridge
  ignition-msgs8::core
  ignition-transport11::core
)

add_executable(control_system_loader src/control_system_loader.cpp)
ament_target_dependencies(control_system_loader controller_manager_msgs rclcpp)
target_link_libraries(control_system_loader
  ignition-msgs8::core
  ignition-transport11::core
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS control_system_loader entity_pose_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/assisted_grasp_manager
  scripts/demo_completion_gate
  scripts/episode_reset_manager
  scripts/execution_collision_monitor
  scripts/joint_state_canonicalizer
  scripts/physical_grasp_monitor
  scripts/stack_ready_gate
  scripts/supervise_visual_demo
  scripts/validate_episode_reset
  scripts/validate_robot_model
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  set(panda_gz_python_sources
    panda_gz_bringup
    launch
    test
    scripts/assisted_grasp_manager
    scripts/demo_completion_gate
    scripts/episode_reset_manager
    scripts/execution_collision_monitor
    scripts/joint_state_canonicalizer
    scripts/physical_grasp_monitor
    scripts/stack_ready_gate
    scripts/supervise_visual_demo
    scripts/validate_episode_reset
    scripts/validate_robot_model
  )
  ament_flake8(MAX_LINE_LENGTH 100 ${panda_gz_python_sources})
  ament_pep257(${panda_gz_python_sources})
  ament_add_pytest_test(panda_gz_bringup_pytest test)
endif()

ament_package()
