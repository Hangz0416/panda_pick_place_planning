cmake_minimum_required(VERSION 3.8)
project(panda_moveit_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(panda_pick_place_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(moveit_bridge_node src/moveit_bridge_node.cpp)
target_compile_features(moveit_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(moveit_bridge_node
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  panda_pick_place_interfaces
  rclcpp
  rclcpp_action
  trajectory_msgs
)

install(TARGETS moveit_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS test/mock_bridge_smoke.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moveit_mock_smoke
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mock_bridge_helpers test/test_mock_bridge_helpers.py)
endif()

ament_package()
