cmake_minimum_required(VERSION 3.8)
project(panda_pick_place_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManipulationTask.msg"
  "msg/StageMetric.msg"
  "msg/EpisodeResult.msg"
  "msg/SceneObjectEstimate.msg"
  "msg/SceneObjectTask.msg"
  "msg/ObjectSelectionMetric.msg"
  "action/PlanExecuteMotion.action"
  "srv/UpsertWorldObject.srv"
  "srv/RemoveWorldObject.srv"
  "srv/SetAttachedObject.srv"
  "srv/ResetEpisode.srv"
  "srv/GetPhysicalGraspState.srv"
  "srv/GetExecutionCollisionState.srv"
  "srv/EstimateScene.srv"
  "srv/SetAssistedObject.srv"
  DEPENDENCIES geometry_msgs moveit_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
