#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/entity_plugin_v.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/world_stats.pb.h>
#include <ignition/transport/Node.hh>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
std::string xml_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '&':
        escaped += "&amp;";
        break;
      case '<':
        escaped += "&lt;";
        break;
      case '>':
        escaped += "&gt;";
        break;
      default:
        escaped += character;
        break;
    }
  }
  return escaped;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("panda_control_system_loader");
  const auto world_name = node->declare_parameter<std::string>(
    "world_name", "panda_pick_place");
  const auto entity_name = node->declare_parameter<std::string>(
    "entity_name", "panda");
  const auto controllers_file = node->declare_parameter<std::string>(
    "controllers_file", "");
  const auto timeout_sec = node->declare_parameter<double>(
    "timeout_sec", 60.0);
  if (controllers_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "controllers_file must not be empty");
    rclcpp::shutdown();
    return 2;
  }

  ignition::transport::Node transport;
  std::atomic<std::uint64_t> iterations{0};
  std::atomic<std::uint64_t> model_entity_id{0};
  const std::string stats_topic = "/world/" + world_name + "/stats";
  const std::string pose_topic = "/world/" + world_name + "/pose/info";
  std::function<void(const ignition::msgs::WorldStatistics &)> on_stats =
    [&iterations](const ignition::msgs::WorldStatistics & message) {
      iterations.store(message.iterations());
    };
  if (!transport.Subscribe(stats_topic, on_stats)) {
    RCLCPP_ERROR(
      node->get_logger(), "failed to subscribe to %s", stats_topic.c_str());
    rclcpp::shutdown();
    return 3;
  }
  std::function<void(const ignition::msgs::Pose_V &)> on_poses =
    [&model_entity_id, &entity_name](const ignition::msgs::Pose_V & message) {
      for (const auto & pose : message.pose()) {
        if (pose.name() == entity_name) {
          model_entity_id.store(pose.id());
          return;
        }
      }
    };
  if (!transport.Subscribe(pose_topic, on_poses)) {
    RCLCPP_ERROR(
      node->get_logger(), "failed to subscribe to %s", pose_topic.c_str());
    rclcpp::shutdown();
    return 4;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() &&
    (iterations.load() < 100 || model_entity_id.load() == 0) &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (iterations.load() < 100 || model_entity_id.load() == 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Gazebo readiness failed within %.1f seconds "
      "(iterations=%lu, model_entity_id=%lu)",
      timeout_sec,
      static_cast<unsigned long>(iterations.load()),
      static_cast<unsigned long>(model_entity_id.load()));
    rclcpp::shutdown();
    return 5;
  }

  ignition::msgs::EntityPlugin_V request;
  request.mutable_entity()->set_id(model_entity_id.load());
  request.mutable_entity()->set_name(entity_name);
  request.mutable_entity()->set_type(ignition::msgs::Entity::MODEL);
  auto * plugin = request.add_plugins();
  plugin->set_name("gz_ros2_control::GazeboSimROS2ControlPlugin");
  plugin->set_filename("gz_ros2_control-system");
  plugin->set_innerxml(
    "<robot_param>robot_description</robot_param>"
    "<robot_param_node>robot_state_publisher</robot_param_node>"
    "<parameters>" + xml_escape(controllers_file) + "</parameters>"
    "<controller_manager_name>controller_manager</controller_manager_name>");

  ignition::msgs::Boolean reply;
  bool service_result = false;
  const std::string service =
    "/world/" + world_name + "/entity/system/add";
  const bool executed = transport.Request(
    service, request, 5000, reply, service_result);
  if (!executed || !service_result || !reply.data()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "failed to add gz_ros2_control to model '%s' via %s "
      "(executed=%s, service_result=%s, reply=%s)",
      entity_name.c_str(), service.c_str(),
      executed ? "true" : "false",
      service_result ? "true" : "false",
      reply.data() ? "true" : "false");
    rclcpp::shutdown();
    return 6;
  }

  auto controller_service =
    node->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");
  if (!controller_service->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(
      node->get_logger(),
      "gz_ros2_control service did not appear after plugin load for entity %lu",
      static_cast<unsigned long>(model_entity_id.load()));
    rclcpp::shutdown();
    return 7;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "loaded gz_ros2_control for '%s' (entity=%lu) after %lu iterations",
    entity_name.c_str(),
    static_cast<unsigned long>(model_entity_id.load()),
    static_cast<unsigned long>(iterations.load()));
  rclcpp::shutdown();
  return 0;
}
