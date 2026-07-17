// Copyright 2026 Panda Pick Place Maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

namespace panda_gz_bringup
{

class EntityPoseBridge : public rclcpp::Node
{
public:
  using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

  EntityPoseBridge()
  : Node("panda_gz_entity_pose_bridge")
  {
    const auto world = declare_parameter<std::string>("world_name", "panda_pick_place");
    timeout_ms_ = static_cast<unsigned int>(
      declare_parameter<std::int64_t>("request_timeout_ms", 5000));
    service_name_ = "/world/" + world + "/set_pose";
    service_ = create_service<SetEntityPose>(
      service_name_,
      std::bind(
        &EntityPoseBridge::set_pose, this, std::placeholders::_1,
        std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(), "Bridging ROS %s to Gazebo Transport %s",
      service_name_.c_str(), service_name_.c_str());
  }

private:
  void set_pose(
    const std::shared_ptr<SetEntityPose::Request> request,
    std::shared_ptr<SetEntityPose::Response> response)
  {
    if (request->entity.name.empty() && request->entity.id == 0U) {
      RCLCPP_ERROR(get_logger(), "SetEntityPose requires an entity name or id");
      response->success = false;
      return;
    }

    ignition::msgs::Pose gz_request;
    if (!request->entity.name.empty()) {
      gz_request.set_name(request->entity.name);
    }
    if (request->entity.id != 0U) {
      gz_request.set_id(request->entity.id);
    }
    auto * position = gz_request.mutable_position();
    position->set_x(request->pose.position.x);
    position->set_y(request->pose.position.y);
    position->set_z(request->pose.position.z);
    auto * orientation = gz_request.mutable_orientation();
    orientation->set_x(request->pose.orientation.x);
    orientation->set_y(request->pose.orientation.y);
    orientation->set_z(request->pose.orientation.z);
    orientation->set_w(request->pose.orientation.w);

    ignition::msgs::Boolean gz_response;
    bool transport_result = false;
    const bool executed = transport_node_.Request(
      service_name_, gz_request, timeout_ms_, gz_response, transport_result);
    response->success = executed && transport_result && gz_response.data();
    if (!response->success) {
      RCLCPP_ERROR(
        get_logger(), "Gazebo set_pose failed for entity [%s]",
        request->entity.name.c_str());
    }
  }

  ignition::transport::Node transport_node_;
  rclcpp::Service<SetEntityPose>::SharedPtr service_;
  std::string service_name_;
  unsigned int timeout_ms_{5000U};
};

}  // namespace panda_gz_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<panda_gz_bringup::EntityPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
