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
#include <stdexcept>
#include <string>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#include <geometry_msgs/msg/pose_array.hpp>
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
    pose_info_topic_ = "/world/" + world + "/pose/info";
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/panda_gz/target_pose", 10);
    obstacle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/panda_gz/obstacle_pose", 10);
    service_ = create_service<SetEntityPose>(
      service_name_,
      std::bind(
        &EntityPoseBridge::set_pose, this, std::placeholders::_1,
        std::placeholders::_2));
    if (!transport_node_.Subscribe(
        pose_info_topic_, &EntityPoseBridge::on_pose_info, this))
    {
      throw std::runtime_error(
              "failed to subscribe to Gazebo world pose info: " + pose_info_topic_);
    }
    RCLCPP_INFO(
      get_logger(),
      "Bridging ROS %s and Gazebo pose readback from %s",
      service_name_.c_str(), pose_info_topic_.c_str());
  }

private:
  static geometry_msgs::msg::Pose to_ros_pose(const ignition::msgs::Pose & pose)
  {
    geometry_msgs::msg::Pose result;
    result.position.x = pose.position().x();
    result.position.y = pose.position().y();
    result.position.z = pose.position().z();
    result.orientation.x = pose.orientation().x();
    result.orientation.y = pose.orientation().y();
    result.orientation.z = pose.orientation().z();
    result.orientation.w = pose.orientation().w();
    return result;
  }

  void on_pose_info(const ignition::msgs::Pose_V & poses)
  {
    geometry_msgs::msg::PoseArray target;
    geometry_msgs::msg::PoseArray obstacle;
    target.header.frame_id = "world";
    obstacle.header.frame_id = "world";
    const auto stamp = now();
    target.header.stamp = stamp;
    obstacle.header.stamp = stamp;
    for (const auto & pose : poses.pose()) {
      if (pose.name() == "target") {
        target.poses.push_back(to_ros_pose(pose));
      } else if (pose.name() == "obstacle") {
        obstacle.poses.push_back(to_ros_pose(pose));
      }
    }
    if (!target.poses.empty()) {
      target_pose_pub_->publish(target);
    }
    if (!obstacle.poses.empty()) {
      obstacle_pose_pub_->publish(obstacle);
    }
  }

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
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pose_pub_;
  rclcpp::Service<SetEntityPose>::SharedPtr service_;
  std::string service_name_;
  std::string pose_info_topic_;
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
