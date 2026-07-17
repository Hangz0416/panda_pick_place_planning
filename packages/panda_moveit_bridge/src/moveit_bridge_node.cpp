#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <panda_pick_place_interfaces/action/plan_execute_motion.hpp>
#include <panda_pick_place_interfaces/srv/remove_world_object.hpp>
#include <panda_pick_place_interfaces/srv/set_attached_object.hpp>
#include <panda_pick_place_interfaces/srv/upsert_world_object.hpp>

namespace panda_moveit_bridge
{

using Motion = panda_pick_place_interfaces::action::PlanExecuteMotion;
using GoalHandleMotion = rclcpp_action::ServerGoalHandle<Motion>;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
using RemoveWorldObject = panda_pick_place_interfaces::srv::RemoveWorldObject;
using SetAttachedObject = panda_pick_place_interfaces::srv::SetAttachedObject;
using UpsertWorldObject = panda_pick_place_interfaces::srv::UpsertWorldObject;

class MoveItBridge : public std::enable_shared_from_this<MoveItBridge>
{
public:
  explicit MoveItBridge(rclcpp::Node::SharedPtr node) : node_(std::move(node))
  {
    planning_group_ = parameter<std::string>("planning_group", "panda_arm");
    action_name_ = parameter<std::string>("action_name", "plan_execute_motion");
    default_planner_id_ =
      parameter<std::string>("default_planner_id", "RRTConnectkConfigDefault");
    default_planning_time_sec_ = parameter<double>("default_planning_time_sec", 3.0);
    default_velocity_scaling_factor_ =
      parameter<double>("default_velocity_scaling_factor", 0.2);
    default_acceleration_scaling_factor_ =
      parameter<double>("default_acceleration_scaling_factor", 0.2);
    max_planning_attempts_ = parameter<int>("max_planning_attempts", 3);
    default_end_effector_link_ =
      parameter<std::string>("default_end_effector_link", "panda_grasptarget");
    default_attach_link_ = parameter<std::string>("default_attach_link", "panda_hand");
    default_touch_links_ = parameter<std::vector<std::string>>(
      "default_touch_links", { "panda_hand", "panda_leftfinger", "panda_rightfinger" });
    scene_sync_timeout_sec_ = parameter<double>("scene_sync_timeout_sec", 1.0);
    const double server_wait_sec = parameter<double>("move_group_server_wait_sec", 10.0);

    const moveit::planning_interface::MoveGroupInterface::Options options(planning_group_);
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, options, nullptr, rclcpp::Duration::from_seconds(server_wait_sec));
    planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    service_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    get_planning_scene_client_ = node_->create_client<GetPlanningScene>(
      "get_planning_scene", rmw_qos_profile_services_default, service_callback_group_);

    action_server_ = rclcpp_action::create_server<Motion>(
      node_, action_name_,
      std::bind(&MoveItBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveItBridge::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveItBridge::handle_accepted, this, std::placeholders::_1));

    upsert_service_ = node_->create_service<UpsertWorldObject>(
      "upsert_world_object",
      std::bind(&MoveItBridge::upsert_world_object, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);
    remove_service_ = node_->create_service<RemoveWorldObject>(
      "remove_world_object",
      std::bind(&MoveItBridge::remove_world_object, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);
    attach_service_ = node_->create_service<SetAttachedObject>(
      "set_attached_object",
      std::bind(&MoveItBridge::set_attached_object, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

    RCLCPP_INFO(
      node_->get_logger(), "MoveIt bridge ready: group='%s', action='%s'", planning_group_.c_str(),
      action_name_.c_str());
  }

private:
  template<typename T>
  T parameter(const std::string& name, const T& default_value)
  {
    if (node_->has_parameter(name)) {
      return node_->get_parameter(name).get_value<T>();
    }
    return node_->declare_parameter<T>(name, default_value);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&, const std::shared_ptr<const Motion::Goal>)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "Rejecting motion goal while another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMotion>)
  {
    cancel_requested_.store(true);
    // MoveGroupInterface::stop() is intentionally called from the cancel callback: waiting for
    // the worker's operation mutex would make cancellation wait until execution had completed.
    move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMotion> goal_handle)
  {
    auto self = shared_from_this();
    std::thread([self, goal_handle]() { self->execute_motion(goal_handle); }).detach();
  }

  void execute_motion(const std::shared_ptr<GoalHandleMotion> goal_handle)
  {
    // Planning, execution and PlanningScene mutations all share MoveGroupInterface state.  Keep
    // them serialized; scene services use try_lock and return bridge_busy instead of racing.
    std::unique_lock<std::mutex> operation_lock(operation_mutex_);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Motion::Result>();
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;

    const auto finish = [this, &goal_handle, &result](bool canceled = false) {
      publish_feedback(goal_handle, "completed");
      if (canceled || goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else if (result->success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
      busy_.store(false);
    };

    try {
      std::string validation_error;
      if (!validate_goal(*goal, validation_error)) {
        result->failure_type = "invalid_request:" + validation_error;
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        finish();
        return;
      }

      move_group_->clearPoseTargets();
      move_group_->setStartStateToCurrentState();
      move_group_->setPlannerId(goal->planner_id.empty() ? default_planner_id_ : goal->planner_id);
      move_group_->setPlanningTime(
        goal->allowed_planning_time_sec == 0.0 ? default_planning_time_sec_ : goal->allowed_planning_time_sec);
      move_group_->setMaxVelocityScalingFactor(
        goal->max_velocity_scaling_factor == 0.0 ? default_velocity_scaling_factor_ :
                                                   goal->max_velocity_scaling_factor);
      move_group_->setMaxAccelerationScalingFactor(
        goal->max_acceleration_scaling_factor == 0.0 ? default_acceleration_scaling_factor_ :
                                                       goal->max_acceleration_scaling_factor);

      const std::string end_effector_link =
        goal->end_effector_link.empty() ? default_end_effector_link_ : goal->end_effector_link;
      if (!end_effector_link.empty() && !move_group_->setEndEffectorLink(end_effector_link)) {
        result->failure_type = "invalid_end_effector_link";
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
        finish();
        return;
      }

      bool target_set = false;
      if (goal->target_type == Motion::Goal::TARGET_TYPE_NAMED) {
        target_set = move_group_->setNamedTarget(goal->named_target);
      } else {
        auto pose_target = goal->pose_target;
        if (pose_target.header.frame_id.empty()) {
          pose_target.header.frame_id = move_group_->getPlanningFrame();
        }
        target_set = move_group_->setPoseTarget(pose_target, end_effector_link);
      }
      if (!target_set) {
        result->failure_type = "target_rejected";
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        finish();
        return;
      }

      publish_feedback(goal_handle, "planning");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode planning_code(
        moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
      const double planning_budget_sec =
        goal->allowed_planning_time_sec == 0.0 ? default_planning_time_sec_ :
                                                goal->allowed_planning_time_sec;
      const auto planning_started = std::chrono::steady_clock::now();
      double accumulated_planning_time_sec = 0.0;
      const int planning_attempts = std::max(1, max_planning_attempts_);
      for (int attempt = 1; attempt <= planning_attempts; ++attempt) {
        const double elapsed_sec = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - planning_started).count();
        const double remaining_sec = planning_budget_sec - elapsed_sec;
        if (remaining_sec <= 0.0) {
          planning_code = moveit::core::MoveItErrorCode(
            moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT);
          break;
        }
        move_group_->setPlanningTime(remaining_sec);
        plan = moveit::planning_interface::MoveGroupInterface::Plan();
        const auto attempt_started = std::chrono::steady_clock::now();
        planning_code = move_group_->plan(plan);
        // MoveGroupInterface does not always copy planning_time_ into Plan when
        // MoveIt rejects a candidate during final trajectory validation.  Use
        // wall time per attempt so rejected attempts are not silently omitted.
        accumulated_planning_time_sec += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - attempt_started).count();
        if (planning_code || !retryable_planning_failure(planning_code.val) ||
          attempt == planning_attempts)
        {
          break;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "Planning attempt %d/%d returned retryable MoveIt error %d; retrying within %.3f s total budget",
          attempt, planning_attempts, planning_code.val, planning_budget_sec);
      }
      result->moveit_error_code = planning_code.val;
      result->planning_time_sec = accumulated_planning_time_sec;
      result->trajectory_points =
        static_cast<std::uint32_t>(plan.trajectory_.joint_trajectory.points.size());
      result->path_length_rad = path_length(plan.trajectory_.joint_trajectory.points);

      if (!planning_code) {
        result->failure_type = failure_type(planning_code.val, "planning");
        finish();
        return;
      }
      if (cancel_requested_.load() || goal_handle->is_canceling()) {
        result->failure_type = "canceled";
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
        finish(true);
        return;
      }

      if (goal->execute) {
        publish_feedback(goal_handle, "executing");
        const auto started = std::chrono::steady_clock::now();
        const auto execution_code = move_group_->execute(plan);
        result->execution_time_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        result->moveit_error_code = execution_code.val;
        if (cancel_requested_.load() || goal_handle->is_canceling()) {
          result->failure_type = "canceled";
          result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
          finish(true);
          return;
        }
        if (!execution_code) {
          result->failure_type = failure_type(execution_code.val, "execution");
          finish();
          return;
        }
      }

      result->success = true;
      result->failure_type.clear();
      finish();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(), "Motion request failed with exception: %s", error.what());
      result->success = false;
      result->failure_type = "bridge_exception";
      result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      finish();
    }
  }

  bool validate_goal(const Motion::Goal& goal, std::string& reason) const
  {
    if (goal.target_type != Motion::Goal::TARGET_TYPE_NAMED &&
      goal.target_type != Motion::Goal::TARGET_TYPE_POSE)
    {
      reason = "unsupported_target_type";
      return false;
    }
    if (goal.target_type == Motion::Goal::TARGET_TYPE_NAMED && goal.named_target.empty()) {
      reason = "empty_named_target";
      return false;
    }
    if (goal.allowed_planning_time_sec < 0.0 || goal.max_velocity_scaling_factor < 0.0 ||
      goal.max_velocity_scaling_factor > 1.0 || goal.max_acceleration_scaling_factor < 0.0 ||
      goal.max_acceleration_scaling_factor > 1.0)
    {
      reason = "invalid_planning_or_scaling_parameter";
      return false;
    }
    if (goal.target_type == Motion::Goal::TARGET_TYPE_POSE) {
      const auto& p = goal.pose_target.pose.position;
      const auto& q = goal.pose_target.pose.orientation;
      const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm_squared) || norm_squared < 1.0e-12)
      {
        reason = "invalid_pose";
        return false;
      }
    }
    return true;
  }

  static double path_length(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points)
  {
    double total = 0.0;
    for (std::size_t index = 1; index < points.size(); ++index) {
      const auto& previous = points[index - 1].positions;
      const auto& current = points[index].positions;
      const std::size_t count = std::min(previous.size(), current.size());
      double squared_distance = 0.0;
      for (std::size_t joint = 0; joint < count; ++joint) {
        const double delta = current[joint] - previous[joint];
        squared_distance += delta * delta;
      }
      total += std::sqrt(squared_distance);
    }
    return total;
  }

  static std::string failure_type(int code, const std::string& phase)
  {
    using Codes = moveit_msgs::msg::MoveItErrorCodes;
    switch (code) {
      case Codes::TIMED_OUT:
        return phase + "_timeout";
      case Codes::PLANNING_FAILED:
        return "planning_failed";
      case Codes::INVALID_MOTION_PLAN:
        return "invalid_motion_plan";
      case Codes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "environment_changed";
      case Codes::START_STATE_IN_COLLISION:
        return "start_state_collision";
      case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "start_state_path_constraint";
      case Codes::GOAL_IN_COLLISION:
        return "goal_collision";
      case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "goal_path_constraint";
      case Codes::GOAL_CONSTRAINTS_VIOLATED:
        return "goal_constraints_violated";
      case Codes::NO_IK_SOLUTION:
        return "no_ik_solution";
      case Codes::CONTROL_FAILED:
        return "execution_control_failed";
      case Codes::INVALID_GROUP_NAME:
        return "invalid_group_name";
      case Codes::INVALID_GOAL_CONSTRAINTS:
        return "invalid_goal_constraints";
      case Codes::INVALID_ROBOT_STATE:
        return "invalid_robot_state";
      case Codes::INVALID_LINK_NAME:
        return "invalid_link_name";
      case Codes::INVALID_OBJECT_NAME:
        return "invalid_object_name";
      case Codes::FRAME_TRANSFORM_FAILURE:
        return "frame_transform_failure";
      case Codes::COLLISION_CHECKING_UNAVAILABLE:
        return "collision_checking_unavailable";
      case Codes::ROBOT_STATE_STALE:
        return "robot_state_stale";
      case Codes::SENSOR_INFO_STALE:
        return "sensor_info_stale";
      case Codes::COMMUNICATION_FAILURE:
        return "communication_failure";
      case Codes::CRASH:
        return "moveit_crash";
      case Codes::ABORT:
        return "moveit_abort";
      case Codes::PREEMPTED:
        return "canceled";
      default:
        return phase + "_failed";
    }
  }

  static bool retryable_planning_failure(int code)
  {
    using Codes = moveit_msgs::msg::MoveItErrorCodes;
    // OMPL is stochastic, and MoveIt's final trajectory validation can reject a
    // candidate after planning (for example when a shortcut grazes an obstacle).
    // Retry only these transient planner outcomes. Invalid goals, collisions at
    // the start/goal, missing IK and scene errors remain immediate failures.
    return code == Codes::PLANNING_FAILED || code == Codes::INVALID_MOTION_PLAN;
  }

  static void publish_feedback(const std::shared_ptr<GoalHandleMotion>& goal_handle, const std::string& phase)
  {
    auto feedback = std::make_shared<Motion::Feedback>();
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
  }

  void upsert_world_object(
    const std::shared_ptr<UpsertWorldObject::Request> request,
    std::shared_ptr<UpsertWorldObject::Response> response)
  {
    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->message = "bridge_busy";
      return;
    }
    auto object = request->collision_object;
    if (object.id.empty()) {
      response->message = "collision_object.id must not be empty";
      return;
    }
    if (object.header.frame_id.empty()) {
      object.header.frame_id = move_group_->getPlanningFrame();
    }
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    response->success = planning_scene_->applyCollisionObject(object);
    if (response->success) {
      response->success = wait_for_world_state(object.id, true);
    }
    if (response->success &&
      !update_allowed_collisions(object.id, request->allowed_collision_links, false))
    {
      moveit_msgs::msg::CollisionObject rollback;
      rollback.id = object.id;
      rollback.header.frame_id = object.header.frame_id;
      rollback.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      const bool rollback_applied = planning_scene_->applyCollisionObject(rollback);
      const bool rollback_synchronized = rollback_applied && wait_for_world_state(object.id, false);
      response->success = false;
      response->message = rollback_synchronized ?
        "allowed-collision matrix update failed; world-object rollback completed" :
        "allowed-collision matrix update failed; world-object rollback failed";
      return;
    }
    response->message = response->success ? "world object applied and synchronized" :
      "planning scene rejected or did not synchronize world object";
  }

  void remove_world_object(
    const std::shared_ptr<RemoveWorldObject::Request> request,
    std::shared_ptr<RemoveWorldObject::Response> response)
  {
    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->message = "bridge_busy";
      return;
    }
    if (request->object_id.empty()) {
      response->message = "object_id must not be empty";
      return;
    }
    moveit_msgs::msg::CollisionObject object;
    object.id = request->object_id;
    object.header.frame_id = move_group_->getPlanningFrame();
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    const bool currently_present =
      planning_scene_->getObjects({ request->object_id }).count(request->object_id) > 0U;
    if (currently_present) {
      response->success = planning_scene_->applyCollisionObject(object);
      if (response->success) {
        response->success = wait_for_world_state(request->object_id, false);
      }
    } else {
      // Episode reset and rollback paths must be idempotent. An object that is
      // already absent satisfies the requested world state; the ACM cleanup
      // below still runs so stale collision permissions cannot leak forward.
      response->success = true;
    }
    if (response->success) {
      response->success = update_allowed_collisions(request->object_id, {}, true);
    }
    response->message = response->success ?
      (currently_present ? "world object removed and synchronized" :
      "world object already absent and ACM synchronized") :
      "planning scene rejected or did not synchronize removal";
  }

  void set_attached_object(
    const std::shared_ptr<SetAttachedObject::Request> request,
    std::shared_ptr<SetAttachedObject::Response> response)
  {
    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->message = "bridge_busy";
      return;
    }
    if (request->object_id.empty()) {
      response->message = "object_id must not be empty";
      return;
    }
    const bool currently_attached =
      planning_scene_->getAttachedObjects({ request->object_id }).count(request->object_id) > 0U;
    if (request->attach && currently_attached) {
      response->success = true;
      response->message = "object is already attached";
    } else if (request->attach) {
      const std::string link = request->link_name.empty() ? default_attach_link_ : request->link_name;
      const auto& touch_links = request->touch_links.empty() ? default_touch_links_ : request->touch_links;
      response->success = move_group_->attachObject(request->object_id, link, touch_links);
      response->message = response->success ? "attach request accepted" : "attach request failed";
    } else if (!currently_attached) {
      // Reset and rollback paths are deliberately idempotent.  Treating an
      // already-detached object as an error only produces noisy MoveIt logs
      // and makes recovery depend on state left by the previous episode.
      response->success = true;
      response->message = "object is already detached";
    } else {
      response->success = move_group_->detachObject(request->object_id);
      response->message = response->success ? "detach request accepted" : "detach request failed";
    }

    if (response->success && !wait_for_attached_state(request->object_id, request->attach)) {
      response->success = false;
      if (request->attach) {
        const bool rollback_requested = move_group_->detachObject(request->object_id);
        const bool rollback_synchronized = rollback_requested &&
          wait_for_attached_state(request->object_id, false);
        response->message = rollback_synchronized ?
          "attachment did not synchronize; detach rollback completed" :
          "attachment did not synchronize; detach rollback failed";
      } else {
        response->message = "detachment state did not synchronize before timeout";
      }
    }

    // Touch links on AttachedCollisionObject replace the temporary world-object
    // ACM entries used while approaching a grasp.  Remove those explicit pairs
    // once attachment is synchronized so they cannot leak into later episodes.
    if (response->success && request->attach &&
      !update_allowed_collisions(request->object_id, {}, true))
    {
      const bool rollback_requested = move_group_->detachObject(request->object_id);
      const bool rollback_synchronized = rollback_requested &&
        wait_for_attached_state(request->object_id, false);
      response->success = false;
      response->message = rollback_synchronized ?
        "world ACM cleanup failed; detach rollback completed" :
        "world ACM cleanup failed; detach rollback failed";
    } else if (response->success && request->attach) {
      response->message = "attachment synchronized and world ACM entry removed";
    }
  }

  bool wait_for_world_state(const std::string& object_id, bool expected_present) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(scene_sync_timeout_sec_));
    while (std::chrono::steady_clock::now() < deadline) {
      const auto objects = planning_scene_->getObjects({ object_id });
      if ((objects.find(object_id) != objects.end()) == expected_present) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool update_allowed_collisions(
    const std::string& object_id, const std::vector<std::string>& allowed_links,
    bool remove_object_entry)
  {
    const auto timeout = std::chrono::duration<double>(scene_sync_timeout_sec_);
    collision_detection::AllowedCollisionMatrix matrix;
    if (!read_allowed_collision_matrix(matrix, timeout)) {
      return false;
    }
    // Drop stale pairs when an object is updated with a different allow-list.
    matrix.removeEntry(object_id);
    if (!remove_object_entry) {
      for (const auto& link : allowed_links) {
        matrix.setEntry(object_id, link, true);
      }
    }
    moveit_msgs::msg::PlanningScene scene_update;
    scene_update.is_diff = true;
    matrix.getMessage(scene_update.allowed_collision_matrix);
    if (!planning_scene_->applyPlanningScene(scene_update)) {
      return false;
    }
    return wait_for_allowed_collisions(
      object_id, remove_object_entry ? std::vector<std::string>{} : allowed_links);
  }

  bool read_allowed_collision_matrix(
    collision_detection::AllowedCollisionMatrix& matrix,
    const std::chrono::duration<double>& timeout) const
  {
    if (!get_planning_scene_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(node_->get_logger(), "get_planning_scene service is unavailable");
      return false;
    }
    auto request = std::make_shared<GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = get_planning_scene_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Timed out while reading AllowedCollisionMatrix");
      return false;
    }
    matrix = collision_detection::AllowedCollisionMatrix(
      future.get()->scene.allowed_collision_matrix);
    return true;
  }

  static bool allowed_collisions_match(
    const collision_detection::AllowedCollisionMatrix& matrix,
    const std::string& object_id, const std::vector<std::string>& allowed_links)
  {
    if (allowed_links.empty()) {
      return !matrix.hasEntry(object_id);
    }
    if (!matrix.hasEntry(object_id)) {
      return false;
    }

    const std::set<std::string> expected(allowed_links.begin(), allowed_links.end());
    std::set<std::string> observed;
    std::vector<std::string> names;
    matrix.getAllEntryNames(names);
    for (const auto& name : names) {
      if (name == object_id) {
        continue;
      }
      collision_detection::AllowedCollision::Type type;
      if (matrix.getEntry(object_id, name, type)) {
        if (type == collision_detection::AllowedCollision::ALWAYS) {
          observed.insert(name);
        } else if (type == collision_detection::AllowedCollision::CONDITIONAL) {
          return false;
        }
      }
    }
    return observed == expected;
  }

  bool wait_for_allowed_collisions(
    const std::string& object_id, const std::vector<std::string>& allowed_links) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(scene_sync_timeout_sec_));
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration<double>(
        deadline - std::chrono::steady_clock::now());
      collision_detection::AllowedCollisionMatrix current;
      if (read_allowed_collision_matrix(current, remaining) &&
        allowed_collisions_match(current, object_id, allowed_links))
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_ERROR(
      node_->get_logger(), "AllowedCollisionMatrix did not synchronize for object [%s]",
      object_id.c_str());
    return false;
  }

  bool wait_for_attached_state(const std::string& object_id, bool expected_attached) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(scene_sync_timeout_sec_));
    while (std::chrono::steady_clock::now() < deadline) {
      const auto attached = planning_scene_->getAttachedObjects({ object_id });
      if ((attached.find(object_id) != attached.end()) == expected_attached) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  rclcpp::Client<GetPlanningScene>::SharedPtr get_planning_scene_client_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp_action::Server<Motion>::SharedPtr action_server_;
  rclcpp::Service<UpsertWorldObject>::SharedPtr upsert_service_;
  rclcpp::Service<RemoveWorldObject>::SharedPtr remove_service_;
  rclcpp::Service<SetAttachedObject>::SharedPtr attach_service_;

  std::atomic_bool busy_{ false };
  std::atomic_bool cancel_requested_{ false };
  std::mutex operation_mutex_;
  std::string planning_group_;
  std::string action_name_;
  std::string default_planner_id_;
  std::string default_end_effector_link_;
  std::string default_attach_link_;
  std::vector<std::string> default_touch_links_;
  double default_planning_time_sec_;
  double default_velocity_scaling_factor_;
  double default_acceleration_scaling_factor_;
  double scene_sync_timeout_sec_;
  int max_planning_attempts_;
};

}  // namespace panda_moveit_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("panda_moveit_bridge", options);
  try {
    auto bridge = std::make_shared<panda_moveit_bridge::MoveItBridge>(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize MoveIt bridge: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
