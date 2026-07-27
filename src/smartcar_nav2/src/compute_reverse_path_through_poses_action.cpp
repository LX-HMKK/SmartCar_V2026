#include "smartcar_nav2/compute_reverse_path_through_poses_action.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smartcar_nav2
{

ComputeReversePathThroughPosesAction::ComputeReversePathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & configuration)
: nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(
    xml_tag_name, action_name, configuration)
{
  tf_buffer_ = configuration.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_base_frame", robot_base_frame_);
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

void ComputeReversePathThroughPosesAction::on_tick()
{
  goal_ = nav2_msgs::action::ComputePathThroughPoses::Goal();
  if (!getInput("goals", real_goals_)) {
    RCLCPP_ERROR(node_->get_logger(), "Reverse through-poses planner goals are missing");
    should_send_goal_ = false;
    return;
  }
  if (real_goals_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Reverse through-poses planner goals are empty");
    should_send_goal_ = false;
    return;
  }
  if (!getInput("planner_id", goal_.planner_id) || !readValidationOptions()) {
    RCLCPP_ERROR(node_->get_logger(), "Reverse through-poses planner ports are invalid");
    should_send_goal_ = false;
    return;
  }
  for (size_t i = 0; i < real_goals_.size(); ++i) {
    if (real_goals_[i].header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Reverse through-poses planner goal[%zu] frame '%s' must equal global frame '%s'",
        i, real_goals_[i].header.frame_id.c_str(), global_frame_.c_str());
      should_send_goal_ = false;
      return;
    }
  }
  if (!nav2_util::getCurrentPose(
      real_start_, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Could not obtain a fresh reverse through-poses planning start pose");
    should_send_goal_ = false;
    return;
  }
  if (!rotatePoseYawByPi(real_start_, goal_.start)) {
    RCLCPP_ERROR(node_->get_logger(),
      "Reverse through-poses planner received an invalid start quaternion");
    should_send_goal_ = false;
    return;
  }
  goal_.goals.resize(real_goals_.size());
  for (size_t i = 0; i < real_goals_.size(); ++i) {
    if (!rotatePoseYawByPi(real_goals_[i], goal_.goals[i])) {
      RCLCPP_ERROR(node_->get_logger(),
        "Reverse through-poses planner received an invalid quaternion in goal[%zu]", i);
      should_send_goal_ = false;
      return;
    }
  }
  goal_.use_start = true;
}

BT::NodeStatus ComputeReversePathThroughPosesAction::on_success()
{
  if (!result_.result) {
    clearPathOutput();
    RCLCPP_ERROR(node_->get_logger(), "Reverse through-poses planner returned no result");
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path restored_path = result_.result->path;
  for (auto & pose : restored_path.poses) {
    geometry_msgs::msg::PoseStamped restored_pose;
    if (!rotatePoseYawByPi(pose, restored_pose)) {
      clearPathOutput();
      RCLCPP_ERROR(node_->get_logger(),
        "Reverse through-poses path contains an invalid quaternion");
      return BT::NodeStatus::FAILURE;
    }
    pose = restored_pose;
  }

  // Deduplicate consecutive poses: Smac Hybrid may emit duplicate
  // poses at angle-bin boundaries, which produce zero-length segments
  // that fail the minimum_segment_length validation check.
  {
    nav_msgs::msg::Path deduped;
    deduped.header = restored_path.header;
    deduped.poses.reserve(restored_path.poses.size());
    constexpr double kDedupEpsilon = 1.0e-6;
    for (size_t i = 0; i < restored_path.poses.size(); ++i) {
      if (deduped.poses.empty()) {
        deduped.poses.push_back(restored_path.poses[i]);
        continue;
      }
      const auto & prev = deduped.poses.back().pose.position;
      const auto & cur = restored_path.poses[i].pose.position;
      double dx = cur.x - prev.x;
      double dy = cur.y - prev.y;
      if (dx * dx + dy * dy > kDedupEpsilon) {
        deduped.poses.push_back(restored_path.poses[i]);
      }
    }
    restored_path = std::move(deduped);
  }

  const auto & last_real_goal = real_goals_.back();
  const auto validation = validateReversePath(
    restored_path, real_start_, last_real_goal, validation_options_);
  if (!validation.valid) {
    clearPathOutput();
    RCLCPP_ERROR(
      node_->get_logger(),
      "Rejected reverse through-poses path: %s at segment %zu/%zu (observed=%.6f, limit=%.6f)",
      validation.reason.c_str(), validation.segment_index,
      restored_path.poses.size(),
      validation.observed_value, validation.limit);
    if (
      validation.reason == "curvature_exceeded" &&
      validation.segment_index > 0 &&
      validation.segment_index + 1 < restored_path.poses.size())
    {
      const auto & previous =
        restored_path.poses[validation.segment_index - 1].pose.position;
      const auto & current =
        restored_path.poses[validation.segment_index].pose.position;
      const auto & next =
        restored_path.poses[validation.segment_index + 1].pose.position;
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rejected curvature samples: previous=(%.6f, %.6f), current=(%.6f, %.6f), "
        "next=(%.6f, %.6f)",
        previous.x, previous.y, current.x, current.y, next.x, next.y);
    }
    return BT::NodeStatus::FAILURE;
  }

  setOutput("path", restored_path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeReversePathThroughPosesAction::on_aborted()
{
  clearPathOutput();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeReversePathThroughPosesAction::on_cancelled()
{
  clearPathOutput();
  return BT::NodeStatus::FAILURE;
}

void ComputeReversePathThroughPosesAction::halt()
{
  clearPathOutput();
  nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathThroughPoses>::halt();
}

bool ComputeReversePathThroughPosesAction::readValidationOptions()
{
  return getInput(
    "minimum_turning_radius", validation_options_.minimum_turning_radius) &&
         getInput("curvature_tolerance", validation_options_.curvature_tolerance) &&
         getInput(
    "maximum_direction_error", validation_options_.maximum_direction_error) &&
         getInput(
    "start_position_tolerance", validation_options_.start_position_tolerance) &&
         getInput("start_yaw_tolerance", validation_options_.start_yaw_tolerance) &&
         getInput(
    "goal_position_tolerance", validation_options_.goal_position_tolerance) &&
         getInput("goal_yaw_tolerance", validation_options_.goal_yaw_tolerance) &&
         getInput("minimum_segment_length", validation_options_.minimum_segment_length);
}

void ComputeReversePathThroughPosesAction::clearPathOutput()
{
  setOutput("path", nav_msgs::msg::Path());
}

}  // namespace smartcar_nav2

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<smartcar_nav2::ComputeReversePathThroughPosesAction>(
        name, "compute_path_through_poses", configuration);
    };
  factory.registerBuilder<smartcar_nav2::ComputeReversePathThroughPosesAction>(
    "ComputeReversePathThroughPoses", builder);
}
