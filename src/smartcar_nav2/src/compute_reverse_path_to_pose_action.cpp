#include "smartcar_nav2/compute_reverse_path_to_pose_action.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smartcar_nav2
{

ComputeReversePathToPoseAction::ComputeReversePathToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & configuration)
: nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathToPose>(
    xml_tag_name, action_name, configuration)
{
  tf_buffer_ = configuration.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_base_frame", robot_base_frame_);
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

void ComputeReversePathToPoseAction::on_tick()
{
  goal_ = nav2_msgs::action::ComputePathToPose::Goal();
  if (!getInput("goal", real_goal_)) {
    RCLCPP_ERROR(node_->get_logger(), "Reverse planner goal is missing");
    should_send_goal_ = false;
    return;
  }
  if (!getInput("planner_id", goal_.planner_id) || !readValidationOptions()) {
    RCLCPP_ERROR(node_->get_logger(), "Reverse planner ports are invalid");
    should_send_goal_ = false;
    return;
  }
  if (real_goal_.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Reverse planner goal frame '%s' must equal global frame '%s'",
      real_goal_.header.frame_id.c_str(), global_frame_.c_str());
    should_send_goal_ = false;
    return;
  }
  if (!nav2_util::getCurrentPose(
      real_start_, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not obtain a fresh reverse planning start pose");
    should_send_goal_ = false;
    return;
  }
  if (!rotatePoseYawByPi(real_start_, goal_.start) ||
    !rotatePoseYawByPi(real_goal_, goal_.goal))
  {
    RCLCPP_ERROR(node_->get_logger(), "Reverse planner received an invalid quaternion");
    should_send_goal_ = false;
    return;
  }
  goal_.use_start = true;
}

BT::NodeStatus ComputeReversePathToPoseAction::on_success()
{
  if (!result_.result) {
    clearPathOutput();
    RCLCPP_ERROR(node_->get_logger(), "Reverse planner returned no result");
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path restored_path = result_.result->path;
  for (auto & pose : restored_path.poses) {
    geometry_msgs::msg::PoseStamped restored_pose;
    if (!rotatePoseYawByPi(pose, restored_pose)) {
      clearPathOutput();
      RCLCPP_ERROR(node_->get_logger(), "Reverse planner path contains an invalid quaternion");
      return BT::NodeStatus::FAILURE;
    }
    pose = restored_pose;
  }

  const auto validation = validateReversePath(
    restored_path, real_start_, real_goal_, validation_options_);
  if (!validation.valid) {
    clearPathOutput();
    RCLCPP_ERROR(
      node_->get_logger(),
      "Rejected reverse path: %s at segment %zu/%zu (observed=%.6f, limit=%.6f)",
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

BT::NodeStatus ComputeReversePathToPoseAction::on_aborted()
{
  clearPathOutput();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeReversePathToPoseAction::on_cancelled()
{
  clearPathOutput();
  return BT::NodeStatus::FAILURE;
}

void ComputeReversePathToPoseAction::halt()
{
  clearPathOutput();
  nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathToPose>::halt();
}

bool ComputeReversePathToPoseAction::readValidationOptions()
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

void ComputeReversePathToPoseAction::clearPathOutput()
{
  setOutput("path", nav_msgs::msg::Path());
}

}  // namespace smartcar_nav2

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<smartcar_nav2::ComputeReversePathToPoseAction>(
        name, "compute_path_to_pose", configuration);
    };
  factory.registerBuilder<smartcar_nav2::ComputeReversePathToPoseAction>(
    "ComputeReversePathToPose", builder);
}
