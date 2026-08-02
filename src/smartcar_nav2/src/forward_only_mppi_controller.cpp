#include "smartcar_nav2/forward_only_mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smartcar_nav2
{
namespace
{

bool containsCritic(
  const std::vector<std::string> & critics, const std::string & critic_name)
{
  return std::find(critics.begin(), critics.end(), critic_name) != critics.end();
}

}  // namespace

void ForwardOnlyMPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_mppi_controller::MPPIController::configure(
    parent, std::move(name), std::move(tf), std::move(costmap_ros));
  readSafetyParameters();
}

void ForwardOnlyMPPIController::readSafetyParameters()
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("ForwardOnlyMPPIController parent node expired");
  }

  double vx_min = 0.0;
  double vx_max = 0.0;
  double wz_max = 0.0;
  double min_turning_radius = 0.0;
  bool consider_footprint = false;
  bool forward_preference = false;
  std::string motion_model;
  std::vector<std::string> critics;
  const std::string prefix = name_ + ".";
  const std::string ackermann_prefix = prefix + "AckermannConstraints.";
  if (!node->get_parameter(prefix + "vx_min", vx_min) ||
    !node->get_parameter(prefix + "vx_max", vx_max) ||
    !node->get_parameter(prefix + "wz_max", wz_max) ||
    !node->get_parameter(prefix + "motion_model", motion_model) ||
    !node->get_parameter(prefix + "critics", critics) ||
    !node->get_parameter(prefix + "CostCritic.consider_footprint", consider_footprint) ||
    !node->get_parameter(prefix + "PathAngleCritic.forward_preference", forward_preference) ||
    !node->get_parameter(ackermann_prefix + "min_turning_r", min_turning_radius))
  {
    throw std::runtime_error(
            "ForwardOnlyMPPIController could not read Ackermann safety parameters");
  }

  if (!finiteForwardValue(vx_min) || !finiteForwardValue(vx_max) ||
    !finiteForwardValue(wz_max) || !finiteForwardValue(min_turning_radius) ||
    vx_min < 0.0 || vx_max <= 0.0 || vx_min > vx_max || wz_max < 0.0 ||
    min_turning_radius <= 0.0 || motion_model != "Ackermann" || !consider_footprint ||
    !forward_preference ||
    !containsCritic(critics, "GoalAngleCritic") ||
    !containsCritic(critics, "PathAngleCritic"))
  {
    throw std::runtime_error(
            "ForwardOnlyMPPIController requires non-negative forward velocity, "
            "Ackermann motion, footprint CostCritic, GoalAngleCritic, and "
            "forward-preferring PathAngleCritic");
  }

  std::lock_guard<std::mutex> lock(limits_mutex_);
  configured_limits_ = ForwardCommandLimits{vx_max, wz_max, min_turning_radius};
  active_limits_ = configured_limits_;
  configured_ = true;
}

geometry_msgs::msg::TwistStamped ForwardOnlyMPPIController::stoppedCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = robot_pose.header.stamp;
  if (costmap_ros_) {
    command.header.frame_id = costmap_ros_->getBaseFrameID();
  }
  return command;
}

geometry_msgs::msg::TwistStamped ForwardOnlyMPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  auto command = nav2_mppi_controller::MPPIController::computeVelocityCommands(
    robot_pose, robot_speed, goal_checker);

  ForwardCommandLimits limits;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    limits = active_limits_;
  }

  const ForwardCommand input{
    command.twist.linear.x,
    command.twist.linear.y,
    command.twist.linear.z,
    command.twist.angular.x,
    command.twist.angular.y,
    command.twist.angular.z};
  const auto filtered = enforceForwardCommandLimits(input, limits);
  if (filtered.status != ForwardCommandFilterStatus::kAccepted) {
    if (clock_) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 1000,
        "Forward-only MPPI rejected command: %s",
        forwardCommandFilterStatusName(filtered.status));
    } else {
      RCLCPP_ERROR(
        logger_, "Forward-only MPPI rejected command: %s",
        forwardCommandFilterStatusName(filtered.status));
    }
    command.twist = geometry_msgs::msg::Twist();
    return command;
  }

  if (filtered.limited && clock_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Forward-only MPPI clamped a command to the Ackermann envelope");
  }
  command.twist.linear.x = filtered.command.linear_x;
  command.twist.linear.y = filtered.command.linear_y;
  command.twist.linear.z = filtered.command.linear_z;
  command.twist.angular.x = filtered.command.angular_x;
  command.twist.angular.y = filtered.command.angular_y;
  command.twist.angular.z = filtered.command.angular_z;
  return command;
}

void ForwardOnlyMPPIController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  ForwardCommandLimits configured;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    if (!configured_) {
      RCLCPP_ERROR(logger_, "Forward-only MPPI received speed limit before configure");
      return;
    }
    configured = configured_limits_;
  }

  const auto translation = translateForwardSpeedLimit(
    speed_limit, percentage, configured.vx_max);
  if (!translation.valid) {
    RCLCPP_ERROR(
      logger_, "Forward-only MPPI ignored invalid speed limit %.6f", speed_limit);
    return;
  }

  nav2_mppi_controller::MPPIController::setSpeedLimit(
    translation.forwarded_speed_limit, translation.forwarded_percentage);

  std::lock_guard<std::mutex> lock(limits_mutex_);
  active_limits_.vx_max = configured_limits_.vx_max * translation.guard_scale;
  active_limits_.wz_max = configured_limits_.wz_max * translation.guard_scale;
  active_limits_.min_turning_radius = configured_limits_.min_turning_radius;
}

}  // namespace smartcar_nav2

PLUGINLIB_EXPORT_CLASS(smartcar_nav2::ForwardOnlyMPPIController, nav2_core::Controller)
