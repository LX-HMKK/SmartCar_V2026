#include "smartcar_nav2/reverse_only_mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "smartcar_nav2/reverse_path_utils.hpp"

namespace smartcar_nav2
{
namespace
{

constexpr double kFootprintSymmetryTolerance = 1.0e-6;

bool containsCritic(
  const std::vector<std::string> & critics, const std::string & critic_name)
{
  return std::find(critics.begin(), critics.end(), critic_name) != critics.end();
}

bool piSymmetricFootprint(const std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (footprint.empty()) {
    return false;
  }

  for (const auto & point : footprint) {
    const auto opposite = std::find_if(
      footprint.begin(), footprint.end(), [&point](const auto & candidate) {
        return std::hypot(
          point.x + candidate.x, point.y + candidate.y) <=
               kFootprintSymmetryTolerance;
      });
    if (opposite == footprint.end()) {
      return false;
    }
  }
  return true;
}

}  // namespace

void ReverseOnlyMPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_mppi_controller::MPPIController::configure(
    parent, std::move(name), std::move(tf), std::move(costmap_ros));
  readSafetyParameters();
}

void ReverseOnlyMPPIController::readSafetyParameters()
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("ReverseOnlyMPPIController parent node expired");
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
    !node->get_parameter(
      prefix + "PathAngleCritic.forward_preference", forward_preference) ||
    !node->get_parameter(
      prefix + "CostCritic.consider_footprint", consider_footprint) ||
    !node->get_parameter(ackermann_prefix + "min_turning_r", min_turning_radius))
  {
    throw std::runtime_error(
            "ReverseOnlyMPPIController could not read virtual-forward MPPI parameters");
  }

  if (!finiteValue(vx_min) || !finiteValue(vx_max) || !finiteValue(wz_max) ||
    !finiteValue(min_turning_radius) || vx_min < 0.0 || vx_max <= 0.0 ||
    vx_min > vx_max || wz_max < 0.0 || min_turning_radius <= 0.0 ||
    motion_model != "Ackermann" || !forward_preference ||
    !containsCritic(critics, "GoalAngleCritic") ||
    !containsCritic(critics, "PathAngleCritic"))
  {
    throw std::runtime_error(
            "ReverseOnlyMPPIController requires 0 <= vx_min <= vx_max, Ackermann motion, "
            "GoalAngleCritic, and forward-preferring PathAngleCritic");
  }
  if (consider_footprint && !piSymmetricFootprint(costmap_ros_->getRobotFootprint())) {
    throw std::runtime_error(
            "Virtual-forward MPPI requires a pi-symmetric footprint when "
            "CostCritic.consider_footprint is enabled");
  }

  std::lock_guard<std::mutex> lock(limits_mutex_);
  configured_limits_ = ReverseCommandLimits{-vx_max, wz_max, min_turning_radius};
  active_limits_ = configured_limits_;
  configured_ = true;
}

void ReverseOnlyMPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path virtual_plan = path;
  for (auto & pose : virtual_plan.poses) {
    geometry_msgs::msg::PoseStamped virtual_pose;
    if (!rotatePoseYawByPi(pose, virtual_pose)) {
      throw std::runtime_error(
              "ReverseOnlyMPPIController received a path with an invalid orientation");
    }
    pose = std::move(virtual_pose);
  }
  nav2_mppi_controller::MPPIController::setPlan(virtual_plan);
}

geometry_msgs::msg::TwistStamped ReverseOnlyMPPIController::stoppedCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = robot_pose.header.stamp;
  if (costmap_ros_) {
    command.header.frame_id = costmap_ros_->getBaseFrameID();
  }
  return command;
}

geometry_msgs::msg::TwistStamped ReverseOnlyMPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::PoseStamped virtual_pose;
  if (!rotatePoseYawByPi(robot_pose, virtual_pose)) {
    RCLCPP_ERROR(logger_, "Reverse-only MPPI received an invalid robot orientation");
    return stoppedCommand(robot_pose);
  }

  const ReverseCommand measured_speed{
    robot_speed.linear.x,
    robot_speed.linear.y,
    robot_speed.linear.z,
    robot_speed.angular.x,
    robot_speed.angular.y,
    robot_speed.angular.z};
  if (!finiteReverseCommand(measured_speed)) {
    RCLCPP_ERROR(logger_, "Reverse-only MPPI received a non-finite measured velocity");
    return stoppedCommand(robot_pose);
  }

  geometry_msgs::msg::Twist virtual_speed = robot_speed;
  virtual_speed.linear.x = -robot_speed.linear.x;
  virtual_speed.linear.y = -robot_speed.linear.y;
  auto command = nav2_mppi_controller::MPPIController::computeVelocityCommands(
    virtual_pose, virtual_speed, goal_checker);

  ReverseCommandLimits limits;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    limits = active_limits_;
  }

  const ReverseCommand virtual_output{
    command.twist.linear.x,
    command.twist.linear.y,
    command.twist.linear.z,
    command.twist.angular.x,
    command.twist.angular.y,
    command.twist.angular.z};
  const auto reverse_output = mapVirtualForwardCommandToReverse(virtual_output);
  const auto filtered = enforceReverseCommandLimits(reverse_output, limits);
  if (filtered.status != ReverseCommandFilterStatus::kAccepted) {
    if (clock_) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 1000,
        "Reverse-only MPPI rejected command: %s",
        reverseCommandFilterStatusName(filtered.status));
    } else {
      RCLCPP_ERROR(
        logger_, "Reverse-only MPPI rejected command: %s",
        reverseCommandFilterStatusName(filtered.status));
    }
    command.twist = geometry_msgs::msg::Twist();
    return command;
  }

  command.twist.linear.x = filtered.command.linear_x;
  command.twist.linear.y = filtered.command.linear_y;
  command.twist.linear.z = filtered.command.linear_z;
  command.twist.angular.x = filtered.command.angular_x;
  command.twist.angular.y = filtered.command.angular_y;
  command.twist.angular.z = filtered.command.angular_z;
  return command;
}

void ReverseOnlyMPPIController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  ReverseCommandLimits configured;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    if (!configured_) {
      RCLCPP_ERROR(logger_, "Reverse-only MPPI received speed limit before configure");
      return;
    }
    configured = configured_limits_;
  }

  const auto translation = translateReverseSpeedLimit(
    speed_limit, percentage, configured.vx_min);
  if (!translation.valid) {
    RCLCPP_ERROR(
      logger_, "Reverse-only MPPI ignored invalid speed limit %.6f", speed_limit);
    return;
  }

  // Keep the virtual positive-forward optimizer and the real reverse command
  // guard on the same scale for percentage and absolute speed limits.
  nav2_mppi_controller::MPPIController::setSpeedLimit(
    translation.forwarded_speed_limit, translation.forwarded_percentage);

  std::lock_guard<std::mutex> lock(limits_mutex_);
  active_limits_.vx_min = configured_limits_.vx_min * translation.guard_scale;
  active_limits_.wz_max = configured_limits_.wz_max * translation.guard_scale;
  active_limits_.min_turning_radius = configured_limits_.min_turning_radius;
}

}  // namespace smartcar_nav2

PLUGINLIB_EXPORT_CLASS(smartcar_nav2::ReverseOnlyMPPIController, nav2_core::Controller)
