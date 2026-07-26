#ifndef SMARTCAR_NAV2__REVERSE_ONLY_MPPI_CONTROLLER_HPP_
#define SMARTCAR_NAV2__REVERSE_ONLY_MPPI_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/reverse_command_filter.hpp"

namespace smartcar_nav2
{

class ReverseOnlyMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  ReverseOnlyMPPIController() = default;
  ~ReverseOnlyMPPIController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void readSafetyParameters();
  geometry_msgs::msg::TwistStamped stoppedCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  mutable std::mutex limits_mutex_;
  ReverseCommandLimits configured_limits_{};
  ReverseCommandLimits active_limits_{};
  bool configured_{false};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__REVERSE_ONLY_MPPI_CONTROLLER_HPP_
