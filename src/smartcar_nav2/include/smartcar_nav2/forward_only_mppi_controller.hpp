#ifndef SMARTCAR_NAV2__FORWARD_ONLY_MPPI_CONTROLLER_HPP_
#define SMARTCAR_NAV2__FORWARD_ONLY_MPPI_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/forward_command_filter.hpp"

namespace smartcar_nav2
{

class ForwardOnlyMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  ForwardOnlyMPPIController() = default;
  ~ForwardOnlyMPPIController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void readSafetyParameters();
  geometry_msgs::msg::TwistStamped stoppedCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  mutable std::mutex limits_mutex_;
  ForwardCommandLimits configured_limits_{};
  ForwardCommandLimits active_limits_{};
  bool configured_{false};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FORWARD_ONLY_MPPI_CONTROLLER_HPP_
