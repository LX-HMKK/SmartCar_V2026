#ifndef SMARTCAR_NAV2__FORWARD_ONLY_RPP_CONTROLLER_HPP_
#define SMARTCAR_NAV2__FORWARD_ONLY_RPP_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/forward_command_filter.hpp"
#include "smartcar_nav2/forward_path_tracking_guard.hpp"

namespace smartcar_nav2
{

/// RPP tracking with a final forward-only Ackermann command guard.
class ForwardOnlyRPPController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  ForwardOnlyRPPController() = default;
  ~ForwardOnlyRPPController() override = default;

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
  geometry_msgs::msg::TwistStamped computeCurvatureTrackingCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker,
    const nav_msgs::msg::Path & confirmed_plan,
    const ForwardPathTrackingProjection & projection);

  mutable std::mutex limits_mutex_;
  ForwardCommandLimits configured_limits_{};
  ForwardCommandLimits active_limits_{};
  bool configured_{false};

  // This copy is populated only after the base controller accepts setPlan().
  // It remains independent of RPP's pruned tracking plan, so a local shortcut
  // cannot make an already-swept global segment disappear from the guard.
  mutable std::mutex path_tracking_mutex_;
  nav_msgs::msg::Path confirmed_plan_;
  double forward_path_max_cross_track_error_{0.0};
  double forward_terminal_lookahead_m_{0.0};
  double forward_terminal_activation_distance_m_{0.0};
  bool forward_path_use_curvature_tracking_{false};
  double forward_path_heading_gain_{0.0};
  double forward_path_cross_track_gain_{0.0};
  double forward_path_collision_projection_m_{0.0};
  double forward_path_tight_turn_speed_mps_{0.0};
  double forward_path_tight_turn_radius_m_{0.0};
  double forward_path_tight_turn_preview_m_{0.0};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FORWARD_ONLY_RPP_CONTROLLER_HPP_
