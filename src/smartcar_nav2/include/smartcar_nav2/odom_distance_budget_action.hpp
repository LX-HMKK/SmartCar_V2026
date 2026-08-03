#ifndef SMARTCAR_NAV2__ODOM_DISTANCE_BUDGET_ACTION_HPP_
#define SMARTCAR_NAV2__ODOM_DISTANCE_BUDGET_ACTION_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "smartcar_nav2/odom_distance_budget_tracker.hpp"

namespace smartcar_nav2
{

// A per-navigation travel fuse. It supports NavigateToPose's single goal and
// NavigateThroughPoses' ordered goals, using the initial terminal target for
// one fixed measured-travel budget. It has no command publisher: exceeding
// the budget halts the child tree, which cancels FollowPath, then returns
// FAILURE to the navigator instead of entering recovery motion.
class OdomDistanceBudgetAction : public BT::DecoratorNode
{
public:
  OdomDistanceBudgetAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & configuration);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  struct OdomSample
  {
    OdomPlanarPose pose{};
    std::string frame_id;
    std::string child_frame_id;
    std::chrono::steady_clock::time_point received_at{};
  };

  enum class ArmResult
  {
    kReady,
    kWaitingForOdom,
    kFailure,
  };

  ArmResult beginNavigation();
  bool readConfiguration();
  bool readGoalPoses();
  bool hasAbortCondition(std::string & reason, double & travelled_m, double & budget_m);
  void stopMonitoring();
  void abortNavigation(const std::string & reason, double travelled_m, double budget_m);
  void updateOdometry(nav_msgs::msg::Odometry::SharedPtr odometry);
  bool sampleMatchesFrames(const OdomSample & sample) const;

  rclcpp::Node::SharedPtr node_;
  // bt_navigator may execute the tree without servicing subscriptions in its
  // default callback group. Keep measured odometry on a private group that
  // this decorator explicitly advances before it evaluates the safety fuse.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  std::mutex mutex_;
  std::optional<OdomSample> latest_odom_;
  OdomDistanceBudgetTracker tracker_;
  std::vector<OdomPlanarPose> goal_poses_;
  bool awaiting_initial_odom_{false};
  std::chrono::steady_clock::time_point initial_odom_deadline_{};
  bool monitoring_{false};
  bool odom_fault_{false};
  std::string odom_fault_reason_;

  std::string odom_frame_{"odom_combined"};
  std::string robot_base_frame_{"base_footprint"};
  double max_distance_ratio_{2.0};
  double distance_slack_m_{0.80};
  double max_odom_step_m_{0.50};
  std::chrono::milliseconds odom_timeout_{500};
  std::chrono::milliseconds initial_odom_wait_timeout_{1000};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__ODOM_DISTANCE_BUDGET_ACTION_HPP_
