#ifndef SMARTCAR_NAV2__RECORD_FOLLOW_PATH_ACTION_HPP_
#define SMARTCAR_NAV2__RECORD_FOLLOW_PATH_ACTION_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"

namespace smartcar_nav2
{

// Mirrors Nav2's FollowPath BT action and records only goals acknowledged by
// controller_server. Planner candidates never reach this publisher.
class RecordFollowPathAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  RecordFollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & configuration);
  ~RecordFollowPathAction() override;

  BT::NodeStatus tick() override;
  void on_tick() override;
  void on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback) override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::InputPort<std::string>("goal_checker_id", ""),
        BT::InputPort<bool>(
          "verify_physical_terminal_pose", false,
          "Require the measured robot pose to match the supplied physical terminal goal"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "terminal_goal",
          "Physical terminal goal to verify after FollowPath succeeds"),
        BT::InputPort<double>(
          "terminal_position_tolerance_m", 0.0,
          "Measured terminal position tolerance when physical verification is enabled"),
        BT::InputPort<double>(
          "terminal_yaw_tolerance_rad", 0.0,
          "Measured terminal yaw tolerance when physical verification is enabled"),
        BT::InputPort<int>(
          "terminal_verification_delay_ms", 0,
          "Delay after FollowPath success before reading the physical terminal pose"),
        BT::OutputPort<bool>(
          "terminal_recovery_eligible",
          "True only after FollowPath reports success outside the physical terminal envelope"),
      });
  }

private:
  enum class TerminalVerificationResult
  {
    kPassed,
    kWaiting,
    kFailed,
  };

  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using FollowPathClient = rclcpp_action::Client<FollowPath>;

  void sendRecordedGoal();
  bool isGoalHandleComplete(std::chrono::milliseconds & elapsed);
  std::chrono::milliseconds goalAcknowledgementTimeout() const;
  void handleGoalResponse(
    std::uint64_t generation,
    const nav_msgs::msg::Path & path,
    FollowPathGoalHandle::SharedPtr goal_handle);
  void cancelOutstandingDispatch();
  void startLateGoalAcknowledgementGuard();
  void reapLateGoalAcknowledgementGuard();
  void stopLateGoalAcknowledgementGuard();
  void clearAcceptedGoalHandle();
  void publishAcceptedPath(const nav_msgs::msg::Path & path) const;
  TerminalVerificationResult verifyPhysicalTerminalPose();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  bool terminal_verification_pending_{false};
  std::chrono::steady_clock::time_point terminal_verification_deadline_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr accepted_path_publisher_;
  std::mutex dispatch_mutex_;
  std::uint64_t next_dispatch_generation_{0U};
  std::uint64_t active_dispatch_generation_{0U};
  std::uint64_t cancelled_dispatch_generation_{0U};
  FollowPathGoalHandle::SharedPtr accepted_goal_handle_;
  std::shared_future<FollowPathGoalHandle::SharedPtr> unacknowledged_goal_future_;
  std::atomic<bool> late_goal_acknowledgement_pending_{false};
  std::atomic<bool> stop_late_goal_acknowledgement_guard_{false};
  std::thread late_goal_acknowledgement_guard_;
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__RECORD_FOLLOW_PATH_ACTION_HPP_
