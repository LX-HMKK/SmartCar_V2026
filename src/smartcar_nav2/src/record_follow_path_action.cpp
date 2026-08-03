#include "smartcar_nav2/record_follow_path_action.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smartcar_nav2
{
namespace
{

constexpr char kAcceptedGlobalPlanTopic[] = "/smartcar/accepted_global_plan";
constexpr std::chrono::milliseconds kMinimumGoalAcknowledgementTimeout{500};
constexpr std::chrono::milliseconds kLateGoalAcknowledgementSpinPeriod{50};
constexpr double kQuaternionNormTolerance = 1.0e-3;

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

bool quaternionYaw(const geometry_msgs::msg::Quaternion & quaternion, double & yaw)
{
  if (!finiteQuaternion(quaternion)) {
    return false;
  }
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > kQuaternionNormTolerance) {
    return false;
  }
  yaw = std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
  return std::isfinite(yaw);
}

double angularDistance(const double first, const double second)
{
  return std::abs(std::atan2(std::sin(first - second), std::cos(first - second)));
}

}  // namespace

class ReverseRecoveryEligible : public BT::ConditionNode
{
public:
  ReverseRecoveryEligible(
    const std::string & name,
    const BT::NodeConfiguration & configuration)
  : BT::ConditionNode(name, configuration)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("eligible", false, "Whether reverse retreat is authorized"),
    };
  }

  BT::NodeStatus tick() override
  {
    bool eligible = false;
    if (!getInput("eligible", eligible)) {
      return BT::NodeStatus::FAILURE;
    }
    return eligible ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

RecordFollowPathAction::RecordFollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & configuration)
: nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>(
    xml_tag_name, action_name, configuration)
{
  tf_buffer_ = configuration.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_base_frame", robot_base_frame_);
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  accepted_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    kAcceptedGlobalPlanTopic, rclcpp::QoS(1).reliable().transient_local());
}

RecordFollowPathAction::~RecordFollowPathAction()
{
  stopLateGoalAcknowledgementGuard();
  cancelOutstandingDispatch();
}

void RecordFollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  terminal_verification_pending_ = false;
  terminal_verification_deadline_ = std::chrono::steady_clock::time_point();

  bool verify_terminal_pose = false;
  if (getInput("verify_physical_terminal_pose", verify_terminal_pose) &&
    verify_terminal_pose)
  {
    setOutput("terminal_recovery_eligible", false);
  }
}

void RecordFollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> /*feedback*/)
{
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);
  if (goal_.path != new_path && new_path != nav_msgs::msg::Path()) {
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);
  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);
  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

BT::NodeStatus RecordFollowPathAction::tick()
{
  reapLateGoalAcknowledgementGuard();
  if (late_goal_acknowledgement_pending_.load()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Refusing to dispatch %s while a cancelled FollowPath goal acknowledgement is unresolved",
      action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Keep Nav2 BtActionNode control flow intact. The only extension is the
  // goal-response callback in sendRecordedGoal(), which records paths after
  // controller_server acknowledges each FollowPath goal.
  if (status() == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);
    should_send_goal_ = true;
    on_tick();
    if (!should_send_goal_) {
      return BT::NodeStatus::FAILURE;
    }
    sendRecordedGoal();
  }

  try {
    if (future_goal_handle_) {
      auto elapsed =
        (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
      if (!isGoalHandleComplete(elapsed)) {
        if (elapsed < goalAcknowledgementTimeout()) {
          return BT::NodeStatus::RUNNING;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "Timed out while waiting for action server to acknowledge goal request for %s",
          action_name_.c_str());
        cancelOutstandingDispatch();
        future_goal_handle_.reset();
        startLateGoalAcknowledgementGuard();
        return BT::NodeStatus::FAILURE;
      }
      unacknowledged_goal_future_ = std::shared_future<FollowPathGoalHandle::SharedPtr>();
    }

    if (rclcpp::ok() && !goal_result_available_) {
      on_wait_for_result(feedback_);
      feedback_.reset();

      const auto goal_status = goal_handle_->get_status();
      if (goal_updated_ &&
        (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
      {
        goal_updated_ = false;
        sendRecordedGoal();
        auto elapsed =
          (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
        if (!isGoalHandleComplete(elapsed)) {
          if (elapsed < goalAcknowledgementTimeout()) {
            return BT::NodeStatus::RUNNING;
          }
          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          cancelOutstandingDispatch();
          future_goal_handle_.reset();
          startLateGoalAcknowledgementGuard();
          return BT::NodeStatus::FAILURE;
        }
        unacknowledged_goal_future_ = std::shared_future<FollowPathGoalHandle::SharedPtr>();
      }

      callback_group_executor_.spin_some();
      if (!goal_result_available_) {
        return BT::NodeStatus::RUNNING;
      }
    }
  } catch (const std::runtime_error & error) {
    if (error.what() == std::string("send_goal failed") ||
      error.what() == std::string("Goal was rejected by the action server"))
    {
      cancelOutstandingDispatch();
      future_goal_handle_.reset();
      startLateGoalAcknowledgementGuard();
      return BT::NodeStatus::FAILURE;
    }
    throw;
  }

  BT::NodeStatus result_status;
  switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      const auto verification = verifyPhysicalTerminalPose();
      if (verification == TerminalVerificationResult::kWaiting) {
        return BT::NodeStatus::RUNNING;
      }
      result_status = on_success();
      if (result_status == BT::NodeStatus::SUCCESS &&
        verification == TerminalVerificationResult::kFailed)
      {
        result_status = BT::NodeStatus::FAILURE;
      }
      break;
    }
    case rclcpp_action::ResultCode::ABORTED:
      result_status = on_aborted();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      result_status = on_cancelled();
      break;
    default:
      throw std::logic_error("RecordFollowPathAction::tick: invalid result code");
  }

  clearAcceptedGoalHandle();
  goal_handle_.reset();
  return result_status;
}

RecordFollowPathAction::TerminalVerificationResult
RecordFollowPathAction::verifyPhysicalTerminalPose()
{
  bool verify_terminal_pose = false;
  if (!getInput("verify_physical_terminal_pose", verify_terminal_pose) ||
    !verify_terminal_pose)
  {
    return TerminalVerificationResult::kPassed;
  }

  double position_tolerance_m = 0.0;
  double yaw_tolerance_rad = 0.0;
  int verification_delay_ms = 0;
  setOutput("terminal_recovery_eligible", false);
  if (!getInput("terminal_position_tolerance_m", position_tolerance_m) ||
    !getInput("terminal_yaw_tolerance_rad", yaw_tolerance_rad) ||
    !getInput("terminal_verification_delay_ms", verification_delay_ms) ||
    !std::isfinite(position_tolerance_m) || !std::isfinite(yaw_tolerance_rad) ||
    position_tolerance_m <= 0.0 || yaw_tolerance_rad <= 0.0 ||
    verification_delay_ms < 0 || verification_delay_ms > 2000)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Physical terminal-pose verification ports are invalid");
    return TerminalVerificationResult::kFailed;
  }

  if (!terminal_verification_pending_) {
    terminal_verification_pending_ = true;
    terminal_verification_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(verification_delay_ms);
  }
  if (std::chrono::steady_clock::now() < terminal_verification_deadline_) {
    return TerminalVerificationResult::kWaiting;
  }
  terminal_verification_pending_ = false;

  geometry_msgs::msg::PoseStamped target;
  if (!getInput("terminal_goal", target)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Physical terminal-pose verification has no terminal goal");
    return TerminalVerificationResult::kFailed;
  }
  if (target.header.frame_id.empty() || target.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Physical terminal-pose verification requires a %s-frame terminal goal",
      global_frame_.c_str());
    return TerminalVerificationResult::kFailed;
  }

  geometry_msgs::msg::PoseStamped current;
  if (!nav2_util::getCurrentPose(
      current, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Physical terminal-pose verification has no fresh robot pose");
    return TerminalVerificationResult::kFailed;
  }

  double current_yaw = 0.0;
  double target_yaw = 0.0;
  if (!quaternionYaw(current.pose.orientation, current_yaw) ||
    !quaternionYaw(target.pose.orientation, target_yaw) ||
    !std::isfinite(current.pose.position.x) || !std::isfinite(current.pose.position.y) ||
    !std::isfinite(target.pose.position.x) || !std::isfinite(target.pose.position.y))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Physical terminal-pose verification received an invalid pose");
    return TerminalVerificationResult::kFailed;
  }

  const double position_error = std::hypot(
    current.pose.position.x - target.pose.position.x,
    current.pose.position.y - target.pose.position.y);
  const double yaw_error = angularDistance(current_yaw, target_yaw);
  if (position_error <= position_tolerance_m && yaw_error <= yaw_tolerance_rad) {
    return TerminalVerificationResult::kPassed;
  }

  if (position_error > position_tolerance_m) {
    RCLCPP_WARN(
      node_->get_logger(),
      "FollowPath reported success outside the physical terminal position envelope "
      "(position=%.3f/%.3f m, yaw=%.3f/%.3f rad); clearing and replanning without retreat",
      position_error, position_tolerance_m, yaw_error, yaw_tolerance_rad);
    return TerminalVerificationResult::kFailed;
  }

  // The vehicle is still at the intended terminal position but misses its
  // locked heading. The enclosing reverse-only tree may safely retreat once,
  // clear both maps, and replan from the measured physical pose.
  setOutput("terminal_recovery_eligible", true);
  RCLCPP_WARN(
    node_->get_logger(),
    "FollowPath reported success outside the physical terminal envelope "
    "(position=%.3f/%.3f m, yaw=%.3f/%.3f rad)",
    position_error, position_tolerance_m, yaw_error, yaw_tolerance_rad);
  return TerminalVerificationResult::kFailed;
}

void RecordFollowPathAction::halt()
{
  // BtActionNode cannot cancel an action whose response has not arrived yet.
  // Marking this dispatch first makes a late acceptance fail closed in our
  // goal-response callback; an already accepted handle is cancelled below.
  reapLateGoalAcknowledgementGuard();
  terminal_verification_pending_ = false;
  terminal_verification_deadline_ = std::chrono::steady_clock::time_point();
  if (late_goal_acknowledgement_pending_.load()) {
    setStatus(BT::NodeStatus::IDLE);
    return;
  }

  const bool has_unacknowledged_goal = unacknowledged_goal_future_.valid();
  cancelOutstandingDispatch();
  nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>::halt();
  if (has_unacknowledged_goal) {
    startLateGoalAcknowledgementGuard();
  }
}

void RecordFollowPathAction::sendRecordedGoal()
{
  goal_result_available_ = false;
  const auto accepted_path = goal_.path;
  std::uint64_t generation = 0U;
  {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    generation = ++next_dispatch_generation_;
    active_dispatch_generation_ = generation;
  }
  auto send_goal_options = FollowPathClient::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, generation, accepted_path](FollowPathGoalHandle::SharedPtr goal_handle) {
      handleGoalResponse(generation, accepted_path, std::move(goal_handle));
    };
  send_goal_options.result_callback =
    [this](const FollowPathGoalHandle::WrappedResult & result) {
      if (future_goal_handle_ || !goal_handle_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Ignoring a result for %s before the active goal response is available",
          action_name_.c_str());
        return;
      }

      if (goal_handle_->get_goal_id() == result.goal_id) {
        goal_result_available_ = true;
        result_ = result;
      }
    };
  send_goal_options.feedback_callback =
    [this](FollowPathGoalHandle::SharedPtr,
      const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback) {
        feedback_ = feedback;
      };

  const auto goal_future = action_client_->async_send_goal(goal_, send_goal_options);
  future_goal_handle_ = std::make_shared<std::shared_future<FollowPathGoalHandle::SharedPtr>>(
    goal_future);
  unacknowledged_goal_future_ = goal_future;
  time_goal_sent_ = node_->now();
}

bool RecordFollowPathAction::isGoalHandleComplete(std::chrono::milliseconds & elapsed)
{
  const auto acknowledgement_timeout = goalAcknowledgementTimeout();
  const auto remaining = acknowledgement_timeout - elapsed;
  if (remaining <= std::chrono::milliseconds::zero()) {
    future_goal_handle_.reset();
    return false;
  }

  const auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
  const auto result = callback_group_executor_.spin_until_future_complete(
    *future_goal_handle_, timeout);
  elapsed += timeout;

  if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
    future_goal_handle_.reset();
    throw std::runtime_error("send_goal failed");
  }

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  goal_handle_ = future_goal_handle_->get();
  future_goal_handle_.reset();
  if (!goal_handle_) {
    throw std::runtime_error("Goal was rejected by the action server");
  }
  return true;
}

std::chrono::milliseconds RecordFollowPathAction::goalAcknowledgementTimeout() const
{
  // Controller server may cold-start after the planner has produced its first
  // path. A 100 ms global BT action timeout incorrectly turns that startup
  // latency into an unreachable-path recovery.
  return std::max(server_timeout_, kMinimumGoalAcknowledgementTimeout);
}

void RecordFollowPathAction::handleGoalResponse(
  const std::uint64_t generation,
  const nav_msgs::msg::Path & path,
  FollowPathGoalHandle::SharedPtr goal_handle)
{
  if (!goal_handle) {
    return;
  }

  bool cancel_goal = false;
  {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    cancel_goal = active_dispatch_generation_ != generation ||
      cancelled_dispatch_generation_ >= generation;
    if (!cancel_goal) {
      accepted_goal_handle_ = goal_handle;
      publishAcceptedPath(path);
    }
  }

  if (!cancel_goal) {
    return;
  }
  try {
    action_client_->async_cancel_goal(goal_handle);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Late RecordFollowPath goal could not be cancelled: %s", error.what());
  }
}

void RecordFollowPathAction::cancelOutstandingDispatch()
{
  FollowPathGoalHandle::SharedPtr accepted_goal_handle;
  {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    cancelled_dispatch_generation_ = std::max(
      cancelled_dispatch_generation_, active_dispatch_generation_);
    accepted_goal_handle = accepted_goal_handle_;
  }

  if (!accepted_goal_handle) {
    return;
  }
  try {
    action_client_->async_cancel_goal(accepted_goal_handle);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "RecordFollowPath goal cancellation failed: %s", error.what());
  }
}

void RecordFollowPathAction::startLateGoalAcknowledgementGuard()
{
  reapLateGoalAcknowledgementGuard();
  if (!unacknowledged_goal_future_.valid() ||
    late_goal_acknowledgement_pending_.exchange(true))
  {
    return;
  }

  const auto goal_future = unacknowledged_goal_future_;
  unacknowledged_goal_future_ = std::shared_future<FollowPathGoalHandle::SharedPtr>();
  stop_late_goal_acknowledgement_guard_.store(false);
  late_goal_acknowledgement_guard_ = std::thread(
    [this, goal_future]() {
      while (rclcpp::ok() && !stop_late_goal_acknowledgement_guard_.load()) {
        const auto result = callback_group_executor_.spin_until_future_complete(
          goal_future, kLateGoalAcknowledgementSpinPeriod);
        if (result == rclcpp::FutureReturnCode::SUCCESS ||
          (result == rclcpp::FutureReturnCode::INTERRUPTED &&
          stop_late_goal_acknowledgement_guard_.load()))
        {
          break;
        }
      }
      late_goal_acknowledgement_pending_.store(false);
    });
}

void RecordFollowPathAction::reapLateGoalAcknowledgementGuard()
{
  if (!late_goal_acknowledgement_pending_.load() &&
    late_goal_acknowledgement_guard_.joinable())
  {
    late_goal_acknowledgement_guard_.join();
  }
}

void RecordFollowPathAction::stopLateGoalAcknowledgementGuard()
{
  stop_late_goal_acknowledgement_guard_.store(true);
  callback_group_executor_.cancel();
  if (late_goal_acknowledgement_guard_.joinable()) {
    late_goal_acknowledgement_guard_.join();
  }
  late_goal_acknowledgement_pending_.store(false);
}

void RecordFollowPathAction::clearAcceptedGoalHandle()
{
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  accepted_goal_handle_.reset();
}

void RecordFollowPathAction::publishAcceptedPath(const nav_msgs::msg::Path & path) const
{
  accepted_path_publisher_->publish(path);
}

}  // namespace smartcar_nav2

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & configuration)
    {
      return std::make_unique<smartcar_nav2::RecordFollowPathAction>(
        name, "follow_path", configuration);
    };

  factory.registerBuilder<smartcar_nav2::RecordFollowPathAction>(
    "RecordFollowPath", builder);
  factory.registerNodeType<smartcar_nav2::ReverseRecoveryEligible>(
    "ReverseRecoveryEligible");
}
