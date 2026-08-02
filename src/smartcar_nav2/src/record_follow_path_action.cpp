#include "smartcar_nav2/record_follow_path_action.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace smartcar_nav2
{
namespace
{

constexpr char kAcceptedGlobalPlanTopic[] = "/smartcar/accepted_global_plan";
constexpr std::chrono::milliseconds kMinimumGoalAcknowledgementTimeout{500};
constexpr std::chrono::milliseconds kLateGoalAcknowledgementSpinPeriod{50};

}  // namespace

RecordFollowPathAction::RecordFollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & configuration)
: nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>(
    xml_tag_name, action_name, configuration)
{
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
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_status = on_success();
      break;
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

void RecordFollowPathAction::halt()
{
  // BtActionNode cannot cancel an action whose response has not arrived yet.
  // Marking this dispatch first makes a late acceptance fail closed in our
  // goal-response callback; an already accepted handle is cancelled below.
  reapLateGoalAcknowledgementGuard();
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
}
