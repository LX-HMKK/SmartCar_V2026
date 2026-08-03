#include "smartcar_nav2/odom_distance_budget_action.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace smartcar_nav2
{

OdomDistanceBudgetAction::OdomDistanceBudgetAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & configuration)
: BT::DecoratorNode(xml_tag_name, configuration)
{
  node_ = configuration.blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  std::string odom_topic{"/odom_combined"};
  if (!getInput("odom_topic", odom_topic) || odom_topic.empty()) {
    throw std::runtime_error("OdomDistanceBudget requires a nonempty odom_topic");
  }
  rclcpp::SubscriptionOptions odometry_options;
  odometry_options.callback_group = callback_group_;
  odometry_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::QoS(10).reliable(),
    [this](nav_msgs::msg::Odometry::SharedPtr odometry) {
      updateOdometry(std::move(odometry));
    }, odometry_options);
}

BT::PortsList OdomDistanceBudgetAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "NavigateToPose target used to derive the per-action budget"),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
      "goals", "NavigateThroughPoses targets; the terminal target sets the per-action budget"),
    BT::InputPort<std::string>(
      "odom_topic", "/odom_combined", "Measured odometry topic"),
    BT::InputPort<std::string>(
      "odom_frame", "odom_combined", "Expected odometry and goal frame"),
    BT::InputPort<std::string>(
      "robot_base_frame", "base_footprint", "Expected odometry child frame"),
    BT::InputPort<double>(
      "max_distance_ratio", 2.0,
      "Maximum measured travel as a multiple of the initial goal chord"),
    BT::InputPort<double>(
      "distance_slack_m", 0.80,
      "Absolute measured-travel allowance added to the initial goal chord"),
    BT::InputPort<double>(
      "max_odom_step_m", 0.50,
      "Reject one implausibly large odometry step instead of continuing blind"),
    BT::InputPort<double>(
      "odom_timeout_sec", 0.50,
      "Fail closed when the measured odometry stream becomes stale"),
    BT::InputPort<double>(
      "initial_odom_wait_sec", 1.0,
      "Startup-only fail-closed wait for the first fresh measured odometry sample"),
  };
}

BT::NodeStatus OdomDistanceBudgetAction::tick()
{
  // This callback group is intentionally not attached to bt_navigator's
  // executor. Drain its odometry work before checking the initial freshness
  // gate or the active travel budget.
  callback_group_executor_.spin_some();

  if (status() == BT::NodeStatus::IDLE || awaiting_initial_odom_) {
    const ArmResult arm_result = beginNavigation();
    if (arm_result == ArmResult::kFailure) {
      return BT::NodeStatus::FAILURE;
    }
    if (arm_result == ArmResult::kWaitingForOdom) {
      // The child is deliberately not ticked until a fresh measured pose
      // exists, so planner/controller actions cannot begin without the fuse.
      setStatus(BT::NodeStatus::RUNNING);
      return BT::NodeStatus::RUNNING;
    }
  }

  setStatus(BT::NodeStatus::RUNNING);

  std::string reason;
  double travelled_m = 0.0;
  double budget_m = 0.0;
  if (hasAbortCondition(reason, travelled_m, budget_m)) {
    abortNavigation(reason, travelled_m, budget_m);
    return BT::NodeStatus::FAILURE;
  }

  const BT::NodeStatus child_status = child_node_->executeTick();
  if (child_status != BT::NodeStatus::RUNNING) {
    stopMonitoring();
    return child_status;
  }

  if (hasAbortCondition(reason, travelled_m, budget_m)) {
    abortNavigation(reason, travelled_m, budget_m);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void OdomDistanceBudgetAction::halt()
{
  stopMonitoring();
  BT::DecoratorNode::halt();
}

OdomDistanceBudgetAction::ArmResult OdomDistanceBudgetAction::beginNavigation()
{
  if (!awaiting_initial_odom_) {
    if (!readConfiguration()) {
      RCLCPP_ERROR(node_->get_logger(), "OdomDistanceBudget ports are invalid");
      return ArmResult::kFailure;
    }
    if (!readGoalPoses()) {
      return ArmResult::kFailure;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      monitoring_ = false;
      odom_fault_ = false;
      odom_fault_reason_.clear();
      tracker_.reset();
    }
    awaiting_initial_odom_ = true;
    initial_odom_deadline_ =
      std::chrono::steady_clock::now() + initial_odom_wait_timeout_;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  const bool has_fresh_matching_sample = latest_odom_.has_value() &&
    sampleMatchesFrames(*latest_odom_) &&
    OdomDistanceBudgetTracker::isFinite(latest_odom_->pose) &&
    now - latest_odom_->received_at <= odom_timeout_;
  if (!has_fresh_matching_sample) {
    if (now < initial_odom_deadline_) {
      return ArmResult::kWaitingForOdom;
    }
    awaiting_initial_odom_ = false;
    RCLCPP_ERROR(
      node_->get_logger(),
      "OdomDistanceBudget did not receive fresh valid odometry within %.3f s",
      initial_odom_wait_timeout_.count() / 1000.0);
    return ArmResult::kFailure;
  }
  if (!tracker_.initialize(
      latest_odom_->pose, goal_poses_, max_distance_ratio_, distance_slack_m_,
      max_odom_step_m_))
  {
    awaiting_initial_odom_ = false;
    RCLCPP_ERROR(node_->get_logger(), "OdomDistanceBudget could not initialize travel budget");
    return ArmResult::kFailure;
  }
  monitoring_ = true;
  awaiting_initial_odom_ = false;
  RCLCPP_INFO(
    node_->get_logger(),
    "OdomDistanceBudget armed: %.3f m measured-travel limit",
    tracker_.budget_m());
  return ArmResult::kReady;
}

bool OdomDistanceBudgetAction::readConfiguration()
{
  double odom_timeout_sec = 0.0;
  double initial_odom_wait_sec = 0.0;
  if (!getInput("odom_frame", odom_frame_) || odom_frame_.empty() ||
    !getInput("robot_base_frame", robot_base_frame_) || robot_base_frame_.empty() ||
    !getInput("max_distance_ratio", max_distance_ratio_) ||
    !getInput("distance_slack_m", distance_slack_m_) ||
    !getInput("max_odom_step_m", max_odom_step_m_) ||
    !getInput("odom_timeout_sec", odom_timeout_sec) ||
    !std::isfinite(max_distance_ratio_) || max_distance_ratio_ < 1.0 ||
    !std::isfinite(distance_slack_m_) || distance_slack_m_ < 0.0 ||
    !std::isfinite(max_odom_step_m_) || max_odom_step_m_ <= 0.0 ||
    !std::isfinite(odom_timeout_sec) || odom_timeout_sec <= 0.0 ||
    !getInput("initial_odom_wait_sec", initial_odom_wait_sec) ||
    !std::isfinite(initial_odom_wait_sec) || initial_odom_wait_sec <= 0.0)
  {
    return false;
  }
  odom_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(odom_timeout_sec));
  initial_odom_wait_timeout_ =
    std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(initial_odom_wait_sec));
  return odom_timeout_.count() > 0 && initial_odom_wait_timeout_.count() > 0;
}

bool OdomDistanceBudgetAction::readGoalPoses()
{
  geometry_msgs::msg::PoseStamped goal;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  const bool has_goal = static_cast<bool>(getInput("goal", goal));
  const bool has_goals = static_cast<bool>(getInput("goals", goals));
  if (has_goal == has_goals) {
    RCLCPP_ERROR(
      node_->get_logger(), "OdomDistanceBudget requires exactly one of goal or goals");
    return false;
  }
  if (has_goals && goals.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "OdomDistanceBudget received an empty goals list");
    return false;
  }

  const auto append_goal = [this](
    const geometry_msgs::msg::PoseStamped & candidate,
    std::vector<OdomPlanarPose> & poses) {
      if (candidate.header.frame_id != odom_frame_) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "OdomDistanceBudget goal frame '%s' does not match odom frame '%s'",
          candidate.header.frame_id.c_str(), odom_frame_.c_str());
        return false;
      }
      if (!std::isfinite(candidate.pose.position.x) ||
        !std::isfinite(candidate.pose.position.y))
      {
        RCLCPP_ERROR(node_->get_logger(), "OdomDistanceBudget has a non-finite goal pose");
        return false;
      }
      poses.push_back(OdomPlanarPose{
        candidate.pose.position.x,
        candidate.pose.position.y,
      });
      return true;
    };

  std::vector<OdomPlanarPose> selected_goals;
  if (has_goal) {
    if (!append_goal(goal, selected_goals)) {
      return false;
    }
  } else {
    selected_goals.reserve(goals.size());
    for (const auto & candidate : goals) {
      if (!append_goal(candidate, selected_goals)) {
        return false;
      }
    }
  }
  goal_poses_ = std::move(selected_goals);
  return true;
}

bool OdomDistanceBudgetAction::hasAbortCondition(
  std::string & reason,
  double & travelled_m,
  double & budget_m)
{
  std::lock_guard<std::mutex> lock(mutex_);
  travelled_m = tracker_.travelled_m();
  budget_m = tracker_.budget_m();
  if (!monitoring_) {
    reason = "monitoring_not_active";
    return true;
  }
  if (odom_fault_) {
    reason = odom_fault_reason_;
    return true;
  }
  if (!latest_odom_.has_value() ||
    std::chrono::steady_clock::now() - latest_odom_->received_at > odom_timeout_)
  {
    reason = "odom_stale";
    return true;
  }
  if (tracker_.travelled_m() >= tracker_.budget_m()) {
    reason = "travel_budget_exceeded";
    return true;
  }
  return false;
}

void OdomDistanceBudgetAction::stopMonitoring()
{
  std::lock_guard<std::mutex> lock(mutex_);
  monitoring_ = false;
  awaiting_initial_odom_ = false;
}

void OdomDistanceBudgetAction::abortNavigation(
  const std::string & reason,
  double travelled_m,
  double budget_m)
{
  RCLCPP_ERROR(
    node_->get_logger(),
    "OdomDistanceBudget aborting navigation (%s): travelled %.3f m / budget %.3f m",
    reason.c_str(), travelled_m, budget_m);
  // FollowPath is below this decorator. Halting it issues Nav2's action
  // cancellation path before this decorator reports a terminal FAILURE.
  haltChild();
  stopMonitoring();
}

void OdomDistanceBudgetAction::updateOdometry(nav_msgs::msg::Odometry::SharedPtr odometry)
{
  if (!odometry) {
    return;
  }

  const OdomSample sample{
    OdomPlanarPose{
      odometry->pose.pose.position.x,
      odometry->pose.pose.position.y,
    },
    odometry->header.frame_id,
    odometry->child_frame_id,
    std::chrono::steady_clock::now(),
  };

  bool notify_tree = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = sample;
    if (!monitoring_) {
      return;
    }
    if (!sampleMatchesFrames(sample)) {
      odom_fault_ = true;
      odom_fault_reason_ = "odom_frame_mismatch";
      notify_tree = true;
    } else if (!OdomDistanceBudgetTracker::isFinite(sample.pose)) {
      odom_fault_ = true;
      odom_fault_reason_ = "odom_nonfinite";
      notify_tree = true;
    } else {
      switch (tracker_.update(sample.pose)) {
        case OdomDistanceBudgetUpdate::kAccepted:
          break;
        case OdomDistanceBudgetUpdate::kBudgetExceeded:
          odom_fault_ = true;
          odom_fault_reason_ = "travel_budget_exceeded";
          notify_tree = true;
          break;
        case OdomDistanceBudgetUpdate::kStepTooLarge:
          odom_fault_ = true;
          odom_fault_reason_ = "odom_step_too_large";
          notify_tree = true;
          break;
        case OdomDistanceBudgetUpdate::kInvalidSample:
        default:
          odom_fault_ = true;
          odom_fault_reason_ = "odom_invalid";
          notify_tree = true;
          break;
      }
    }
  }
  if (notify_tree) {
    emitStateChanged();
  }
}

bool OdomDistanceBudgetAction::sampleMatchesFrames(const OdomSample & sample) const
{
  return sample.frame_id == odom_frame_ &&
         sample.child_frame_id == robot_base_frame_;
}

}  // namespace smartcar_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<smartcar_nav2::OdomDistanceBudgetAction>(
    "OdomDistanceBudget");
}
