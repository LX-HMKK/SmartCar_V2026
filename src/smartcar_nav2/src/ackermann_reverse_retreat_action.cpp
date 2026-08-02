#include "smartcar_nav2/ackermann_reverse_retreat_action.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"

#include "smartcar_nav2/ackermann_reverse_retreat_path.hpp"
#include "smartcar_nav2/costmap_sample_guard.hpp"
#include "smartcar_nav2/footprint_sweep_collision_source.hpp"

namespace smartcar_nav2
{

AckermannReverseRetreatAction::AckermannReverseRetreatAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & configuration)
: BT::StatefulActionNode(xml_tag_name, configuration), action_name_(action_name)
{
  node_ = configuration.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = configuration.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_base_frame", robot_base_frame_);
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  // FollowPath intentionally stays on bt_navigator's default callback group.
  // If this BT node is halted while its goal response is in flight, that group
  // continues to run the response callback and immediately cancels a late
  // acceptance. The perception subscriptions remain private and are advanced
  // explicitly once per BT tick below.
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(node_, action_name_);
  accepted_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/smartcar/accepted_global_plan", rclcpp::QoS(1).reliable().transient_local());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  global_costmap_subscription_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr costmap) {
      updateGlobalCostmap(std::move(costmap));
    }, options);
  local_costmap_subscription_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap_raw", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr costmap) {
      updateLocalCostmap(std::move(costmap));
    }, options);
  scan_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS().keep_last(5),
    [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
      updateScan(std::move(scan));
    }, options);
  odometry_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_combined", rclcpp::QoS(10).reliable(),
    [this](nav_msgs::msg::Odometry::SharedPtr odometry) {
      updateOdometry(std::move(odometry));
    }, options);
}

BT::NodeStatus AckermannReverseRetreatAction::onStart()
{
  callback_group_executor_.spin_some();
  resetOperation();
  clearPathOutput();

  if (!loadInputs()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!follow_path_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath action server is unavailable");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat has no fresh robot pose");
    return BT::NodeStatus::FAILURE;
  }
  if (!buildAckermannReverseRetreatPath(
      current_pose, global_frame_, retreat_distance_m_, retreat_path_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat path inputs are invalid");
    return BT::NodeStatus::FAILURE;
  }

  // The enclosing BT has just completed both ClearEntireCostmap services.
  // From here onward every accepted raw costmap must be newer than this
  // barrier, and its associated scan must be newer too.
  armCostmapBarrier();
  perception_deadline_ = std::chrono::steady_clock::now() + perception_wait_timeout_;
  state_ = State::WAITING_FOR_PERCEPTION;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AckermannReverseRetreatAction::onRunning()
{
  callback_group_executor_.spin_some();
  switch (state_) {
    case State::WAITING_FOR_PERCEPTION:
      return waitForPerception();
    case State::WAITING_FOR_GOAL_HANDLE:
      return waitForGoalHandle();
    case State::WAITING_FOR_RESULT:
      return waitForResult();
    case State::IDLE:
      RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat has no active state");
      return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

void AckermannReverseRetreatAction::onHalted()
{
  clearPathOutput();
  cancelFollowPath();
  resetOperation();
}

void AckermannReverseRetreatAction::clearPathOutput()
{
  setOutput("path", nav_msgs::msg::Path());
}

void AckermannReverseRetreatAction::resetOperation()
{
  const auto completed_generation = active_dispatch_generation_;
  {
    std::lock_guard<std::mutex> lock(goal_response_handle_mutex_);
    if (goal_response_handle_.generation == completed_generation) {
      goal_response_handle_ = GoalResponseHandle();
    }
  }
  state_ = State::IDLE;
  retreat_path_ = nav_msgs::msg::Path();
  goal_ = FollowPath::Goal();
  goal_handle_future_ = std::shared_future<FollowPathGoalHandle::SharedPtr>();
  result_future_ = std::shared_future<FollowPathGoalHandle::WrappedResult>();
  goal_handle_.reset();
  perception_deadline_ = std::chrono::steady_clock::time_point();
  goal_handle_deadline_ = std::chrono::steady_clock::time_point();
  result_deadline_ = std::chrono::steady_clock::time_point();
  retreat_odom_guard_.reset();
  active_dispatch_generation_ = 0U;
}

bool AckermannReverseRetreatAction::loadInputs()
{
  bool allow_retreat = false;
  bool retreat_used = false;
  double retreat_distance_m = 0.0;
  int costmap_max_age_ms = 0;
  int perception_wait_timeout_ms = 0;
  int follow_path_goal_timeout_ms = 0;
  int follow_path_result_timeout_ms = 0;
  int scan_costmap_fusion_lag_ms = 0;
  std::int64_t costmap_min_stamp_ns = 0;
  std::int64_t local_costmap_min_stamp_ns = 0;
  std::string static_keepout_mask_topic;
  if (!getInput("allow_retreat", allow_retreat) || !allow_retreat) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Refusing Ackermann reverse retreat because the planner failure is not recoverable");
    return false;
  }
  if (!getInput("retreat_used", retreat_used) || retreat_used) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Refusing a second Ackermann reverse retreat in the same navigation action");
    return false;
  }
  if (!getInput("retreat_distance_m", retreat_distance_m) ||
    !getInput("costmap_max_age_ms", costmap_max_age_ms) ||
    !getInput("perception_wait_timeout_ms", perception_wait_timeout_ms) ||
    !getInput("follow_path_goal_timeout_ms", follow_path_goal_timeout_ms) ||
    !getInput("follow_path_result_timeout_ms", follow_path_result_timeout_ms) ||
    !getInput("scan_costmap_fusion_lag_ms", scan_costmap_fusion_lag_ms) ||
    !getInput("costmap_min_stamp_ns", costmap_min_stamp_ns) ||
    !getInput("local_costmap_min_stamp_ns", local_costmap_min_stamp_ns) ||
    !getInput("static_keepout_mask_topic", static_keepout_mask_topic) ||
    !std::isfinite(retreat_distance_m) || retreat_distance_m <= 0.0 ||
    costmap_min_stamp_ns <= 0 || local_costmap_min_stamp_ns <= 0 ||
    costmap_max_age_ms < 100 || costmap_max_age_ms > 2000 ||
    perception_wait_timeout_ms < 2500 || perception_wait_timeout_ms > 5000 ||
    follow_path_goal_timeout_ms < 100 || follow_path_goal_timeout_ms > 2000 ||
    follow_path_result_timeout_ms < 1000 || follow_path_result_timeout_ms > 15000 ||
    scan_costmap_fusion_lag_ms < 100 || scan_costmap_fusion_lag_ms > costmap_max_age_ms ||
    !readSweepOptions() || !readScanWitnessOptions() || !readOdomOptions(retreat_distance_m))
  {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat ports are invalid");
    return false;
  }

  costmap_max_age_ = std::chrono::milliseconds(costmap_max_age_ms);
  perception_wait_timeout_ = std::chrono::milliseconds(perception_wait_timeout_ms);
  follow_path_goal_timeout_ = std::chrono::milliseconds(follow_path_goal_timeout_ms);
  follow_path_result_timeout_ = std::chrono::milliseconds(follow_path_result_timeout_ms);
  scan_costmap_fusion_lag_ = std::chrono::milliseconds(scan_costmap_fusion_lag_ms);
  retreat_distance_m_ = retreat_distance_m;
  costmap_min_stamp_ns_ = costmap_min_stamp_ns;
  local_costmap_min_stamp_ns_ = local_costmap_min_stamp_ns;
  if (!configureKeepoutMaskSubscription(static_keepout_mask_topic)) {
    return false;
  }
  std::string controller_id;
  std::string goal_checker_id;
  if (!getInput("controller_id", controller_id) ||
    !getInput("goal_checker_id", goal_checker_id) ||
    controller_id != "ReverseRecovery" ||
    goal_checker_id != "recovery_goal_checker")
  {
    // This node is the only physical recovery motion allowed by the reverse
    // trees.  Do not let a BT wiring regression substitute a controller that
    // can issue a positive, curved, or tolerance-shortened command.
    RCLCPP_ERROR(
      node_->get_logger(),
      "Ackermann reverse retreat requires ReverseRecovery and recovery_goal_checker");
    return false;
  }
  goal_.controller_id = controller_id;
  goal_.goal_checker_id = goal_checker_id;
  return true;
}

bool AckermannReverseRetreatAction::readSweepOptions()
{
  int lethal_cost = 0;
  if (!getInput("footprint_half_length_m", footprint_sweep_options_.half_length_m) ||
    !getInput("footprint_half_width_m", footprint_sweep_options_.half_width_m) ||
    !getInput("footprint_sweep_step_m", footprint_sweep_options_.sample_spacing_m) ||
    !getInput("footprint_lethal_cost", lethal_cost) ||
    !std::isfinite(footprint_sweep_options_.half_length_m) ||
    !std::isfinite(footprint_sweep_options_.half_width_m) ||
    !std::isfinite(footprint_sweep_options_.sample_spacing_m) ||
    footprint_sweep_options_.half_length_m <= 0.0 ||
    footprint_sweep_options_.half_width_m <= 0.0 ||
    footprint_sweep_options_.sample_spacing_m <= 0.0 ||
    lethal_cost < 1 || lethal_cost > 253)
  {
    return false;
  }
  footprint_sweep_options_.lethal_cost_threshold = static_cast<std::uint8_t>(lethal_cost);
  return true;
}

bool AckermannReverseRetreatAction::readScanWitnessOptions()
{
  if (!getInput("scan_min_obstacle_range_m", scan_min_obstacle_range_m_) ||
    !getInput("scan_max_obstacle_range_m", scan_max_obstacle_range_m_) ||
    !getInput("scan_costmap_match_radius_m", scan_costmap_match_radius_m_) ||
    !std::isfinite(scan_min_obstacle_range_m_) ||
    !std::isfinite(scan_max_obstacle_range_m_) ||
    !std::isfinite(scan_costmap_match_radius_m_) ||
    scan_min_obstacle_range_m_ < 0.0 ||
    scan_max_obstacle_range_m_ <= scan_min_obstacle_range_m_ ||
    scan_costmap_match_radius_m_ <= 0.0 || scan_costmap_match_radius_m_ > 0.50)
  {
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::readOdomOptions(double retreat_distance_m)
{
  int odom_max_age_ms = 0;
  if (!getInput("retreat_odom_max_age_ms", odom_max_age_ms) ||
    !getInput("retreat_odom_max_step_m", retreat_odom_limits_.maximum_step_m) ||
    !getInput("retreat_odom_max_travel_m", retreat_odom_limits_.maximum_travel_m) ||
    !getInput(
      "retreat_odom_max_displacement_m", retreat_odom_limits_.maximum_displacement_m) ||
    odom_max_age_ms < 100 || odom_max_age_ms > 1000 ||
    !std::isfinite(retreat_odom_limits_.maximum_step_m) ||
    retreat_odom_limits_.maximum_step_m <= 0.0 || retreat_odom_limits_.maximum_step_m > 0.10 ||
    !std::isfinite(retreat_odom_limits_.maximum_travel_m) ||
    retreat_odom_limits_.maximum_travel_m < retreat_distance_m ||
    retreat_odom_limits_.maximum_travel_m > retreat_distance_m + 0.05 ||
    !std::isfinite(retreat_odom_limits_.maximum_displacement_m) ||
    retreat_odom_limits_.maximum_displacement_m < retreat_distance_m ||
    retreat_odom_limits_.maximum_displacement_m > retreat_distance_m + 0.05)
  {
    return false;
  }
  retreat_odom_limits_.maximum_age = std::chrono::milliseconds(odom_max_age_ms);
  return true;
}

bool AckermannReverseRetreatAction::configureKeepoutMaskSubscription(const std::string & topic)
{
  if (topic == static_keepout_mask_topic_) {
    return true;
  }
  static_keepout_mask_subscription_.reset();
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    static_keepout_mask_.reset();
  }
  static_keepout_mask_topic_ = topic;
  if (topic.empty()) {
    // Real navigation intentionally has no static localization map. Its
    // recovery proof remains the fresh raw global/local costmaps, their
    // associated scan endpoints at scan-time TF, and the complete footprint
    // sweep. A configured mask is additional, not a substitute for that
    // evidence.
    return true;
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  try {
    static_keepout_mask_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr mask) {
        updateStaticKeepoutMask(std::move(mask));
      }, options);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat cannot subscribe to keepout mask '%s': %s",
      topic.c_str(), error.what());
    return false;
  }
  return true;
}

void AckermannReverseRetreatAction::armCostmapBarrier()
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  // Capture the observations present after the BT ClearEntireCostmap calls.
  // A valid retreat needs strictly later raw local/global maps and a strictly
  // later scan, so a transient-local replay or a freshly emptied map cannot
  // authorize physical motion.
  global_costmap_barrier_sequence_ = global_costmap_sequence_;
  local_costmap_barrier_sequence_ = local_costmap_sequence_;
  scan_barrier_sequence_ = scan_.sequence;
  scan_barrier_stamp_ns_ = scan_.stamp_ns;
  post_clear_ros_stamp_ns_ = node_->get_clock()->now().nanoseconds();
}

bool AckermannReverseRetreatAction::odomIsFresh(
  OdomSample & sample, std::string & reason) const
{
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!latest_odom_.has_value()) {
      reason = "odometry missing";
      return false;
    }
    sample = *latest_odom_;
  }
  if (sample.frame_id != global_frame_ || sample.child_frame_id != robot_base_frame_) {
    reason = "odometry frame mismatch";
    return false;
  }
  if (sample.stamp_ns <= 0) {
    reason = "odometry source stamp is invalid";
    return false;
  }
  const auto ros_now = node_->get_clock()->now();
  if (ros_now.nanoseconds() <= 0) {
    reason = "ROS clock unavailable for odometry freshness";
    return false;
  }
  constexpr std::int64_t kFutureStampToleranceNs = 50000000LL;
  const auto source_age_ns = ros_now.nanoseconds() - sample.stamp_ns;
  const auto maximum_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    retreat_odom_limits_.maximum_age).count();
  const auto steady_now = std::chrono::steady_clock::now();
  if (source_age_ns < -kFutureStampToleranceNs || source_age_ns > maximum_age_ns ||
    steady_now < sample.sample.received_at ||
    steady_now - sample.sample.received_at > retreat_odom_limits_.maximum_age)
  {
    reason = "odometry is stale or from the future";
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::armRetreatOdomGuard(std::string & reason)
{
  OdomSample sample;
  if (!odomIsFresh(sample, reason)) {
    return false;
  }
  if (!retreat_odom_guard_.arm(
      sample.sample, retreat_odom_limits_, std::chrono::steady_clock::now()))
  {
    reason = "odometry guard could not arm";
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::dispatchedRetreatIsSafe(std::string & reason)
{
  if (!retreatPathIsClear(retreat_path_, reason)) {
    return false;
  }
  OdomSample sample;
  if (!odomIsFresh(sample, reason)) {
    return false;
  }
  const auto odom_result = retreat_odom_guard_.observe(
    sample.sample, std::chrono::steady_clock::now());
  if (odom_result != AckermannReverseRetreatOdomResult::kClear) {
    reason = std::string("retreat odometry ") +
      ackermannReverseRetreatOdomResultName(odom_result);
    return false;
  }
  return true;
}

BT::NodeStatus AckermannReverseRetreatAction::waitForPerception()
{
  std::string reason;
  if (retreatPathIsClear(retreat_path_, reason)) {
    OdomSample odom_sample;
    if (odomIsFresh(odom_sample, reason)) {
      return dispatchRetreat();
    }
  }
  if (std::chrono::steady_clock::now() >= perception_deadline_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ackermann reverse retreat denied after %ld ms waiting for post-clear safety evidence: %s",
      static_cast<long>(perception_wait_timeout_.count()), reason.c_str());
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AckermannReverseRetreatAction::dispatchRetreat()
{
  if (!follow_path_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath server disappeared before dispatch");
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  std::string reason;
  if (!armRetreatOdomGuard(reason)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat has no safe odometry baseline: %s",
      reason.c_str());
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  // Latch before dispatch. An action-server abort can occur after physical
  // motion has begun, so it must not permit a second reverse attempt.
  setOutput("retreat_used", true);
  goal_.path = retreat_path_;
  setOutput("path", retreat_path_);
  active_dispatch_generation_ = next_dispatch_generation_.fetch_add(1U) + 1U;
  const auto generation = active_dispatch_generation_;
  const auto accepted_path = retreat_path_;
  auto options = FollowPathClient::SendGoalOptions();
  options.goal_response_callback =
    [this, generation, accepted_path](FollowPathGoalHandle::SharedPtr goal_handle) {
      if (!goal_handle) {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(goal_response_handle_mutex_);
        // A delayed callback for an older request must never replace the
        // acknowledged handle for a newer physical-recovery dispatch.
        if (generation >= goal_response_handle_.generation) {
          goal_response_handle_.generation = generation;
          goal_response_handle_.handle = goal_handle;
        }
      }
      if (cancelled_dispatch_generation_.load() < generation) {
        accepted_path_publisher_->publish(accepted_path);
        return;
      }
      try {
        follow_path_client_->async_cancel_goal(goal_handle);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Late Ackermann reverse retreat goal could not be cancelled: %s", error.what());
      }
    };
  try {
    goal_handle_future_ = follow_path_client_->async_send_goal(goal_, options);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath dispatch failed: %s", error.what());
    // async_send_goal() may fail after handing the request to the middleware.
    // Preserve the cancellation generation so a late acceptance cannot move
    // the robot after this BT branch has failed.
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  goal_handle_deadline_ = std::chrono::steady_clock::now() + follow_path_goal_timeout_;
  state_ = State::WAITING_FOR_GOAL_HANDLE;
  RCLCPP_WARN(
    node_->get_logger(),
    "Reverse planner exhausted its candidates; retreating %.3f m before one replan",
    retreat_distance_m_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AckermannReverseRetreatAction::waitForGoalHandle()
{
  std::string safety_reason;
  if (!dispatchedRetreatIsSafe(safety_reason)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Ackermann reverse retreat cancelled while awaiting FollowPath acceptance: %s",
      safety_reason.c_str());
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  if (!goal_handle_future_.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat goal future is invalid");
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= goal_handle_deadline_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Ackermann reverse retreat FollowPath goal acknowledgement timed out");
      // Mark this dispatch as cancelled before returning FAILURE. If the
      // server accepts it after this deadline, goal_response_callback()
      // observes the generation and cancels it before it can move the robot.
      cancelFollowPath();
      clearPathOutput();
      state_ = State::IDLE;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  try {
    goal_handle_ = goal_handle_future_.get();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath goal transport failed: %s", error.what());
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  goal_handle_future_ = std::shared_future<FollowPathGoalHandle::SharedPtr>();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat FollowPath goal was rejected");
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  if (cancelled_dispatch_generation_.load() >= active_dispatch_generation_) {
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  try {
    result_future_ = follow_path_client_->async_get_result(goal_handle_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath result transport failed: %s",
      error.what());
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  result_deadline_ = std::chrono::steady_clock::now() + follow_path_result_timeout_;
  state_ = State::WAITING_FOR_RESULT;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AckermannReverseRetreatAction::waitForResult()
{
  std::string safety_reason;
  if (!dispatchedRetreatIsSafe(safety_reason)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Ackermann reverse retreat cancelled during FollowPath execution: %s",
      safety_reason.c_str());
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  if (!result_future_.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat result future is invalid");
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  if (result_deadline_ == std::chrono::steady_clock::time_point() ||
    std::chrono::steady_clock::now() >= result_deadline_)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath result timed out");
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  FollowPathGoalHandle::WrappedResult result;
  try {
    result = result_future_.get();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath result failed: %s", error.what());
    cancelFollowPath();
    clearPathOutput();
    state_ = State::IDLE;
    goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }
  result_future_ = std::shared_future<FollowPathGoalHandle::WrappedResult>();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    goal_handle_.reset();
    clearPathOutput();
    state_ = State::IDLE;
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(
    node_->get_logger(), "Ackermann reverse retreat FollowPath ended with result code %d",
    static_cast<int>(result.code));
  // The server reports terminal here, but preserve the cancellation generation
  // and issue a best-effort cancel before exposing FAILURE.  This keeps every
  // non-success exit fail-closed even if a transport implementation delivers
  // an inconsistent late state transition.
  cancelFollowPath();
  goal_handle_.reset();
  clearPathOutput();
  state_ = State::IDLE;
  return BT::NodeStatus::FAILURE;
}

void AckermannReverseRetreatAction::cancelFollowPath()
{
  if (active_dispatch_generation_ != 0U) {
    auto cancelled_generation = cancelled_dispatch_generation_.load();
    while (cancelled_generation < active_dispatch_generation_ &&
      !cancelled_dispatch_generation_.compare_exchange_weak(
        cancelled_generation, active_dispatch_generation_))
    {
    }
  }
  FollowPathGoalHandle::SharedPtr handle_to_cancel = goal_handle_;
  if (!handle_to_cancel && active_dispatch_generation_ != 0U) {
    std::lock_guard<std::mutex> lock(goal_response_handle_mutex_);
    if (goal_response_handle_.generation == active_dispatch_generation_) {
      handle_to_cancel = goal_response_handle_.handle;
    }
  }
  if (!handle_to_cancel) {
    return;
  }
  try {
    follow_path_client_->async_cancel_goal(handle_to_cancel);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Ackermann reverse retreat FollowPath cancellation failed: %s", error.what());
  }
}

bool AckermannReverseRetreatAction::scanIsFresh(
  const ScanSample & sample, std::string & reason) const
{
  if (!sample.scan || sample.received_at == std::chrono::steady_clock::time_point() ||
    sample.sequence == 0U || sample.stamp_ns <= 0 || sample.scan->header.frame_id.empty())
  {
    reason = "scan missing or malformed";
    return false;
  }
  if (sample.sequence <= scan_barrier_sequence_ ||
    (scan_barrier_stamp_ns_ > 0 && sample.stamp_ns <= scan_barrier_stamp_ns_) ||
    post_clear_ros_stamp_ns_ <= 0 || sample.stamp_ns <= post_clear_ros_stamp_ns_)
  {
    reason = "scan predates post-clear barrier";
    return false;
  }
  const auto ros_now = node_->get_clock()->now();
  if (ros_now.nanoseconds() <= 0) {
    reason = "ROS clock unavailable for scan freshness";
    return false;
  }
  constexpr std::int64_t kFutureStampToleranceNs = 50000000LL;
  const auto scan_age_ns = ros_now.nanoseconds() - sample.stamp_ns;
  const auto maximum_age_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(costmap_max_age_).count();
  if (scan_age_ns < -kFutureStampToleranceNs || scan_age_ns > maximum_age_ns ||
    std::chrono::steady_clock::now() < sample.received_at ||
    std::chrono::steady_clock::now() - sample.received_at > costmap_max_age_)
  {
    reason = "scan is stale or from the future";
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::scanCouldHaveBeenFused(
  const CostmapSample & sample, const ScanSample & scan, std::string & reason) const
{
  const auto association = costmapScanAssociationFreshness(sample, scan_costmap_fusion_lag_);
  if (association != CostmapScanAssociationFreshness::kFresh) {
    reason = std::string("scan/costmap association ") +
      costmapScanAssociationFreshnessName(association);
    return false;
  }
  if (sample.scan_sequence != scan.sequence || sample.scan_stamp_ns != scan.stamp_ns) {
    reason = "scan/costmap association changed while evaluating";
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::scanPointsInGlobalFrame(
  const ScanSample & sample,
  std::vector<std::pair<double, double>> & points,
  std::string & reason) const
{
  points.clear();
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      global_frame_, sample.scan->header.frame_id, rclcpp::Time(sample.scan->header.stamp));
  } catch (const tf2::TransformException & error) {
    reason = std::string("scan TF unavailable: ") + error.what();
    return false;
  }
  const auto & rotation = transform.transform.rotation;
  const double sin_yaw = 2.0 * (
    rotation.w * rotation.z + rotation.x * rotation.y);
  const double cos_yaw = 1.0 - 2.0 * (
    rotation.y * rotation.y + rotation.z * rotation.z);
  const double lower = std::max(
    scan_min_obstacle_range_m_, static_cast<double>(sample.scan->range_min));
  const double upper = std::min(
    scan_max_obstacle_range_m_, static_cast<double>(sample.scan->range_max));
  if (!std::isfinite(lower) || !std::isfinite(upper) || upper <= lower) {
    reason = "scan range limits are invalid";
    return false;
  }
  points.reserve(sample.scan->ranges.size());
  for (std::size_t index = 0; index < sample.scan->ranges.size(); ++index) {
    const double range = static_cast<double>(sample.scan->ranges[index]);
    if (!std::isfinite(range) || range <= lower || range >= upper) {
      continue;
    }
    const double angle = static_cast<double>(sample.scan->angle_min) +
      static_cast<double>(index) * static_cast<double>(sample.scan->angle_increment);
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);
    points.emplace_back(
      transform.transform.translation.x + cos_yaw * x - sin_yaw * y,
      transform.transform.translation.y + sin_yaw * x + cos_yaw * y);
  }
  if (points.empty()) {
    reason = "scan has no valid obstacle endpoints";
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::staticKeepoutMaskPathIsClear(
  const nav_msgs::msg::OccupancyGrid * mask,
  const nav_msgs::msg::Path & path,
  std::string & reason) const
{
  if (mask == nullptr) {
    reason = "required static keepout mask was not received";
    return false;
  }
  const auto result = staticKeepoutMaskFootprintPathSweep(
    mask, global_frame_, path, footprint_sweep_options_);
  if (result != StaticKeepoutMaskSweepResult::kClear) {
    reason = std::string("static keepout footprint sweep ") +
      staticKeepoutMaskSweepResultName(result);
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::filterStaticKeepoutPoints(
  const std::vector<std::pair<double, double>> & points,
  const nav_msgs::msg::OccupancyGrid * mask,
  std::vector<std::pair<double, double>> & filtered_points,
  std::string & reason) const
{
  if (mask == nullptr) {
    reason = "required static keepout mask was not received";
    return false;
  }
  const auto result = filterPointsOutsideStaticKeepoutMask(
    points, mask, global_frame_, filtered_points);
  // The static full-body sweep runs before this dynamic-observation filter,
  // but retain a fail-closed check here as well.  A missing mask or any future
  // non-success filter result must never turn static keepout returns into
  // evidence that the raw costmaps observed a dynamic obstacle.
  if (result != StaticKeepoutMaskFilterResult::kFiltered) {
    reason = std::string("keepout mask ") + staticKeepoutMaskFilterResultName(result);
    return false;
  }
  if (filtered_points.empty()) {
    reason = "scan has no endpoints outside occupied or unknown keepout mask cells";
    return false;
  }
  return true;
}

bool AckermannReverseRetreatAction::retreatPathIsClear(
  const nav_msgs::msg::Path & path, std::string & reason) const
{
  CostmapSample global_sample;
  CostmapSample local_sample;
  ScanSample global_scan_sample;
  ScanSample local_scan_sample;
  nav_msgs::msg::OccupancyGrid::SharedPtr keepout_mask;
  const bool static_keepout_required = !static_keepout_mask_topic_.empty();
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    global_sample = {
      global_costmap_, global_costmap_received_at_, global_costmap_sequence_, global_costmap_stamp_ns_,
      global_costmap_scan_sequence_, global_costmap_scan_stamp_ns_};
    local_sample = {
      local_costmap_, local_costmap_received_at_, local_costmap_sequence_, local_costmap_stamp_ns_,
      local_costmap_scan_sequence_, local_costmap_scan_stamp_ns_};
    global_scan_sample = global_costmap_scan_;
    local_scan_sample = local_costmap_scan_;
    keepout_mask = static_keepout_mask_;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto ros_now = node_->get_clock()->now();
  const auto global_minimum_stamp_ns = std::max(
    costmap_min_stamp_ns_, post_clear_ros_stamp_ns_);
  const auto local_minimum_stamp_ns = std::max(
    local_costmap_min_stamp_ns_, post_clear_ros_stamp_ns_);
  const auto minimum_scan_stamp_ns = std::max(
    scan_barrier_stamp_ns_, post_clear_ros_stamp_ns_);
  const auto global_freshness = costmapSampleFreshness(
    global_sample, global_frame_, costmap_max_age_, ros_now, now,
    global_minimum_stamp_ns, global_costmap_barrier_sequence_, minimum_scan_stamp_ns,
    scan_barrier_sequence_, true);
  const auto local_freshness = costmapSampleFreshness(
    local_sample, global_frame_, costmap_max_age_, ros_now, now,
    local_minimum_stamp_ns, local_costmap_barrier_sequence_, minimum_scan_stamp_ns,
    scan_barrier_sequence_, true);
  if (global_freshness != CostmapSampleFreshness::kFresh ||
    local_freshness != CostmapSampleFreshness::kFresh)
  {
    reason = std::string("costmap freshness global=") +
      costmapSampleFreshnessName(global_freshness) + ", local=" +
      costmapSampleFreshnessName(local_freshness);
    return false;
  }
  if (static_keepout_required &&
    !staticKeepoutMaskPathIsClear(keepout_mask.get(), path, reason))
  {
    return false;
  }
  if (!scanIsFresh(global_scan_sample, reason)) {
    reason = "global map " + reason;
    return false;
  }
  if (!scanIsFresh(local_scan_sample, reason)) {
    reason = "local map " + reason;
    return false;
  }
  if (!scanCouldHaveBeenFused(global_sample, global_scan_sample, reason)) {
    reason = "global map " + reason;
    return false;
  }
  if (!scanCouldHaveBeenFused(local_sample, local_scan_sample, reason)) {
    reason = "local map " + reason;
    return false;
  }

  std::vector<std::pair<double, double>> global_points;
  if (!scanPointsInGlobalFrame(global_scan_sample, global_points, reason)) {
    reason = "global map " + reason;
    return false;
  }
  std::vector<std::pair<double, double>> local_points;
  if (!scanPointsInGlobalFrame(local_scan_sample, local_points, reason)) {
    reason = "local map " + reason;
    return false;
  }
  // A configured static mask removes static-route cells from the dynamic
  // obstacle witness set. Without a mask every valid scan endpoint remains
  // mandatory sensor evidence; neither mode permits a blind retreat.
  std::vector<std::pair<double, double>> global_witness_points = global_points;
  std::vector<std::pair<double, double>> local_witness_points = local_points;
  if (static_keepout_required) {
    if (!filterStaticKeepoutPoints(
        global_points, keepout_mask.get(), global_witness_points, reason))
    {
      reason = "global map " + reason;
      return false;
    }
    if (!filterStaticKeepoutPoints(
        local_points, keepout_mask.get(), local_witness_points, reason))
    {
      reason = "local map " + reason;
      return false;
    }
  }
  if (!costmapHasLethalObservationAtPoints(
      *global_sample.costmap, global_witness_points,
      footprint_sweep_options_.lethal_cost_threshold, scan_costmap_match_radius_m_) ||
    !costmapHasLethalObservationAtPoints(
      *local_sample.costmap, local_witness_points,
      footprint_sweep_options_.lethal_cost_threshold, scan_costmap_match_radius_m_))
  {
    reason = static_keepout_required ?
      "non-keepout post-clear scan endpoints are not marked in both raw costmaps" :
      "post-clear scan endpoints are not marked in both raw costmaps";
    return false;
  }

  const auto global_result = costmapFootprintPathSweep(
    path, *global_sample.costmap, footprint_sweep_options_);
  const auto local_result = costmapFootprintPathSweep(
    path, *local_sample.costmap, footprint_sweep_options_);
  if (global_result != CostmapFootprintSweepResult::kClear ||
    local_result != CostmapFootprintSweepResult::kClear)
  {
    reason = std::string("footprint sweep global=") +
      costmapFootprintSweepResultName(global_result) + ", local=" +
      costmapFootprintSweepResultName(local_result);
    return false;
  }
  return true;
}

AckermannReverseRetreatAction::ScanSample
AckermannReverseRetreatAction::newestScanAtOrBefore(
  const std::deque<ScanSample> & samples, std::int64_t costmap_stamp_ns)
{
  for (auto scan = samples.rbegin(); scan != samples.rend(); ++scan) {
    if (scan->stamp_ns > 0 && scan->stamp_ns <= costmap_stamp_ns) {
      return *scan;
    }
  }
  return ScanSample();
}

void AckermannReverseRetreatAction::updateScan(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (!scan) {
    return;
  }
  const auto stamp_ns = costmapStampNanoseconds(scan->header.stamp);
  if (!stamp_ns.has_value() || scan->header.frame_id.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring scan with an invalid source stamp or frame");
    return;
  }
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (*stamp_ns <= scan_.stamp_ns) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Ignoring non-monotonic scan stamp %lld (last=%lld)",
      static_cast<long long>(*stamp_ns), static_cast<long long>(scan_.stamp_ns));
    return;
  }
  ScanSample sample;
  sample.scan = std::move(scan);
  sample.received_at = std::chrono::steady_clock::now();
  sample.sequence = scan_.sequence + 1U;
  sample.stamp_ns = *stamp_ns;
  scan_ = sample;
  scan_history_.push_back(sample);
  constexpr std::size_t kMaximumScanHistory = 64U;
  while (scan_history_.size() > kMaximumScanHistory) {
    scan_history_.pop_front();
  }
}

void AckermannReverseRetreatAction::updateGlobalCostmap(
  nav2_msgs::msg::Costmap::SharedPtr costmap)
{
  if (!costmap) {
    return;
  }
  const auto stamp_ns = costmapSourceStampNanoseconds(*costmap);
  if (!stamp_ns.has_value()) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring global raw costmap with an invalid source stamp");
    return;
  }
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (*stamp_ns <= global_costmap_stamp_ns_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Ignoring non-monotonic global raw costmap stamp %lld (last=%lld)",
      static_cast<long long>(*stamp_ns), static_cast<long long>(global_costmap_stamp_ns_));
    return;
  }
  global_costmap_ = std::move(costmap);
  global_costmap_received_at_ = std::chrono::steady_clock::now();
  global_costmap_stamp_ns_ = *stamp_ns;
  global_costmap_scan_ = newestScanAtOrBefore(scan_history_, *stamp_ns);
  global_costmap_scan_sequence_ = global_costmap_scan_.sequence;
  global_costmap_scan_stamp_ns_ = global_costmap_scan_.stamp_ns;
  ++global_costmap_sequence_;
}

void AckermannReverseRetreatAction::updateLocalCostmap(
  nav2_msgs::msg::Costmap::SharedPtr costmap)
{
  if (!costmap) {
    return;
  }
  const auto stamp_ns = costmapSourceStampNanoseconds(*costmap);
  if (!stamp_ns.has_value()) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring local raw costmap with an invalid source stamp");
    return;
  }
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (*stamp_ns <= local_costmap_stamp_ns_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Ignoring non-monotonic local raw costmap stamp %lld (last=%lld)",
      static_cast<long long>(*stamp_ns), static_cast<long long>(local_costmap_stamp_ns_));
    return;
  }
  local_costmap_ = std::move(costmap);
  local_costmap_received_at_ = std::chrono::steady_clock::now();
  local_costmap_stamp_ns_ = *stamp_ns;
  local_costmap_scan_ = newestScanAtOrBefore(scan_history_, *stamp_ns);
  local_costmap_scan_sequence_ = local_costmap_scan_.sequence;
  local_costmap_scan_stamp_ns_ = local_costmap_scan_.stamp_ns;
  ++local_costmap_sequence_;
}

void AckermannReverseRetreatAction::updateStaticKeepoutMask(
  nav_msgs::msg::OccupancyGrid::SharedPtr mask)
{
  if (!mask) {
    return;
  }
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  static_keepout_mask_ = std::move(mask);
}

void AckermannReverseRetreatAction::updateOdometry(
  nav_msgs::msg::Odometry::SharedPtr odometry)
{
  if (!odometry) {
    return;
  }
  const auto stamp_ns = costmapStampNanoseconds(odometry->header.stamp);
  if (!stamp_ns.has_value()) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring odometry with an invalid source stamp");
    return;
  }
  if (!std::isfinite(odometry->pose.pose.position.x) ||
    !std::isfinite(odometry->pose.pose.position.y))
  {
    RCLCPP_WARN(node_->get_logger(), "Ignoring odometry with a non-finite planar pose");
    return;
  }

  OdomSample sample;
  sample.sample.x = odometry->pose.pose.position.x;
  sample.sample.y = odometry->pose.pose.position.y;
  sample.sample.received_at = std::chrono::steady_clock::now();
  sample.frame_id = odometry->header.frame_id;
  sample.child_frame_id = odometry->child_frame_id;
  sample.stamp_ns = *stamp_ns;
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (latest_odom_.has_value() && sample.stamp_ns <= latest_odom_->stamp_ns) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Ignoring non-monotonic odometry stamp %lld (last=%lld)",
      static_cast<long long>(sample.stamp_ns),
      static_cast<long long>(latest_odom_->stamp_ns));
    return;
  }
  sample.sample.sequence = latest_odom_.has_value() ?
    latest_odom_->sample.sequence + 1U : 1U;
  latest_odom_ = std::move(sample);
}

}  // namespace smartcar_nav2

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<smartcar_nav2::AckermannReverseRetreatAction>(
        name, "follow_path", configuration);
    };
  factory.registerBuilder<smartcar_nav2::AckermannReverseRetreatAction>(
    "AckermannReverseRetreat", builder);
}
