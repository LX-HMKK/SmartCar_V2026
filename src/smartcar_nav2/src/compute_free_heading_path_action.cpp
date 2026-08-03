#include "smartcar_nav2/compute_free_heading_path_action.hpp"
#include "smartcar_nav2/footprint_sweep_collision_source.hpp"
#include "smartcar_nav2/forward_path_geometry_validation.hpp"
#include "smartcar_nav2/free_transit_goal_samples.hpp"
#include "smartcar_nav2/planner_path_start_contract.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <thread>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp_action/create_client.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace smartcar_nav2
{

// Preserve a completed global route while FollowPath is running.  Unlike
// RateController and DistanceController, this decorator does not re-enter a
// successful child on later BT ticks.  PipelineSequence halts its children on
// a genuine FollowPath failure, which clears the latch and permits recovery to
// build a fresh route from the current pose.
class LatchSuccess : public BT::DecoratorNode
{
public:
  LatchSuccess(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & configuration)
  : BT::DecoratorNode(xml_tag_name, configuration)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      success_latched_ = false;
    }
    setStatus(BT::NodeStatus::RUNNING);
    if (success_latched_) {
      return BT::NodeStatus::SUCCESS;
    }

    const BT::NodeStatus child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::SUCCESS) {
      success_latched_ = true;
    }
    return child_status;
  }

  void halt() override
  {
    success_latched_ = false;
    BT::DecoratorNode::halt();
  }

private:
  bool success_latched_{false};
};

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr char kFreeHeadingMinimumTurningRadiusParameter[] =
  "free_heading_minimum_turning_radius";
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr double kPositionEpsilon = 1.0e-6;
constexpr double kJoinPositionTolerance = 1.0e-3;
constexpr double kJoinYawTolerance = 1.0e-3;
constexpr std::size_t kMaximumThroughCandidateQueries = 64U;

bool finite(double value)
{
  return std::isfinite(value);
}

ForwardPathGeometryValidationOptions forwardPathGeometryOptions(
  const ReversePathValidationOptions & validation_options)
{
  return ForwardPathGeometryValidationOptions{
    validation_options.minimum_turning_radius,
    validation_options.curvature_tolerance,
    validation_options.maximum_direction_error,
    std::min(
      validation_options.maximum_direction_error,
      validation_options.goal_yaw_tolerance),
    validation_options.minimum_segment_length,
  };
}

}  // namespace

ComputeFreeHeadingPathAction::ComputeFreeHeadingPathAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & configuration,
  bool reverse,
  bool through_poses)
: BT::StatefulActionNode(xml_tag_name, configuration),
  reverse_(reverse),
  through_poses_(through_poses)
{
  node_ = configuration.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = configuration.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_base_frame", robot_base_frame_);
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  if (!node_->has_parameter(kFreeHeadingMinimumTurningRadiusParameter)) {
    node_->declare_parameter<double>(
      kFreeHeadingMinimumTurningRadiusParameter, 0.55);
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
  planner_client_ = rclcpp_action::create_client<ComputePathToPose>(
    node_, "compute_path_to_pose", callback_group_);
  // Keep the planner's rolling costmap snapshot in the same callback group as
  // the action client.  This node explicitly spins that group while it waits
  // for planner results; leaving the subscription in the default executor can
  // silently leave every candidate with an apparent zero-cost snapshot.
  rclcpp::SubscriptionOptions costmap_options;
  costmap_options.callback_group = callback_group_;
  global_costmap_subscription_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr costmap) {
      updateGlobalCostmap(std::move(costmap));
    }, costmap_options);
  local_costmap_subscription_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap_raw", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr costmap) {
      updateLocalCostmap(std::move(costmap));
    }, costmap_options);
  // Retain the raw local map above for reverse-recovery scan barriers. The
  // forward tracking envelope must instead see Nav2's filtered master grid,
  // because this is the grid used by the controller's collision projection.
  local_filtered_costmap_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
      updateLocalFilteredCostmap(std::move(costmap));
    }, costmap_options);
}

BT::PortsList ComputeFreeHeadingPathAction::providedPorts()
{
  return {
    BT::OutputPort<nav_msgs::msg::Path>(
      "path", "Costmap-aware path with free transit headings resolved"),
    BT::OutputPort<bool>(
      "recovery_eligible",
      "True only when the bounded candidate search exhausts every feasible path"),
    BT::OutputPort<std::int64_t>(
      "costmap_stamp_ns",
      "Source timestamp used as the post-clear recovery barrier"),
    BT::OutputPort<std::int64_t>(
      "local_costmap_stamp_ns",
      "Local raw-costmap timestamp used as the post-clear recovery barrier"),
    BT::OutputPort<std::uint64_t>(
      "costmap_sequence",
      "Monotonic raw costmap sequence used for recovery diagnostics"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "Single real-frame destination"),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
      "goals", "Ordered real-frame destinations"),
    BT::InputPort<std::string>("planner_id", "", "Planner plugin name"),
    BT::InputPort<int>(
      "heading_samples", 12,
      "Fallback terminal headings sampled after the live geometric heading fails"),
    BT::InputPort<int>(
      "candidate_timeout_ms", 3500,
      "Maximum duration of one ComputePathToPose query in milliseconds"),
    BT::InputPort<int>(
      "search_budget_ms", 2400,
      "Maximum duration of one free-heading target decision in milliseconds"),
    BT::InputPort<int>(
      "through_search_budget_ms", 12000,
      "Maximum duration of the bounded through-poses search in milliseconds"),
    BT::InputPort<int>(
      "cancellation_timeout_ms", 500,
      "Maximum duration spent waiting for a cancelled planner query in milliseconds"),
    BT::InputPort<int>(
      "through_solution_limit", 4,
      "Maximum complete through-poses heading chains compared before publication"),
    BT::InputPort<double>(
      "max_start_drift_m", 0.10,
      "Reject a plan whose captured robot start has become stale"),
    BT::InputPort<int>(
      "costmap_wait_timeout_ms", 1500,
      "Maximum wait for a fresh global costmap snapshot in milliseconds"),
    BT::InputPort<int>(
      "costmap_max_age_ms", 2000,
      "Maximum accepted global costmap snapshot age in milliseconds"),
    BT::InputPort<int>(
      "maximum_path_cost", 252,
      "Maximum permitted sampled global costmap cost for a candidate path"),
    BT::InputPort<double>(
      "footprint_half_length_m", 0.2491,
      "Padded vehicle half length used for global-costmap footprint checks"),
    BT::InputPort<double>(
      "footprint_half_width_m", 0.095,
      "Padded vehicle half width used for global-costmap footprint checks"),
    BT::InputPort<double>(
      "footprint_sweep_step_m", 0.025,
      "Maximum pose spacing for global-costmap footprint sweep"),
    BT::InputPort<int>(
      "footprint_lethal_cost", 254,
      "Costmap value at or above which the footprint sweep rejects a path"),
    BT::InputPort<double>(
      "local_tracking_cross_track_error_m", 0.0,
      "Maximum admitted forward-controller cross-track error added to the local footprint"),
    BT::InputPort<double>(
      "local_tracking_horizon_m", 0.0,
      "Visible forward path horizon swept in the filtered local costmap"),
    BT::InputPort<int>(
      "local_tracking_lethal_cost", 254,
      "Filtered local cost at or above which the tracking envelope rejects a path"),
    BT::InputPort<std::string>(
      "local_tracking_lateral_profile", kForwardPathLateralProfileSymmetric,
      "Forward tracking-tube profile; the P departure profile is Gazebo-only"),
    BT::InputPort<double>(
      "local_tracking_profile_start_position_tolerance_m", 0.001,
      "Strict P-start tolerance required before activating a P lateral profile"),
    BT::InputPort<double>(
      "local_tracking_profile_start_yaw_tolerance_rad", 0.001,
      "Strict P-start yaw tolerance required before activating a P lateral profile"),
    BT::InputPort<bool>(
      "departure_connector_enabled", false,
      "Enable P-only internal safe departure connectors before the first planner query"),
    BT::InputPort<double>(
      "departure_connector_radius_margin_m", 0.08,
      "Positive margin added to the active kinematic radius for a P departure arc"),
    BT::InputPort<double>(
      "departure_connector_maximum_active_radius_m", 0.0,
      "Maximum active kinematic radius; larger real-vehicle radii skip the P connector"),
    BT::InputPort<double>(
      "departure_connector_terminal_radius_m", 0.0,
      "P-only RSL terminal radius, constrained to the active simulator connector envelope"),
    BT::InputPort<double>(
      "departure_connector_high_right_turn_radius_m", 0.0,
      "P-only high right-turn radius, constrained to the active simulator connector envelope"),
    BT::InputPort<double>(
      "departure_connector_start_x_m", 0.0,
      "Configured P-start x coordinate required before injecting a connector"),
    BT::InputPort<double>(
      "departure_connector_start_y_m", 0.0,
      "Configured P-start y coordinate required before injecting a connector"),
    BT::InputPort<double>(
      "departure_connector_start_yaw_rad", 0.0,
      "Configured P-start yaw required before injecting a connector"),
    BT::InputPort<double>(
      "departure_connector_start_position_tolerance_m", 0.10,
      "Maximum P-start position error allowed to inject a connector"),
    BT::InputPort<double>(
      "departure_connector_start_yaw_tolerance_rad", 0.15,
      "Maximum P-start yaw error allowed to inject a connector"),
    BT::InputPort<int>(
      "departure_connector_heading_bins", 0,
      "Smac heading-bin count required for an exact P-connector lattice handoff"),
    BT::InputPort<std::string>(
      "static_keepout_mask_topic", "/keepout_filter_mask",
      "Optional static KeepoutFilter mask swept as a hard body and field-boundary constraint"),
    BT::InputPort<double>(
      "minimum_turning_radius", 0.0,
      "Optional per-tree minimum radius; zero follows the navigator kinematic radius"),
    BT::InputPort<double>(
      "curvature_tolerance", 0.20, "Discrete curvature allowance"),
    BT::InputPort<double>(
      "maximum_direction_error", 0.35, "Maximum reverse tangent error"),
    BT::InputPort<double>(
      "start_position_tolerance", 0.10, "Maximum path start position error"),
    BT::InputPort<double>(
      "start_yaw_tolerance", 0.15, "Maximum path start yaw error"),
    BT::InputPort<double>(
      "goal_position_tolerance", 0.20, "Maximum path goal position error"),
    BT::InputPort<double>(
      "goal_yaw_tolerance", 0.15, "Maximum path goal yaw error"),
    BT::InputPort<double>(
      "minimum_segment_length", 1.0e-4, "Minimum nonzero path segment"),
  };
}

BT::NodeStatus ComputeFreeHeadingPathAction::onStart()
{
  callback_group_executor_.spin_some();
  if (cancellationInProgress()) {
    const auto cancellation_status = waitForCancellation();
    if (cancellation_status != BT::NodeStatus::SUCCESS) {
      return cancellation_status;
    }
  }
  if (query_state_ != QueryState::IDLE) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Free-heading planner cannot start while a previous query is still active");
    cancelActiveQuery(true);
    return waitForCancellation();
  }

  clearPathOutput();
  setRecoveryEligible(false);
  virtual_path_ = nav_msgs::msg::Path();
  candidate_goals_.clear();
  lookahead_candidates_.clear();
  best_candidate_.reset();
  best_trial_path_.reset();
  best_continuation_.reset();
  pending_candidate_.reset();
  departure_connectors_.clear();
  departure_connector_index_ = 0U;
  through_search_frames_.clear();
  through_candidate_query_count_ = 0U;
  best_through_path_.reset();
  best_through_quality_.reset();
  through_complete_path_count_ = 0U;
  target_index_ = 0;
  candidate_index_ = 0;
  lookahead_candidate_index_ = 0;
  planner_query_failed_ = false;
  search_budget_exhausted_ = false;
  failure_after_cancellation_ = false;
  waiting_for_costmap_ = false;
  setOutput("costmap_stamp_ns", static_cast<std::int64_t>(0));
  setOutput("local_costmap_stamp_ns", static_cast<std::int64_t>(0));
  setOutput("costmap_sequence", static_cast<std::uint64_t>(0U));

  if (!loadInputs()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!planner_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
    RCLCPP_ERROR(
      node_->get_logger(), "Free-heading planner action server is unavailable");
    return BT::NodeStatus::FAILURE;
  }
  if (!nav2_util::getCurrentPose(
      real_start_, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not obtain a fresh planning start pose");
    return BT::NodeStatus::FAILURE;
  }
  if (reverse_) {
    if (!rotatePoseYawByPi(real_start_, virtual_start_)) {
      RCLCPP_ERROR(node_->get_logger(), "Reverse planner start quaternion is invalid");
      return BT::NodeStatus::FAILURE;
    }
  } else {
    virtual_start_ = real_start_;
  }
  planning_virtual_start_ = virtual_start_;
  costmap_wait_deadline_ =
    std::chrono::steady_clock::now() + costmap_wait_timeout_;
  if (!hasFreshPlanningCostmaps()) {
    waiting_for_costmap_ = true;
    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for fresh planning costmaps before free-heading planning");
    return BT::NodeStatus::RUNNING;
  }
  publishCostmapBarrier();
  if (!beginCandidateSearch()) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeFreeHeadingPathAction::onRunning()
{
  callback_group_executor_.spin_some();
  if (cancellationInProgress()) {
    const BT::NodeStatus cancellation_status = waitForCancellation();
    return cancellation_status;
  }
  if (waiting_for_costmap_) {
    if (!hasFreshPlanningCostmaps()) {
      if (std::chrono::steady_clock::now() >= costmap_wait_deadline_) {
        RCLCPP_ERROR(
          node_->get_logger(), "Free-heading planner timed out waiting for required costmaps");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    waiting_for_costmap_ = false;
    publishCostmapBarrier();
    if (!beginCandidateSearch()) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (through_poses_ && goalsChanged()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Free-heading goals changed while planning; cancelling the stale query");
    cancelActiveQuery(true);
    planner_query_failed_ = true;
    setRecoveryEligible(false);
    clearPathOutput();
    return waitForCancellation();
  }
  if (query_state_ == QueryState::IDLE) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading planner has no active query");
    return BT::NodeStatus::FAILURE;
  }
  if (std::chrono::steady_clock::now() > query_deadline_) {
    // A query timeout is an infrastructure/planner failure.  It is never
    // evidence that the geometry is unreachable. Do not publish a previously
    // cached candidate after this timeout: it may no longer be fresh enough
    // for the costmap contract, and it must not arm retreat.
    planner_query_failed_ = true;
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading planner candidate %zu for goal %zu timed out; cancelling and failing closed",
      candidate_index_ + 1, target_index_ + 1);
    cancelActiveQuery(true);
    if (!cancellationInProgress()) {
      clearPathOutput();
      return BT::NodeStatus::FAILURE;
    }
    const BT::NodeStatus cancellation_status = waitForCancellation();
    clearPathOutput();
    return cancellation_status;
  }

  const bool waiting_for_goal_handle =
    query_state_ == QueryState::WAITING_FOR_GOAL_HANDLE ||
    query_state_ == QueryState::WAITING_FOR_LOOKAHEAD_GOAL_HANDLE;
  if (waiting_for_goal_handle) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    const bool lookahead_query =
      query_state_ == QueryState::WAITING_FOR_LOOKAHEAD_GOAL_HANDLE;
    try {
      active_goal_handle_ = goal_handle_future_.get();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Free-heading planner goal request failed: %s", error.what());
      clearCompletedQuery();
      return failPlannerQuery("planner goal transport error", lookahead_query);
    }
    goal_handle_future_ = std::shared_future<PlannerGoalHandle::SharedPtr>();
    if (!active_goal_handle_) {
      clearCompletedQuery();
      return failPlannerQuery("planner goal was rejected", lookahead_query);
    }
    try {
      result_future_ = planner_client_->async_get_result(active_goal_handle_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Free-heading planner result request failed: %s", error.what());
      clearCompletedQuery();
      return failPlannerQuery("planner result transport error", lookahead_query);
    }
    query_state_ = lookahead_query ?
      QueryState::WAITING_FOR_LOOKAHEAD_RESULT : QueryState::WAITING_FOR_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }
  const bool lookahead_query =
    query_state_ == QueryState::WAITING_FOR_LOOKAHEAD_RESULT;
  PlannerGoalHandle::WrappedResult result;
  try {
    result = result_future_.get();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_->get_logger(), "Free-heading planner result failed: %s", error.what());
    clearCompletedQuery();
    return failPlannerQuery("planner result transport error", lookahead_query);
  }
  clearCompletedQuery();
  if (result.code == rclcpp_action::ResultCode::ABORTED) {
    // Humble ComputePathToPose exposes no planner error code in its action
    // result. PlannerServer uses ABORTED for NoValidPathCouldBeFound, so an
    // accepted query that terminates on time may only reject this heading
    // candidate. It does not arm retreat on its own: complete*() advances the
    // bounded candidate search and recovery remains gated on full exhaustion.
    // CANCELED, UNKNOWN, transport failures, goal rejection, and timeouts are
    // still handled below as fail-closed infrastructure failures.
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Free-heading planner aborted candidate %zu for goal %zu; Humble action has no typed "
      "planner failure reason, continuing bounded candidate search",
      candidate_index_ + 1, target_index_ + 1);
    return lookahead_query ?
      completeLookahead(nullptr, true) : completeCandidate(nullptr, true);
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Free-heading planner returned canceled or unknown action result code %d; "
      "refusing recovery eligibility",
      static_cast<int>(result.code));
    return failPlannerQuery("planner action canceled or returned an unknown result", lookahead_query);
  }
  if (!result.result) {
    return failPlannerQuery("planner returned a null result", lookahead_query);
  }
  if (result.result->path.poses.empty()) {
    // Some planners report an empty successful result for a rejected terminal
    // heading. Treat it like the Humble ABORTED no-path representation above:
    // only this candidate is rejected and recovery still needs full exhaustion.
    return lookahead_query ?
      completeLookahead(nullptr, true) : completeCandidate(nullptr, true);
  }
  return lookahead_query ?
    completeLookahead(&result.result->path) : completeCandidate(&result.result->path);
}

void ComputeFreeHeadingPathAction::onHalted()
{
  cancelActiveQuery(false);
  while (cancellationInProgress() &&
    std::chrono::steady_clock::now() <= cancellation_deadline_)
  {
    callback_group_executor_.spin_some();
    if (waitForCancellation() != BT::NodeStatus::RUNNING) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (cancellationInProgress()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Free-heading planner cancellation exceeded the halt deadline");
    clearQueryState();
  }
  candidate_goals_.clear();
  lookahead_candidates_.clear();
  pending_candidate_.reset();
  departure_connectors_.clear();
  departure_connector_index_ = 0U;
  best_trial_path_.reset();
  best_continuation_.reset();
  through_search_frames_.clear();
  through_candidate_query_count_ = 0U;
  best_through_path_.reset();
  best_through_quality_.reset();
  through_complete_path_count_ = 0U;
  through_search_deadline_ = std::chrono::steady_clock::time_point();
  waiting_for_costmap_ = false;
  clearPathOutput();
}

bool ComputeFreeHeadingPathAction::loadInputs()
{
  if (!getInput("planner_id", planner_id_) || planner_id_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading planner_id is missing");
    return false;
  }
  if (!getInput("heading_samples", heading_samples_) || heading_samples_ < 4 ||
    heading_samples_ > 72)
  {
    RCLCPP_ERROR(node_->get_logger(), "heading_samples must lie in [4, 72]");
    return false;
  }
  int timeout_ms = 0;
  if (!getInput("candidate_timeout_ms", timeout_ms) || timeout_ms <= 0 || timeout_ms > 5000) {
    RCLCPP_ERROR(node_->get_logger(), "candidate_timeout_ms must lie in [1, 5000]");
    return false;
  }
  candidate_timeout_ = std::chrono::milliseconds(timeout_ms);
  int search_budget_ms = 0;
  if (!getInput("search_budget_ms", search_budget_ms) ||
    search_budget_ms < 200 || search_budget_ms > 5000)
  {
    RCLCPP_ERROR(node_->get_logger(), "search_budget_ms must lie in [200, 5000]");
    return false;
  }
  search_budget_ = std::chrono::milliseconds(search_budget_ms);
  int through_search_budget_ms = 0;
  if (!getInput("through_search_budget_ms", through_search_budget_ms) ||
    through_search_budget_ms < 2000 || through_search_budget_ms > 30000)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "through_search_budget_ms must lie in [2000, 30000]");
    return false;
  }
  through_search_budget_ = std::chrono::milliseconds(through_search_budget_ms);
  int cancellation_timeout_ms = 0;
  if (!getInput("cancellation_timeout_ms", cancellation_timeout_ms) ||
    cancellation_timeout_ms < 100 || cancellation_timeout_ms > 2000)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "cancellation_timeout_ms must lie in [100, 2000]");
    return false;
  }
  cancellation_timeout_ = std::chrono::milliseconds(cancellation_timeout_ms);
  int through_solution_limit = 0;
  if (!getInput("through_solution_limit", through_solution_limit) ||
    through_solution_limit < 1 || through_solution_limit > 8)
  {
    RCLCPP_ERROR(node_->get_logger(), "through_solution_limit must lie in [1, 8]");
    return false;
  }
  through_solution_limit_ = static_cast<std::size_t>(through_solution_limit);
  if (!getInput("max_start_drift_m", max_start_drift_m_) ||
    !finite(max_start_drift_m_) || max_start_drift_m_ <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "max_start_drift_m must be positive");
    return false;
  }
  int costmap_wait_timeout_ms = 0;
  if (!getInput("costmap_wait_timeout_ms", costmap_wait_timeout_ms) ||
    costmap_wait_timeout_ms < 100 || costmap_wait_timeout_ms > 5000)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "costmap_wait_timeout_ms must lie in [100, 5000]");
    return false;
  }
  costmap_wait_timeout_ = std::chrono::milliseconds(costmap_wait_timeout_ms);
  int costmap_max_age_ms = 0;
  if (!getInput("costmap_max_age_ms", costmap_max_age_ms) ||
    costmap_max_age_ms < 100 || costmap_max_age_ms > 10000)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "costmap_max_age_ms must lie in [100, 10000]");
    return false;
  }
  costmap_max_age_ = std::chrono::milliseconds(costmap_max_age_ms);
  int maximum_path_cost = 0;
  if (!getInput("maximum_path_cost", maximum_path_cost) ||
    maximum_path_cost < 0 || maximum_path_cost > 253)
  {
    RCLCPP_ERROR(node_->get_logger(), "maximum_path_cost must lie in [0, 253]");
    return false;
  }
  maximum_path_cost_ = static_cast<std::uint8_t>(maximum_path_cost);
  if (!readFootprintSweepOptions()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading footprint sweep ports are invalid");
    return false;
  }
  if (!readLocalTrackingEnvelopeOptions()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading local tracking envelope ports are invalid");
    return false;
  }
  std::string static_keepout_mask_topic;
  if (!getInput("static_keepout_mask_topic", static_keepout_mask_topic) ||
    !configureKeepoutMaskSubscription(static_keepout_mask_topic))
  {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading keepout-mask diagnostic port is invalid");
    return false;
  }
  if (!getInput("goal_position_tolerance", goal_position_tolerance_) ||
    !finite(goal_position_tolerance_) || goal_position_tolerance_ < 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "goal_position_tolerance is invalid");
    return false;
  }
  if (!readValidationOptions()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading planner validation ports are invalid");
    return false;
  }
  if (!readDepartureConnectorOptions()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading departure connector ports are invalid");
    return false;
  }

  real_goals_.clear();
  if (through_poses_) {
    if (!getInput("goals", real_goals_) || real_goals_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Free-heading through-poses goals are missing");
      return false;
    }
  } else {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_ERROR(node_->get_logger(), "Free-heading goal is missing");
      return false;
    }
    real_goals_.push_back(goal);
  }
  for (const auto & goal : real_goals_) {
    if (goal.header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Free-heading goal frame '%s' must equal global frame '%s'",
        goal.header.frame_id.c_str(), global_frame_.c_str());
      return false;
    }
    if (!finite(goal.pose.position.x) || !finite(goal.pose.position.y) ||
      (!isZeroQuaternion(goal.pose.orientation) && !isUnitQuaternion(goal.pose.orientation)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Free-heading goal has an invalid pose");
      return false;
    }
  }
  return true;
}

bool ComputeFreeHeadingPathAction::beginCandidateSearch()
{
  if (through_poses_) {
    through_search_deadline_ =
      std::chrono::steady_clock::now() + through_search_budget_;
  }
  return prepareDepartureConnectors() && prepareTargetCandidates() && startCandidateQuery();
}

bool ComputeFreeHeadingPathAction::readValidationOptions()
{
  double minimum_turning_radius = 0.0;
  if (!getInput("minimum_turning_radius", minimum_turning_radius) ||
    !getInput("curvature_tolerance", validation_options_.curvature_tolerance) ||
    !getInput("maximum_direction_error", validation_options_.maximum_direction_error) ||
    !getInput("start_position_tolerance", validation_options_.start_position_tolerance) ||
    !getInput("start_yaw_tolerance", validation_options_.start_yaw_tolerance) ||
    !getInput("goal_position_tolerance", validation_options_.goal_position_tolerance) ||
    !getInput("goal_yaw_tolerance", validation_options_.goal_yaw_tolerance) ||
    !getInput("minimum_segment_length", validation_options_.minimum_segment_length))
  {
    return false;
  }
  if (minimum_turning_radius <= 0.0) {
    if (!node_->get_parameter(
        kFreeHeadingMinimumTurningRadiusParameter, minimum_turning_radius))
    {
      return false;
    }
  }
  if (!finite(minimum_turning_radius) || minimum_turning_radius <= 0.0) {
    return false;
  }
  validation_options_.minimum_turning_radius = minimum_turning_radius;
  return true;
}

bool ComputeFreeHeadingPathAction::readFootprintSweepOptions()
{
  int lethal_cost = 0;
  if (!getInput("footprint_half_length_m", footprint_sweep_options_.half_length_m) ||
    !getInput("footprint_half_width_m", footprint_sweep_options_.half_width_m) ||
    !getInput("footprint_sweep_step_m", footprint_sweep_options_.sample_spacing_m) ||
    !getInput("footprint_lethal_cost", lethal_cost) ||
    !finite(footprint_sweep_options_.half_length_m) ||
    !finite(footprint_sweep_options_.half_width_m) ||
    !finite(footprint_sweep_options_.sample_spacing_m) ||
    footprint_sweep_options_.half_length_m <= 0.0 ||
    footprint_sweep_options_.half_width_m <= 0.0 ||
    footprint_sweep_options_.sample_spacing_m <= 0.0 ||
    lethal_cost < 1 || lethal_cost > 254)
  {
    return false;
  }
  footprint_sweep_options_.lethal_cost_threshold =
    static_cast<std::uint8_t>(lethal_cost);
  return true;
}

bool ComputeFreeHeadingPathAction::readLocalTrackingEnvelopeOptions()
{
  int lethal_cost = 0;
  if (!getInput(
      "local_tracking_cross_track_error_m", local_tracking_cross_track_error_m_) ||
    !getInput("local_tracking_horizon_m", local_tracking_horizon_m_) ||
    !getInput("local_tracking_lethal_cost", lethal_cost) ||
    !getInput("local_tracking_lateral_profile", local_tracking_lateral_profile_) ||
    !finite(local_tracking_cross_track_error_m_) ||
    !finite(local_tracking_horizon_m_) ||
    local_tracking_cross_track_error_m_ < 0.0 ||
    local_tracking_horizon_m_ < 0.0 || lethal_cost < 1 || lethal_cost > 254 ||
    !forwardPathLateralProfileKnown(local_tracking_lateral_profile_))
  {
    return false;
  }
  local_tracking_lethal_cost_ = static_cast<std::uint8_t>(lethal_cost);
  local_tracking_envelope_enabled_ = !reverse_ && local_tracking_cross_track_error_m_ > 0.0;
  return !local_tracking_envelope_enabled_ || local_tracking_horizon_m_ > 0.0;
}

bool ComputeFreeHeadingPathAction::readDepartureConnectorOptions()
{
  double radius_margin_m = 0.0;
  if (!getInput("departure_connector_enabled", departure_connector_enabled_) ||
    !getInput("departure_connector_radius_margin_m", radius_margin_m) ||
    !getInput(
      "departure_connector_maximum_active_radius_m",
      departure_connector_maximum_active_radius_m_) ||
    !getInput("departure_connector_start_x_m", departure_connector_start_x_m_) ||
    !getInput("departure_connector_start_y_m", departure_connector_start_y_m_) ||
    !getInput("departure_connector_start_yaw_rad", departure_connector_start_yaw_rad_) ||
    !getInput(
      "departure_connector_start_position_tolerance_m",
      departure_connector_start_position_tolerance_m_) ||
    !getInput(
      "departure_connector_start_yaw_tolerance_rad",
      departure_connector_start_yaw_tolerance_rad_) ||
    !getInput(
      "local_tracking_profile_start_position_tolerance_m",
      local_tracking_lateral_profile_start_.position_tolerance_m) ||
    !getInput(
      "local_tracking_profile_start_yaw_tolerance_rad",
      local_tracking_lateral_profile_start_.yaw_tolerance_rad) ||
    !getInput(
      "departure_connector_terminal_radius_m",
      departure_connector_terminal_radius_m_) ||
    !getInput(
      "departure_connector_high_right_turn_radius_m",
      departure_connector_high_right_turn_radius_m_) ||
    !getInput("departure_connector_heading_bins", departure_connector_heading_bins_))
  {
    return false;
  }
  local_tracking_lateral_profile_start_.frame_id = global_frame_;
  local_tracking_lateral_profile_start_.x_m = departure_connector_start_x_m_;
  local_tracking_lateral_profile_start_.y_m = departure_connector_start_y_m_;
  local_tracking_lateral_profile_start_.yaw_rad = departure_connector_start_yaw_rad_;
  if (!departure_connector_enabled_) {
    return local_tracking_lateral_profile_ == kForwardPathLateralProfileSymmetric;
  }
  if (reverse_ || through_poses_ || !finite(departure_connector_start_x_m_) ||
    !finite(departure_connector_start_y_m_) ||
    !finite(departure_connector_start_yaw_rad_) ||
    !finite(departure_connector_start_position_tolerance_m_) ||
    !finite(departure_connector_start_yaw_tolerance_rad_) ||
    !finite(departure_connector_maximum_active_radius_m_) ||
    !finite(departure_connector_terminal_radius_m_) ||
    !finite(departure_connector_high_right_turn_radius_m_) ||
    departure_connector_start_position_tolerance_m_ <= 0.0 ||
    departure_connector_start_yaw_tolerance_rad_ <= 0.0 ||
    departure_connector_maximum_active_radius_m_ <= 0.0 ||
    departure_connector_heading_bins_ < 4 || departure_connector_heading_bins_ > 720 ||
    !forwardPathLateralProfileStartValid(
      local_tracking_lateral_profile_, local_tracking_lateral_profile_start_) ||
    !forwardPathLateralProfileConfigurationValid(
      local_tracking_lateral_profile_, local_tracking_cross_track_error_m_,
      validation_options_.minimum_turning_radius) ||
    (local_tracking_lateral_profile_ == kForwardPathLateralProfilePDepartureSouthV1 &&
    static_keepout_mask_topic_.empty()))
  {
    return false;
  }
  departure_connector_options_.minimum_turning_radius_m =
    validation_options_.minimum_turning_radius;
  departure_connector_options_.radius_margin_m = radius_margin_m;
  departure_connector_options_.sample_spacing_m = footprint_sweep_options_.sample_spacing_m;
  departure_connector_options_.curvature_tolerance = validation_options_.curvature_tolerance;
  departure_connector_options_.maximum_direction_error =
    validation_options_.maximum_direction_error;
  departure_connector_options_.minimum_segment_length =
    validation_options_.minimum_segment_length;
  if (!departureConnectorOptionsValid(departure_connector_options_)) {
    return false;
  }

  // This tree is shared with the real vehicle, but the P connector is only
  // admitted inside its explicit simulator envelope. Check that gate before
  // validating the simulation-only terminal radius: a real-car run must
  // continue with normal Smac planning rather than rejecting a 0.22 m
  // connector it will never inject.
  if (!departureConnectorRadiusWithinMaximum(
      departure_connector_options_, departure_connector_maximum_active_radius_m_))
  {
    return true;
  }
  if (!departureConnectorHighRightTurnRadiusWithinEnvelope(
      departure_connector_options_, departure_connector_high_right_turn_radius_m_,
      departure_connector_maximum_active_radius_m_))
  {
    return false;
  }
  return departureConnectorTerminalRadiusWithinEnvelope(
    departure_connector_options_, departure_connector_terminal_radius_m_,
    departure_connector_maximum_active_radius_m_);
}

bool ComputeFreeHeadingPathAction::goalsChanged()
{
  std::vector<geometry_msgs::msg::PoseStamped> latest_goals;
  if (!getInput("goals", latest_goals) || latest_goals.empty()) {
    return true;
  }
  if (sameGoalSequence(real_goals_, latest_goals)) {
    return false;
  }

  // RemovePassedGoals only removes a leading run of already-passed goals. A
  // shorter list is therefore equivalent only when every removed goal has
  // already been committed into virtual_path_. Accepting an arbitrary suffix
  // here would allow a newly removed, still-unplanned waypoint to disappear
  // without invalidating this route.
  return !isGoalSequenceSuffix(real_goals_, latest_goals, target_index_);
}

bool ComputeFreeHeadingPathAction::prepareTargetCandidates(bool reset_search)
{
  if (target_index_ >= real_goals_.size()) {
    return false;
  }
  candidate_goals_.clear();
  lookahead_candidates_.clear();
  candidate_index_ = 0;
  lookahead_candidate_index_ = 0;
  if (reset_search) {
    best_candidate_.reset();
    best_trial_path_.reset();
    best_continuation_.reset();
    search_deadline_ = std::chrono::steady_clock::now() + search_budget_;
  }
  candidate_goals_ = goalCandidatesForTarget(target_index_, virtual_start_);
  return !candidate_goals_.empty();
}

bool ComputeFreeHeadingPathAction::prepareDepartureConnectors()
{
  departure_connectors_.clear();
  departure_connector_index_ = 0U;
  if (!departure_connector_enabled_) {
    return true;
  }
  if (reverse_ || through_poses_ || target_index_ != 0U || !virtual_path_.poses.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "P departure connectors are only valid for the first forward NavigateToPose edge");
    planner_query_failed_ = true;
    return false;
  }
  if (!departureConnectorRadiusWithinMaximum(
      departure_connector_options_, departure_connector_maximum_active_radius_m_))
  {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Skipping P departure connector because active radius %.3f exceeds its %.3f m gate",
      departure_connector_options_.minimum_turning_radius_m +
      departure_connector_options_.radius_margin_m,
      departure_connector_maximum_active_radius_m_);
    return true;
  }
  const double start_distance = std::hypot(
    virtual_start_.pose.position.x - departure_connector_start_x_m_,
    virtual_start_.pose.position.y - departure_connector_start_y_m_);
  const double start_yaw_error = angularDistance(
    quaternionYaw(virtual_start_.pose.orientation), departure_connector_start_yaw_rad_);
  if (start_distance > departure_connector_start_position_tolerance_m_ ||
    start_yaw_error > departure_connector_start_yaw_tolerance_rad_)
  {
    // A recovery replan after the vehicle has left P must use its actual pose;
    // do not replay a P-specific connector from a stale coordinate.
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Skipping P departure connector for non-P start distance=%.3f yaw_error=%.3f",
      start_distance, start_yaw_error);
    return true;
  }
  if (!hasFreshPlanningCostmaps()) {
    planner_query_failed_ = true;
    return false;
  }

  nav2_msgs::msg::Costmap::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(global_costmap_mutex_);
    costmap = global_costmap_;
  }
  if (!costmap || costmap->header.frame_id != global_frame_ ||
    costmap->metadata.resolution <= 0.0F || costmap->metadata.size_x == 0U ||
    costmap->metadata.size_y == 0U ||
    !isUnitQuaternion(costmap->metadata.origin.orientation) ||
    angularDistance(quaternionYaw(costmap->metadata.origin.orientation), 0.0) >
    kJoinYawTolerance)
  {
    // The lattice helper deliberately has no hidden field coordinate. It
    // needs an axis-aligned, live raw-costmap grid, and rejects ambiguity
    // rather than manufacturing a P departure in a rotated frame.
    RCLCPP_ERROR(
      node_->get_logger(),
      "P departure lattice connector requires a fresh axis-aligned global raw costmap");
    planner_query_failed_ = true;
    return false;
  }

  const auto generated = buildPDepartureEscapeLatticeConnectors(
    virtual_start_, costmap->metadata.origin.position.x,
    costmap->metadata.origin.position.y,
    static_cast<double>(costmap->metadata.resolution),
    static_cast<std::size_t>(departure_connector_heading_bins_),
    departure_connector_high_right_turn_radius_m_,
    departure_connector_options_);
  for (const auto & connector : generated) {
    if (!connector.lattice_aligned) {
      RCLCPP_ERROR(
        node_->get_logger(), "Refusing a P departure connector without an exact lattice endpoint");
      continue;
    }
    const auto kinematic = validateForwardConnectorPath(
      connector.path, virtual_start_, departure_connector_options_);
    if (!kinematic.valid) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rejected internally generated P departure connector: %s", kinematic.reason.c_str());
      continue;
    }
    const PathQuality quality = pathQuality(connector.path);
    if (staticKeepoutSweepIsInfrastructureFailure(quality)) {
      logStaticKeepoutSweepFailure(quality, "departure_connector", 0U, 1U);
      planner_query_failed_ = true;
      return false;
    }
    if (localTrackingSweepIsInfrastructureFailure(quality)) {
      logLocalTrackingSweepFailure(quality, "departure_connector", 0U, 1U);
      planner_query_failed_ = true;
      return false;
    }
    if (!hasAcceptableCostmapSample(quality)) {
      logFootprintSweepFailure(quality, "departure_connector", 0U, 1U);
      logStaticKeepoutSweepFailure(quality, "departure_connector", 0U, 1U);
      logLocalTrackingSweepFailure(quality, "departure_connector", 0U, 1U);
      continue;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Accepted lattice-aligned P departure connector radius=%.3f high_right_radius=%.3f "
      "arc_deg=%.1f "
      "straight_m=%.3f end=(%.4f,%.4f,%.4f) length=%.3f max_curvature=%.3f",
      connector.radius_m, connector.high_right_turn_radius_m,
      connector.arc_angle_rad * 180.0 / kPi,
      connector.straight_length_m,
      connector.path.poses.back().pose.position.x,
      connector.path.poses.back().pose.position.y,
      quaternionYaw(connector.path.poses.back().pose.orientation),
      kinematic.length_m, kinematic.maximum_curvature);
    departure_connectors_.push_back(connector);
  }
  if (departure_connectors_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "No P departure connector satisfies the fresh raw-costmap and full-footprint contract");
    return false;
  }
  return true;
}

const geometry_msgs::msg::PoseStamped &
ComputeFreeHeadingPathAction::plannerStartForCandidate() const
{
  if (departure_connector_index_ < departure_connectors_.size() &&
    !departure_connectors_[departure_connector_index_].path.poses.empty())
  {
    return departure_connectors_[departure_connector_index_].path.poses.back();
  }
  return virtual_start_;
}

bool ComputeFreeHeadingPathAction::buildCandidateSegment(
  const nav_msgs::msg::Path & planner_segment,
  nav_msgs::msg::Path & candidate_segment,
  std::string & reason) const
{
  candidate_segment = nav_msgs::msg::Path();
  if (departure_connector_index_ < departure_connectors_.size()) {
    const auto & departure = departure_connectors_[departure_connector_index_].path;
    if (!appendSegmentChecked(candidate_segment, departure, reason)) {
      return false;
    }
    if (target_index_ >= real_goals_.size() || departure.poses.empty() ||
      isZeroQuaternion(real_goals_[target_index_].pose.orientation) ||
      planarDistance(active_virtual_goal_, real_goals_[target_index_]) > kPositionEpsilon ||
      angularDistance(
        quaternionYaw(active_virtual_goal_.pose.orientation),
        quaternionYaw(real_goals_[target_index_].pose.orientation)) > kPositionEpsilon)
    {
      reason = "p_terminal_goal_not_exact_locked_task_pose";
      return false;
    }

    // Smac's heading lattice can return a pose quaternion that falls within
    // the task tolerance while its final XY tangent still points elsewhere.
    // The P escape prefix is already swept and tangent-continuous; finish it
    // with a deterministic RSL curve to the authored task pose instead of
    // publishing that quantized final segment to the controller.
    nav_msgs::msg::Path terminal_connector;
    if (!buildPDepartureRslTerminalConnector(
        departure.poses.back(), real_goals_[target_index_],
        departure_connector_terminal_radius_m_,
        departure_connector_options_, terminal_connector))
    {
      reason = "p_terminal_rsl_unavailable";
      return false;
    }
    return appendSegmentChecked(candidate_segment, terminal_connector, reason);
  }
  return appendSegmentChecked(candidate_segment, planner_segment, reason);
}

bool ComputeFreeHeadingPathAction::prepareLookaheadCandidates()
{
  lookahead_candidates_.clear();
  lookahead_candidate_index_ = 0;
  if (!pending_candidate_.has_value() ||
    target_index_ + 1 >= real_goals_.size() ||
    pending_candidate_->path.poses.empty())
  {
    return false;
  }
  lookahead_candidates_ = goalCandidatesForTarget(
    target_index_ + 1, pending_candidate_->path.poses.back());
  return !lookahead_candidates_.empty();
}

std::vector<ComputeFreeHeadingPathAction::GoalCandidate>
ComputeFreeHeadingPathAction::goalCandidatesForTarget(
  std::size_t target_index,
  const geometry_msgs::msg::PoseStamped & start) const
{
  std::vector<GoalCandidate> candidates;
  if (target_index >= real_goals_.size()) {
    return candidates;
  }

  const auto & real_goal = real_goals_[target_index];
  if (!isZeroQuaternion(real_goal.pose.orientation)) {
    const double authored_virtual_yaw = referenceYawForGoal(
      real_goal, target_index, start);
    if (!reverse_ && !through_poses_ && target_index == 0U &&
      departure_connector_index_ < departure_connectors_.size())
    {
      // The P-specific RSL terminal connector is exact by construction. Do
      // not let a tolerated heading sample cause an XY/tangent mismatch at
      // the locked semantic task pose.
      GoalCandidate candidate;
      candidate.pose = real_goal;
      candidate.pose.pose.orientation = quaternionFromYaw(authored_virtual_yaw);
      candidates.push_back(std::move(candidate));
      return candidates;
    }
    // Keep the task position and semantic yaw fixed.  Only the planner's
    // terminal heading bin is varied inside the existing authored tolerance;
    // endpointMatchesCandidateAndRealGoal() below rejects any result that
    // actually leaves that tolerance after quantization.
    const auto headings = lockedGoalHeadingHints(
      authored_virtual_yaw, validation_options_.goal_yaw_tolerance);
    candidates.reserve(headings.size());
    for (const double heading : headings) {
      GoalCandidate candidate;
      candidate.pose = real_goal;
      candidate.pose.pose.orientation = quaternionFromYaw(heading);
      candidates.push_back(std::move(candidate));
    }
    return candidates;
  }

  // A free target remains at its authored position. Its heading list is
  // finite and deterministic; time and query budgets can still stop a route
  // search before every cross-product combination is evaluated.
  geometry_msgs::msg::PoseStamped target = real_goal;
  const double reference_yaw = referenceYawForGoal(target, target_index, start);
  std::vector<double> tangent_yaws;
  tangent_yaws.reserve(2U);
  const double incoming_dx = target.pose.position.x - start.pose.position.x;
  const double incoming_dy = target.pose.position.y - start.pose.position.y;
  if (std::hypot(incoming_dx, incoming_dy) > kPositionEpsilon) {
    tangent_yaws.push_back(std::atan2(incoming_dy, incoming_dx));
  }
  if (target_index + 1 < real_goals_.size()) {
    const auto & next_target = real_goals_[target_index + 1];
    const double outgoing_dx = next_target.pose.position.x - target.pose.position.x;
    const double outgoing_dy = next_target.pose.position.y - target.pose.position.y;
    if (std::hypot(outgoing_dx, outgoing_dy) > kPositionEpsilon) {
      // Prefer the successor tangent in the deterministic hint order. It
      // gives a transit point a direct chance to carry steering state forward.
      tangent_yaws.insert(
        tangent_yaws.begin(), std::atan2(outgoing_dy, outgoing_dx));
    }
  }
  const auto headings = freeTransitHeadingHints(
    reference_yaw, tangent_yaws, heading_samples_);
  candidates.reserve(std::min(
    headings.size(), static_cast<std::size_t>(heading_samples_)));
  for (const double heading : headings) {
    if (candidates.size() >= static_cast<std::size_t>(heading_samples_)) {
      break;
    }
    if (!finite(heading)) {
      continue;
    }
    GoalCandidate candidate;
    candidate.pose = target;
    candidate.pose.pose.orientation = quaternionFromYaw(heading);
    candidates.push_back(std::move(candidate));
  }
  return candidates;
}

double ComputeFreeHeadingPathAction::referenceYawForGoal(
  const geometry_msgs::msg::PoseStamped & target,
  std::size_t target_index,
  const geometry_msgs::msg::PoseStamped & start) const
{
  if (target_index >= real_goals_.size()) {
    return quaternionYaw(start.pose.orientation);
  }
  const auto & real_goal = real_goals_[target_index];
  if (!isZeroQuaternion(real_goal.pose.orientation)) {
    return quaternionYaw(real_goal.pose.orientation) + (reverse_ ? kPi : 0.0);
  }

  // A position-only transit must remain tangent-continuous across its next
  // constraint.  Favor the unit-vector bisector of the incoming and outgoing
  // legs instead of hard-locking to the outgoing leg.  The latter turns a
  // short gate into an abrupt Ackermann turn even though its orientation was
  // intentionally left free.
  const double incoming_dx = target.pose.position.x - start.pose.position.x;
  const double incoming_dy = target.pose.position.y - start.pose.position.y;
  const double incoming_length = std::hypot(incoming_dx, incoming_dy);
  double outgoing_dx = 0.0;
  double outgoing_dy = 0.0;
  double outgoing_length = 0.0;
  if (target_index + 1 < real_goals_.size()) {
    const auto & next_target = real_goals_[target_index + 1];
    outgoing_dx = next_target.pose.position.x - target.pose.position.x;
    outgoing_dy = next_target.pose.position.y - target.pose.position.y;
    outgoing_length = std::hypot(outgoing_dx, outgoing_dy);
  }
  if (incoming_length > kPositionEpsilon && outgoing_length > kPositionEpsilon) {
    const double bisector_x = incoming_dx / incoming_length + outgoing_dx / outgoing_length;
    const double bisector_y = incoming_dy / incoming_length + outgoing_dy / outgoing_length;
    if (std::hypot(bisector_x, bisector_y) > kPositionEpsilon) {
      return std::atan2(bisector_y, bisector_x);
    }
  }
  if (outgoing_length > kPositionEpsilon) {
    return std::atan2(outgoing_dy, outgoing_dx);
  }
  if (incoming_length > kPositionEpsilon) {
    return std::atan2(incoming_dy, incoming_dx);
  }
  return quaternionYaw(start.pose.orientation);
}

bool ComputeFreeHeadingPathAction::endpointMatchesCandidateAndRealGoal(
  const geometry_msgs::msg::PoseStamped & endpoint,
  const geometry_msgs::msg::PoseStamped & candidate,
  const geometry_msgs::msg::PoseStamped & real_goal) const
{
  if (planarDistance(endpoint, candidate) > goal_position_tolerance_ ||
    planarDistance(endpoint, real_goal) > goal_position_tolerance_)
  {
    return false;
  }
  if (isZeroQuaternion(real_goal.pose.orientation)) {
    return true;
  }
  if (!isUnitQuaternion(endpoint.pose.orientation)) {
    return false;
  }
  // Planner queries run in the virtual frame for reverse motion, so compare
  // against the authored task yaw after applying the same half-turn used by
  // the reverse path transform.  This closes the semantic yaw contract for
  // intermediate locked goals as well as the terminal goal.
  const double authored_virtual_yaw = quaternionYaw(real_goal.pose.orientation) +
    (reverse_ ? kPi : 0.0);
  return angularDistance(
    quaternionYaw(endpoint.pose.orientation), authored_virtual_yaw) <=
         validation_options_.goal_yaw_tolerance;
}

bool ComputeFreeHeadingPathAction::startCandidateQuery()
{
  if (target_index_ >= real_goals_.size() || candidate_index_ >= candidate_goals_.size() ||
    query_state_ != QueryState::IDLE || active_goal_handle_ || goal_handle_future_.valid() ||
    result_future_.valid() || cancel_future_.valid())
  {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading attempted to overlap planner queries");
    planner_query_failed_ = true;
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now >= search_deadline_ ||
    (through_poses_ && now >= through_search_deadline_))
  {
    RCLCPP_WARN(node_->get_logger(), "Free-heading candidate search budget exhausted");
    search_budget_exhausted_ = true;
    return false;
  }
  if (through_poses_ &&
    through_candidate_query_count_ >= kMaximumThroughCandidateQueries)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading through-poses search reached its %zu-query limit",
      kMaximumThroughCandidateQueries);
    search_budget_exhausted_ = true;
    return false;
  }
  active_virtual_goal_ = candidate_goals_[candidate_index_].pose;
  ComputePathToPose::Goal request;
  request.planner_id = planner_id_;
  request.use_start = true;
  request.start = plannerStartForCandidate();
  request.goal = active_virtual_goal_;
  goal_handle_future_ = planner_client_->async_send_goal(request);
  if (through_poses_) {
    ++through_candidate_query_count_;
  }
  active_goal_handle_.reset();
  query_state_ = QueryState::WAITING_FOR_GOAL_HANDLE;
  query_deadline_ = std::min(now + candidate_timeout_, search_deadline_);
  if (through_poses_) {
    query_deadline_ = std::min(query_deadline_, through_search_deadline_);
  }
  return true;
}

bool ComputeFreeHeadingPathAction::startLookaheadQuery()
{
  if (!pending_candidate_.has_value() ||
    lookahead_candidate_index_ >= lookahead_candidates_.size() ||
    pending_candidate_->path.poses.empty() || query_state_ != QueryState::IDLE ||
    active_goal_handle_ || goal_handle_future_.valid() || result_future_.valid() ||
    cancel_future_.valid())
  {
    if (query_state_ != QueryState::IDLE || active_goal_handle_ ||
      goal_handle_future_.valid() || result_future_.valid() || cancel_future_.valid())
    {
      RCLCPP_ERROR(node_->get_logger(), "Free-heading attempted to overlap planner queries");
      planner_query_failed_ = true;
    }
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now >= search_deadline_ ||
    (through_poses_ && now >= through_search_deadline_))
  {
    RCLCPP_WARN(node_->get_logger(), "Free-heading candidate search budget exhausted");
    search_budget_exhausted_ = true;
    return false;
  }
  const auto & continuation_start = pending_candidate_->path.poses.back();
  active_lookahead_goal_ = lookahead_candidates_[lookahead_candidate_index_].pose;

  ComputePathToPose::Goal request;
  request.planner_id = planner_id_;
  request.use_start = true;
  request.start = continuation_start;
  request.goal = active_lookahead_goal_;
  goal_handle_future_ = planner_client_->async_send_goal(request);
  active_goal_handle_.reset();
  query_state_ = QueryState::WAITING_FOR_LOOKAHEAD_GOAL_HANDLE;
  query_deadline_ = std::min(now + candidate_timeout_, search_deadline_);
  if (through_poses_) {
    query_deadline_ = std::min(query_deadline_, through_search_deadline_);
  }
  return true;
}

BT::NodeStatus ComputeFreeHeadingPathAction::completeCandidate(
  const nav_msgs::msg::Path * candidate_path,
  bool explicit_no_valid_path)
{
  if (candidate_path == nullptr || candidate_path->poses.empty()) {
    if (!explicit_no_valid_path) {
      return failPlannerQuery("candidate result was not an explicit no-valid-path result", false);
    }
    return advanceCandidate();
  }

  const bool has_departure_connector =
    departure_connector_index_ < departure_connectors_.size();
  // A generated P connector is only physically continuous when Smac starts
  // at its exact lattice endpoint.  Unlike an ordinary explicit start, its
  // permissible grid quantization cannot be charged as a virtual join: doing
  // so would require editing the first planner sample and can change a legal
  // tangent into an infeasible turn.
  const auto start_continuity = validatePlannerPathStartContinuity(
    *candidate_path, plannerStartForCandidate(),
    has_departure_connector ? kJoinPositionTolerance : kPlannerPathStartPositionToleranceM,
    has_departure_connector ? kJoinYawTolerance : kPlannerPathStartYawToleranceRad);
  if (!start_continuity.valid) {
    if (has_departure_connector) {
      const auto & requested_start = plannerStartForCandidate();
      const auto & returned_start = candidate_path->poses.front();
      RCLCPP_WARN(
        node_->get_logger(),
        "Rejected P connector lattice handoff for candidate %zu goal %zu: %s "
        "requested_start=(%.4f,%.4f,%.4f) returned_start=(%.4f,%.4f,%.4f)",
        candidate_index_ + 1, target_index_ + 1, start_continuity.reason.c_str(),
        requested_start.pose.position.x, requested_start.pose.position.y,
        quaternionYaw(requested_start.pose.orientation),
        returned_start.pose.position.x, returned_start.pose.position.y,
        quaternionYaw(returned_start.pose.orientation));
      return advanceCandidate();
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "Free-heading planner candidate %zu for goal %zu violates the requested start contract: %s",
      candidate_index_ + 1, target_index_ + 1, start_continuity.reason.c_str());
    return failPlannerQuery("planner path start continuity contract violation", false);
  }
  const double planner_length = plannerPathLengthIncludingStartJoin(
    *candidate_path, start_continuity);
  if (!finite(planner_length)) {
    return failPlannerQuery("planner path has an invalid start-joined length", false);
  }

  nav_msgs::msg::Path candidate_segment;
  std::string connector_reason;
  if (!buildCandidateSegment(*candidate_path, candidate_segment, connector_reason)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Rejected free-heading candidate %zu for goal %zu at P connector handoff: %s",
      candidate_index_ + 1, target_index_ + 1, connector_reason.c_str());
    return advanceCandidate();
  }
  double length = planner_length;
  if (departure_connector_index_ < departure_connectors_.size()) {
    const auto kinematic = validateForwardConnectorPath(
      candidate_segment, virtual_start_, departure_connector_options_);
    if (!kinematic.valid) {
      const auto & connector_end = plannerStartForCandidate();
      const auto & raw_planner_start = candidate_path->poses.front();
      const auto & raw_planner_next = candidate_path->poses.size() > 1U ?
        candidate_path->poses[1U] : raw_planner_start;
      RCLCPP_WARN(
        node_->get_logger(),
        "Rejected free-heading candidate %zu for goal %zu after P connector: %s "
        "connector_end=(%.4f,%.4f,%.4f) raw_start=(%.4f,%.4f,%.4f) "
        "raw_next=(%.4f,%.4f,%.4f) quantization_gap=(%.4f,%.4f)",
        candidate_index_ + 1, target_index_ + 1, kinematic.reason.c_str(),
        connector_end.pose.position.x, connector_end.pose.position.y,
        quaternionYaw(connector_end.pose.orientation),
        raw_planner_start.pose.position.x, raw_planner_start.pose.position.y,
        quaternionYaw(raw_planner_start.pose.orientation),
        raw_planner_next.pose.position.x, raw_planner_next.pose.position.y,
        quaternionYaw(raw_planner_next.pose.orientation),
        start_continuity.join_gap_m, start_continuity.yaw_error_rad);
      return advanceCandidate();
    }
    length = kinematic.length_m;
  }

  const auto & endpoint = candidate_segment.poses.back();
  const bool heading_matches = isUnitQuaternion(endpoint.pose.orientation) &&
    angularDistance(
    quaternionYaw(endpoint.pose.orientation),
    quaternionYaw(active_virtual_goal_.pose.orientation)) <=
    validation_options_.goal_yaw_tolerance;
  if (!heading_matches ||
    !endpointMatchesCandidateAndRealGoal(
      endpoint, active_virtual_goal_, real_goals_[target_index_]))
  {
    return advanceCandidate();
  }

  // P departure paths are built and checked by their dedicated connector
  // contract. Every ordinary forward replan must prove the same controller
  // envelope before it can reach FollowPath, including its sampled yaw
  // deltas. This keeps a discretized terminal heading from failing later in
  // ForwardOnlyRPP after the vehicle has already committed to the route.
  if (!reverse_ && !has_departure_connector) {
    const auto forward_geometry = validateForwardPathGeometry(
      candidate_segment, forwardPathGeometryOptions(validation_options_),
      !isZeroQuaternion(real_goals_[target_index_].pose.orientation));
    if (!forward_geometry.valid) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Rejected generic forward candidate %zu for goal %zu before FollowPath: %s "
        "at segment %zu (observed=%.6f, limit=%.6f)",
        candidate_index_ + 1, target_index_ + 1,
        forward_geometry.reason.c_str(), forward_geometry.segment_index,
        forward_geometry.observed_value, forward_geometry.limit);
      return advanceCandidate();
    }
  }

  if (!hasFreshPlanningCostmaps()) {
    return failPlannerQuery("required costmap became stale during candidate search", false);
  }
  const PathQuality candidate_quality = pathQuality(candidate_segment);
  if (staticKeepoutSweepIsInfrastructureFailure(candidate_quality)) {
    logStaticKeepoutSweepFailure(
      candidate_quality, "candidate", candidate_index_ + 1, target_index_ + 1);
    return failPlannerQuery("static keepout mask is unavailable or malformed", false);
  }
  if (localTrackingSweepIsInfrastructureFailure(candidate_quality)) {
    logLocalTrackingSweepFailure(
      candidate_quality, "candidate", candidate_index_ + 1, target_index_ + 1);
    return failPlannerQuery("filtered local tracking envelope is unavailable or malformed", false);
  }
  if (!hasAcceptableCostmapSample(candidate_quality)) {
    // This check is deliberately independent of Smac's internal collision
    // checker.  Keep failures visible at the default simulator log level: a
    // silent disagreement would otherwise look like an unreachable goal and
    // can hide a footprint or observation-frame regression.
    if (candidate_quality.footprint_sweep_checked && !candidate_quality.footprint_sweep_clear) {
      logFootprintSweepFailure(
        candidate_quality, "candidate", candidate_index_ + 1, target_index_ + 1);
    }
    if (candidate_quality.static_keepout_sweep_checked &&
      !candidate_quality.static_keepout_sweep_clear)
    {
      logStaticKeepoutSweepFailure(
        candidate_quality, "candidate", candidate_index_ + 1, target_index_ + 1);
    }
    if (candidate_quality.local_tracking_sweep_checked &&
      !candidate_quality.local_tracking_sweep_clear)
    {
      logLocalTrackingSweepFailure(
        candidate_quality, "candidate", candidate_index_ + 1, target_index_ + 1);
    }
    if ((!
        candidate_quality.footprint_sweep_checked || candidate_quality.footprint_sweep_clear) &&
      (!candidate_quality.static_keepout_sweep_checked ||
      candidate_quality.static_keepout_sweep_clear) &&
      (!local_tracking_envelope_enabled_ ||
      (candidate_quality.local_tracking_sweep_checked &&
      candidate_quality.local_tracking_sweep_clear)))
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Rejected free-heading candidate %zu for goal %zu by raw-costmap guard: "
        "sample=%s footprint_checked=%s footprint_clear=%s max_cost=%u limit=%u",
        candidate_index_ + 1, target_index_ + 1,
        candidate_quality.has_costmap_sample ? "true" : "false",
        candidate_quality.footprint_sweep_checked ? "true" : "false",
        candidate_quality.footprint_sweep_checked && candidate_quality.footprint_sweep_clear ?
        "true" : "false",
        static_cast<unsigned int>(candidate_quality.maximum_cost),
        static_cast<unsigned int>(maximum_path_cost_));
    }
    return advanceCandidate();
  }

  if (through_poses_) {
    return completeThroughCandidate(candidate_segment, length);
  }

  // A locally shortest free-heading arrival can leave the next corridor
  // impossible to enter. Before committing a transit candidate, prove that
  // its actual Smac endpoint can reach the immediate successor.
  if (isZeroQuaternion(real_goals_[target_index_].pose.orientation) &&
    target_index_ + 1 < real_goals_.size())
  {
    pending_candidate_ = CandidatePlan{length, candidate_segment};
    if (!prepareLookaheadCandidates() || !startLookaheadQuery()) {
      pending_candidate_.reset();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  nav_msgs::msg::Path trial;
  std::string reason;
  if (!buildAndValidateTrial(candidate_segment, nullptr, trial, reason)) {
    RCLCPP_WARN(
      node_->get_logger(), "Rejected free-heading candidate %zu for goal %zu: %s",
      candidate_index_ + 1, target_index_ + 1, reason.c_str());
    return advanceCandidate();
  }
  if (!best_candidate_.has_value() ||
    length < best_candidate_->length_m - kPositionEpsilon)
  {
    best_candidate_ = CandidatePlan{length, candidate_segment};
    best_trial_path_ = std::move(trial);
    best_continuation_.reset();
  }
  return advanceCandidate();
}

BT::NodeStatus ComputeFreeHeadingPathAction::completeLookahead(
  const nav_msgs::msg::Path * candidate_path,
  bool explicit_no_valid_path)
{
  if (!pending_candidate_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading lookahead lost its candidate");
    return BT::NodeStatus::FAILURE;
  }

  if (candidate_path == nullptr && !explicit_no_valid_path) {
    return failPlannerQuery("lookahead result was not an explicit no-valid-path result", true);
  }
  bool continuation_is_valid = false;
  double continuation_length = 0.0;
  if (candidate_path != nullptr && !candidate_path->poses.empty()) {
    if (!hasFreshPlanningCostmaps()) {
      return failPlannerQuery("required costmap became stale during lookahead search", true);
    }
    const auto start_continuity = validatePlannerPathStartContinuity(
      *candidate_path, pending_candidate_->path.poses.back());
    if (!start_continuity.valid) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Free-heading lookahead candidate %zu for goal %zu violates the requested start contract: %s",
        lookahead_candidate_index_ + 1, target_index_ + 2,
        start_continuity.reason.c_str());
      return failPlannerQuery("planner path start continuity contract violation", true);
    }
    continuation_length = plannerPathLengthIncludingStartJoin(
      *candidate_path, start_continuity);
    if (!finite(continuation_length)) {
      return failPlannerQuery("planner lookahead path has an invalid start-joined length", true);
    }
    const auto & endpoint = candidate_path->poses.back();
    continuation_is_valid = isUnitQuaternion(endpoint.pose.orientation) &&
      angularDistance(
      quaternionYaw(endpoint.pose.orientation),
      quaternionYaw(active_lookahead_goal_.pose.orientation)) <=
      validation_options_.goal_yaw_tolerance &&
      endpointMatchesCandidateAndRealGoal(
      endpoint, active_lookahead_goal_, real_goals_[target_index_ + 1]);
    if (continuation_is_valid && !reverse_ && departure_connectors_.empty()) {
      const auto forward_geometry = validateForwardPathGeometry(
        *candidate_path, forwardPathGeometryOptions(validation_options_),
        !isZeroQuaternion(real_goals_[target_index_ + 1U].pose.orientation));
      if (!forward_geometry.valid) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Rejected generic forward lookahead candidate %zu for goal %zu before FollowPath: %s "
          "at segment %zu (observed=%.6f, limit=%.6f)",
          lookahead_candidate_index_ + 1U, target_index_ + 2U,
          forward_geometry.reason.c_str(), forward_geometry.segment_index,
          forward_geometry.observed_value, forward_geometry.limit);
        return advanceLookaheadCandidate();
      }
    }
    const PathQuality continuation_quality = pathQuality(*candidate_path);
    if (staticKeepoutSweepIsInfrastructureFailure(continuation_quality)) {
      logStaticKeepoutSweepFailure(
        continuation_quality, "lookahead", lookahead_candidate_index_ + 1,
        target_index_ + 2);
      return failPlannerQuery("static keepout mask is unavailable or malformed", true);
    }
    if (localTrackingSweepIsInfrastructureFailure(continuation_quality)) {
      logLocalTrackingSweepFailure(
        continuation_quality, "lookahead", lookahead_candidate_index_ + 1,
        target_index_ + 2);
      return failPlannerQuery(
        "filtered local tracking envelope is unavailable or malformed", true);
    }
    if (!hasAcceptableCostmapSample(continuation_quality) &&
      continuation_quality.footprint_sweep_checked &&
      !continuation_quality.footprint_sweep_clear)
    {
      logFootprintSweepFailure(
        continuation_quality, "lookahead", lookahead_candidate_index_ + 1, target_index_ + 2);
    }
    if (!hasAcceptableCostmapSample(continuation_quality) &&
      continuation_quality.static_keepout_sweep_checked &&
      !continuation_quality.static_keepout_sweep_clear)
    {
      logStaticKeepoutSweepFailure(
        continuation_quality, "lookahead", lookahead_candidate_index_ + 1,
        target_index_ + 2);
    }
    if (!hasAcceptableCostmapSample(continuation_quality) &&
      continuation_quality.local_tracking_sweep_checked &&
      !continuation_quality.local_tracking_sweep_clear)
    {
      logLocalTrackingSweepFailure(
        continuation_quality, "lookahead", lookahead_candidate_index_ + 1,
        target_index_ + 2);
    }
    continuation_is_valid = continuation_is_valid &&
      hasAcceptableCostmapSample(continuation_quality);
  }

  nav_msgs::msg::Path trial;
  std::string reason;
  if (continuation_is_valid &&
    buildAndValidateTrial(
      pending_candidate_->path, candidate_path, trial, reason))
  {
    const double score = pending_candidate_->length_m + continuation_length;
    if (!best_candidate_.has_value() ||
      score < best_candidate_->length_m - kPositionEpsilon)
    {
      best_candidate_ = CandidatePlan{score, pending_candidate_->path};
      best_trial_path_ = std::move(trial);
      best_continuation_ = *candidate_path;
    }
  }
  if (continuation_is_valid && !reason.empty()) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Rejected free-heading candidate %zu for goal %zu: %s",
      candidate_index_ + 1, target_index_ + 1, reason.c_str());
  }
  return advanceLookaheadCandidate();
}

BT::NodeStatus ComputeFreeHeadingPathAction::completeThroughCandidate(
  const nav_msgs::msg::Path & candidate_path,
  double edge_length_m)
{
  nav_msgs::msg::Path trial;
  std::string reason;
  if (!buildAndValidateTrial(candidate_path, nullptr, trial, reason)) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Rejected through-poses candidate %zu for goal %zu: %s",
      candidate_index_ + 1, target_index_ + 1, reason.c_str());
    return advanceThroughCandidate();
  }

  ThroughSearchFrame frame;
  frame.target_index = target_index_;
  frame.candidates = candidate_goals_;
  frame.selected_candidate_index = candidate_index_;
  frame.path_before = virtual_path_;
  frame.start_before = virtual_start_;
  frame.edge_quality = pathQuality(candidate_path);
  frame.edge_quality.length_m = edge_length_m;
  through_search_frames_.push_back(std::move(frame));

  virtual_path_ = std::move(trial);
  virtual_start_ = virtual_path_.poses.back();
  ++target_index_;
  if (target_index_ == real_goals_.size()) {
    return completeThroughPath();
  }
  if (!prepareTargetCandidates() || !startCandidateQuery()) {
    clearPathOutput();
    if (best_through_path_.has_value()) {
      return publishBestThroughPath();
    }
    if (!search_budget_exhausted_) {
      planner_query_failed_ = true;
    }
    setRecoveryEligible(true);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeFreeHeadingPathAction::completeThroughPath()
{
  if (!hasFreshPlanningCostmaps()) {
    return failPlannerQuery("required costmap became stale during through-poses search", false);
  }
  const auto start_continuity = validatePlannerPathStartContinuity(
    virtual_path_, planning_virtual_start_);
  if (!start_continuity.valid) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Free-heading through-poses chain violates the requested start contract: %s",
      start_continuity.reason.c_str());
    return failPlannerQuery("planner path start continuity contract violation", false);
  }
  const double chain_length = plannerPathLengthIncludingStartJoin(
    virtual_path_, start_continuity);
  if (!finite(chain_length)) {
    return failPlannerQuery("planner through-poses path has an invalid start-joined length", false);
  }
  PathQuality quality = pathQuality(virtual_path_);
  quality.length_m = chain_length;
  if (staticKeepoutSweepIsInfrastructureFailure(quality)) {
    logStaticKeepoutSweepFailure(
      quality, "through_chain", through_complete_path_count_ + 1, real_goals_.size());
    return failPlannerQuery("static keepout mask is unavailable or malformed", false);
  }
  if (localTrackingSweepIsInfrastructureFailure(quality)) {
    logLocalTrackingSweepFailure(
      quality, "through_chain", through_complete_path_count_ + 1, real_goals_.size());
    return failPlannerQuery("filtered local tracking envelope is unavailable or malformed", false);
  }
  ++through_complete_path_count_;
  std::string validation_reason;
  const bool chain_is_valid = hasAcceptableCostmapSample(quality) &&
    validateReverseTrial(virtual_path_, validation_reason);
  if (!chain_is_valid) {
    if (quality.footprint_sweep_checked && !quality.footprint_sweep_clear) {
      logFootprintSweepFailure(
        quality, "through_chain", through_complete_path_count_, real_goals_.size());
    }
    if (quality.static_keepout_sweep_checked && !quality.static_keepout_sweep_clear) {
      logStaticKeepoutSweepFailure(
        quality, "through_chain", through_complete_path_count_, real_goals_.size());
    }
    if (quality.local_tracking_sweep_checked && !quality.local_tracking_sweep_clear) {
      logLocalTrackingSweepFailure(
        quality, "through_chain", through_complete_path_count_, real_goals_.size());
    }
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Rejected free-heading through-poses chain %zu after full validation: %s",
      through_complete_path_count_,
      validation_reason.empty() ? "costmap_or_footprint" : validation_reason.c_str());
  }
  // The full chain has passed costmap/footprint and reverse-kinematic
  // validation. Rank those Nav2 candidates by length; do not impose a second
  // hand-authored detour ratio on top of the planner's result.
  if (chain_is_valid &&
    (!best_through_quality_.has_value() ||
    quality.length_m < best_through_quality_->length_m - kPositionEpsilon))
  {
    best_through_path_ = virtual_path_;
    best_through_quality_ = quality;
  }

  const bool solution_limit_reached =
    through_complete_path_count_ >= through_solution_limit_;
  const bool search_budget_exhausted =
    std::chrono::steady_clock::now() >= through_search_deadline_ ||
    through_candidate_query_count_ >= kMaximumThroughCandidateQueries;
  if (search_budget_exhausted) {
    search_budget_exhausted_ = true;
  }
  if (solution_limit_reached) {
    // A bounded solution sample is not proof that every heading chain is
    // unreachable.  It may publish a validated best path, but may not arm a
    // physical retreat when no valid chain was sampled.
    search_budget_exhausted_ = true;
  }
  if (solution_limit_reached || search_budget_exhausted) {
    return publishBestThroughPath();
  }

  const auto next_frame = highestRiskThroughFrame();
  if (!next_frame.has_value()) {
    if (!best_through_path_.has_value()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No free-heading through-poses chain passed Nav2 collision and kinematic validation");
      setRecoveryEligible(true);
      clearPathOutput();
      return BT::NodeStatus::FAILURE;
    }
    return publishBestThroughPath();
  }
  return backtrackThroughCandidate(next_frame);
}

BT::NodeStatus ComputeFreeHeadingPathAction::publishBestThroughPath()
{
  if (!best_through_path_.has_value() || !best_through_quality_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No complete free-heading through-poses path to publish");
    setRecoveryEligible(true);
    clearPathOutput();
    return BT::NodeStatus::FAILURE;
  }

  virtual_path_ = *best_through_path_;
  RCLCPP_INFO(
    node_->get_logger(),
    "Selected free-heading chain %zu/%zu: max_cost=%u mean_cost=%.1f max_curvature=%.3f length=%.3f",
    through_complete_path_count_, through_solution_limit_,
    static_cast<unsigned int>(best_through_quality_->maximum_cost),
    best_through_quality_->mean_cost, best_through_quality_->maximum_curvature,
    best_through_quality_->length_m);
  through_search_frames_.clear();
  return finishPath();
}

BT::NodeStatus ComputeFreeHeadingPathAction::advanceCandidate()
{
  if (through_poses_) {
    return advanceThroughCandidate();
  }
  ++candidate_index_;
  if (candidate_index_ < candidate_goals_.size()) {
    if (!startCandidateQuery()) {
      if (std::chrono::steady_clock::now() >= search_deadline_) {
        return commitBestCandidate();
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!through_poses_ &&
    departure_connector_index_ + 1U < departure_connectors_.size())
  {
    return advanceDepartureConnector();
  }
  return commitBestCandidate();
}

BT::NodeStatus ComputeFreeHeadingPathAction::advanceDepartureConnector()
{
  if (departure_connector_index_ + 1U >= departure_connectors_.size()) {
    return commitBestCandidate();
  }
  ++departure_connector_index_;
  if (!prepareTargetCandidates(false) || !startCandidateQuery()) {
    if (std::chrono::steady_clock::now() >= search_deadline_) {
      return commitBestCandidate();
    }
    planner_query_failed_ = true;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeFreeHeadingPathAction::advanceThroughCandidate()
{
  ++candidate_index_;
  if (candidate_index_ < candidate_goals_.size()) {
    if (!startCandidateQuery()) {
      clearPathOutput();
      if (best_through_path_.has_value()) {
        return publishBestThroughPath();
      }
      if (!search_budget_exhausted_) {
        planner_query_failed_ = true;
      }
      setRecoveryEligible(true);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  return backtrackThroughCandidate();
}

BT::NodeStatus ComputeFreeHeadingPathAction::backtrackThroughCandidate(
  std::optional<std::size_t> preferred_frame_index)
{
  std::optional<std::size_t> frame_index;
  if (preferred_frame_index.has_value() &&
    *preferred_frame_index < through_search_frames_.size() &&
    frameHasAlternative(through_search_frames_[*preferred_frame_index]))
  {
    frame_index = preferred_frame_index;
  }
  if (!frame_index.has_value()) {
    for (std::size_t index = through_search_frames_.size(); index > 0U; --index) {
      const std::size_t candidate_index = index - 1U;
      if (frameHasAlternative(through_search_frames_[candidate_index])) {
        frame_index = candidate_index;
        break;
      }
    }
  }

  if (!frame_index.has_value()) {
    if (best_through_path_.has_value()) {
      return publishBestThroughPath();
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "No complete kinematically continuous heading chain found for %zu through-poses goals",
      real_goals_.size());
    virtual_path_ = nav_msgs::msg::Path();
    setRecoveryEligible(true);
    clearPathOutput();
    return BT::NodeStatus::FAILURE;
  }

  ThroughSearchFrame frame = std::move(through_search_frames_[*frame_index]);
  through_search_frames_.resize(*frame_index);
  target_index_ = frame.target_index;
  candidate_goals_ = std::move(frame.candidates);
  candidate_index_ = frame.selected_candidate_index;
  virtual_path_ = std::move(frame.path_before);
  virtual_start_ = frame.start_before;

  // A returned-to depth gets its own short decision window, while the whole
  // route remains bounded by through_search_deadline_ and the query cap.
  search_deadline_ = std::chrono::steady_clock::now() + search_budget_;
  const BT::NodeStatus status = advanceThroughCandidate();
  if (status == BT::NodeStatus::FAILURE && best_through_path_.has_value()) {
    return publishBestThroughPath();
  }
  return status;
}

bool ComputeFreeHeadingPathAction::frameHasAlternative(
  const ThroughSearchFrame & frame) const
{
  return frame.target_index < real_goals_.size() &&
         frame.selected_candidate_index + 1U < frame.candidates.size();
}

std::optional<std::size_t> ComputeFreeHeadingPathAction::highestRiskThroughFrame() const
{
  std::optional<std::size_t> selected_index;
  for (std::size_t index = 0; index < through_search_frames_.size(); ++index) {
    const auto & frame = through_search_frames_[index];
    if (!frameHasAlternative(frame)) {
      continue;
    }
    if (!selected_index.has_value()) {
      selected_index = index;
      continue;
    }
    const auto & selected = through_search_frames_[*selected_index];
    if (betterPathQuality(selected.edge_quality, frame.edge_quality) ||
      (!betterPathQuality(frame.edge_quality, selected.edge_quality) &&
      frame.target_index < selected.target_index))
    {
      selected_index = index;
    }
  }
  return selected_index;
}

ComputeFreeHeadingPathAction::PathQuality
ComputeFreeHeadingPathAction::pathQuality(const nav_msgs::msg::Path & path) const
{
  PathQuality quality;
  quality.length_m = pathLength(path);

  for (std::size_t index = 1; index + 1U < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U];
    const auto & current = path.poses[index];
    const auto & next = path.poses[index + 1U];
    const double first_length = planarDistance(previous, current);
    const double second_length = planarDistance(current, next);
    const double chord_length = planarDistance(previous, next);
    if (first_length <= kPositionEpsilon || second_length <= kPositionEpsilon ||
      chord_length <= kPositionEpsilon)
    {
      continue;
    }
    const double first_x = current.pose.position.x - previous.pose.position.x;
    const double first_y = current.pose.position.y - previous.pose.position.y;
    const double chord_x = next.pose.position.x - previous.pose.position.x;
    const double chord_y = next.pose.position.y - previous.pose.position.y;
    const double curvature = 2.0 * std::abs(first_x * chord_y - first_y * chord_x) /
      (first_length * second_length * chord_length);
    if (!finite(curvature)) {
      continue;
    }
    quality.maximum_curvature = std::max(quality.maximum_curvature, curvature);
    quality.accumulated_curvature += curvature * (first_length + second_length) * 0.5;
  }

  nav2_msgs::msg::Costmap::SharedPtr costmap;
  nav2_msgs::msg::Costmap::SharedPtr local_tracking_costmap;
  nav_msgs::msg::OccupancyGrid::SharedPtr keepout_mask;
  CostmapSampleFreshness freshness = CostmapSampleFreshness::kMissing;
  CostmapSampleFreshness local_tracking_freshness = CostmapSampleFreshness::kMissing;
  {
    std::lock_guard<std::mutex> lock(global_costmap_mutex_);
    costmap = global_costmap_;
    local_tracking_costmap = local_filtered_costmap_;
    keepout_mask = static_keepout_mask_;
    const CostmapSample sample{
      global_costmap_, global_costmap_received_at_, global_costmap_sequence_,
      global_costmap_stamp_ns_};
    quality.costmap_stamp_ns = global_costmap_stamp_ns_;
    quality.costmap_sequence = global_costmap_sequence_;
    freshness = costmapSampleFreshness(
      sample, global_frame_, costmap_max_age_, node_->get_clock()->now(),
      std::chrono::steady_clock::now());
    const CostmapSample local_tracking_sample{
      local_filtered_costmap_, local_filtered_costmap_received_at_,
      local_filtered_costmap_sequence_, local_filtered_costmap_stamp_ns_};
    quality.local_tracking_costmap_stamp_ns = local_filtered_costmap_stamp_ns_;
    quality.local_tracking_costmap_sequence = local_filtered_costmap_sequence_;
    local_tracking_freshness = costmapSampleFreshness(
      local_tracking_sample, global_frame_, costmap_max_age_, node_->get_clock()->now(),
      std::chrono::steady_clock::now());
  }
  if (freshness != CostmapSampleFreshness::kFresh || !costmap)
  {
    return quality;
  }
  quality.costmap_resolution_m = static_cast<double>(costmap->metadata.resolution);
  quality.costmap_frame = costmap->header.frame_id;
  quality.local_tracking_costmap_fresh =
    local_tracking_freshness == CostmapSampleFreshness::kFresh &&
    local_tracking_costmap != nullptr;
  if (quality.local_tracking_costmap_fresh) {
    quality.local_tracking_costmap_resolution_m =
      static_cast<double>(local_tracking_costmap->metadata.resolution);
    quality.local_tracking_costmap_frame = local_tracking_costmap->header.frame_id;
  }

  // A path centreline can clear a lethal cell while the Ackermann body clips
  // it during a minimum-radius arc. The virtual reverse frame is a pi yaw
  // shift, so the configured pi-symmetric rectangular footprint is unchanged.
  quality.footprint_sweep_checked = true;
  quality.footprint_sweep_result = costmapFootprintPathSweep(
    path, *costmap, footprint_sweep_options_, &quality.footprint_sweep_diagnostic);
  quality.footprint_sweep_clear =
    quality.footprint_sweep_result == CostmapFootprintSweepResult::kClear;
  if (!quality.footprint_sweep_clear &&
    quality.footprint_sweep_diagnostic.has_blocking_cell)
  {
    quality.footprint_collision_source = keepoutCollisionSourceName(
      keepoutMaskCellStateAt(
        keepout_mask.get(), global_frame_,
        quality.footprint_sweep_diagnostic.blocking_cell_world_x,
        quality.footprint_sweep_diagnostic.blocking_cell_world_y));
  } else if (!quality.footprint_sweep_clear) {
    quality.footprint_collision_source = "not_a_lethal_costmap_cell";
  }

  // The raw costmap is deliberately captured before Nav2's filter pipeline
  // so it can remain a fresh sensor-fusion witness for retreat.  That means
  // an occupied KeepoutFilter cell cannot be inferred from it.  Apply the
  // same continuous padded-body sweep to the static mask independently; the
  // finite mask bounds also forbid planner shortcuts outside the field.
  quality.static_keepout_sweep_result = staticKeepoutMaskFootprintPathSweep(
    keepout_mask.get(), global_frame_, path, footprint_sweep_options_,
    &quality.static_keepout_sweep_diagnostic);
  quality.static_keepout_sweep_checked =
    quality.static_keepout_sweep_result != StaticKeepoutMaskSweepResult::kNoMask;
  quality.static_keepout_sweep_clear =
    quality.static_keepout_sweep_result == StaticKeepoutMaskSweepResult::kNoMask ||
    quality.static_keepout_sweep_result == StaticKeepoutMaskSweepResult::kClear;

  if (local_tracking_envelope_enabled_) {
    if (!quality.local_tracking_costmap_fresh) {
      return quality;
    }
    std::string active_lateral_profile = kForwardPathLateralProfileSymmetric;
    if (local_tracking_lateral_profile_ ==
      kForwardPathLateralProfilePDepartureSouthV1 && departure_connector_enabled_)
    {
      // prepareDepartureConnectors() validates a generated connector before
      // adding it to departure_connectors_. The strict path matcher below is
      // the authority for this P-only profile; requiring a stored connector
      // here would incorrectly force that first validation back to symmetric.
      const auto match = forwardPathLateralProfileMatchesPlan(
        local_tracking_lateral_profile_, path, local_tracking_lateral_profile_start_);
      if (match == ForwardPathLateralProfilePathMatch::kInvalid) {
        quality.local_tracking_lateral_profile = local_tracking_lateral_profile_;
        quality.local_tracking_lateral_profile_active = true;
        quality.local_tracking_sweep_checked = true;
        quality.local_tracking_sweep_result = CostmapFootprintSweepResult::kInvalidInput;
        return quality;
      }
      if (match == ForwardPathLateralProfilePathMatch::kMatches) {
        active_lateral_profile = local_tracking_lateral_profile_;
        quality.local_tracking_lateral_profile_active = true;
      }
    }
    quality.local_tracking_lateral_profile = active_lateral_profile;
    CostmapFootprintSweepOptions local_options = footprint_sweep_options_;
    local_options.lethal_cost_threshold = local_tracking_lethal_cost_;
    const double local_horizon = std::min(
      local_tracking_horizon_m_, std::max(0.0, quality.length_m));
    const auto local_result = localCostmapTrackingEnvelopeSweep(
      path, *local_tracking_costmap, local_options, local_horizon,
      active_lateral_profile, local_tracking_cross_track_error_m_);
    quality.local_tracking_sweep_checked = true;
    quality.local_tracking_sweep_result = local_result.sweep_result;
    quality.local_tracking_sweep_diagnostic = local_result.diagnostic;
    quality.local_tracking_requested_horizon_m = local_result.requested_horizon_m;
    quality.local_tracking_covered_horizon_m = local_result.covered_horizon_m;
    quality.local_tracking_sweep_clear = local_result.horizon_covered &&
      local_result.sweep_result == CostmapFootprintSweepResult::kClear;
  }

  std::uint64_t accumulated_cost = 0U;
  std::size_t sampled_cells = 0U;
  const double sampling_step = std::max(
    static_cast<double>(costmap->metadata.resolution) * 0.5, 0.02);
  const auto sample = [this, &costmap, &quality, &accumulated_cost, &sampled_cells](
      double x, double y) {
      const auto cost = costAt(*costmap, x, y);
      if (!cost.has_value()) {
        return;
      }
      quality.maximum_cost = std::max(quality.maximum_cost, *cost);
      accumulated_cost += *cost;
      ++sampled_cells;
    };

  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & current = path.poses[index];
    if (index + 1U == path.poses.size()) {
      sample(current.pose.position.x, current.pose.position.y);
      continue;
    }
    const auto & next = path.poses[index + 1U];
    const double segment_length = planarDistance(current, next);
    const std::size_t steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(segment_length / sampling_step)));
    for (std::size_t step = 0; step < steps; ++step) {
      const double fraction = static_cast<double>(step) / static_cast<double>(steps);
      sample(
        current.pose.position.x +
        (next.pose.position.x - current.pose.position.x) * fraction,
        current.pose.position.y +
        (next.pose.position.y - current.pose.position.y) * fraction);
    }
  }
  if (sampled_cells > 0U) {
    quality.has_costmap_sample = true;
    quality.mean_cost = static_cast<double>(accumulated_cost) /
      static_cast<double>(sampled_cells);
  }
  return quality;
}

bool ComputeFreeHeadingPathAction::betterPathQuality(
  const PathQuality & first, const PathQuality & second) const
{
  // This is intentionally a risk-first ordering used only to decide which
  // ThroughPoses frame should be backtracked next. Complete chains are ranked
  // by length in completeThroughPath() after validation.
  if (first.has_costmap_sample != second.has_costmap_sample) {
    return first.has_costmap_sample;
  }
  if (first.has_costmap_sample) {
    if (first.maximum_cost != second.maximum_cost) {
      return first.maximum_cost < second.maximum_cost;
    }
    if (std::abs(first.mean_cost - second.mean_cost) > 0.5) {
      return first.mean_cost < second.mean_cost;
    }
  }
  // Every published candidate has already passed the Ackermann curvature
  // validation.  Among paths with comparable cost, prefer distance before
  // residual curvature preference: a lower-curvature long loop is not a
  // better transit path through a corridor than a shorter feasible one.
  if (std::abs(first.length_m - second.length_m) > 1.0e-3) {
    return first.length_m < second.length_m;
  }
  if (std::abs(first.maximum_curvature - second.maximum_curvature) > 1.0e-3) {
    return first.maximum_curvature < second.maximum_curvature;
  }
  if (std::abs(first.accumulated_curvature - second.accumulated_curvature) > 1.0e-3) {
    return first.accumulated_curvature < second.accumulated_curvature;
  }
  return false;
}

std::optional<std::uint8_t> ComputeFreeHeadingPathAction::costAt(
  const nav2_msgs::msg::Costmap & costmap, double x, double y) const
{
  if (!finite(x) || !finite(y) || costmap.metadata.resolution <= 0.0f ||
    costmap.metadata.size_x == 0U || costmap.metadata.size_y == 0U)
  {
    return std::nullopt;
  }
  const std::size_t expected_size = static_cast<std::size_t>(costmap.metadata.size_x) *
    static_cast<std::size_t>(costmap.metadata.size_y);
  if (costmap.data.size() < expected_size) {
    return std::nullopt;
  }
  const double origin_yaw = quaternionYaw(costmap.metadata.origin.orientation);
  const double dx = x - costmap.metadata.origin.position.x;
  const double dy = y - costmap.metadata.origin.position.y;
  const double local_x = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
  const double local_y = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  if (local_x < 0.0 || local_y < 0.0) {
    return std::uint8_t{255U};
  }
  const std::size_t map_x = static_cast<std::size_t>(std::floor(local_x / resolution));
  const std::size_t map_y = static_cast<std::size_t>(std::floor(local_y / resolution));
  if (map_x >= costmap.metadata.size_x || map_y >= costmap.metadata.size_y) {
    return std::uint8_t{255U};
  }
  return costmap.data[map_y * static_cast<std::size_t>(costmap.metadata.size_x) + map_x];
}

bool ComputeFreeHeadingPathAction::configureKeepoutMaskSubscription(const std::string & topic)
{
  if (topic == static_keepout_mask_topic_) {
    return true;
  }
  static_keepout_mask_subscription_.reset();
  {
    std::lock_guard<std::mutex> lock(global_costmap_mutex_);
    static_keepout_mask_.reset();
  }
  static_keepout_mask_topic_ = topic;
  if (topic.empty()) {
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
      node_->get_logger(), "Free-heading sweep cannot subscribe to keepout mask '%s': %s",
      topic.c_str(), error.what());
    return false;
  }
  return true;
}

void ComputeFreeHeadingPathAction::updateGlobalCostmap(
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
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
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
  ++global_costmap_sequence_;
}

void ComputeFreeHeadingPathAction::updateLocalCostmap(
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
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
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
  ++local_costmap_sequence_;
}

void ComputeFreeHeadingPathAction::updateLocalFilteredCostmap(
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  if (!costmap) {
    return;
  }
  auto converted_costmap = localCostmapTrackingOccupancyGridToCostmap(*costmap);
  if (!converted_costmap.has_value()) {
    RCLCPP_WARN(
      node_->get_logger(), "Ignoring malformed filtered local OccupancyGrid");
    return;
  }
  const auto stamp_ns = costmapSourceStampNanoseconds(*converted_costmap);
  if (!stamp_ns.has_value()) {
    RCLCPP_WARN(
      node_->get_logger(), "Ignoring filtered local costmap with an invalid source stamp");
    return;
  }
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  if (*stamp_ns <= local_filtered_costmap_stamp_ns_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Ignoring non-monotonic filtered local costmap stamp %lld (last=%lld)",
      static_cast<long long>(*stamp_ns),
      static_cast<long long>(local_filtered_costmap_stamp_ns_));
    return;
  }
  local_filtered_costmap_ = std::make_shared<nav2_msgs::msg::Costmap>(
    std::move(*converted_costmap));
  local_filtered_costmap_received_at_ = std::chrono::steady_clock::now();
  local_filtered_costmap_stamp_ns_ = *stamp_ns;
  ++local_filtered_costmap_sequence_;
}

void ComputeFreeHeadingPathAction::updateStaticKeepoutMask(
  nav_msgs::msg::OccupancyGrid::SharedPtr mask)
{
  if (!mask) {
    return;
  }
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  static_keepout_mask_ = std::move(mask);
}

bool ComputeFreeHeadingPathAction::hasFreshGlobalCostmap() const
{
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  const CostmapSample sample{
    global_costmap_, global_costmap_received_at_, global_costmap_sequence_,
    global_costmap_stamp_ns_};
  return costmapSampleFreshness(
    sample, global_frame_, costmap_max_age_, node_->get_clock()->now(),
    std::chrono::steady_clock::now()) == CostmapSampleFreshness::kFresh;
}

bool ComputeFreeHeadingPathAction::hasFreshLocalFilteredCostmap() const
{
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  const CostmapSample sample{
    local_filtered_costmap_, local_filtered_costmap_received_at_,
    local_filtered_costmap_sequence_, local_filtered_costmap_stamp_ns_};
  return costmapSampleFreshness(
    sample, global_frame_, costmap_max_age_, node_->get_clock()->now(),
    std::chrono::steady_clock::now()) == CostmapSampleFreshness::kFresh;
}

bool ComputeFreeHeadingPathAction::hasStaticKeepoutMask() const
{
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  return static_keepout_mask_ != nullptr;
}

bool ComputeFreeHeadingPathAction::pDepartureStaticKeepoutMaskRequired() const
{
  return departure_connector_enabled_ &&
         local_tracking_lateral_profile_ == kForwardPathLateralProfilePDepartureSouthV1;
}

bool ComputeFreeHeadingPathAction::hasFreshPlanningCostmaps() const
{
  return hasFreshGlobalCostmap() &&
         (!local_tracking_envelope_enabled_ || hasFreshLocalFilteredCostmap()) &&
         (!pDepartureStaticKeepoutMaskRequired() || hasStaticKeepoutMask());
}

bool ComputeFreeHeadingPathAction::hasAcceptableCostmapSample(
  const PathQuality & quality) const
{
  return quality.has_costmap_sample && quality.footprint_sweep_checked &&
         quality.footprint_sweep_clear &&
         quality.static_keepout_sweep_clear &&
         (!pDepartureStaticKeepoutMaskRequired() || quality.static_keepout_sweep_checked) &&
         (!local_tracking_envelope_enabled_ ||
         (quality.local_tracking_costmap_fresh && quality.local_tracking_sweep_checked &&
         quality.local_tracking_sweep_clear)) &&
         quality.maximum_cost <= maximum_path_cost_;
}

bool ComputeFreeHeadingPathAction::staticKeepoutSweepIsInfrastructureFailure(
  const PathQuality & quality) const
{
  return quality.static_keepout_sweep_result == StaticKeepoutMaskSweepResult::kWrongFrame ||
         quality.static_keepout_sweep_result == StaticKeepoutMaskSweepResult::kMalformed ||
         (pDepartureStaticKeepoutMaskRequired() &&
         quality.static_keepout_sweep_result == StaticKeepoutMaskSweepResult::kNoMask);
}

bool ComputeFreeHeadingPathAction::localTrackingSweepIsInfrastructureFailure(
  const PathQuality & quality) const
{
  return local_tracking_envelope_enabled_ &&
         (!quality.local_tracking_costmap_fresh ||
         (quality.local_tracking_sweep_checked &&
         quality.local_tracking_sweep_result == CostmapFootprintSweepResult::kInvalidInput));
}

void ComputeFreeHeadingPathAction::logFootprintSweepFailure(
  const PathQuality & quality,
  const char * scope,
  std::size_t identifier,
  std::size_t goal_index) const
{
  if (!quality.footprint_sweep_checked || quality.footprint_sweep_clear) {
    return;
  }
  const auto & diagnostic = quality.footprint_sweep_diagnostic;
  const double pose_x = diagnostic.has_sample_pose ? diagnostic.sample_pose.pose.position.x : 0.0;
  const double pose_y = diagnostic.has_sample_pose ? diagnostic.sample_pose.pose.position.y : 0.0;
  const double pose_yaw = diagnostic.has_sample_pose ?
    tf2::getYaw(diagnostic.sample_pose.pose.orientation) : 0.0;
  if (diagnostic.has_blocking_cell) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading raw-costmap footprint rejection scope=%s id=%zu goal=%zu result=%s "
      "planner_path_provenance=unavailable_from_ComputePathToPose "
      "costmap_topic=/global_costmap/costmap_raw frame=%s stamp_ns=%lld sequence=%llu "
      "resolution_m=%.4f sample_pose=(%.4f,%.4f,%.4f) path_segment=%zu->%zu "
      "interpolation=%zu/%zu fraction=%.6f blocking_cell=(%zu,%zu) "
      "blocking_cell_world=(%.4f,%.4f) cost=%u cost_semantic=%s obstacle_source=%s "
      "per_layer_attribution=unavailable_from_nav2_msgs_Costmap max_centerline_cost=%u "
      "centerline_limit=%u",
      scope, identifier, goal_index,
      costmapFootprintSweepResultName(diagnostic.result),
      quality.costmap_frame.c_str(), static_cast<long long>(quality.costmap_stamp_ns),
      static_cast<unsigned long long>(quality.costmap_sequence), quality.costmap_resolution_m,
      pose_x, pose_y, pose_yaw,
      diagnostic.segment_start_pose_index, diagnostic.segment_end_pose_index,
      diagnostic.segment_sample_index, diagnostic.segment_sample_count,
      diagnostic.segment_fraction,
      diagnostic.blocking_cell_x, diagnostic.blocking_cell_y,
      diagnostic.blocking_cell_world_x, diagnostic.blocking_cell_world_y,
      static_cast<unsigned int>(diagnostic.blocking_cell_cost),
      costmapFootprintSweepCellCostName(diagnostic.blocking_cell_cost),
      quality.footprint_collision_source.c_str(),
      static_cast<unsigned int>(quality.maximum_cost),
      static_cast<unsigned int>(maximum_path_cost_));
    return;
  }
  if (diagnostic.has_boundary_point) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading raw-costmap footprint rejection scope=%s id=%zu goal=%zu result=%s "
      "planner_path_provenance=unavailable_from_ComputePathToPose "
      "costmap_topic=/global_costmap/costmap_raw frame=%s stamp_ns=%lld sequence=%llu "
      "resolution_m=%.4f sample_pose=(%.4f,%.4f,%.4f) path_segment=%zu->%zu "
      "interpolation=%zu/%zu fraction=%.6f boundary_world=(%.4f,%.4f) "
      "obstacle_source=%s",
      scope, identifier, goal_index,
      costmapFootprintSweepResultName(diagnostic.result),
      quality.costmap_frame.c_str(), static_cast<long long>(quality.costmap_stamp_ns),
      static_cast<unsigned long long>(quality.costmap_sequence), quality.costmap_resolution_m,
      pose_x, pose_y, pose_yaw,
      diagnostic.segment_start_pose_index, diagnostic.segment_end_pose_index,
      diagnostic.segment_sample_index, diagnostic.segment_sample_count,
      diagnostic.segment_fraction,
      diagnostic.boundary_world_x, diagnostic.boundary_world_y,
      quality.footprint_collision_source.c_str());
    return;
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "Free-heading raw-costmap footprint rejection scope=%s id=%zu goal=%zu result=%s "
    "planner_path_provenance=unavailable_from_ComputePathToPose "
    "costmap_topic=/global_costmap/costmap_raw frame=%s stamp_ns=%lld sequence=%llu "
    "resolution_m=%.4f sample_pose=(%.4f,%.4f,%.4f) path_segment=%zu->%zu "
    "interpolation=%zu/%zu fraction=%.6f obstacle_source=%s",
    scope, identifier, goal_index,
    costmapFootprintSweepResultName(diagnostic.result),
    quality.costmap_frame.c_str(), static_cast<long long>(quality.costmap_stamp_ns),
    static_cast<unsigned long long>(quality.costmap_sequence), quality.costmap_resolution_m,
    pose_x, pose_y, pose_yaw,
    diagnostic.segment_start_pose_index, diagnostic.segment_end_pose_index,
    diagnostic.segment_sample_index, diagnostic.segment_sample_count,
    diagnostic.segment_fraction, quality.footprint_collision_source.c_str());
}

void ComputeFreeHeadingPathAction::logStaticKeepoutSweepFailure(
  const PathQuality & quality,
  const char * scope,
  std::size_t identifier,
  std::size_t goal_index) const
{
  if (quality.static_keepout_sweep_clear) {
    return;
  }
  const auto & diagnostic = quality.static_keepout_sweep_diagnostic;
  const double pose_x = diagnostic.has_sample_pose ? diagnostic.sample_pose.pose.position.x : 0.0;
  const double pose_y = diagnostic.has_sample_pose ? diagnostic.sample_pose.pose.position.y : 0.0;
  const double pose_yaw = diagnostic.has_sample_pose ?
    tf2::getYaw(diagnostic.sample_pose.pose.orientation) : 0.0;
  if (diagnostic.has_blocking_cell) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading static-keepout footprint rejection scope=%s id=%zu goal=%zu "
      "result=%s mask_topic=%s sample_pose=(%.4f,%.4f,%.4f) "
      "path_segment=%zu->%zu interpolation=%zu/%zu fraction=%.6f "
      "blocking_cell=(%zu,%zu) blocking_cell_world=(%.4f,%.4f) occupancy=%u",
      scope, identifier, goal_index,
      staticKeepoutMaskSweepResultName(quality.static_keepout_sweep_result),
      static_keepout_mask_topic_.c_str(), pose_x, pose_y, pose_yaw,
      diagnostic.segment_start_pose_index, diagnostic.segment_end_pose_index,
      diagnostic.segment_sample_index, diagnostic.segment_sample_count,
      diagnostic.segment_fraction, diagnostic.blocking_cell_x, diagnostic.blocking_cell_y,
      diagnostic.blocking_cell_world_x, diagnostic.blocking_cell_world_y,
      static_cast<unsigned int>(diagnostic.blocking_cell_cost));
    return;
  }
  if (diagnostic.has_boundary_point) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading static-keepout footprint rejection scope=%s id=%zu goal=%zu "
      "result=%s mask_topic=%s sample_pose=(%.4f,%.4f,%.4f) "
      "path_segment=%zu->%zu interpolation=%zu/%zu fraction=%.6f "
      "boundary_world=(%.4f,%.4f)",
      scope, identifier, goal_index,
      staticKeepoutMaskSweepResultName(quality.static_keepout_sweep_result),
      static_keepout_mask_topic_.c_str(), pose_x, pose_y, pose_yaw,
      diagnostic.segment_start_pose_index, diagnostic.segment_end_pose_index,
      diagnostic.segment_sample_index, diagnostic.segment_sample_count,
      diagnostic.segment_fraction, diagnostic.boundary_world_x, diagnostic.boundary_world_y);
    return;
  }
  RCLCPP_ERROR(
    node_->get_logger(),
    "Free-heading static-keepout sweep is unusable scope=%s id=%zu goal=%zu result=%s "
    "mask_topic=%s",
    scope, identifier, goal_index,
    staticKeepoutMaskSweepResultName(quality.static_keepout_sweep_result),
    static_keepout_mask_topic_.c_str());
}

void ComputeFreeHeadingPathAction::logLocalTrackingSweepFailure(
  const PathQuality & quality,
  const char * scope,
  std::size_t identifier,
  std::size_t goal_index) const
{
  if (!local_tracking_envelope_enabled_) {
    return;
  }
  if (!quality.local_tracking_costmap_fresh) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Free-heading filtered-local tracking envelope unavailable scope=%s id=%zu goal=%zu "
      "costmap_topic=/local_costmap/costmap",
      scope, identifier, goal_index);
    return;
  }
  if (!quality.local_tracking_sweep_checked || quality.local_tracking_sweep_clear) {
    return;
  }
  const auto & diagnostic = quality.local_tracking_sweep_diagnostic;
  const double pose_x = diagnostic.has_sample_pose ? diagnostic.sample_pose.pose.position.x : 0.0;
  const double pose_y = diagnostic.has_sample_pose ? diagnostic.sample_pose.pose.position.y : 0.0;
  const double pose_yaw = diagnostic.has_sample_pose ?
    tf2::getYaw(diagnostic.sample_pose.pose.orientation) : 0.0;
  if (diagnostic.has_blocking_cell) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading filtered-local tracking envelope rejection scope=%s id=%zu goal=%zu "
      "result=%s lateral_profile=%s lateral_profile_active=%s "
      "costmap_topic=/local_costmap/costmap frame=%s stamp_ns=%lld sequence=%llu "
      "resolution_m=%.4f cross_track_envelope_m=%.3f horizon_m=%.3f covered_m=%.3f "
      "sample_pose=(%.4f,%.4f,%.4f) path_segment=%zu->%zu interpolation=%zu/%zu "
      "fraction=%.6f blocking_cell=(%zu,%zu) blocking_cell_world=(%.4f,%.4f) cost=%u",
      scope, identifier, goal_index,
      costmapFootprintSweepResultName(quality.local_tracking_sweep_result),
      quality.local_tracking_lateral_profile.c_str(),
      quality.local_tracking_lateral_profile_active ? "true" : "false",
      quality.local_tracking_costmap_frame.c_str(),
      static_cast<long long>(quality.local_tracking_costmap_stamp_ns),
      static_cast<unsigned long long>(quality.local_tracking_costmap_sequence),
      quality.local_tracking_costmap_resolution_m, local_tracking_cross_track_error_m_,
      quality.local_tracking_requested_horizon_m, quality.local_tracking_covered_horizon_m,
      pose_x, pose_y, pose_yaw,
      diagnostic.segment_start_pose_index, diagnostic.segment_end_pose_index,
      diagnostic.segment_sample_index, diagnostic.segment_sample_count,
      diagnostic.segment_fraction,
      diagnostic.blocking_cell_x, diagnostic.blocking_cell_y,
      diagnostic.blocking_cell_world_x, diagnostic.blocking_cell_world_y,
      static_cast<unsigned int>(diagnostic.blocking_cell_cost));
    return;
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "Free-heading filtered-local tracking envelope rejection scope=%s id=%zu goal=%zu "
    "result=%s lateral_profile=%s lateral_profile_active=%s "
    "costmap_topic=/local_costmap/costmap frame=%s stamp_ns=%lld sequence=%llu "
    "resolution_m=%.4f cross_track_envelope_m=%.3f horizon_m=%.3f covered_m=%.3f "
    "sample_pose=(%.4f,%.4f,%.4f)",
    scope, identifier, goal_index,
    costmapFootprintSweepResultName(quality.local_tracking_sweep_result),
    quality.local_tracking_lateral_profile.c_str(),
    quality.local_tracking_lateral_profile_active ? "true" : "false",
    quality.local_tracking_costmap_frame.c_str(),
    static_cast<long long>(quality.local_tracking_costmap_stamp_ns),
    static_cast<unsigned long long>(quality.local_tracking_costmap_sequence),
    quality.local_tracking_costmap_resolution_m, local_tracking_cross_track_error_m_,
    quality.local_tracking_requested_horizon_m, quality.local_tracking_covered_horizon_m,
    pose_x, pose_y, pose_yaw);
}

bool ComputeFreeHeadingPathAction::publishCostmapBarrier()
{
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  const CostmapSample global_sample{
    global_costmap_, global_costmap_received_at_, global_costmap_sequence_,
    global_costmap_stamp_ns_};
  const CostmapSample local_sample{
    local_costmap_, local_costmap_received_at_, local_costmap_sequence_,
    local_costmap_stamp_ns_};
  const auto global_freshness = costmapSampleFreshness(
    global_sample, global_frame_, costmap_max_age_, node_->get_clock()->now(),
    std::chrono::steady_clock::now());
  const auto local_freshness = costmapSampleFreshness(
    local_sample, global_frame_, costmap_max_age_, node_->get_clock()->now(),
    std::chrono::steady_clock::now());
  if (global_freshness != CostmapSampleFreshness::kFresh ||
    local_freshness != CostmapSampleFreshness::kFresh)
  {
    setOutput("costmap_stamp_ns", static_cast<std::int64_t>(0));
    setOutput("local_costmap_stamp_ns", static_cast<std::int64_t>(0));
    setOutput("costmap_sequence", static_cast<std::uint64_t>(0U));
    return false;
  }
  setOutput("costmap_stamp_ns", global_costmap_stamp_ns_);
  setOutput("local_costmap_stamp_ns", local_costmap_stamp_ns_);
  setOutput("costmap_sequence", global_costmap_sequence_);
  return true;
}

BT::NodeStatus ComputeFreeHeadingPathAction::failPlannerQuery(
  const char * reason, bool lookahead_query)
{
  planner_query_failed_ = true;
  setRecoveryEligible(false);
  clearPathOutput();
  RCLCPP_ERROR(
    node_->get_logger(),
    "Free-heading %s query failed closed: %s",
    lookahead_query ? "lookahead planner" : "planner", reason);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeFreeHeadingPathAction::advanceLookaheadCandidate()
{
  ++lookahead_candidate_index_;
  if (lookahead_candidate_index_ < lookahead_candidates_.size()) {
    if (!startLookaheadQuery()) {
      pending_candidate_.reset();
      if (std::chrono::steady_clock::now() >= search_deadline_) {
        return advanceCandidate();
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  pending_candidate_.reset();
  return advanceCandidate();
}

BT::NodeStatus ComputeFreeHeadingPathAction::commitBestCandidate()
{
  if (!best_candidate_.has_value() || !best_trial_path_.has_value()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "No kinematically continuous heading found for position-only goal %zu/%zu",
      target_index_ + 1, real_goals_.size());
    setRecoveryEligible(true);
    clearPathOutput();
    return BT::NodeStatus::FAILURE;
  }
  const bool has_continuation = best_continuation_.has_value();
  virtual_path_ = std::move(*best_trial_path_);
  if (virtual_path_.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Selected free-heading path is empty");
    return BT::NodeStatus::FAILURE;
  }
  virtual_start_ = virtual_path_.poses.back();
  target_index_ += has_continuation ? 2 : 1;
  if (target_index_ == real_goals_.size()) {
    return finishPath();
  }
  if (!prepareTargetCandidates() || !startCandidateQuery()) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

bool ComputeFreeHeadingPathAction::appendSegmentChecked(
  nav_msgs::msg::Path & destination,
  const nav_msgs::msg::Path & segment,
  std::string & reason) const
{
  if (segment.poses.empty()) {
    reason = "segment_empty";
    return false;
  }
  if (destination.poses.empty()) {
    destination.header = segment.header;
  } else if (destination.header.frame_id != segment.header.frame_id) {
    reason = "segment_frame_mismatch";
    return false;
  }

  for (std::size_t index = 0; index < segment.poses.size(); ++index) {
    const auto & pose = segment.poses[index];
    if (!isUnitQuaternion(pose.pose.orientation)) {
      reason = "segment_pose_orientation_invalid";
      return false;
    }
    if (destination.poses.empty()) {
      destination.poses.push_back(pose);
      continue;
    }

    const auto & previous = destination.poses.back();
    const double distance = planarDistance(previous, pose);
    if (distance <= kJoinPositionTolerance) {
      if (angularDistance(
          quaternionYaw(previous.pose.orientation),
          quaternionYaw(pose.pose.orientation)) > kJoinYawTolerance)
      {
        reason = index == 0 ? "handoff_yaw_mismatch" : "duplicate_pose_yaw_mismatch";
        return false;
      }
      continue;
    }
    if (index == 0 && !destination.poses.empty()) {
      reason = "handoff_position_mismatch";
      return false;
    }
    if (distance < validation_options_.minimum_segment_length) {
      reason = "segment_too_short";
      return false;
    }
    destination.poses.push_back(pose);
  }
  return true;
}

bool ComputeFreeHeadingPathAction::buildAndValidateTrial(
  const nav_msgs::msg::Path & first,
  const nav_msgs::msg::Path * continuation,
  nav_msgs::msg::Path & trial,
  std::string & reason) const
{
  trial = virtual_path_;
  if (!appendSegmentChecked(trial, first, reason)) {
    return false;
  }
  if (continuation != nullptr && !appendSegmentChecked(trial, *continuation, reason)) {
    return false;
  }
  return validateReverseTrial(trial, reason);
}

bool ComputeFreeHeadingPathAction::validateReverseTrial(
  const nav_msgs::msg::Path & virtual_trial,
  std::string & reason) const
{
  if (!reverse_ || virtual_trial.poses.size() < 2) {
    return true;
  }

  nav_msgs::msg::Path restored = virtual_trial;
  for (auto & pose : restored.poses) {
    geometry_msgs::msg::PoseStamped restored_pose;
    if (!rotatePoseYawByPi(pose, restored_pose)) {
      reason = "reverse_trial_quaternion_invalid";
      return false;
    }
    pose = restored_pose;
  }
  const auto validation = validateReversePath(
    restored, real_start_, restored.poses.back(), validation_options_);
  if (!validation.valid) {
    reason = validation.reason;
    return false;
  }
  return true;
}

BT::NodeStatus ComputeFreeHeadingPathAction::finishPath()
{
  if (virtual_path_.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading path is too short");
    clearPathOutput();
    return BT::NodeStatus::FAILURE;
  }
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_) ||
    planarDistance(current_pose, real_start_) > max_start_drift_m_)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading plan start became stale before publication");
    clearPathOutput();
    return BT::NodeStatus::FAILURE;
  }
  nav_msgs::msg::Path output = virtual_path_;
  if (reverse_) {
    for (auto & pose : output.poses) {
      geometry_msgs::msg::PoseStamped restored;
      if (!rotatePoseYawByPi(pose, restored)) {
        RCLCPP_ERROR(node_->get_logger(), "Reverse free-heading path has an invalid quaternion");
        clearPathOutput();
        return BT::NodeStatus::FAILURE;
      }
      pose = restored;
    }
    geometry_msgs::msg::PoseStamped expected_goal = real_goals_.back();
    if (isZeroQuaternion(expected_goal.pose.orientation)) {
      expected_goal = output.poses.back();
    }
    if (output.poses.size() >= 2) {
      const auto validation = validateReversePath(
        output, real_start_, expected_goal, validation_options_);
      if (!validation.valid) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Rejected reverse free-heading path: %s at segment %zu (observed=%.6f, limit=%.6f)",
          validation.reason.c_str(), validation.segment_index,
          validation.observed_value, validation.limit);
        clearPathOutput();
        return BT::NodeStatus::FAILURE;
      }
    } else if (
      planarDistance(output.poses.back(), expected_goal) > goal_position_tolerance_ ||
      angularDistance(
        quaternionYaw(output.poses.back().pose.orientation),
        quaternionYaw(expected_goal.pose.orientation)) > validation_options_.goal_yaw_tolerance)
    {
      RCLCPP_ERROR(node_->get_logger(), "Single-pose reverse path does not satisfy its goal");
      clearPathOutput();
      return BT::NodeStatus::FAILURE;
    }
  }
  // Planner endpoints are allowed to use a bounded sample around a
  // position-only transit target.  Recheck the final, restored output against
  // the original semantic point before it reaches FollowPath; this keeps a
  // malformed or stale candidate from silently widening that contract.
  const auto & semantic_goal = real_goals_.back();
  if (planarDistance(output.poses.back(), semantic_goal) > goal_position_tolerance_) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading path endpoint left its semantic goal disk");
    clearPathOutput();
    return BT::NodeStatus::FAILURE;
  }
  if (!isZeroQuaternion(semantic_goal.pose.orientation) &&
    angularDistance(
      quaternionYaw(output.poses.back().pose.orientation),
      quaternionYaw(semantic_goal.pose.orientation)) > validation_options_.goal_yaw_tolerance)
  {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading path endpoint does not satisfy its authored yaw");
    clearPathOutput();
    return BT::NodeStatus::FAILURE;
  }
  if (!reverse_ && departure_connectors_.empty()) {
    const auto forward_geometry = validateForwardPathGeometry(
      output, forwardPathGeometryOptions(validation_options_),
      !isZeroQuaternion(semantic_goal.pose.orientation));
    if (!forward_geometry.valid) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rejected generic forward free-heading path before FollowPath: %s at segment %zu "
        "(observed=%.6f, limit=%.6f)",
        forward_geometry.reason.c_str(), forward_geometry.segment_index,
        forward_geometry.observed_value, forward_geometry.limit);
      clearPathOutput();
      return BT::NodeStatus::FAILURE;
    }
  }
  setOutput("path", output);
  return BT::NodeStatus::SUCCESS;
}

void ComputeFreeHeadingPathAction::cancelActiveQuery(bool fail_after_terminal)
{
  if (query_state_ == QueryState::IDLE) {
    return;
  }
  failure_after_cancellation_ = failure_after_cancellation_ || fail_after_terminal;
  if (cancellationInProgress()) {
    return;
  }
  cancellation_deadline_ = std::chrono::steady_clock::now() + cancellation_timeout_;

  if (query_state_ == QueryState::WAITING_FOR_GOAL_HANDLE ||
    query_state_ == QueryState::WAITING_FOR_LOOKAHEAD_GOAL_HANDLE)
  {
    // async_send_goal() gives us no UUID before this future resolves. Waiting
    // for it is the only way to guarantee that a late acceptance is cancelled
    // before any replacement candidate is dispatched.
    query_state_ = QueryState::WAITING_FOR_CANCELLATION_GOAL_HANDLE;
    return;
  }

  if (query_state_ == QueryState::WAITING_FOR_RESULT ||
    query_state_ == QueryState::WAITING_FOR_LOOKAHEAD_RESULT)
  {
    if (requestCancellationForActiveGoal()) {
      return;
    }
    // Both client-side futures can be absent only when the action handle has
    // already become terminal/unknown. Keep an explicit terminal wait state so
    // the caller still observes the requested failure outcome.
    query_state_ = QueryState::WAITING_FOR_CANCELLATION_TERMINAL;
    return;
  }

  RCLCPP_ERROR(
    node_->get_logger(), "Free-heading planner lost the state required to cancel its query");
  clearQueryState();
}

BT::NodeStatus ComputeFreeHeadingPathAction::waitForCancellation()
{
  if (std::chrono::steady_clock::now() > cancellation_deadline_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Free-heading planner cancellation did not reach a terminal state");
    const bool fail = failure_after_cancellation_;
    clearQueryState();
    clearPathOutput();
    return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  if (query_state_ == QueryState::WAITING_FOR_CANCELLATION_GOAL_HANDLE) {
    if (!goal_handle_future_.valid()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Free-heading cancellation lost its goal-handle future");
      const bool fail = failure_after_cancellation_;
      clearQueryState();
      return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    try {
      active_goal_handle_ = goal_handle_future_.get();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Free-heading cancellation goal request failed: %s", error.what());
      const bool fail = failure_after_cancellation_;
      clearQueryState();
      return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    goal_handle_future_ = std::shared_future<PlannerGoalHandle::SharedPtr>();
    if (!active_goal_handle_) {
      const bool fail = failure_after_cancellation_;
      clearQueryState();
      return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    if (!requestCancellationForActiveGoal()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Free-heading cancellation could not track its accepted goal");
      query_state_ = QueryState::WAITING_FOR_CANCELLATION_TERMINAL;
      return waitForCancellation();
    }
    return BT::NodeStatus::RUNNING;
  }

  if (query_state_ != QueryState::WAITING_FOR_CANCELLATION_TERMINAL) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading cancellation has an invalid state");
    const bool fail = failure_after_cancellation_;
    clearQueryState();
    return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  const bool cancel_ready = !cancel_future_.valid() ||
    cancel_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  const bool result_ready = !result_future_.valid() ||
    result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  if (!cancel_ready || !result_ready) {
    return BT::NodeStatus::RUNNING;
  }

  if (cancel_future_.valid()) {
    try {
      (void)cancel_future_.get();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "Free-heading cancellation response failed: %s", error.what());
    }
  }
  if (result_future_.valid()) {
    try {
      (void)result_future_.get();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "Free-heading cancellation result failed: %s", error.what());
    }
  }

  const bool fail = failure_after_cancellation_;
  clearQueryState();
  return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

bool ComputeFreeHeadingPathAction::requestCancellationForActiveGoal()
{
  if (!active_goal_handle_) {
    return false;
  }

  if (!result_future_.valid()) {
    try {
      result_future_ = planner_client_->async_get_result(active_goal_handle_);
    } catch (const std::exception & error) {
      // An action client only rejects this request when the handle is no longer
      // known or is terminal. In either case there is no live result to wait on.
      RCLCPP_WARN(
        node_->get_logger(), "Free-heading cancellation result request failed: %s", error.what());
    }
  }
  try {
    cancel_future_ = planner_client_->async_cancel_goal(active_goal_handle_);
  } catch (const std::exception & error) {
    // Keep waiting on result_future_ when possible. A failed cancel request
    // must not make a replacement query race an action that may still execute.
    RCLCPP_WARN(
      node_->get_logger(), "Free-heading cancellation request failed: %s", error.what());
  }

  if (!result_future_.valid() && !cancel_future_.valid()) {
    return false;
  }
  query_state_ = QueryState::WAITING_FOR_CANCELLATION_TERMINAL;
  return true;
}

bool ComputeFreeHeadingPathAction::cancellationInProgress() const
{
  return query_state_ == QueryState::WAITING_FOR_CANCELLATION_GOAL_HANDLE ||
         query_state_ == QueryState::WAITING_FOR_CANCELLATION_TERMINAL;
}

void ComputeFreeHeadingPathAction::clearCompletedQuery()
{
  active_goal_handle_.reset();
  goal_handle_future_ = std::shared_future<PlannerGoalHandle::SharedPtr>();
  result_future_ = std::shared_future<PlannerGoalHandle::WrappedResult>();
  cancel_future_ = std::shared_future<CancelResponse::SharedPtr>();
  query_state_ = QueryState::IDLE;
}

void ComputeFreeHeadingPathAction::clearQueryState()
{
  clearCompletedQuery();
  failure_after_cancellation_ = false;
  cancellation_deadline_ = std::chrono::steady_clock::time_point();
}

void ComputeFreeHeadingPathAction::clearPathOutput()
{
  setOutput("path", nav_msgs::msg::Path());
}

void ComputeFreeHeadingPathAction::setRecoveryEligible(bool eligible)
{
  bool recovery_is_eligible =
    eligible && !planner_query_failed_ && !search_budget_exhausted_;
  // Capture the final planning snapshot immediately before the BT clears
  // costmaps. AckermannReverseRetreat requires both raw maps to advance beyond
  // this stamp, creating a real post-clear sample barrier instead of merely
  // accepting the map that happened to arrive when planning started.
  if (recovery_is_eligible && !publishCostmapBarrier()) {
    recovery_is_eligible = false;
  }
  setOutput("recovery_eligible", recovery_is_eligible);
}

bool ComputeFreeHeadingPathAction::isZeroQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  return finite(orientation.x) && finite(orientation.y) &&
         finite(orientation.z) && finite(orientation.w) &&
         std::abs(orientation.x) <= kQuaternionNormTolerance &&
         std::abs(orientation.y) <= kQuaternionNormTolerance &&
         std::abs(orientation.z) <= kQuaternionNormTolerance &&
         std::abs(orientation.w) <= kQuaternionNormTolerance;
}

bool ComputeFreeHeadingPathAction::isUnitQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!finite(orientation.x) || !finite(orientation.y) ||
    !finite(orientation.z) || !finite(orientation.w))
  {
    return false;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  return finite(norm) && std::abs(norm - 1.0) <= kQuaternionNormTolerance;
}

bool ComputeFreeHeadingPathAction::sameGoalSequence(
  const std::vector<geometry_msgs::msg::PoseStamped> & first,
  const std::vector<geometry_msgs::msg::PoseStamped> & second)
{
  return isGoalSequenceSuffix(first, second, 0U);
}

bool ComputeFreeHeadingPathAction::isGoalSequenceSuffix(
  const std::vector<geometry_msgs::msg::PoseStamped> & original,
  const std::vector<geometry_msgs::msg::PoseStamped> & suffix,
  std::size_t maximum_removed_prefix)
{
  if (suffix.empty() || suffix.size() > original.size()) {
    return false;
  }
  const std::size_t removed_prefix = original.size() - suffix.size();
  if (removed_prefix > maximum_removed_prefix) {
    return false;
  }
  for (std::size_t index = 0; index < suffix.size(); ++index) {
    const auto & lhs = original[removed_prefix + index];
    const auto & rhs = suffix[index];
    if (lhs.header.frame_id != rhs.header.frame_id ||
      std::abs(lhs.pose.position.x - rhs.pose.position.x) > kPositionEpsilon ||
      std::abs(lhs.pose.position.y - rhs.pose.position.y) > kPositionEpsilon ||
      std::abs(lhs.pose.position.z - rhs.pose.position.z) > kPositionEpsilon ||
      std::abs(lhs.pose.orientation.x - rhs.pose.orientation.x) > kQuaternionNormTolerance ||
      std::abs(lhs.pose.orientation.y - rhs.pose.orientation.y) > kQuaternionNormTolerance ||
      std::abs(lhs.pose.orientation.z - rhs.pose.orientation.z) > kQuaternionNormTolerance ||
      std::abs(lhs.pose.orientation.w - rhs.pose.orientation.w) > kQuaternionNormTolerance)
    {
      return false;
    }
  }
  return true;
}

double ComputeFreeHeadingPathAction::quaternionYaw(
  const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(orientation);
}

geometry_msgs::msg::Quaternion ComputeFreeHeadingPathAction::quaternionFromYaw(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

double ComputeFreeHeadingPathAction::angularDistance(double first, double second)
{
  return std::abs(std::remainder(second - first, 2.0 * kPi));
}

double ComputeFreeHeadingPathAction::pathLength(const nav_msgs::msg::Path & path)
{
  double total = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    total += planarDistance(path.poses[index - 1], path.poses[index]);
  }
  return total;
}

double ComputeFreeHeadingPathAction::planarDistance(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(
    second.pose.position.x - first.pose.position.x,
    second.pose.position.y - first.pose.position.y);
}

bool ComputeFreeHeadingPathAction::rotatePoseYawByPi(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output)
{
  return smartcar_nav2::rotatePoseYawByPi(input, output);
}

}  // namespace smartcar_nav2

BT_REGISTER_NODES(factory)
{
  using smartcar_nav2::ComputeFreeHeadingPathAction;
  using smartcar_nav2::LatchSuccess;
  factory.registerNodeType<LatchSuccess>("LatchSuccess");
  factory.registerBuilder<ComputeFreeHeadingPathAction>(
    "ComputeFreeHeadingPathToPose",
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<ComputeFreeHeadingPathAction>(name, configuration, false, false);
    });
  factory.registerBuilder<ComputeFreeHeadingPathAction>(
    "ComputeFreeHeadingPathThroughPoses",
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<ComputeFreeHeadingPathAction>(name, configuration, false, true);
    });
  factory.registerBuilder<ComputeFreeHeadingPathAction>(
    "ComputeReverseFreeHeadingPathToPose",
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<ComputeFreeHeadingPathAction>(name, configuration, true, false);
    });
  factory.registerBuilder<ComputeFreeHeadingPathAction>(
    "ComputeReverseFreeHeadingPathThroughPoses",
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<ComputeFreeHeadingPathAction>(name, configuration, true, true);
    });
}
