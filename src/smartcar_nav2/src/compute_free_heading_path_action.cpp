#include "smartcar_nav2/compute_free_heading_path_action.hpp"
#include "smartcar_nav2/free_transit_goal_samples.hpp"

#include <algorithm>
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
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr double kPositionEpsilon = 1.0e-6;
constexpr double kJoinPositionTolerance = 1.0e-3;
constexpr double kJoinYawTolerance = 1.0e-3;
constexpr std::size_t kMaximumThroughCandidateQueries = 64U;

bool finite(double value)
{
  return std::isfinite(value);
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
}

BT::PortsList ComputeFreeHeadingPathAction::providedPorts()
{
  return {
    BT::OutputPort<nav_msgs::msg::Path>(
      "path", "Costmap-aware path with free transit headings resolved"),
    BT::OutputPort<bool>(
      "recovery_eligible",
      "True only when the bounded candidate search exhausts every feasible path"),
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
      "fallback_candidate_limit", 2,
      "Legacy compatibility limit; free headings are initialized from heading_samples"),
    BT::InputPort<int>(
      "lookahead_fallback_candidate_limit", 0,
      "Legacy compatibility limit; free lookaheads use the full heading list"),
    BT::InputPort<int>(
      "through_solution_limit", 4,
      "Maximum complete through-poses heading chains compared before publication"),
    BT::InputPort<double>(
      "max_initial_path_length_ratio", 1.60,
      "Expand fallback headings when the live geometric candidate is this much longer than direct"),
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
      "footprint_half_length_m", 0.30,
      "Padded vehicle half length used for global-costmap footprint checks"),
    BT::InputPort<double>(
      "footprint_half_width_m", 0.16,
      "Padded vehicle half width used for global-costmap footprint checks"),
    BT::InputPort<double>(
      "footprint_sweep_step_m", 0.025,
      "Maximum pose spacing for global-costmap footprint sweep"),
    BT::InputPort<int>(
      "footprint_lethal_cost", 253,
      "Costmap value at or above which the footprint sweep rejects a path"),
    BT::InputPort<double>(
      "minimum_turning_radius", 0.55, "Minimum feasible turning radius"),
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
  through_search_frames_.clear();
  through_candidate_query_count_ = 0U;
  best_through_path_.reset();
  best_through_quality_.reset();
  through_complete_path_count_ = 0U;
  target_index_ = 0;
  candidate_index_ = 0;
  lookahead_candidate_index_ = 0;
  failure_after_cancellation_ = false;
  commit_best_after_cancellation_ = false;
  waiting_for_costmap_ = false;

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
  costmap_wait_deadline_ =
    std::chrono::steady_clock::now() + costmap_wait_timeout_;
  if (!hasFreshGlobalCostmap()) {
    waiting_for_costmap_ = true;
    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for a fresh global costmap before free-heading planning");
    return BT::NodeStatus::RUNNING;
  }
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
    if (cancellation_status != BT::NodeStatus::SUCCESS) {
      commit_best_after_cancellation_ = false;
      return cancellation_status;
    }
    if (!commit_best_after_cancellation_) {
      return cancellation_status;
    }
    commit_best_after_cancellation_ = false;
    return commitBestCandidate();
  }
  if (waiting_for_costmap_) {
    if (!hasFreshGlobalCostmap()) {
      if (std::chrono::steady_clock::now() >= costmap_wait_deadline_) {
        RCLCPP_ERROR(
          node_->get_logger(), "Free-heading planner timed out waiting for global costmap");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    waiting_for_costmap_ = false;
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
    clearPathOutput();
    return waitForCancellation();
  }
  if (query_state_ == QueryState::IDLE) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading planner has no active query");
    return BT::NodeStatus::FAILURE;
  }
  if (std::chrono::steady_clock::now() > query_deadline_) {
    const bool retain_best = !through_poses_ && best_candidate_.has_value() &&
      best_trial_path_.has_value();
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading planner candidate %zu for goal %zu timed out%s",
      candidate_index_ + 1, target_index_ + 1,
      retain_best ? "; retaining the best validated candidate" : "");
    commit_best_after_cancellation_ = retain_best;
    cancelActiveQuery(!retain_best);
    if (!cancellationInProgress()) {
      commit_best_after_cancellation_ = false;
      clearPathOutput();
      return BT::NodeStatus::FAILURE;
    }
    const BT::NodeStatus cancellation_status = waitForCancellation();
    if (cancellation_status != BT::NodeStatus::SUCCESS) {
      commit_best_after_cancellation_ = false;
      clearPathOutput();
      return cancellation_status;
    }
    if (retain_best) {
      commit_best_after_cancellation_ = false;
      return commitBestCandidate();
    }
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
      return lookahead_query ? completeLookahead(nullptr) : completeCandidate(nullptr);
    }
    goal_handle_future_ = std::shared_future<PlannerGoalHandle::SharedPtr>();
    if (!active_goal_handle_) {
      query_state_ = QueryState::IDLE;
      return lookahead_query ? completeLookahead(nullptr) : completeCandidate(nullptr);
    }
    try {
      result_future_ = planner_client_->async_get_result(active_goal_handle_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Free-heading planner result request failed: %s", error.what());
      clearCompletedQuery();
      return lookahead_query ? completeLookahead(nullptr) : completeCandidate(nullptr);
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
    return lookahead_query ? completeLookahead(nullptr) : completeCandidate(nullptr);
  }
  clearCompletedQuery();
  if (
    result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !result.result || result.result->path.poses.empty())
  {
    return lookahead_query ? completeLookahead(nullptr) : completeCandidate(nullptr);
  }
  return lookahead_query ?
    completeLookahead(&result.result->path) : completeCandidate(&result.result->path);
}

void ComputeFreeHeadingPathAction::onHalted()
{
  commit_best_after_cancellation_ = false;
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
  // Existing BT XML exposes these compatibility attributes. Validate their
  // ranges, but candidate generation is now wholly bounded by heading_samples.
  int fallback_candidate_limit = 0;
  if (!getInput("fallback_candidate_limit", fallback_candidate_limit) ||
    fallback_candidate_limit < 0 || fallback_candidate_limit > 4)
  {
    RCLCPP_ERROR(node_->get_logger(), "fallback_candidate_limit must lie in [0, 4]");
    return false;
  }
  int lookahead_fallback_candidate_limit = 0;
  if (!getInput(
      "lookahead_fallback_candidate_limit", lookahead_fallback_candidate_limit) ||
    lookahead_fallback_candidate_limit < 0 || lookahead_fallback_candidate_limit > 2)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "lookahead_fallback_candidate_limit must lie in [0, 2]");
    return false;
  }
  int through_solution_limit = 0;
  if (!getInput("through_solution_limit", through_solution_limit) ||
    through_solution_limit < 1 || through_solution_limit > 8)
  {
    RCLCPP_ERROR(node_->get_logger(), "through_solution_limit must lie in [1, 8]");
    return false;
  }
  through_solution_limit_ = static_cast<std::size_t>(through_solution_limit);
  if (!getInput("max_initial_path_length_ratio", max_initial_path_length_ratio_) ||
    !finite(max_initial_path_length_ratio_) || max_initial_path_length_ratio_ < 1.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "max_initial_path_length_ratio must be at least 1.0");
    return false;
  }
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
  return prepareTargetCandidates() && startCandidateQuery();
}

bool ComputeFreeHeadingPathAction::readValidationOptions()
{
  return getInput("minimum_turning_radius", validation_options_.minimum_turning_radius) &&
         getInput("curvature_tolerance", validation_options_.curvature_tolerance) &&
         getInput("maximum_direction_error", validation_options_.maximum_direction_error) &&
         getInput("start_position_tolerance", validation_options_.start_position_tolerance) &&
         getInput("start_yaw_tolerance", validation_options_.start_yaw_tolerance) &&
         getInput("goal_position_tolerance", validation_options_.goal_position_tolerance) &&
         getInput("goal_yaw_tolerance", validation_options_.goal_yaw_tolerance) &&
         getInput("minimum_segment_length", validation_options_.minimum_segment_length);
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
    lethal_cost < 1 || lethal_cost > 253)
  {
    return false;
  }
  footprint_sweep_options_.lethal_cost_threshold =
    static_cast<std::uint8_t>(lethal_cost);
  return true;
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

bool ComputeFreeHeadingPathAction::prepareTargetCandidates()
{
  if (target_index_ >= real_goals_.size()) {
    return false;
  }
  candidate_goals_.clear();
  lookahead_candidates_.clear();
  candidate_index_ = 0;
  lookahead_candidate_index_ = 0;
  best_candidate_.reset();
  best_trial_path_.reset();
  best_continuation_.reset();
  search_deadline_ = std::chrono::steady_clock::now() + search_budget_;
  candidate_goals_ = goalCandidatesForTarget(target_index_, virtual_start_);
  return !candidate_goals_.empty();
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
    GoalCandidate candidate;
    candidate.pose = real_goal;
    candidate.pose.pose.orientation = quaternionFromYaw(
      referenceYawForGoal(candidate.pose, target_index, start));
    candidates.push_back(std::move(candidate));
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
  return planarDistance(endpoint, candidate) <= goal_position_tolerance_ &&
         planarDistance(endpoint, real_goal) <= goal_position_tolerance_;
}

bool ComputeFreeHeadingPathAction::startCandidateQuery()
{
  if (target_index_ >= real_goals_.size() || candidate_index_ >= candidate_goals_.size() ||
    query_state_ != QueryState::IDLE || active_goal_handle_ || goal_handle_future_.valid() ||
    result_future_.valid() || cancel_future_.valid())
  {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading attempted to overlap planner queries");
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now >= search_deadline_ ||
    (through_poses_ && now >= through_search_deadline_))
  {
    RCLCPP_WARN(node_->get_logger(), "Free-heading candidate search budget exhausted");
    return false;
  }
  if (through_poses_ &&
    through_candidate_query_count_ >= kMaximumThroughCandidateQueries)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Free-heading through-poses search reached its %zu-query limit",
      kMaximumThroughCandidateQueries);
    return false;
  }
  active_virtual_goal_ = candidate_goals_[candidate_index_].pose;
  ComputePathToPose::Goal request;
  request.planner_id = planner_id_;
  request.use_start = true;
  request.start = virtual_start_;
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
    }
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now >= search_deadline_ ||
    (through_poses_ && now >= through_search_deadline_))
  {
    RCLCPP_WARN(node_->get_logger(), "Free-heading candidate search budget exhausted");
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
  const nav_msgs::msg::Path * candidate_path)
{
  if (candidate_path == nullptr || candidate_path->poses.empty()) {
    return advanceCandidate();
  }

  const auto & endpoint = candidate_path->poses.back();
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

  const PathQuality candidate_quality = pathQuality(*candidate_path);
  if (!hasAcceptableCostmapSample(candidate_quality)) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Rejected free-heading candidate %zu for goal %zu: footprint_clear=%s, "
      "max_cost=%u, limit=%u",
      candidate_index_ + 1, target_index_ + 1,
      candidate_quality.footprint_sweep_checked && candidate_quality.footprint_sweep_clear ?
      "true" : "false",
      static_cast<unsigned int>(candidate_quality.maximum_cost),
      static_cast<unsigned int>(maximum_path_cost_));
    return advanceCandidate();
  }

  const double length = pathLength(*candidate_path);
  if (!finite(length)) {
    return advanceCandidate();
  }

  if (through_poses_) {
    return completeThroughCandidate(*candidate_path);
  }

  // A locally shortest free-heading arrival can leave the next corridor
  // impossible to enter. Before committing a transit candidate, prove that
  // its actual Smac endpoint can reach the immediate successor.
  if (isZeroQuaternion(real_goals_[target_index_].pose.orientation) &&
    target_index_ + 1 < real_goals_.size())
  {
    pending_candidate_ = CandidatePlan{length, *candidate_path};
    if (!prepareLookaheadCandidates() || !startLookaheadQuery()) {
      pending_candidate_.reset();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  nav_msgs::msg::Path trial;
  std::string reason;
  if (!buildAndValidateTrial(*candidate_path, nullptr, trial, reason)) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Rejected free-heading candidate %zu for goal %zu: %s",
      candidate_index_ + 1, target_index_ + 1, reason.c_str());
    return advanceCandidate();
  }
  if (!best_candidate_.has_value() ||
    length < best_candidate_->length_m - kPositionEpsilon)
  {
    best_candidate_ = CandidatePlan{length, *candidate_path};
    best_trial_path_ = std::move(trial);
    best_continuation_.reset();
  }
  return advanceCandidate();
}

BT::NodeStatus ComputeFreeHeadingPathAction::completeLookahead(
  const nav_msgs::msg::Path * candidate_path)
{
  if (!pending_candidate_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "Free-heading lookahead lost its candidate");
    return BT::NodeStatus::FAILURE;
  }

  bool continuation_is_valid = false;
  double continuation_length = 0.0;
  if (candidate_path != nullptr && !candidate_path->poses.empty()) {
    const auto & endpoint = candidate_path->poses.back();
    continuation_is_valid = isUnitQuaternion(endpoint.pose.orientation) &&
      angularDistance(
      quaternionYaw(endpoint.pose.orientation),
      quaternionYaw(active_lookahead_goal_.pose.orientation)) <=
      validation_options_.goal_yaw_tolerance &&
      endpointMatchesCandidateAndRealGoal(
      endpoint, active_lookahead_goal_, real_goals_[target_index_ + 1]);
    continuation_length = pathLength(*candidate_path);
    continuation_is_valid = continuation_is_valid && finite(continuation_length);
    continuation_is_valid = continuation_is_valid &&
      hasAcceptableCostmapSample(pathQuality(*candidate_path));
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
  const nav_msgs::msg::Path & candidate_path)
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
    setRecoveryEligible(true);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeFreeHeadingPathAction::completeThroughPath()
{
  const PathQuality quality = pathQuality(virtual_path_);
  ++through_complete_path_count_;
  std::string validation_reason;
  const bool chain_is_valid = hasAcceptableCostmapSample(quality) &&
    validateReverseTrial(virtual_path_, validation_reason);
  if (!chain_is_valid) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Rejected free-heading through-poses chain %zu after full validation: %s",
      through_complete_path_count_,
      validation_reason.empty() ? "costmap_or_footprint" : validation_reason.c_str());
  }
  double direct_constraint_length = 0.0;
  if (!virtual_path_.poses.empty()) {
    geometry_msgs::msg::PoseStamped previous = virtual_path_.poses.front();
    for (const auto & goal : real_goals_) {
      direct_constraint_length += planarDistance(previous, goal);
      previous = goal;
    }
  }
  const bool path_is_reasonable =
    direct_constraint_length <= kPositionEpsilon ||
    quality.length_m <= direct_constraint_length * max_initial_path_length_ratio_;
  if (!path_is_reasonable) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Rejected free-heading through-poses chain %zu: length %.3f exceeds %.3f x %.2f",
      through_complete_path_count_, quality.length_m, direct_constraint_length,
      max_initial_path_length_ratio_);
  }

  // Every candidate reaching this point has already passed edge validation;
  // the full chain above repeats costmap/footprint and reverse-kinematic
  // validation. Rank only those valid chains by length. A bounded search does
  // not establish a global continuous-heading optimum.
  if (chain_is_valid && path_is_reasonable &&
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
  if (solution_limit_reached || search_budget_exhausted) {
    return publishBestThroughPath();
  }

  const auto next_frame = highestRiskThroughFrame();
  if (!next_frame.has_value()) {
    if (!best_through_path_.has_value()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No free-heading through-poses chain satisfies the path-length bound");
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
  return commitBestCandidate();
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
  bool costmap_is_fresh = false;
  {
    std::lock_guard<std::mutex> lock(global_costmap_mutex_);
    costmap = global_costmap_;
    costmap_is_fresh = global_costmap_received_at_ !=
      std::chrono::steady_clock::time_point() &&
      std::chrono::steady_clock::now() - global_costmap_received_at_ <= costmap_max_age_;
  }
  if (!costmap_is_fresh || !costmap || costmap->header.frame_id != global_frame_ ||
    costmap->metadata.resolution <= 0.0f || costmap->metadata.size_x == 0U ||
    costmap->metadata.size_y == 0U)
  {
    return quality;
  }

  // A path centreline can clear a lethal cell while the Ackermann body clips
  // it during a minimum-radius arc. The virtual reverse frame is a pi yaw
  // shift, so the configured pi-symmetric rectangular footprint is unchanged.
  quality.footprint_sweep_checked = true;
  quality.footprint_sweep_clear =
    costmapFootprintPathSweep(path, *costmap, footprint_sweep_options_) ==
    CostmapFootprintSweepResult::kClear;

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

void ComputeFreeHeadingPathAction::updateGlobalCostmap(
  nav2_msgs::msg::Costmap::SharedPtr costmap)
{
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  global_costmap_ = std::move(costmap);
  global_costmap_received_at_ = std::chrono::steady_clock::now();
}

bool ComputeFreeHeadingPathAction::hasFreshGlobalCostmap() const
{
  std::lock_guard<std::mutex> lock(global_costmap_mutex_);
  if (!global_costmap_ || global_costmap_->header.frame_id != global_frame_ ||
    global_costmap_->metadata.resolution <= 0.0f ||
    global_costmap_->metadata.size_x == 0U || global_costmap_->metadata.size_y == 0U ||
    global_costmap_received_at_ == std::chrono::steady_clock::time_point())
  {
    return false;
  }
  const std::size_t expected_size =
    static_cast<std::size_t>(global_costmap_->metadata.size_x) *
    static_cast<std::size_t>(global_costmap_->metadata.size_y);
  return global_costmap_->data.size() >= expected_size &&
         std::chrono::steady_clock::now() - global_costmap_received_at_ <=
         costmap_max_age_;
}

bool ComputeFreeHeadingPathAction::hasAcceptableCostmapSample(
  const PathQuality & quality) const
{
  return quality.has_costmap_sample && quality.footprint_sweep_checked &&
         quality.footprint_sweep_clear &&
         quality.maximum_cost <= maximum_path_cost_;
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
    const bool fail = failure_after_cancellation_ || commit_best_after_cancellation_;
    clearQueryState();
    clearPathOutput();
    return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  if (query_state_ == QueryState::WAITING_FOR_CANCELLATION_GOAL_HANDLE) {
    if (!goal_handle_future_.valid()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Free-heading cancellation lost its goal-handle future");
      const bool fail = failure_after_cancellation_ || commit_best_after_cancellation_;
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
      const bool fail = failure_after_cancellation_ || commit_best_after_cancellation_;
      clearQueryState();
      return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    goal_handle_future_ = std::shared_future<PlannerGoalHandle::SharedPtr>();
    if (!active_goal_handle_) {
      const bool fail = failure_after_cancellation_ || commit_best_after_cancellation_;
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
    const bool fail = failure_after_cancellation_ || commit_best_after_cancellation_;
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
  setOutput("recovery_eligible", eligible);
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
