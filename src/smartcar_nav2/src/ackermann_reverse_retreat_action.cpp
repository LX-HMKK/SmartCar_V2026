#include "smartcar_nav2/ackermann_reverse_retreat_action.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "smartcar_nav2/ackermann_reverse_retreat_path.hpp"

namespace smartcar_nav2
{

namespace
{

enum class CostmapFreshness
{
  kFresh,
  kMissing,
  kWrongFrame,
  kMalformed,
  kStale,
};

CostmapFreshness costmapFreshness(
  const nav2_msgs::msg::Costmap::SharedPtr & costmap,
  const std::chrono::steady_clock::time_point & received_at,
  const std::string & expected_frame,
  const std::chrono::milliseconds & maximum_age,
  const std::chrono::steady_clock::time_point & now)
{
  if (!costmap || received_at == std::chrono::steady_clock::time_point()) {
    return CostmapFreshness::kMissing;
  }
  if (costmap->header.frame_id != expected_frame) {
    return CostmapFreshness::kWrongFrame;
  }
  const auto expected_size = static_cast<std::size_t>(costmap->metadata.size_x) *
    static_cast<std::size_t>(costmap->metadata.size_y);
  if (costmap->metadata.resolution <= 0.0F || costmap->metadata.size_x == 0U ||
    costmap->metadata.size_y == 0U || costmap->data.size() < expected_size)
  {
    return CostmapFreshness::kMalformed;
  }
  if (now - received_at > maximum_age) {
    return CostmapFreshness::kStale;
  }
  return CostmapFreshness::kFresh;
}

const char * costmapFreshnessName(CostmapFreshness freshness)
{
  switch (freshness) {
    case CostmapFreshness::kFresh:
      return "fresh";
    case CostmapFreshness::kMissing:
      return "missing";
    case CostmapFreshness::kWrongFrame:
      return "wrong frame";
    case CostmapFreshness::kMalformed:
      return "malformed";
    case CostmapFreshness::kStale:
      return "stale";
  }
  return "unknown";
}

long long costmapAgeMilliseconds(
  const std::chrono::steady_clock::time_point & received_at,
  const std::chrono::steady_clock::time_point & now)
{
  if (received_at == std::chrono::steady_clock::time_point()) {
    return -1;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - received_at).count();
}

}  // namespace

AckermannReverseRetreatAction::AckermannReverseRetreatAction(
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
}

void AckermannReverseRetreatAction::on_tick()
{
  callback_group_executor_.spin_some();
  clearPathOutput();
  goal_ = nav2_msgs::action::FollowPath::Goal();

  bool allow_retreat = false;
  bool retreat_used = false;
  double retreat_distance_m = 0.0;
  int costmap_max_age_ms = 0;
  if (!getInput("allow_retreat", allow_retreat) || !allow_retreat) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Refusing Ackermann reverse retreat because the planner failure is not recoverable");
    should_send_goal_ = false;
    return;
  }
  if (!getInput("retreat_used", retreat_used) || retreat_used) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Refusing a second Ackermann reverse retreat in the same navigation action");
    should_send_goal_ = false;
    return;
  }
  if (!getInput("retreat_distance_m", retreat_distance_m) ||
    !getInput("costmap_max_age_ms", costmap_max_age_ms) ||
    costmap_max_age_ms < 100 || costmap_max_age_ms > 2000 ||
    !readSweepOptions())
  {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat ports are invalid");
    should_send_goal_ = false;
    return;
  }
  costmap_max_age_ = std::chrono::milliseconds(costmap_max_age_ms);
  if (!getInput("controller_id", goal_.controller_id) || goal_.controller_id.empty() ||
    !getInput("goal_checker_id", goal_.goal_checker_id) || goal_.goal_checker_id.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat controller ports are missing");
    should_send_goal_ = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat has no fresh robot pose");
    should_send_goal_ = false;
    return;
  }

  nav_msgs::msg::Path retreat_path;
  if (!buildAckermannReverseRetreatPath(
      current_pose, global_frame_, retreat_distance_m, retreat_path))
  {
    RCLCPP_ERROR(node_->get_logger(), "Ackermann reverse retreat path inputs are invalid");
    should_send_goal_ = false;
    return;
  }
  if (!retreatPathIsClear(retreat_path)) {
    should_send_goal_ = false;
    return;
  }

  // Latch before dispatch. An action-server abort can occur after physical
  // motion has begun, so it must not permit a second reverse attempt.
  setOutput("retreat_used", true);
  goal_.path = retreat_path;
  setOutput("path", retreat_path);
  RCLCPP_WARN(
    node_->get_logger(),
    "Reverse planner exhausted its candidates; retreating %.3f m before one replan",
    retreat_distance_m);
}

BT::NodeStatus AckermannReverseRetreatAction::on_aborted()
{
  clearPathOutput();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus AckermannReverseRetreatAction::on_cancelled()
{
  clearPathOutput();
  return BT::NodeStatus::FAILURE;
}

void AckermannReverseRetreatAction::halt()
{
  clearPathOutput();
  nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>::halt();
}

void AckermannReverseRetreatAction::clearPathOutput()
{
  setOutput("path", nav_msgs::msg::Path());
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

bool AckermannReverseRetreatAction::retreatPathIsClear(const nav_msgs::msg::Path & path) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  const auto now = std::chrono::steady_clock::now();
  const auto global_freshness = costmapFreshness(
    global_costmap_, global_costmap_received_at_, global_frame_, costmap_max_age_, now);
  const auto local_freshness = costmapFreshness(
    local_costmap_, local_costmap_received_at_, global_frame_, costmap_max_age_, now);
  const auto & start = path.poses.front().pose.position;
  const auto & end = path.poses.back().pose.position;
  if (global_freshness != CostmapFreshness::kFresh ||
    local_freshness != CostmapFreshness::kFresh)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ackermann reverse retreat denied: global_costmap=%s (age=%lld ms), "
      "local_costmap=%s (age=%lld ms), limit=%ld ms; retreat=(%.3f, %.3f)->(%.3f, %.3f)",
      costmapFreshnessName(global_freshness),
      costmapAgeMilliseconds(global_costmap_received_at_, now),
      costmapFreshnessName(local_freshness),
      costmapAgeMilliseconds(local_costmap_received_at_, now),
      static_cast<long>(costmap_max_age_.count()),
      start.x, start.y, end.x, end.y);
    return false;
  }

  const auto global_result = costmapFootprintPathSweep(
    path, *global_costmap_, footprint_sweep_options_);
  const auto local_result = costmapFootprintPathSweep(
    path, *local_costmap_, footprint_sweep_options_);
  if (global_result != CostmapFootprintSweepResult::kClear ||
    local_result != CostmapFootprintSweepResult::kClear)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ackermann reverse retreat denied: global_footprint=%s, local_footprint=%s; "
      "retreat=(%.3f, %.3f)->(%.3f, %.3f)",
      costmapFootprintSweepResultName(global_result),
      costmapFootprintSweepResultName(local_result),
      start.x, start.y, end.x, end.y);
    return false;
  }
  return true;
}

void AckermannReverseRetreatAction::updateGlobalCostmap(
  nav2_msgs::msg::Costmap::SharedPtr costmap)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  global_costmap_ = std::move(costmap);
  global_costmap_received_at_ = std::chrono::steady_clock::now();
}

void AckermannReverseRetreatAction::updateLocalCostmap(
  nav2_msgs::msg::Costmap::SharedPtr costmap)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  local_costmap_ = std::move(costmap);
  local_costmap_received_at_ = std::chrono::steady_clock::now();
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
