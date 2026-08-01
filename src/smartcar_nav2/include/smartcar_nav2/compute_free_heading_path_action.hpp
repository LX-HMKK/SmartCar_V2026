#ifndef SMARTCAR_NAV2__COMPUTE_FREE_HEADING_PATH_ACTION_HPP_
#define SMARTCAR_NAV2__COMPUTE_FREE_HEADING_PATH_ACTION_HPP_

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"
#include "smartcar_nav2/reverse_path_utils.hpp"

namespace smartcar_nav2
{

// Resolve zero-quaternion transit goals from the live path geometry. The
// initial terminal tangent bisects incoming and outgoing route legs, then a
// bounded heading scan handles blocked or long alternatives. This keeps
// position-only waypoints out of Smac's hard-yaw goal state without imposing
// an artificial right-angle turn at every pass-through constraint.
class ComputeFreeHeadingPathAction : public BT::StatefulActionNode
{
public:
  ComputeFreeHeadingPathAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & configuration,
    bool reverse,
    bool through_poses);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using PlannerGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using PlannerClient = rclcpp_action::Client<ComputePathToPose>;
  using CancelResponse = PlannerClient::CancelResponse;

  enum class QueryState
  {
    IDLE,
    WAITING_FOR_GOAL_HANDLE,
    WAITING_FOR_RESULT,
    WAITING_FOR_LOOKAHEAD_GOAL_HANDLE,
    WAITING_FOR_LOOKAHEAD_RESULT,
    WAITING_FOR_CANCELLATION_GOAL_HANDLE,
    WAITING_FOR_CANCELLATION_TERMINAL,
  };

  struct CandidatePlan
  {
    double length_m{0.0};
    nav_msgs::msg::Path path;
  };

  struct GoalCandidate
  {
    geometry_msgs::msg::PoseStamped pose;
  };

  struct PathQuality
  {
    double length_m{0.0};
    double maximum_curvature{0.0};
    double accumulated_curvature{0.0};
    std::uint8_t maximum_cost{0U};
    double mean_cost{0.0};
    bool has_costmap_sample{false};
    bool footprint_sweep_checked{false};
    bool footprint_sweep_clear{false};
  };

  struct ThroughSearchFrame
  {
    std::size_t target_index{0};
    std::vector<GoalCandidate> candidates;
    std::size_t selected_candidate_index{0};
    nav_msgs::msg::Path path_before;
    geometry_msgs::msg::PoseStamped start_before;
    PathQuality edge_quality;
  };

  bool loadInputs();
  bool readValidationOptions();
  bool readFootprintSweepOptions();
  bool beginCandidateSearch();
  bool goalsChanged();
  bool prepareTargetCandidates();
  bool prepareLookaheadCandidates();
  bool startCandidateQuery();
  bool startLookaheadQuery();
  BT::NodeStatus completeCandidate(
    const nav_msgs::msg::Path * candidate_path);
  BT::NodeStatus completeThroughCandidate(
    const nav_msgs::msg::Path & candidate_path);
  BT::NodeStatus completeThroughPath();
  BT::NodeStatus completeLookahead(
    const nav_msgs::msg::Path * candidate_path);
  BT::NodeStatus advanceCandidate();
  BT::NodeStatus advanceThroughCandidate();
  BT::NodeStatus backtrackThroughCandidate(
    std::optional<std::size_t> preferred_frame_index = std::nullopt);
  BT::NodeStatus advanceLookaheadCandidate();
  BT::NodeStatus commitBestCandidate();
  BT::NodeStatus finishPath();
  BT::NodeStatus publishBestThroughPath();
  bool appendSegmentChecked(
    nav_msgs::msg::Path & destination,
    const nav_msgs::msg::Path & segment,
    std::string & reason) const;
  bool buildAndValidateTrial(
    const nav_msgs::msg::Path & first,
    const nav_msgs::msg::Path * continuation,
    nav_msgs::msg::Path & trial,
    std::string & reason) const;
  bool validateReverseTrial(
    const nav_msgs::msg::Path & virtual_trial,
    std::string & reason) const;
  bool frameHasAlternative(const ThroughSearchFrame & frame) const;
  std::optional<std::size_t> highestRiskThroughFrame() const;
  PathQuality pathQuality(const nav_msgs::msg::Path & path) const;
  bool betterPathQuality(const PathQuality & first, const PathQuality & second) const;
  bool hasFreshGlobalCostmap() const;
  bool hasAcceptableCostmapSample(const PathQuality & quality) const;
  std::optional<std::uint8_t> costAt(
    const nav2_msgs::msg::Costmap & costmap, double x, double y) const;
  void updateGlobalCostmap(nav2_msgs::msg::Costmap::SharedPtr costmap);
  void cancelActiveQuery(bool fail_after_terminal);
  BT::NodeStatus waitForCancellation();
  bool requestCancellationForActiveGoal();
  bool cancellationInProgress() const;
  void clearCompletedQuery();
  void clearQueryState();
  void clearPathOutput();
  void setRecoveryEligible(bool eligible);

  static bool isZeroQuaternion(const geometry_msgs::msg::Quaternion & orientation);
  static bool isUnitQuaternion(const geometry_msgs::msg::Quaternion & orientation);
  static bool sameGoalSequence(
    const std::vector<geometry_msgs::msg::PoseStamped> & first,
    const std::vector<geometry_msgs::msg::PoseStamped> & second);
  static bool isGoalSequenceSuffix(
    const std::vector<geometry_msgs::msg::PoseStamped> & original,
    const std::vector<geometry_msgs::msg::PoseStamped> & suffix,
    std::size_t maximum_removed_prefix);
  static double quaternionYaw(const geometry_msgs::msg::Quaternion & orientation);
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);
  static double angularDistance(double first, double second);
  static double pathLength(const nav_msgs::msg::Path & path);
  std::vector<GoalCandidate> goalCandidatesForTarget(
    std::size_t target_index,
    const geometry_msgs::msg::PoseStamped & start) const;
  double referenceYawForGoal(
    const geometry_msgs::msg::PoseStamped & target,
    std::size_t target_index,
    const geometry_msgs::msg::PoseStamped & start) const;
  bool endpointMatchesCandidateAndRealGoal(
    const geometry_msgs::msg::PoseStamped & endpoint,
    const geometry_msgs::msg::PoseStamped & candidate,
    const geometry_msgs::msg::PoseStamped & real_goal) const;
  static double planarDistance(
    const geometry_msgs::msg::PoseStamped & first,
    const geometry_msgs::msg::PoseStamped & second);
  static bool rotatePoseYawByPi(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;

  bool reverse_{false};
  bool through_poses_{false};
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  std::string planner_id_;
  int heading_samples_{24};
  std::chrono::milliseconds candidate_timeout_{750};
  std::chrono::milliseconds search_budget_{2400};
  std::chrono::milliseconds through_search_budget_{12000};
  std::chrono::milliseconds cancellation_timeout_{500};
  std::chrono::milliseconds costmap_wait_timeout_{1500};
  std::chrono::milliseconds costmap_max_age_{2000};
  std::chrono::steady_clock::time_point search_deadline_;
  std::chrono::steady_clock::time_point through_search_deadline_;
  std::chrono::steady_clock::time_point cancellation_deadline_;
  std::chrono::steady_clock::time_point costmap_wait_deadline_;
  std::size_t through_solution_limit_{4};
  double goal_position_tolerance_{0.20};
  double max_initial_path_length_ratio_{1.60};
  double max_start_drift_m_{0.10};
  std::uint8_t maximum_path_cost_{252U};
  CostmapFootprintSweepOptions footprint_sweep_options_;
  ReversePathValidationOptions validation_options_;

  geometry_msgs::msg::PoseStamped real_start_;
  geometry_msgs::msg::PoseStamped virtual_start_;
  std::vector<geometry_msgs::msg::PoseStamped> real_goals_;
  std::size_t target_index_{0};
  std::vector<GoalCandidate> candidate_goals_;
  std::size_t candidate_index_{0};
  std::vector<GoalCandidate> lookahead_candidates_;
  std::size_t lookahead_candidate_index_{0};
  geometry_msgs::msg::PoseStamped active_virtual_goal_;
  geometry_msgs::msg::PoseStamped active_lookahead_goal_;
  std::optional<CandidatePlan> best_candidate_;
  std::optional<nav_msgs::msg::Path> best_trial_path_;
  std::optional<nav_msgs::msg::Path> best_continuation_;
  std::optional<CandidatePlan> pending_candidate_;
  nav_msgs::msg::Path virtual_path_;
  std::vector<ThroughSearchFrame> through_search_frames_;
  std::size_t through_candidate_query_count_{0};
  std::optional<nav_msgs::msg::Path> best_through_path_;
  std::optional<PathQuality> best_through_quality_;
  std::size_t through_complete_path_count_{0};
  bool waiting_for_costmap_{false};

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_subscription_;
  mutable std::mutex global_costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr global_costmap_;
  std::chrono::steady_clock::time_point global_costmap_received_at_;

  QueryState query_state_{QueryState::IDLE};
  std::shared_future<PlannerGoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<PlannerGoalHandle::WrappedResult> result_future_;
  std::shared_future<CancelResponse::SharedPtr> cancel_future_;
  PlannerGoalHandle::SharedPtr active_goal_handle_;
  std::chrono::steady_clock::time_point query_deadline_;
  bool failure_after_cancellation_{false};
  bool commit_best_after_cancellation_{false};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__COMPUTE_FREE_HEADING_PATH_ACTION_HPP_
