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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"
#include "smartcar_nav2/costmap_sample_guard.hpp"
#include "smartcar_nav2/departure_connector.hpp"
#include "smartcar_nav2/footprint_sweep_collision_source.hpp"
#include "smartcar_nav2/local_costmap_tracking_envelope.hpp"
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
    CostmapFootprintSweepResult footprint_sweep_result{
      CostmapFootprintSweepResult::kInvalidInput};
    CostmapFootprintSweepDiagnostic footprint_sweep_diagnostic;
    std::string footprint_collision_source{"not_applicable"};
    bool local_tracking_costmap_fresh{false};
    bool local_tracking_sweep_checked{false};
    bool local_tracking_sweep_clear{false};
    CostmapFootprintSweepResult local_tracking_sweep_result{
      CostmapFootprintSweepResult::kInvalidInput};
    CostmapFootprintSweepDiagnostic local_tracking_sweep_diagnostic;
    std::int64_t local_tracking_costmap_stamp_ns{0};
    std::uint64_t local_tracking_costmap_sequence{0U};
    double local_tracking_costmap_resolution_m{0.0};
    std::string local_tracking_costmap_frame;
    double local_tracking_requested_horizon_m{0.0};
    double local_tracking_covered_horizon_m{0.0};
    std::string local_tracking_lateral_profile{kForwardPathLateralProfileSymmetric};
    bool local_tracking_lateral_profile_active{false};
    bool static_keepout_sweep_checked{false};
    bool static_keepout_sweep_clear{true};
    StaticKeepoutMaskSweepResult static_keepout_sweep_result{
      StaticKeepoutMaskSweepResult::kNoMask};
    CostmapFootprintSweepDiagnostic static_keepout_sweep_diagnostic;
    std::int64_t costmap_stamp_ns{0};
    std::uint64_t costmap_sequence{0U};
    double costmap_resolution_m{0.0};
    std::string costmap_frame;
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
  bool readLocalTrackingEnvelopeOptions();
  bool beginCandidateSearch();
  bool goalsChanged();
  bool prepareTargetCandidates(bool reset_search = true);
  bool prepareLookaheadCandidates();
  bool readDepartureConnectorOptions();
  bool prepareDepartureConnectors();
  BT::NodeStatus advanceDepartureConnector();
  const geometry_msgs::msg::PoseStamped & plannerStartForCandidate() const;
  bool buildCandidateSegment(
    const nav_msgs::msg::Path & planner_segment,
    nav_msgs::msg::Path & candidate_segment,
    std::string & reason) const;
  bool startCandidateQuery();
  bool startLookaheadQuery();
  BT::NodeStatus completeCandidate(
    const nav_msgs::msg::Path * candidate_path,
    bool explicit_no_valid_path = false);
  BT::NodeStatus completeThroughCandidate(
    const nav_msgs::msg::Path & candidate_path,
    double edge_length_m);
  BT::NodeStatus completeThroughPath();
  BT::NodeStatus completeLookahead(
    const nav_msgs::msg::Path * candidate_path,
    bool explicit_no_valid_path = false);
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
  bool hasFreshPlanningCostmaps() const;
  bool hasFreshGlobalCostmap() const;
  bool hasFreshLocalFilteredCostmap() const;
  bool hasStaticKeepoutMask() const;
  bool pDepartureStaticKeepoutMaskRequired() const;
  bool hasAcceptableCostmapSample(const PathQuality & quality) const;
  bool staticKeepoutSweepIsInfrastructureFailure(const PathQuality & quality) const;
  bool localTrackingSweepIsInfrastructureFailure(const PathQuality & quality) const;
  void logFootprintSweepFailure(
    const PathQuality & quality,
    const char * scope,
    std::size_t identifier,
    std::size_t goal_index) const;
  void logStaticKeepoutSweepFailure(
    const PathQuality & quality,
    const char * scope,
    std::size_t identifier,
    std::size_t goal_index) const;
  void logLocalTrackingSweepFailure(
    const PathQuality & quality,
    const char * scope,
    std::size_t identifier,
    std::size_t goal_index) const;
  bool publishCostmapBarrier();
  BT::NodeStatus failPlannerQuery(const char * reason, bool lookahead_query);
  std::optional<std::uint8_t> costAt(
    const nav2_msgs::msg::Costmap & costmap, double x, double y) const;
  bool configureKeepoutMaskSubscription(const std::string & topic);
  void updateGlobalCostmap(nav2_msgs::msg::Costmap::SharedPtr costmap);
  void updateLocalCostmap(nav2_msgs::msg::Costmap::SharedPtr costmap);
  void updateLocalFilteredCostmap(nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
  void updateStaticKeepoutMask(nav_msgs::msg::OccupancyGrid::SharedPtr mask);
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
  double max_start_drift_m_{0.10};
  std::uint8_t maximum_path_cost_{252U};
  CostmapFootprintSweepOptions footprint_sweep_options_;
  bool local_tracking_envelope_enabled_{false};
  double local_tracking_cross_track_error_m_{0.0};
  double local_tracking_horizon_m_{0.0};
  std::uint8_t local_tracking_lethal_cost_{254U};
  std::string local_tracking_lateral_profile_{kForwardPathLateralProfileSymmetric};
  ForwardPathLateralProfileStart local_tracking_lateral_profile_start_;
  ReversePathValidationOptions validation_options_;
  bool departure_connector_enabled_{false};
  double departure_connector_start_x_m_{0.0};
  double departure_connector_start_y_m_{0.0};
  double departure_connector_start_yaw_rad_{0.0};
  double departure_connector_start_position_tolerance_m_{0.10};
  double departure_connector_start_yaw_tolerance_rad_{0.15};
  double departure_connector_maximum_active_radius_m_{0.0};
  double departure_connector_terminal_radius_m_{0.0};
  double departure_connector_high_right_turn_radius_m_{0.0};
  int departure_connector_heading_bins_{0};
  DepartureConnectorOptions departure_connector_options_;

  geometry_msgs::msg::PoseStamped real_start_;
  geometry_msgs::msg::PoseStamped planning_virtual_start_;
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
  std::vector<DepartureConnector> departure_connectors_;
  std::size_t departure_connector_index_{0};
  std::vector<ThroughSearchFrame> through_search_frames_;
  std::size_t through_candidate_query_count_{0};
  std::optional<nav_msgs::msg::Path> best_through_path_;
  std::optional<PathQuality> best_through_quality_;
  std::size_t through_complete_path_count_{0};
  bool waiting_for_costmap_{false};
  bool planner_query_failed_{false};
  bool search_budget_exhausted_{false};

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_filtered_costmap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_keepout_mask_subscription_;
  mutable std::mutex global_costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr global_costmap_;
  nav2_msgs::msg::Costmap::SharedPtr local_costmap_;
  nav2_msgs::msg::Costmap::SharedPtr local_filtered_costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_keepout_mask_;
  std::string static_keepout_mask_topic_;
  std::chrono::steady_clock::time_point global_costmap_received_at_;
  std::chrono::steady_clock::time_point local_costmap_received_at_;
  std::chrono::steady_clock::time_point local_filtered_costmap_received_at_;
  std::int64_t global_costmap_stamp_ns_{0};
  std::int64_t local_costmap_stamp_ns_{0};
  std::int64_t local_filtered_costmap_stamp_ns_{0};
  std::uint64_t global_costmap_sequence_{0U};
  std::uint64_t local_costmap_sequence_{0U};
  std::uint64_t local_filtered_costmap_sequence_{0U};

  QueryState query_state_{QueryState::IDLE};
  std::shared_future<PlannerGoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<PlannerGoalHandle::WrappedResult> result_future_;
  std::shared_future<CancelResponse::SharedPtr> cancel_future_;
  PlannerGoalHandle::SharedPtr active_goal_handle_;
  std::chrono::steady_clock::time_point query_deadline_;
  bool failure_after_cancellation_{false};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__COMPUTE_FREE_HEADING_PATH_ACTION_HPP_
