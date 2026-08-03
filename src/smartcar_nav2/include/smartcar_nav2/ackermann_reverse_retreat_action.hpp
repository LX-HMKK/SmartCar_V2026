#ifndef SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ACTION_HPP_
#define SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ACTION_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/ackermann_reverse_retreat_odom_guard.hpp"
#include "smartcar_nav2/costmap_footprint_sweep.hpp"
#include "smartcar_nav2/costmap_sample_guard.hpp"

namespace smartcar_nav2
{

// A reverse-only recovery action. It never publishes Twist directly: the
// generated path goes through controller_server, velocity_smoother, the
// direction lease gate, and smartcar_safety like every other navigation path.
class AckermannReverseRetreatAction
  : public BT::StatefulActionNode
{
public:
  AckermannReverseRetreatAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & configuration);
  ~AckermannReverseRetreatAction() override;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<bool>(
          "allow_retreat", false,
          "True only after the reverse planner exhausts its feasible candidates"),
        BT::BidirectionalPort<bool>(
          "retreat_used", false,
          "Persistent per-navigation-action gate preventing a second physical retreat"),
        BT::InputPort<std::string>(
          "controller_id", "ReverseRecovery", "Reverse-only controller plugin"),
        BT::InputPort<std::string>(
          "goal_checker_id", "recovery_goal_checker", "Strict recovery goal checker"),
        BT::InputPort<double>(
          "retreat_distance_m", 0.15, "Straight physical recovery distance in meters"),
        BT::InputPort<std::string>(
          "retreat_direction", "reverse",
          "Physical recovery direction: reverse normally, forward for a reverse-arrival handoff"),
        BT::InputPort<int>(
          "costmap_max_age_ms", 1500, "Maximum accepted local/global costmap age"),
        BT::InputPort<int>(
          "perception_wait_timeout_ms", 2600,
          "Post-clear nonblocking wait for fresh local/global raw costmaps"),
        BT::InputPort<int>(
          "follow_path_goal_timeout_ms", 1000,
          "Maximum asynchronous FollowPath goal-acceptance wait"),
        BT::InputPort<int>(
          "follow_path_result_timeout_ms", 12000,
          "Maximum physical FollowPath execution time for the short retreat"),
        BT::InputPort<int>(
          "scan_costmap_fusion_lag_ms", 1250,
          "Maximum source-time lag from an associated scan to its raw costmap update"),
        BT::InputPort<std::string>(
          "static_keepout_mask_topic", "",
          "Optional static KeepoutFilter mask; when configured, its full padded-body sweep must clear"),
        BT::InputPort<bool>(
          "allow_static_scan_only_evidence", false,
          "Allow a static-map-only scan witness after the keepout full-body sweep"),
        BT::InputPort<std::int64_t>(
          "costmap_min_stamp_ns", 0,
          "Planner global-costmap stamp; retreat requires a strictly newer raw sample"),
        BT::InputPort<std::int64_t>(
          "local_costmap_min_stamp_ns", 0,
          "Planner local-costmap stamp; retreat requires a strictly newer raw sample"),
        BT::InputPort<double>(
          "footprint_half_length_m", 0.2491,
          "Padded vehicle half length for the recovery footprint sweep"),
        BT::InputPort<double>(
          "footprint_half_width_m", 0.095,
          "Padded vehicle half width for the recovery footprint sweep"),
        BT::InputPort<double>(
          "footprint_sweep_step_m", 0.025,
          "Maximum footprint-sweep sample spacing"),
        BT::InputPort<int>(
          "footprint_lethal_cost", 253,
          "Costmap value at or above which recovery is rejected"),
        BT::InputPort<double>(
          "scan_min_obstacle_range_m", 0.25,
          "Nearest scan return accepted as obstacle evidence"),
        BT::InputPort<double>(
          "scan_max_obstacle_range_m", 2.50,
          "Farthest scan return accepted as obstacle evidence"),
        BT::InputPort<double>(
          "scan_costmap_match_radius_m", 0.12,
          "Maximum scan-endpoint to lethal-costmap witness distance"),
        BT::InputPort<int>(
          "retreat_odom_max_age_ms", 500,
          "Maximum accepted /odom_combined sample age while retreating"),
        BT::InputPort<double>(
          "retreat_odom_max_step_m", 0.05,
          "Reject an implausible single odometry displacement during retreat"),
        BT::InputPort<double>(
          "retreat_odom_max_travel_m", 0.19,
          "Hard cumulative odometry travel cap for the 0.15 m recovery"),
        BT::InputPort<double>(
          "retreat_odom_max_displacement_m", 0.19,
          "Hard odometry displacement cap for the 0.15 m recovery"),
        BT::OutputPort<nav_msgs::msg::Path>(
          "path", "Validated short reverse recovery path"),
      };
  }

private:
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using FollowPathClient = rclcpp_action::Client<FollowPath>;

  enum class State
  {
    IDLE,
    WAITING_FOR_PERCEPTION,
    WAITING_FOR_GOAL_HANDLE,
    WAITING_FOR_RESULT,
  };

  struct ScanSample
  {
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    std::chrono::steady_clock::time_point received_at;
    std::uint64_t sequence{0U};
    std::int64_t stamp_ns{0};
  };

  struct OdomSample
  {
    AckermannReverseRetreatOdomSample sample;
    std::string frame_id;
    std::string child_frame_id;
    std::int64_t stamp_ns{0};
  };

  // The action response callback can win the race against a BT halt, before
  // waitForGoalHandle() transfers the future result into goal_handle_. Keep
  // that acknowledged handle available to cancelFollowPath() in either order.
  struct GoalResponseHandle
  {
    std::uint64_t generation{0U};
    FollowPathGoalHandle::SharedPtr handle;
  };

  void clearPathOutput();
  void resetOperation();
  bool loadInputs();
  bool readSweepOptions();
  bool readScanWitnessOptions();
  bool readOdomOptions(double retreat_distance_m);
  bool configureKeepoutMaskSubscription(const std::string & topic);
  void armCostmapBarrier();
  BT::NodeStatus waitForPerception();
  BT::NodeStatus dispatchRetreat();
  BT::NodeStatus waitForGoalHandle();
  BT::NodeStatus waitForResult();
  void cancelFollowPath();
  void startLateGoalAcknowledgementGuard();
  void reapLateGoalAcknowledgementGuard();
  void stopLateGoalAcknowledgementGuard();
  bool armRetreatOdomGuard(std::string & reason);
  bool dispatchedRetreatIsSafe(std::string & reason);
  bool odomIsFresh(OdomSample & sample, std::string & reason) const;
  bool retreatPathIsClear(
    const nav_msgs::msg::Path & path, std::string & reason) const;
  bool scanIsFresh(const ScanSample & sample, std::string & reason) const;
  bool scanCouldHaveBeenFused(
    const CostmapSample & sample, const ScanSample & scan, std::string & reason) const;
  bool scanPointsInGlobalFrame(
    const ScanSample & sample,
    std::vector<std::pair<double, double>> & points,
    std::string & reason) const;
  bool staticKeepoutMaskPathIsClear(
    const nav_msgs::msg::OccupancyGrid * mask,
    const nav_msgs::msg::Path & path,
    std::string & reason) const;
  bool filterStaticKeepoutPoints(
    const std::vector<std::pair<double, double>> & points,
    const nav_msgs::msg::OccupancyGrid * mask,
    std::vector<std::pair<double, double>> & filtered_points,
    std::string & reason) const;
  static ScanSample newestScanAtOrBefore(
    const std::deque<ScanSample> & samples, std::int64_t costmap_stamp_ns);
  void updateScan(sensor_msgs::msg::LaserScan::SharedPtr scan);
  void updateGlobalCostmap(nav2_msgs::msg::Costmap::SharedPtr costmap);
  void updateLocalCostmap(nav2_msgs::msg::Costmap::SharedPtr costmap);
  void updateStaticKeepoutMask(nav_msgs::msg::OccupancyGrid::SharedPtr mask);
  void updateOdometry(nav_msgs::msg::Odometry::SharedPtr odometry);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  FollowPathClient::SharedPtr follow_path_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr accepted_path_publisher_;
  std::string action_name_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  std::chrono::milliseconds costmap_max_age_{1500};
  std::chrono::milliseconds perception_wait_timeout_{2600};
  std::chrono::milliseconds follow_path_goal_timeout_{1000};
  std::chrono::milliseconds follow_path_result_timeout_{12000};
  std::chrono::milliseconds scan_costmap_fusion_lag_{1250};
  double retreat_distance_m_{0.15};
  bool retreat_forward_{false};
  std::int64_t costmap_min_stamp_ns_{0};
  std::int64_t local_costmap_min_stamp_ns_{0};
  CostmapFootprintSweepOptions footprint_sweep_options_;
  double scan_min_obstacle_range_m_{0.25};
  double scan_max_obstacle_range_m_{2.50};
  double scan_costmap_match_radius_m_{0.12};
  bool allow_static_scan_only_evidence_{false};
  AckermannReverseRetreatOdomLimits retreat_odom_limits_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_keepout_mask_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  mutable std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr global_costmap_;
  nav2_msgs::msg::Costmap::SharedPtr local_costmap_;
  std::chrono::steady_clock::time_point global_costmap_received_at_;
  std::chrono::steady_clock::time_point local_costmap_received_at_;
  std::int64_t global_costmap_stamp_ns_{0};
  std::int64_t local_costmap_stamp_ns_{0};
  std::uint64_t global_costmap_sequence_{0U};
  std::uint64_t local_costmap_sequence_{0U};
  std::uint64_t global_costmap_barrier_sequence_{0U};
  std::uint64_t local_costmap_barrier_sequence_{0U};
  std::uint64_t global_costmap_scan_sequence_{0U};
  std::uint64_t local_costmap_scan_sequence_{0U};
  std::int64_t global_costmap_scan_stamp_ns_{0};
  std::int64_t local_costmap_scan_stamp_ns_{0};
  ScanSample scan_;
  ScanSample global_costmap_scan_;
  ScanSample local_costmap_scan_;
  std::deque<ScanSample> scan_history_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_keepout_mask_;
  std::string static_keepout_mask_topic_;
  std::uint64_t scan_barrier_sequence_{0U};
  std::int64_t scan_barrier_stamp_ns_{0};
  std::int64_t post_clear_ros_stamp_ns_{0};
  mutable std::mutex odom_mutex_;
  std::optional<OdomSample> latest_odom_;
  AckermannReverseRetreatOdomGuard retreat_odom_guard_;

  State state_{State::IDLE};
  nav_msgs::msg::Path retreat_path_;
  FollowPath::Goal goal_;
  std::shared_future<FollowPathGoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<FollowPathGoalHandle::SharedPtr> late_goal_handle_future_;
  std::shared_future<FollowPathGoalHandle::WrappedResult> result_future_;
  FollowPathGoalHandle::SharedPtr goal_handle_;
  std::mutex goal_response_handle_mutex_;
  GoalResponseHandle goal_response_handle_;
  std::chrono::steady_clock::time_point perception_deadline_;
  std::chrono::steady_clock::time_point goal_handle_deadline_;
  std::chrono::steady_clock::time_point result_deadline_;
  std::uint64_t active_dispatch_generation_{0U};
  std::atomic<std::uint64_t> next_dispatch_generation_{0U};
  std::atomic<std::uint64_t> cancelled_dispatch_generation_{0U};
  std::atomic<bool> late_goal_acknowledgement_pending_{false};
  std::atomic<bool> stop_late_goal_acknowledgement_guard_{false};
  std::thread late_goal_acknowledgement_guard_;
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ACTION_HPP_
