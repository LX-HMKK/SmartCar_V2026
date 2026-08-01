#ifndef SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ACTION_HPP_
#define SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ACTION_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"

namespace smartcar_nav2
{

// A reverse-only recovery action. It never publishes Twist directly: the
// generated path goes through controller_server, velocity_smoother, the
// direction lease gate, and smartcar_safety like every other navigation path.
class AckermannReverseRetreatAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  AckermannReverseRetreatAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & configuration);

  void on_tick() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
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
          "retreat_distance_m", 0.15, "Straight physical reverse distance in meters"),
        BT::InputPort<int>(
          "costmap_max_age_ms", 1500, "Maximum accepted local/global costmap age"),
        BT::InputPort<double>(
          "footprint_half_length_m", 0.30,
          "Padded vehicle half length for the recovery footprint sweep"),
        BT::InputPort<double>(
          "footprint_half_width_m", 0.16,
          "Padded vehicle half width for the recovery footprint sweep"),
        BT::InputPort<double>(
          "footprint_sweep_step_m", 0.025,
          "Maximum footprint-sweep sample spacing"),
        BT::InputPort<int>(
          "footprint_lethal_cost", 253,
          "Costmap value at or above which recovery is rejected"),
        BT::OutputPort<nav_msgs::msg::Path>(
          "path", "Validated short reverse recovery path"),
      });
  }

private:
  void clearPathOutput();
  bool readSweepOptions();
  bool retreatPathIsClear(const nav_msgs::msg::Path & path) const;
  void updateGlobalCostmap(nav2_msgs::msg::Costmap::SharedPtr costmap);
  void updateLocalCostmap(nav2_msgs::msg::Costmap::SharedPtr costmap);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  std::chrono::milliseconds costmap_max_age_{1500};
  CostmapFootprintSweepOptions footprint_sweep_options_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_subscription_;
  mutable std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr global_costmap_;
  nav2_msgs::msg::Costmap::SharedPtr local_costmap_;
  std::chrono::steady_clock::time_point global_costmap_received_at_;
  std::chrono::steady_clock::time_point local_costmap_received_at_;
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ACTION_HPP_
