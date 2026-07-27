#ifndef SMART_CAR_NAV2__COMPUTE_REVERSE_PATH_THROUGH_POSES_ACTION_HPP_
#define SMART_CAR_NAV2__COMPUTE_REVERSE_PATH_THROUGH_POSES_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

#include "smartcar_nav2/reverse_path_utils.hpp"

namespace smartcar_nav2
{

class ComputeReversePathThroughPosesAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathThroughPoses>
{
public:
  ComputeReversePathThroughPosesAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & configuration);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>(
          "path", "Validated reverse path through poses in the real vehicle frame"),
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
          "goals", "Real vehicle destinations to plan through"),
        BT::InputPort<std::string>(
          "planner_id", "", "Planner plugin name"),
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
      });
  }

private:
  bool readValidationOptions();
  void clearPathOutput();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  geometry_msgs::msg::PoseStamped real_start_;
  std::vector<geometry_msgs::msg::PoseStamped> real_goals_;
  ReversePathValidationOptions validation_options_;
};

}  // namespace smartcar_nav2

#endif  // SMART_CAR_NAV2__COMPUTE_REVERSE_PATH_THROUGH_POSES_ACTION_HPP_
