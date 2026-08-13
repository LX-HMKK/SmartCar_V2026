#include "smartcar_nav2/remove_duplicate_path_points_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace smartcar_nav2
{

BT::NodeStatus RemoveDuplicatePathPoints::tick()
{
  nav_msgs::msg::Path input_path;
  if (!getInput("input_path", input_path)) {
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path output_path;
  output_path.header = input_path.header;
  output_path.poses.reserve(input_path.poses.size());
  for (const auto & pose : input_path.poses) {
    // ConstrainedSmoother computes segment length in x/y. Planner segments
    // may carry distinct headers or orientations at their shared endpoint, so
    // remove only exactly coincident consecutive planar positions.
    const auto & previous_position = output_path.poses.empty() ?
      pose.pose.position : output_path.poses.back().pose.position;
    if (output_path.poses.empty() ||
      previous_position.x != pose.pose.position.x ||
      previous_position.y != pose.pose.position.y) {
      output_path.poses.push_back(pose);
    }
  }

  setOutput("output_path", output_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace smartcar_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<smartcar_nav2::RemoveDuplicatePathPoints>(
    "RemoveDuplicatePathPoints");
}
