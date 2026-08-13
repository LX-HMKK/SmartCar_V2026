#ifndef SMARTCAR_NAV2__REMOVE_DUPLICATE_PATH_POINTS_ACTION_HPP_
#define SMARTCAR_NAV2__REMOVE_DUPLICATE_PATH_POINTS_ACTION_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"

namespace smartcar_nav2
{

class RemoveDuplicatePathPoints : public BT::SyncActionNode
{
public:
  RemoveDuplicatePathPoints(
    const std::string & name,
    const BT::NodeConfiguration & configuration)
  : BT::SyncActionNode(name, configuration)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Path to de-duplicate"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path without consecutive duplicates"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__REMOVE_DUPLICATE_PATH_POINTS_ACTION_HPP_
