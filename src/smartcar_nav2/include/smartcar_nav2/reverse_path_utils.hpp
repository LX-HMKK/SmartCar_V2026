#ifndef SMARTCAR_NAV2__REVERSE_PATH_UTILS_HPP_
#define SMARTCAR_NAV2__REVERSE_PATH_UTILS_HPP_

#include <cstddef>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"

namespace smartcar_nav2
{

struct ReversePathValidationOptions
{
  double minimum_turning_radius{0.55};
  double curvature_tolerance{0.20};
  double maximum_direction_error{0.35};
  double start_position_tolerance{0.10};
  double start_yaw_tolerance{0.15};
  double goal_position_tolerance{0.20};
  double goal_yaw_tolerance{0.15};
  double minimum_segment_length{1.0e-4};
};

struct ReversePathValidationResult
{
  bool valid{false};
  std::string reason;
  std::size_t segment_index{0};
  double observed_value{0.0};
  double limit{0.0};
};

bool rotateYawByPi(
  const geometry_msgs::msg::Quaternion & input,
  geometry_msgs::msg::Quaternion & output);

bool rotatePoseYawByPi(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output);

ReversePathValidationResult validateReversePath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & expected_start,
  const geometry_msgs::msg::PoseStamped & expected_goal,
  const ReversePathValidationOptions & options);

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__REVERSE_PATH_UTILS_HPP_
