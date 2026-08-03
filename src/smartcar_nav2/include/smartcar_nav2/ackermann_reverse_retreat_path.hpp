#ifndef SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_PATH_HPP_
#define SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_PATH_HPP_

#include <cmath>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"

namespace smartcar_nav2
{

// Construct a short, zero-curvature recovery trajectory along the vehicle's
// physical longitudinal axis. The caller selects -1 for reverse or +1 for
// forward motion and must separately bind the matching controller/lease.
inline bool buildAckermannLinearRetreatPath(
  const geometry_msgs::msg::PoseStamped & start,
  const std::string & global_frame,
  double retreat_distance_m,
  double longitudinal_direction,
  nav_msgs::msg::Path & path)
{
  if (global_frame.empty() || start.header.frame_id != global_frame ||
    !std::isfinite(retreat_distance_m) || retreat_distance_m < 0.05 ||
    retreat_distance_m > 0.25 || !std::isfinite(longitudinal_direction) ||
    std::abs(std::abs(longitudinal_direction) - 1.0) > 1.0e-12 ||
    !std::isfinite(start.pose.position.x) ||
    !std::isfinite(start.pose.position.y))
  {
    return false;
  }

  const auto & orientation = start.pose.orientation;
  const double orientation_norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
    !std::isfinite(orientation_norm) || std::abs(orientation_norm - 1.0) > 1.0e-3)
  {
    return false;
  }

  const double yaw = tf2::getYaw(orientation);
  if (!std::isfinite(yaw)) {
    return false;
  }

  geometry_msgs::msg::PoseStamped end = start;
  end.pose.position.x += longitudinal_direction * retreat_distance_m * std::cos(yaw);
  end.pose.position.y += longitudinal_direction * retreat_distance_m * std::sin(yaw);
  if (!std::isfinite(end.pose.position.x) || !std::isfinite(end.pose.position.y)) {
    return false;
  }

  path = nav_msgs::msg::Path();
  path.header = start.header;
  path.poses = {start, end};
  return true;
}

// Construct the normal reverse-only recovery trajectory along physical -X.
inline bool buildAckermannReverseRetreatPath(
  const geometry_msgs::msg::PoseStamped & start,
  const std::string & global_frame,
  double retreat_distance_m,
  nav_msgs::msg::Path & path)
{
  return buildAckermannLinearRetreatPath(
    start, global_frame, retreat_distance_m, -1.0, path);
}

// C1's reverse-arrival recovery must pull forward, away from the terminal
// pose, before planning another reverse approach.
inline bool buildAckermannForwardRetreatPath(
  const geometry_msgs::msg::PoseStamped & start,
  const std::string & global_frame,
  double retreat_distance_m,
  nav_msgs::msg::Path & path)
{
  return buildAckermannLinearRetreatPath(
    start, global_frame, retreat_distance_m, 1.0, path);
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_PATH_HPP_
