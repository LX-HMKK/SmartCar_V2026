#ifndef SMARTCAR_NAV2__FORWARD_PATH_TRACKING_GUARD_HPP_
#define SMARTCAR_NAV2__FORWARD_PATH_TRACKING_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace smartcar_nav2
{

constexpr double kForwardPathTrackingQuaternionNormTolerance = 1.0e-3;
constexpr double kForwardPathTrackingMinimumSegmentLengthM = 1.0e-6;
constexpr double kForwardPathTrackingTwoPi = 6.28318530717958647692;

struct ForwardPathTrackingProjection
{
  bool valid{false};
  std::string reason;
  double station_m{std::numeric_limits<double>::quiet_NaN()};
  double cross_track_m{std::numeric_limits<double>::quiet_NaN()};
  double signed_cross_track_m{std::numeric_limits<double>::quiet_NaN()};
  double path_heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double path_tangent_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double projected_x_m{std::numeric_limits<double>::quiet_NaN()};
  double projected_y_m{std::numeric_limits<double>::quiet_NaN()};
  double remaining_path_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t segment_index{0U};
};

struct ForwardPathTrackingCurvature
{
  bool valid{false};
  std::string reason;
  double curvature_m_inv{std::numeric_limits<double>::quiet_NaN()};
};

inline bool forwardPathTrackingFinite(double value)
{
  return std::isfinite(value);
}

inline bool forwardPathTrackingQuaternionIsUnit(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!forwardPathTrackingFinite(orientation.x) ||
    !forwardPathTrackingFinite(orientation.y) ||
    !forwardPathTrackingFinite(orientation.z) ||
    !forwardPathTrackingFinite(orientation.w))
  {
    return false;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  return std::abs(norm - 1.0) <= kForwardPathTrackingQuaternionNormTolerance;
}

inline bool forwardPathTrackingPoseYaw(
  const geometry_msgs::msg::PoseStamped & pose, double & yaw)
{
  if (!forwardPathTrackingFinite(pose.pose.position.x) ||
    !forwardPathTrackingFinite(pose.pose.position.y) ||
    !forwardPathTrackingQuaternionIsUnit(pose.pose.orientation))
  {
    return false;
  }
  const auto & orientation = pose.pose.orientation;
  yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  return forwardPathTrackingFinite(yaw);
}

inline ForwardPathTrackingProjection projectForwardPathTrackingPose(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  ForwardPathTrackingProjection result;
  if (path.header.frame_id.empty()) {
    result.reason = "path_frame_missing";
    return result;
  }
  if (robot_pose.header.frame_id != path.header.frame_id) {
    result.reason = "robot_path_frame_mismatch";
    return result;
  }
  if (path.poses.size() < 2U) {
    result.reason = "path_too_short";
    return result;
  }

  double robot_yaw = 0.0;
  if (!forwardPathTrackingPoseYaw(robot_pose, robot_yaw)) {
    result.reason = "robot_pose_invalid";
    return result;
  }
  for (const auto & pose : path.poses) {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) ||
      !forwardPathTrackingFinite(pose.pose.position.x) ||
      !forwardPathTrackingFinite(pose.pose.position.y))
    {
      result.reason = "path_pose_invalid";
      return result;
    }
  }

  double accumulated_station_m = 0.0;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  bool have_segment = false;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & first = path.poses[index - 1U].pose.position;
    const auto & second = path.poses[index].pose.position;
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double segment_length_squared = dx * dx + dy * dy;
    if (!forwardPathTrackingFinite(segment_length_squared)) {
      result.reason = "path_segment_invalid";
      return result;
    }
    const double segment_length = std::sqrt(segment_length_squared);
    if (segment_length <= kForwardPathTrackingMinimumSegmentLengthM) {
      continue;
    }

    have_segment = true;
    const double projection_factor = std::clamp(
      ((robot_pose.pose.position.x - first.x) * dx +
      (robot_pose.pose.position.y - first.y) * dy) / segment_length_squared,
      0.0, 1.0);
    const double projected_x = first.x + projection_factor * dx;
    const double projected_y = first.y + projection_factor * dy;
    const double cross_track_x = robot_pose.pose.position.x - projected_x;
    const double cross_track_y = robot_pose.pose.position.y - projected_y;
    const double distance_squared =
      cross_track_x * cross_track_x + cross_track_y * cross_track_y;
    if (!forwardPathTrackingFinite(distance_squared)) {
      result.reason = "projection_invalid";
      return result;
    }
    const double candidate_station_m =
      accumulated_station_m + projection_factor * segment_length;
    if (distance_squared < best_distance_squared) {
      const double tangent_yaw = std::atan2(dy, dx);
      best_distance_squared = distance_squared;
      result.station_m = candidate_station_m;
      result.cross_track_m = std::sqrt(distance_squared);
      result.signed_cross_track_m =
        -std::sin(tangent_yaw) * cross_track_x + std::cos(tangent_yaw) * cross_track_y;
      result.path_heading_error_rad = std::remainder(
        robot_yaw - tangent_yaw, kForwardPathTrackingTwoPi);
      result.path_tangent_yaw_rad = tangent_yaw;
      result.projected_x_m = projected_x;
      result.projected_y_m = projected_y;
      result.segment_index = index - 1U;
    }
    accumulated_station_m += segment_length;
  }

  if (!have_segment) {
    result.reason = "path_has_no_nonzero_segment";
    return result;
  }
  if (!forwardPathTrackingFinite(result.station_m) ||
    !forwardPathTrackingFinite(result.cross_track_m) ||
    !forwardPathTrackingFinite(result.signed_cross_track_m) ||
    !forwardPathTrackingFinite(result.path_heading_error_rad) ||
    !forwardPathTrackingFinite(result.path_tangent_yaw_rad) ||
    !forwardPathTrackingFinite(result.projected_x_m) ||
    !forwardPathTrackingFinite(result.projected_y_m))
  {
    result.reason = "projection_invalid";
    return result;
  }
  result.remaining_path_m = accumulated_station_m - result.station_m;
  if (!forwardPathTrackingFinite(result.remaining_path_m) || result.remaining_path_m < 0.0) {
    result.reason = "remaining_path_invalid";
    return result;
  }
  result.valid = true;
  return result;
}

// The planner publishes a tangent orientation at each sampled path pose. Use
// that local differential, rather than a distant pure-pursuit carrot, when a
// narrow swept connector changes radius beside a keepout boundary.
inline ForwardPathTrackingCurvature forwardPathTrackingLocalCurvature(
  const nav_msgs::msg::Path & path,
  const ForwardPathTrackingProjection & projection)
{
  ForwardPathTrackingCurvature result;
  if (!projection.valid || projection.segment_index + 1U >= path.poses.size()) {
    result.reason = "projection_segment_unavailable";
    return result;
  }

  const auto & first = path.poses[projection.segment_index];
  const auto & second = path.poses[projection.segment_index + 1U];
  if ((!first.header.frame_id.empty() && first.header.frame_id != path.header.frame_id) ||
    (!second.header.frame_id.empty() && second.header.frame_id != path.header.frame_id))
  {
    result.reason = "path_frame_mismatch";
    return result;
  }
  const double segment_length = std::hypot(
    second.pose.position.x - first.pose.position.x,
    second.pose.position.y - first.pose.position.y);
  if (!forwardPathTrackingFinite(segment_length) ||
    segment_length <= kForwardPathTrackingMinimumSegmentLengthM)
  {
    result.reason = "curvature_segment_invalid";
    return result;
  }

  double first_yaw = 0.0;
  double second_yaw = 0.0;
  if (!forwardPathTrackingPoseYaw(first, first_yaw) ||
    !forwardPathTrackingPoseYaw(second, second_yaw))
  {
    result.reason = "curvature_orientation_invalid";
    return result;
  }
  const double yaw_delta = std::remainder(
    second_yaw - first_yaw, kForwardPathTrackingTwoPi);
  result.curvature_m_inv = yaw_delta / segment_length;
  if (!forwardPathTrackingFinite(result.curvature_m_inv)) {
    result.reason = "curvature_invalid";
    return result;
  }
  result.valid = true;
  return result;
}

// Return true when a sampled section of the accepted path at, or shortly
// ahead of, the current projection requires a radius no larger than the
// supplied threshold. This lets a simulator controller slow before a steering
// actuator has to move into a tight Ackermann arc; it never changes the path
// or relaxes collision checking.
inline bool forwardPathTrackingTightTurnAhead(
  const nav_msgs::msg::Path & path,
  const ForwardPathTrackingProjection & projection,
  double tight_turn_radius_m,
  double preview_distance_m)
{
  if (!projection.valid || !forwardPathTrackingFinite(tight_turn_radius_m) ||
    !forwardPathTrackingFinite(preview_distance_m) || tight_turn_radius_m <= 0.0 ||
    preview_distance_m < 0.0 || projection.segment_index + 1U >= path.poses.size())
  {
    return false;
  }

  const double minimum_curvature_m_inv = 1.0 / tight_turn_radius_m;
  double remaining_preview_m = preview_distance_m;
  for (std::size_t index = projection.segment_index;
    index + 1U < path.poses.size(); ++index)
  {
    ForwardPathTrackingProjection local_projection = projection;
    local_projection.segment_index = index;
    const auto curvature = forwardPathTrackingLocalCurvature(path, local_projection);
    if (!curvature.valid) {
      return false;
    }
    if (std::abs(curvature.curvature_m_inv) + 1.0e-9 >= minimum_curvature_m_inv) {
      return true;
    }

    const auto & first = path.poses[index].pose.position;
    const auto & second = path.poses[index + 1U].pose.position;
    double distance_to_next_segment_m = std::hypot(second.x - first.x, second.y - first.y);
    if (index == projection.segment_index) {
      distance_to_next_segment_m = std::hypot(
        second.x - projection.projected_x_m,
        second.y - projection.projected_y_m);
    }
    if (!forwardPathTrackingFinite(distance_to_next_segment_m) ||
      distance_to_next_segment_m <= kForwardPathTrackingMinimumSegmentLengthM ||
      distance_to_next_segment_m > remaining_preview_m + 1.0e-9)
    {
      return false;
    }
    remaining_preview_m -= distance_to_next_segment_m;
  }
  return false;
}

inline bool forwardPathTrackingCrossTrackExceeded(
  const ForwardPathTrackingProjection & projection, double maximum_cross_track_m)
{
  return !projection.valid || !forwardPathTrackingFinite(maximum_cross_track_m) ||
         maximum_cross_track_m <= 0.0 ||
         projection.cross_track_m > maximum_cross_track_m;
}

inline double forwardPathTrackingPathLength(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double length = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & first = path.poses[index - 1U].pose.position;
    const auto & second = path.poses[index].pose.position;
    if (!forwardPathTrackingFinite(first.x) || !forwardPathTrackingFinite(first.y) ||
      !forwardPathTrackingFinite(second.x) || !forwardPathTrackingFinite(second.y))
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double segment_length = std::hypot(second.x - first.x, second.y - first.y);
    if (!forwardPathTrackingFinite(segment_length)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    length += segment_length;
  }
  return forwardPathTrackingFinite(length) ? length :
         std::numeric_limits<double>::quiet_NaN();
}

inline bool forwardPathTrackingTerminalLookaheadActive(
  const nav_msgs::msg::Path & path,
  const ForwardPathTrackingProjection & projection,
  double terminal_lookahead_m,
  double activation_distance_m)
{
  if (!projection.valid || !forwardPathTrackingFinite(terminal_lookahead_m) ||
    !forwardPathTrackingFinite(activation_distance_m) || terminal_lookahead_m <= 0.0 ||
    activation_distance_m < terminal_lookahead_m)
  {
    return false;
  }
  const double path_length = forwardPathTrackingPathLength(path);
  if (!forwardPathTrackingFinite(path_length) ||
    projection.station_m < 0.0 || projection.station_m > path_length + 1.0e-6)
  {
    return false;
  }
  return path_length - projection.station_m <= activation_distance_m + 1.0e-6;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FORWARD_PATH_TRACKING_GUARD_HPP_
