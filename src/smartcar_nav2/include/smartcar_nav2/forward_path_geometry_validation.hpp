#ifndef SMARTCAR_NAV2__FORWARD_PATH_GEOMETRY_VALIDATION_HPP_
#define SMARTCAR_NAV2__FORWARD_PATH_GEOMETRY_VALIDATION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace smartcar_nav2
{

constexpr double kForwardPathGeometryTwoPi = 6.28318530717958647692;
constexpr double kForwardPathGeometryQuaternionNormTolerance = 1.0e-3;

struct ForwardPathGeometryValidationOptions
{
  double minimum_turning_radius{0.55};
  double curvature_tolerance{0.20};
  double maximum_direction_error{0.35};
  double terminal_tangent_tolerance{0.15};
  double minimum_segment_length{1.0e-4};
};

struct ForwardPathGeometryValidationResult
{
  bool valid{false};
  std::string reason;
  std::size_t segment_index{0U};
  double observed_value{0.0};
  double limit{0.0};
};

inline bool forwardPathGeometryFinite(double value)
{
  return std::isfinite(value);
}

inline bool forwardPathGeometryUnitQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!forwardPathGeometryFinite(orientation.x) ||
    !forwardPathGeometryFinite(orientation.y) ||
    !forwardPathGeometryFinite(orientation.z) ||
    !forwardPathGeometryFinite(orientation.w))
  {
    return false;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  return forwardPathGeometryFinite(norm) &&
         std::abs(norm - 1.0) <= kForwardPathGeometryQuaternionNormTolerance;
}

inline bool forwardPathGeometryPoseYaw(
  const geometry_msgs::msg::PoseStamped & pose, double & yaw)
{
  if (!forwardPathGeometryFinite(pose.pose.position.x) ||
    !forwardPathGeometryFinite(pose.pose.position.y) ||
    !forwardPathGeometryFinite(pose.pose.position.z) ||
    !forwardPathGeometryUnitQuaternion(pose.pose.orientation))
  {
    return false;
  }
  const auto & orientation = pose.pose.orientation;
  yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  return forwardPathGeometryFinite(yaw);
}

inline double forwardPathGeometryAngularDistance(double first, double second)
{
  return std::abs(std::remainder(
      second - first, kForwardPathGeometryTwoPi));
}

inline ForwardPathGeometryValidationResult forwardPathGeometryInvalid(
  const std::string & reason, std::size_t segment_index = 0U,
  double observed_value = 0.0, double limit = 0.0)
{
  return ForwardPathGeometryValidationResult{
    false, reason, segment_index, observed_value, limit};
}

inline bool forwardPathGeometryOptionsValid(
  const ForwardPathGeometryValidationOptions & options)
{
  const std::array<double, 5U> values = {
    options.minimum_turning_radius,
    options.curvature_tolerance,
    options.maximum_direction_error,
    options.terminal_tangent_tolerance,
    options.minimum_segment_length,
  };
  if (!std::all_of(
      values.begin(), values.end(),
      [](double value) {return forwardPathGeometryFinite(value);}))
  {
    return false;
  }
  return options.minimum_turning_radius > 0.0 &&
         options.curvature_tolerance >= 0.0 &&
         options.maximum_direction_error > 0.0 &&
         options.maximum_direction_error < kForwardPathGeometryTwoPi * 0.25 &&
         options.terminal_tangent_tolerance >= 0.0 &&
         options.terminal_tangent_tolerance <= options.maximum_direction_error &&
         options.minimum_segment_length > 0.0;
}

// Validate the geometry that a forward-only controller will be asked to
// track. This intentionally checks both the sampled centreline and adjacent
// pose headings: a quantized terminal quaternion can otherwise look safe to
// a footprint sweep but demand an impossible instantaneous steering change.
inline ForwardPathGeometryValidationResult validateForwardPathGeometry(
  const nav_msgs::msg::Path & path,
  const ForwardPathGeometryValidationOptions & options,
  bool require_terminal_tangent)
{
  if (!forwardPathGeometryOptionsValid(options)) {
    return forwardPathGeometryInvalid("options_invalid");
  }
  if (path.poses.size() < 2U) {
    return forwardPathGeometryInvalid("path_too_short");
  }
  if (path.header.frame_id.empty()) {
    return forwardPathGeometryInvalid("path_frame_missing");
  }

  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index];
    double ignored_yaw = 0.0;
    if (pose.header.frame_id != path.header.frame_id ||
      !forwardPathGeometryPoseYaw(pose, ignored_yaw))
    {
      return forwardPathGeometryInvalid("pose_invalid", index);
    }
  }

  const double minimum_projection = std::cos(options.maximum_direction_error);
  const double maximum_curvature =
    1.0 / options.minimum_turning_radius + options.curvature_tolerance;
  for (std::size_t index = 0U; index + 1U < path.poses.size(); ++index) {
    const auto & current = path.poses[index];
    const auto & next = path.poses[index + 1U];
    const double delta_x = next.pose.position.x - current.pose.position.x;
    const double delta_y = next.pose.position.y - current.pose.position.y;
    const double segment_length = std::hypot(delta_x, delta_y);
    if (!forwardPathGeometryFinite(segment_length) ||
      segment_length < options.minimum_segment_length)
    {
      return forwardPathGeometryInvalid("segment_too_short", index);
    }

    double current_yaw = 0.0;
    double next_yaw = 0.0;
    if (!forwardPathGeometryPoseYaw(current, current_yaw) ||
      !forwardPathGeometryPoseYaw(next, next_yaw))
    {
      return forwardPathGeometryInvalid("pose_invalid", index);
    }
    const double current_projection =
      (delta_x * std::cos(current_yaw) + delta_y * std::sin(current_yaw)) /
      segment_length;
    const double next_projection =
      (delta_x * std::cos(next_yaw) + delta_y * std::sin(next_yaw)) /
      segment_length;
    const double observed_projection = std::min(current_projection, next_projection);
    if (!forwardPathGeometryFinite(observed_projection) ||
      observed_projection < minimum_projection)
    {
      return forwardPathGeometryInvalid(
        "segment_not_forward", index, observed_projection, minimum_projection);
    }

    const double orientation_curvature = forwardPathGeometryAngularDistance(
      current_yaw, next_yaw) / segment_length;
    if (!forwardPathGeometryFinite(orientation_curvature) ||
      orientation_curvature > maximum_curvature)
    {
      return forwardPathGeometryInvalid(
        "orientation_curvature_exceeded", index, orientation_curvature, maximum_curvature);
    }
  }

  for (std::size_t index = 1U; index + 1U < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1U].pose.position;
    const double first_x = current.x - previous.x;
    const double first_y = current.y - previous.y;
    const double second_x = next.x - current.x;
    const double second_y = next.y - current.y;
    const double chord_x = next.x - previous.x;
    const double chord_y = next.y - previous.y;
    const double first_length = std::hypot(first_x, first_y);
    const double second_length = std::hypot(second_x, second_y);
    const double chord_length = std::hypot(chord_x, chord_y);
    if (!forwardPathGeometryFinite(first_length) ||
      !forwardPathGeometryFinite(second_length) ||
      !forwardPathGeometryFinite(chord_length) ||
      first_length < options.minimum_segment_length ||
      second_length < options.minimum_segment_length ||
      chord_length < options.minimum_segment_length)
    {
      return forwardPathGeometryInvalid("curvature_segment_too_short", index);
    }
    const double curvature = 2.0 * std::abs(first_x * chord_y - first_y * chord_x) /
      (first_length * second_length * chord_length);
    if (!forwardPathGeometryFinite(curvature) || curvature > maximum_curvature) {
      return forwardPathGeometryInvalid(
        "geometric_curvature_exceeded", index, curvature, maximum_curvature);
    }
  }

  if (require_terminal_tangent) {
    const auto & penultimate = path.poses[path.poses.size() - 2U];
    const auto & terminal = path.poses.back();
    const double tangent_yaw = std::atan2(
      terminal.pose.position.y - penultimate.pose.position.y,
      terminal.pose.position.x - penultimate.pose.position.x);
    double terminal_yaw = 0.0;
    if (!forwardPathGeometryFinite(tangent_yaw) ||
      !forwardPathGeometryPoseYaw(terminal, terminal_yaw))
    {
      return forwardPathGeometryInvalid("terminal_tangent_invalid", path.poses.size() - 2U);
    }
    const double tangent_error = forwardPathGeometryAngularDistance(tangent_yaw, terminal_yaw);
    if (tangent_error > options.terminal_tangent_tolerance) {
      return forwardPathGeometryInvalid(
        "terminal_tangent_mismatch", path.poses.size() - 2U,
        tangent_error, options.terminal_tangent_tolerance);
    }
  }

  return ForwardPathGeometryValidationResult{true, "ok", 0U, 0.0, 0.0};
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FORWARD_PATH_GEOMETRY_VALIDATION_HPP_
