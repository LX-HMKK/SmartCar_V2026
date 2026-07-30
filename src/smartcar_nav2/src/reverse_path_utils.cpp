#include "smartcar_nav2/reverse_path_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace smartcar_nav2
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kQuaternionNormTolerance = 1.0e-3;

bool finite(double value)
{
  return std::isfinite(value);
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return finite(quaternion.x) && finite(quaternion.y) &&
         finite(quaternion.z) && finite(quaternion.w);
}

double quaternionNorm(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
}

bool validUnitQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!finiteQuaternion(quaternion)) {
    return false;
  }
  const double norm = quaternionNorm(quaternion);
  return finite(norm) && std::abs(norm - 1.0) <= kQuaternionNormTolerance;
}

bool validExpectedPose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!finite(pose.pose.position.x) || !finite(pose.pose.position.y) ||
      !finite(pose.pose.position.z) || !finiteQuaternion(pose.pose.orientation))
  {
    return false;
  }
  // Zero quaternion is the Nav2 convention for "orientation unconstrained"
  // (pass-through waypoint). Expected start and goal poses may therefore
  // omit orientation, unlike planner-returned path poses.
  const double norm = quaternionNorm(pose.pose.orientation);
  return norm <= 1.0e-6 || std::abs(norm - 1.0) <= kQuaternionNormTolerance;
}

bool validPathPose(const geometry_msgs::msg::PoseStamped & pose)
{
  return finite(pose.pose.position.x) && finite(pose.pose.position.y) &&
         finite(pose.pose.position.z) && validUnitQuaternion(pose.pose.orientation);
}

double planarDistance(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(
    second.pose.position.x - first.pose.position.x,
    second.pose.position.y - first.pose.position.y);
}

double angularDistance(double first, double second)
{
  return std::abs(std::remainder(second - first, 2.0 * kPi));
}

ReversePathValidationResult invalidResult(
  const std::string & reason, std::size_t segment_index = 0,
  double observed_value = 0.0, double limit = 0.0)
{
  return ReversePathValidationResult{
    false, reason, segment_index, observed_value, limit};
}

bool validOptions(const ReversePathValidationOptions & options)
{
  const std::array<double, 8> values = {
    options.minimum_turning_radius,
    options.curvature_tolerance,
    options.maximum_direction_error,
    options.start_position_tolerance,
    options.start_yaw_tolerance,
    options.goal_position_tolerance,
    options.goal_yaw_tolerance,
    options.minimum_segment_length,
  };
  if (!std::all_of(values.begin(), values.end(), finite)) {
    return false;
  }
  return options.minimum_turning_radius > 0.0 &&
         options.curvature_tolerance >= 0.0 &&
         options.maximum_direction_error > 0.0 &&
         options.maximum_direction_error < kPi / 2.0 &&
         options.start_position_tolerance >= 0.0 &&
         options.start_yaw_tolerance >= 0.0 &&
         options.goal_position_tolerance >= 0.0 &&
         options.goal_yaw_tolerance >= 0.0 &&
         options.minimum_segment_length > 0.0;
}

}  // namespace

bool rotateYawByPi(
  const geometry_msgs::msg::Quaternion & input,
  geometry_msgs::msg::Quaternion & output)
{
  if (!finiteQuaternion(input)) {
    return false;
  }
  const double norm = quaternionNorm(input);
  // Zero quaternion: pass-through waypoint with no orientation constraint.
  // Preserve as-is (no yaw to rotate).
  if (norm <= 1.0e-6) {
    output = input;
    return true;
  }
  if (!validUnitQuaternion(input)) {
    return false;
  }

  tf2::Quaternion source;
  tf2::fromMsg(input, source);
  tf2::Quaternion half_turn;
  half_turn.setRPY(0.0, 0.0, kPi);
  tf2::Quaternion rotated = half_turn * source;
  rotated.normalize();
  output = tf2::toMsg(rotated);
  return finiteQuaternion(output);
}

bool rotatePoseYawByPi(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output)
{
  output = input;
  return rotateYawByPi(input.pose.orientation, output.pose.orientation);
}

ReversePathValidationResult validateReversePath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & expected_start,
  const geometry_msgs::msg::PoseStamped & expected_goal,
  const ReversePathValidationOptions & options)
{
  if (!validOptions(options)) {
    return invalidResult("options_invalid");
  }
  if (path.poses.size() < 2) {
    return invalidResult("path_too_short");
  }
  if (path.header.frame_id.empty() ||
    expected_start.header.frame_id != path.header.frame_id ||
    expected_goal.header.frame_id != path.header.frame_id)
  {
    return invalidResult("path_frame_mismatch");
  }
  if (!validExpectedPose(expected_start) || !validExpectedPose(expected_goal)) {
    return invalidResult("expected_pose_invalid");
  }

  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index];
    if (pose.header.frame_id != path.header.frame_id) {
      return invalidResult("pose_frame_mismatch", index);
    }
    if (!validPathPose(pose)) {
      return invalidResult("pose_invalid", index);
    }
  }

  const auto & path_start = path.poses.front();
  const auto & path_goal = path.poses.back();
  const bool start_orient_constrained =
      quaternionNorm(expected_start.pose.orientation) > 1.0e-6;
  const bool goal_orient_constrained =
      quaternionNorm(expected_goal.pose.orientation) > 1.0e-6;

  if (planarDistance(path_start, expected_start) > options.start_position_tolerance) {
    return invalidResult("start_position_mismatch", 0,
      planarDistance(path_start, expected_start), options.start_position_tolerance);
  }
  if (start_orient_constrained &&
      angularDistance(
        tf2::getYaw(path_start.pose.orientation),
        tf2::getYaw(expected_start.pose.orientation)) > options.start_yaw_tolerance)
  {
    return invalidResult("start_yaw_mismatch", 0,
      angularDistance(tf2::getYaw(path_start.pose.orientation),
                      tf2::getYaw(expected_start.pose.orientation)),
      options.start_yaw_tolerance);
  }
  if (planarDistance(path_goal, expected_goal) > options.goal_position_tolerance) {
    return invalidResult("goal_position_mismatch", 0,
      planarDistance(path_goal, expected_goal), options.goal_position_tolerance);
  }
  if (goal_orient_constrained &&
      angularDistance(
        tf2::getYaw(path_goal.pose.orientation),
        tf2::getYaw(expected_goal.pose.orientation)) > options.goal_yaw_tolerance)
  {
    return invalidResult("goal_yaw_mismatch", 0,
      angularDistance(tf2::getYaw(path_goal.pose.orientation),
                      tf2::getYaw(expected_goal.pose.orientation)),
      options.goal_yaw_tolerance);
  }

  const double maximum_projection = -std::cos(options.maximum_direction_error);
  const double maximum_curvature =
    1.0 / options.minimum_turning_radius + options.curvature_tolerance;

  for (std::size_t index = 0; index + 1 < path.poses.size(); ++index) {
    const auto & current = path.poses[index];
    const auto & next = path.poses[index + 1];
    const double dx = next.pose.position.x - current.pose.position.x;
    const double dy = next.pose.position.y - current.pose.position.y;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length < options.minimum_segment_length) {
      return invalidResult("segment_too_short", index);
    }

    const double current_yaw = tf2::getYaw(current.pose.orientation);
    const double next_yaw = tf2::getYaw(next.pose.orientation);
    const double current_projection =
      (dx * std::cos(current_yaw) + dy * std::sin(current_yaw)) / segment_length;
    const double next_projection =
      (dx * std::cos(next_yaw) + dy * std::sin(next_yaw)) / segment_length;
    const double observed_projection = std::max(current_projection, next_projection);
    if (observed_projection > maximum_projection) {
      return invalidResult(
        "segment_not_reverse", index, observed_projection, maximum_projection);
    }
  }

  // Smac quantizes pose yaw to angle bins. Dividing adjacent yaw deltas by
  // short sampled segments therefore reports false curvature spikes near the
  // path endpoints. The circumcircle through three positions measures the
  // actual path geometry while the projection checks above still enforce
  // reverse-only motion and reject cusps.
  for (std::size_t index = 1; index + 1 < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1];
    const auto & current = path.poses[index];
    const auto & next = path.poses[index + 1];
    const double first_length = planarDistance(previous, current);
    const double second_length = planarDistance(current, next);
    const double chord_length = planarDistance(previous, next);
    if (chord_length < options.minimum_segment_length) {
      return invalidResult("curvature_exceeded", index);
    }

    const double first_x =
      current.pose.position.x - previous.pose.position.x;
    const double first_y =
      current.pose.position.y - previous.pose.position.y;
    const double chord_x = next.pose.position.x - previous.pose.position.x;
    const double chord_y = next.pose.position.y - previous.pose.position.y;
    const double twice_area = std::abs(first_x * chord_y - first_y * chord_x);
    const double curvature =
      2.0 * twice_area / (first_length * second_length * chord_length);
    if (!finite(curvature) || curvature > maximum_curvature) {
      return invalidResult(
        "curvature_exceeded", index, curvature, maximum_curvature);
    }
  }

  return ReversePathValidationResult{true, "ok", 0, 0.0, 0.0};
}

}  // namespace smartcar_nav2
