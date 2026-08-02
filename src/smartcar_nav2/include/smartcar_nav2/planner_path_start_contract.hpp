#ifndef SMARTCAR_NAV2__PLANNER_PATH_START_CONTRACT_HPP_
#define SMARTCAR_NAV2__PLANNER_PATH_START_CONTRACT_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace smartcar_nav2
{

// Smac normally returns its first pose at the requested start.  A small
// allowance covers grid-cell and heading-bin quantization, but remains below
// the action's 0.10 m stale-start limit.  Any accepted spatial gap is charged
// to the edge length before detour budgets are evaluated.
constexpr double kPlannerPathStartPositionToleranceM = 0.05;
constexpr double kPlannerPathStartYawToleranceRad = 0.10;
constexpr double kPlannerPathStartQuaternionNormTolerance = 1.0e-3;
constexpr double kPlannerPathStartTwoPi = 6.28318530717958647692;

struct PlannerPathStartContinuityResult
{
  bool valid{false};
  std::string reason;
  double join_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double yaw_error_rad{std::numeric_limits<double>::quiet_NaN()};
};

inline bool plannerPathStartFinite(double value)
{
  return std::isfinite(value);
}

inline bool plannerPathStartQuaternionIsUnit(const geometry_msgs::msg::Quaternion & orientation)
{
  if (!plannerPathStartFinite(orientation.x) || !plannerPathStartFinite(orientation.y) ||
    !plannerPathStartFinite(orientation.z) || !plannerPathStartFinite(orientation.w))
  {
    return false;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  return std::abs(norm - 1.0) <= kPlannerPathStartQuaternionNormTolerance;
}

inline double plannerPathStartYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double sine = 2.0 * (
    orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosine = 1.0 - 2.0 * (
    orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sine, cosine);
}

inline double plannerPathStartAngularDistance(double first, double second)
{
  return std::abs(std::remainder(second - first, kPlannerPathStartTwoPi));
}

inline PlannerPathStartContinuityResult validatePlannerPathStartContinuity(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & requested_start,
  double position_tolerance_m = kPlannerPathStartPositionToleranceM,
  double yaw_tolerance_rad = kPlannerPathStartYawToleranceRad)
{
  PlannerPathStartContinuityResult result;
  if (!plannerPathStartFinite(position_tolerance_m) || position_tolerance_m < 0.0 ||
    !plannerPathStartFinite(yaw_tolerance_rad) || yaw_tolerance_rad < 0.0)
  {
    result.reason = "start_contract_tolerance_invalid";
    return result;
  }
  if (path.poses.empty()) {
    result.reason = "path_empty";
    return result;
  }
  if (requested_start.header.frame_id.empty()) {
    result.reason = "requested_start_frame_missing";
    return result;
  }
  if (path.header.frame_id != requested_start.header.frame_id) {
    result.reason = "path_frame_mismatch";
    return result;
  }

  const auto & first = path.poses.front();
  if (first.header.frame_id != requested_start.header.frame_id) {
    result.reason = "first_pose_frame_mismatch";
    return result;
  }
  if (!plannerPathStartFinite(requested_start.pose.position.x) ||
    !plannerPathStartFinite(requested_start.pose.position.y) ||
    !plannerPathStartFinite(first.pose.position.x) ||
    !plannerPathStartFinite(first.pose.position.y) ||
    !plannerPathStartQuaternionIsUnit(requested_start.pose.orientation) ||
    !plannerPathStartQuaternionIsUnit(first.pose.orientation))
  {
    result.reason = "start_pose_invalid";
    return result;
  }

  result.join_gap_m = std::hypot(
    first.pose.position.x - requested_start.pose.position.x,
    first.pose.position.y - requested_start.pose.position.y);
  if (result.join_gap_m > position_tolerance_m) {
    result.reason = "start_position_mismatch";
    return result;
  }

  result.yaw_error_rad = plannerPathStartAngularDistance(
    plannerPathStartYaw(requested_start.pose.orientation),
    plannerPathStartYaw(first.pose.orientation));
  if (result.yaw_error_rad > yaw_tolerance_rad) {
    result.reason = "start_yaw_mismatch";
    return result;
  }

  result.valid = true;
  return result;
}

inline double plannerPathLengthIncludingStartJoin(
  const nav_msgs::msg::Path & path,
  const PlannerPathStartContinuityResult & continuity)
{
  if (!continuity.valid || !plannerPathStartFinite(continuity.join_gap_m) ||
    continuity.join_gap_m < 0.0 || path.poses.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double length_m = continuity.join_gap_m;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    if (!plannerPathStartFinite(previous.x) || !plannerPathStartFinite(previous.y) ||
      !plannerPathStartFinite(current.x) || !plannerPathStartFinite(current.y))
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    length_m += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return plannerPathStartFinite(length_m) ? length_m :
         std::numeric_limits<double>::quiet_NaN();
}

// Smac may quantize a requested start onto its grid while keeping the gap
// within validatePlannerPathStartContinuity()'s explicit contract.  When an
// already validated, physically executable prefix must be prepended, replace
// that quantized first sample with the prefix endpoint rather than publishing
// a discontinuous join.  The caller must still validate the resulting path's
// kinematics and collision sweep before accepting it.
inline bool snapPlannerPathStartToRequestedPose(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & requested_start,
  const PlannerPathStartContinuityResult & continuity)
{
  if (!continuity.valid || path.poses.empty() ||
    path.header.frame_id != requested_start.header.frame_id ||
    path.poses.front().header.frame_id != requested_start.header.frame_id)
  {
    return false;
  }
  path.poses.front() = requested_start;
  return true;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__PLANNER_PATH_START_CONTRACT_HPP_
