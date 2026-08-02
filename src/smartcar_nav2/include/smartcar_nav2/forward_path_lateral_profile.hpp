#ifndef SMARTCAR_NAV2__FORWARD_PATH_LATERAL_PROFILE_HPP_
#define SMARTCAR_NAV2__FORWARD_PATH_LATERAL_PROFILE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace smartcar_nav2
{

// The P start is close to the south field boundary.  A symmetric tracking
// tube is therefore physically impossible during the first left turn even
// though the nominal padded footprint is clear.  This Gazebo-only profile
// limits only the path-right (south/outside) error until the connector has
// gained enough northward clearance.  The companion controller guard uses
// the same table; all other paths retain a symmetric tube.
inline constexpr char kForwardPathLateralProfileSymmetric[] = "symmetric";
inline constexpr char kForwardPathLateralProfilePDepartureSouthV1[] =
  "p_departure_south_v1";

struct ForwardPathLateralEnvelope
{
  double left_cross_track_error_m{0.0};
  double right_cross_track_error_m{0.0};
};

// The P-specific profile is only legal for a plan which still begins at the
// configured P pose. A replan from a later vehicle pose must fall back to the
// normal symmetric envelope instead of applying a station table with the
// wrong origin.
struct ForwardPathLateralProfileStart
{
  std::string frame_id;
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double position_tolerance_m{0.0};
  double yaw_tolerance_rad{0.0};
};

enum class ForwardPathLateralProfilePathMatch
{
  kMatches,
  kDoesNotMatch,
  kInvalid,
};

inline bool forwardPathLateralProfileFinite(double value)
{
  return std::isfinite(value);
}

inline bool forwardPathLateralProfileKnown(const std::string & profile)
{
  return profile == kForwardPathLateralProfileSymmetric ||
         profile == kForwardPathLateralProfilePDepartureSouthV1;
}

inline bool forwardPathLateralProfileStartValid(
  const std::string & profile,
  const ForwardPathLateralProfileStart & start)
{
  if (!forwardPathLateralProfileKnown(profile)) {
    return false;
  }
  if (profile == kForwardPathLateralProfileSymmetric) {
    return true;
  }
  return !start.frame_id.empty() && forwardPathLateralProfileFinite(start.x_m) &&
         forwardPathLateralProfileFinite(start.y_m) &&
         forwardPathLateralProfileFinite(start.yaw_rad) &&
         forwardPathLateralProfileFinite(start.position_tolerance_m) &&
         forwardPathLateralProfileFinite(start.yaw_tolerance_rad) &&
         start.position_tolerance_m > 0.0 && start.yaw_tolerance_rad > 0.0;
}

inline bool forwardPathLateralProfilePoseYaw(
  const geometry_msgs::msg::PoseStamped & pose,
  double & yaw_rad)
{
  const auto & orientation = pose.pose.orientation;
  if (!forwardPathLateralProfileFinite(pose.pose.position.x) ||
    !forwardPathLateralProfileFinite(pose.pose.position.y) ||
    !forwardPathLateralProfileFinite(orientation.x) ||
    !forwardPathLateralProfileFinite(orientation.y) ||
    !forwardPathLateralProfileFinite(orientation.z) ||
    !forwardPathLateralProfileFinite(orientation.w))
  {
    return false;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!forwardPathLateralProfileFinite(norm) || std::abs(norm - 1.0) > 1.0e-3) {
    return false;
  }
  yaw_rad = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  return forwardPathLateralProfileFinite(yaw_rad);
}

inline ForwardPathLateralProfilePathMatch forwardPathLateralProfileMatchesPlan(
  const std::string & profile,
  const nav_msgs::msg::Path & path,
  const ForwardPathLateralProfileStart & start)
{
  if (!forwardPathLateralProfileStartValid(profile, start)) {
    return ForwardPathLateralProfilePathMatch::kInvalid;
  }
  if (profile == kForwardPathLateralProfileSymmetric) {
    return ForwardPathLateralProfilePathMatch::kMatches;
  }
  if (path.header.frame_id != start.frame_id || path.poses.size() < 2U) {
    return ForwardPathLateralProfilePathMatch::kDoesNotMatch;
  }
  if (path.poses.front().header.frame_id != start.frame_id) {
    return ForwardPathLateralProfilePathMatch::kInvalid;
  }
  double path_start_yaw = 0.0;
  if (!forwardPathLateralProfilePoseYaw(path.poses.front(), path_start_yaw)) {
    return ForwardPathLateralProfilePathMatch::kInvalid;
  }
  const double position_error = std::hypot(
    path.poses.front().pose.position.x - start.x_m,
    path.poses.front().pose.position.y - start.y_m);
  const double yaw_error = std::abs(std::remainder(path_start_yaw - start.yaw_rad,
        6.28318530717958647692));
  if (!forwardPathLateralProfileFinite(position_error) ||
    !forwardPathLateralProfileFinite(yaw_error))
  {
    return ForwardPathLateralProfilePathMatch::kInvalid;
  }
  if (position_error > start.position_tolerance_m || yaw_error > start.yaw_tolerance_rad) {
    return ForwardPathLateralProfilePathMatch::kDoesNotMatch;
  }

  // The profile is not a general P-area relaxation. Match the first 0.375 m
  // of the actual staged connector: a 0.45 m, 32.5-degree lead-in followed by
  // the 0.22 m left arc. This prevents a later generic replan which merely
  // happens to start near P from inheriting the south-side allowance.
  struct PrefixAnchor
  {
    double station_m;
    double turn_rad;
  };
  constexpr std::array<PrefixAnchor, 3U> kPDepartureAnchors = {{
      {0.100, 0.2222222222222222},
      {0.225, 0.5000000000000000},
      {0.375, 1.1115301746064720},
    }};
  constexpr double kLeadRadiusM = 0.45;
  constexpr double kTightRadiusM = 0.22;
  constexpr double kLeadTurnRad = 13.0 * 3.14159265358979323846 / 72.0;
  constexpr double kAnchorPositionToleranceM = 0.015;
  constexpr double kAnchorYawToleranceRad = 0.03;
  const double lead_length = kLeadRadiusM * kLeadTurnRad;
  double accumulated_station_m = 0.0;
  std::size_t next_anchor = 0U;
  double previous_yaw = path_start_yaw;
  for (std::size_t index = 1U; index < path.poses.size() &&
    next_anchor < kPDepartureAnchors.size(); ++index)
  {
    const auto & first = path.poses[index - 1U];
    const auto & second = path.poses[index];
    if (first.header.frame_id != start.frame_id || second.header.frame_id != start.frame_id) {
      return ForwardPathLateralProfilePathMatch::kInvalid;
    }
    double first_yaw = 0.0;
    double second_yaw = 0.0;
    if (!forwardPathLateralProfilePoseYaw(first, first_yaw) ||
      !forwardPathLateralProfilePoseYaw(second, second_yaw))
    {
      return ForwardPathLateralProfilePathMatch::kInvalid;
    }
    const double segment_length = std::hypot(
      second.pose.position.x - first.pose.position.x,
      second.pose.position.y - first.pose.position.y);
    const double yaw_delta = std::remainder(second_yaw - first_yaw, 6.28318530717958647692);
    if (!forwardPathLateralProfileFinite(segment_length) ||
      !forwardPathLateralProfileFinite(yaw_delta) || segment_length <= 1.0e-6 ||
      yaw_delta < -1.0e-4 || std::abs(std::remainder(first_yaw - previous_yaw,
      6.28318530717958647692)) > 1.0e-3)
    {
      return ForwardPathLateralProfilePathMatch::kDoesNotMatch;
    }
    while (next_anchor < kPDepartureAnchors.size() &&
      accumulated_station_m + segment_length + 1.0e-9 >=
      kPDepartureAnchors[next_anchor].station_m)
    {
      const double fraction = std::clamp(
        (kPDepartureAnchors[next_anchor].station_m - accumulated_station_m) /
        segment_length, 0.0, 1.0);
      const double observed_yaw = first_yaw + yaw_delta * fraction;
      const double expected_turn = kPDepartureAnchors[next_anchor].turn_rad;
      double expected_local_x = 0.0;
      double expected_local_y = 0.0;
      if (kPDepartureAnchors[next_anchor].station_m <= lead_length) {
        expected_local_x = kLeadRadiusM * std::sin(expected_turn);
        expected_local_y = kLeadRadiusM * (1.0 - std::cos(expected_turn));
      } else {
        expected_local_x = kLeadRadiusM * std::sin(kLeadTurnRad) + kTightRadiusM *
          (std::sin(expected_turn) - std::sin(kLeadTurnRad));
        expected_local_y = kLeadRadiusM * (1.0 - std::cos(kLeadTurnRad)) +
          kTightRadiusM * (std::cos(kLeadTurnRad) - std::cos(expected_turn));
      }
      const double observed_x = first.pose.position.x +
        (second.pose.position.x - first.pose.position.x) * fraction;
      const double observed_y = first.pose.position.y +
        (second.pose.position.y - first.pose.position.y) * fraction;
      const double expected_x = start.x_m + std::cos(start.yaw_rad) * expected_local_x -
        std::sin(start.yaw_rad) * expected_local_y;
      const double expected_y = start.y_m + std::sin(start.yaw_rad) * expected_local_x +
        std::cos(start.yaw_rad) * expected_local_y;
      if (std::abs(std::remainder(
          observed_yaw - (start.yaw_rad + expected_turn), 6.28318530717958647692)) >
        kAnchorYawToleranceRad ||
        std::hypot(observed_x - expected_x, observed_y - expected_y) >
        kAnchorPositionToleranceM)
      {
        return ForwardPathLateralProfilePathMatch::kDoesNotMatch;
      }
      ++next_anchor;
    }
    accumulated_station_m += segment_length;
    previous_yaw = second_yaw;
  }
  return next_anchor == kPDepartureAnchors.size() ?
         ForwardPathLateralProfilePathMatch::kMatches :
         ForwardPathLateralProfilePathMatch::kDoesNotMatch;
}

inline bool forwardPathLateralProfileConfigurationValid(
  const std::string & profile,
  double maximum_cross_track_error_m,
  double minimum_turning_radius_m)
{
  if (!forwardPathLateralProfileKnown(profile) ||
    !forwardPathLateralProfileFinite(maximum_cross_track_error_m) ||
    !forwardPathLateralProfileFinite(minimum_turning_radius_m) ||
    maximum_cross_track_error_m < 0.0 || minimum_turning_radius_m <= 0.0)
  {
    return false;
  }
  return profile != kForwardPathLateralProfilePDepartureSouthV1 ||
         (std::abs(maximum_cross_track_error_m - 0.12) <= 1.0e-9 &&
         std::abs(minimum_turning_radius_m - 0.22) <= 1.0e-9);
}

inline std::optional<ForwardPathLateralEnvelope> forwardPathLateralEnvelopeAtStation(
  const std::string & profile,
  double station_m,
  double maximum_cross_track_error_m)
{
  if (!forwardPathLateralProfileKnown(profile) ||
    !forwardPathLateralProfileFinite(station_m) || station_m < 0.0 ||
    !forwardPathLateralProfileFinite(maximum_cross_track_error_m) ||
    maximum_cross_track_error_m <= 0.0)
  {
    return std::nullopt;
  }
  if (profile == kForwardPathLateralProfileSymmetric) {
    return ForwardPathLateralEnvelope{
      maximum_cross_track_error_m, maximum_cross_track_error_m};
  }

  // Right-side caps are derived from the P connector plus its deterministic
  // RSL terminal's continuous padded-footprint sweep against pFieldCostmap.
  // The first staged turn remains constrained by the south field boundary.
  // After the northbound leg has cleared that boundary and A1, retain a
  // deliberately capped 50 mm right allowance: zero is not a usable
  // closed-loop contract, because normal Gazebo pose/projection quantization
  // can report a sub-millimetre path-right error on an otherwise clear pose.
  struct RightLimitPoint
  {
    double station_m;
    double right_limit_m;
  };
  constexpr std::array<RightLimitPoint, 8U> kPDepartureRightLimits = {{
      {0.000, 0.075},
      {0.100, 0.025},
      {0.150, 0.010},
      {0.200, 0.0075},
      {0.225, 0.0075},
      {1.350, 0.0075},
      {1.450, 0.050},
      {4.750, 0.050},
    }};

  double right_limit = kPDepartureRightLimits.front().right_limit_m;
  if (station_m >= kPDepartureRightLimits.back().station_m) {
    right_limit = kPDepartureRightLimits.back().right_limit_m;
  } else {
    for (std::size_t index = 1U; index < kPDepartureRightLimits.size(); ++index) {
      const auto & previous = kPDepartureRightLimits[index - 1U];
      const auto & current = kPDepartureRightLimits[index];
      if (station_m <= current.station_m) {
        const double fraction = (station_m - previous.station_m) /
          (current.station_m - previous.station_m);
        right_limit = previous.right_limit_m +
          (current.right_limit_m - previous.right_limit_m) * fraction;
        break;
      }
    }
  }
  if (!forwardPathLateralProfileFinite(right_limit) || right_limit < 0.0) {
    return std::nullopt;
  }
  return ForwardPathLateralEnvelope{
    maximum_cross_track_error_m,
    std::min(maximum_cross_track_error_m, right_limit)};
}

inline std::optional<double> forwardPathLateralCrossTrackLimit(
  const std::string & profile,
  double station_m,
  double signed_cross_track_m,
  double maximum_cross_track_error_m)
{
  if (!forwardPathLateralProfileFinite(signed_cross_track_m)) {
    return std::nullopt;
  }
  const auto envelope = forwardPathLateralEnvelopeAtStation(
    profile, station_m, maximum_cross_track_error_m);
  if (!envelope.has_value()) {
    return std::nullopt;
  }
  return signed_cross_track_m >= 0.0 ?
         envelope->left_cross_track_error_m : envelope->right_cross_track_error_m;
}

inline bool forwardPathLateralCrossTrackExceeded(
  const std::string & profile,
  double station_m,
  double signed_cross_track_m,
  double maximum_cross_track_error_m)
{
  const auto limit = forwardPathLateralCrossTrackLimit(
    profile, station_m, signed_cross_track_m, maximum_cross_track_error_m);
  return !limit.has_value() || std::abs(signed_cross_track_m) > *limit + 1.0e-9;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FORWARD_PATH_LATERAL_PROFILE_HPP_
