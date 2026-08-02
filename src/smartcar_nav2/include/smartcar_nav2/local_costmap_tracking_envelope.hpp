#ifndef SMARTCAR_NAV2__LOCAL_COSTMAP_TRACKING_ENVELOPE_HPP_
#define SMARTCAR_NAV2__LOCAL_COSTMAP_TRACKING_ENVELOPE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"
#include "smartcar_nav2/forward_path_lateral_profile.hpp"

namespace smartcar_nav2
{

// A local rolling costmap cannot prove clearance for an entire global route.
// This helper sweeps only the prefix that a controller can reach before the
// next planning/control update, while expanding the body by its admitted
// closed-loop cross-track error.  It intentionally reports an uncovered
// horizon separately from a collision: callers must fail closed rather than
// treating an out-of-window path as clear.
struct LocalCostmapTrackingEnvelopeResult
{
  CostmapFootprintSweepResult sweep_result{CostmapFootprintSweepResult::kInvalidInput};
  CostmapFootprintSweepDiagnostic diagnostic;
  double requested_horizon_m{0.0};
  double covered_horizon_m{0.0};
  bool horizon_covered{false};
};

inline bool localCostmapTrackingFinite(double value)
{
  return std::isfinite(value);
}

// Nav2 publishes its post-filter master grid on ``costmap`` as an
// OccupancyGrid, while ``costmap_raw`` is a nav2_msgs/Costmap.  Convert the
// former into the latter only for the shared continuous footprint sweep.
// Unknown OccupancyGrid cells deliberately become NO_INFORMATION (255), so a
// tracking envelope cannot mistake an unobservable cell for free space.
inline std::optional<nav2_msgs::msg::Costmap>
localCostmapTrackingOccupancyGridToCostmap(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (!localCostmapTrackingFinite(grid.info.resolution) || grid.info.resolution <= 0.0F ||
    grid.info.width == 0U || grid.info.height == 0U)
  {
    return std::nullopt;
  }
  const auto width = static_cast<std::size_t>(grid.info.width);
  const auto height = static_cast<std::size_t>(grid.info.height);
  if (width > std::numeric_limits<std::size_t>::max() / height) {
    return std::nullopt;
  }
  const auto expected_size = width * height;
  if (grid.data.size() != expected_size) {
    return std::nullopt;
  }

  nav2_msgs::msg::Costmap result;
  result.header = grid.header;
  result.metadata.map_load_time = grid.info.map_load_time;
  result.metadata.update_time = grid.header.stamp;
  result.metadata.layer = "filtered_local_occupancy_grid";
  result.metadata.resolution = grid.info.resolution;
  result.metadata.size_x = grid.info.width;
  result.metadata.size_y = grid.info.height;
  result.metadata.origin = grid.info.origin;
  result.data.reserve(expected_size);
  for (const std::int8_t occupancy : grid.data) {
    if (occupancy < 0) {
      result.data.push_back(255U);
    } else if (occupancy > 100) {
      return std::nullopt;
    } else {
      // Costmap2DPublisher maps lethal 254 to OccupancyGrid value 100. Keep
      // the existing 254 collision threshold semantically identical.
      result.data.push_back(static_cast<std::uint8_t>(
        (static_cast<unsigned int>(occupancy) * 254U + 50U) / 100U));
    }
  }
  return result;
}

inline double localCostmapTrackingYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

inline geometry_msgs::msg::PoseStamped localCostmapTrackingInterpolatedPose(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second,
  double fraction)
{
  geometry_msgs::msg::PoseStamped result = first;
  result.pose.position.x = first.pose.position.x +
    (second.pose.position.x - first.pose.position.x) * fraction;
  result.pose.position.y = first.pose.position.y +
    (second.pose.position.y - first.pose.position.y) * fraction;
  result.pose.position.z = first.pose.position.z +
    (second.pose.position.z - first.pose.position.z) * fraction;
  const double first_yaw = localCostmapTrackingYaw(first.pose.orientation);
  const double yaw_delta = std::remainder(
    localCostmapTrackingYaw(second.pose.orientation) - first_yaw,
    6.28318530717958647692);
  const double yaw = first_yaw + yaw_delta * fraction;
  result.pose.orientation.x = 0.0;
  result.pose.orientation.y = 0.0;
  result.pose.orientation.z = std::sin(yaw * 0.5);
  result.pose.orientation.w = std::cos(yaw * 0.5);
  return result;
}

// For an asymmetric tracking tube, the admitted body positions at one path
// station form a rectangle whose centre is shifted toward the larger side and
// whose half-width grows by the mean of both allowances. Sweep that exact
// rectangle at the same conservative spacing as the symmetric body sweep.
inline CostmapFootprintSweepResult localCostmapTrackingProfiledFootprintPathSweep(
  const nav_msgs::msg::Path & path,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options,
  const std::string & lateral_profile,
  double maximum_cross_track_error_m,
  CostmapFootprintSweepDiagnostic * diagnostic = nullptr)
{
  if (!forwardPathLateralProfileKnown(lateral_profile) ||
    !forwardPathLateralProfileFinite(maximum_cross_track_error_m) ||
    maximum_cross_track_error_m < 0.0)
  {
    if (diagnostic != nullptr) {
      *diagnostic = CostmapFootprintSweepDiagnostic();
    }
    detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
    return CostmapFootprintSweepResult::kInvalidInput;
  }
  if (lateral_profile == kForwardPathLateralProfileSymmetric) {
    CostmapFootprintSweepOptions symmetric_options = options;
    symmetric_options.half_width_m += maximum_cross_track_error_m;
    return costmapFootprintPathSweep(path, costmap, symmetric_options, diagnostic);
  }
  if (diagnostic != nullptr) {
    *diagnostic = CostmapFootprintSweepDiagnostic();
  }
  if (path.poses.empty() || !detail::validCostmap(costmap) || !detail::validOptions(options) ||
    maximum_cross_track_error_m <= 0.0)
  {
    detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
    return CostmapFootprintSweepResult::kInvalidInput;
  }

  const auto check_pose = [&costmap, &options, &lateral_profile,
      maximum_cross_track_error_m, diagnostic](
      const geometry_msgs::msg::PoseStamped & pose,
      double station_m) {
      const auto envelope = forwardPathLateralEnvelopeAtStation(
        lateral_profile, station_m, maximum_cross_track_error_m);
      if (!envelope.has_value()) {
        detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
        return CostmapFootprintSweepResult::kInvalidInput;
      }
      double yaw = 0.0;
      if (!forwardPathLateralProfilePoseYaw(pose, yaw)) {
        detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
        return CostmapFootprintSweepResult::kInvalidInput;
      }
      const double centre_shift_m = 0.5 * (
        envelope->left_cross_track_error_m - envelope->right_cross_track_error_m);
      CostmapFootprintSweepOptions shifted_options = options;
      shifted_options.half_width_m += 0.5 * (
        envelope->left_cross_track_error_m + envelope->right_cross_track_error_m);
      if (!localCostmapTrackingFinite(centre_shift_m) ||
        !localCostmapTrackingFinite(shifted_options.half_width_m) ||
        shifted_options.half_width_m <= 0.0)
      {
        detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
        return CostmapFootprintSweepResult::kInvalidInput;
      }
      geometry_msgs::msg::PoseStamped shifted_pose = pose;
      shifted_pose.pose.position.x -= std::sin(yaw) * centre_shift_m;
      shifted_pose.pose.position.y += std::cos(yaw) * centre_shift_m;
      return detail::footprintPoseIsClear(
        shifted_pose, costmap, shifted_options, diagnostic);
    };

  if (diagnostic != nullptr) {
    diagnostic->segment_start_pose_index = 0U;
    diagnostic->segment_end_pose_index = 0U;
  }
  auto result = check_pose(path.poses.front(), 0.0);
  if (result != CostmapFootprintSweepResult::kClear) {
    return result;
  }

  const double spacing = std::min(
    options.sample_spacing_m, static_cast<double>(costmap.metadata.resolution) * 0.5);
  double accumulated_station_m = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U];
    const auto & current = path.poses[index];
    const double delta_x = current.pose.position.x - previous.pose.position.x;
    const double delta_y = current.pose.position.y - previous.pose.position.y;
    const double segment_length = std::hypot(delta_x, delta_y);
    double previous_yaw = 0.0;
    double current_yaw = 0.0;
    if (!localCostmapTrackingFinite(segment_length) || !localCostmapTrackingFinite(spacing) ||
      spacing <= 0.0 || !forwardPathLateralProfilePoseYaw(previous, previous_yaw) ||
      !forwardPathLateralProfilePoseYaw(current, current_yaw))
    {
      detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
      return CostmapFootprintSweepResult::kInvalidInput;
    }
    const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(segment_length / spacing)));
    const double yaw_delta = std::remainder(
      current_yaw - previous_yaw, 6.28318530717958647692);
    if (!localCostmapTrackingFinite(yaw_delta)) {
      detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
      return CostmapFootprintSweepResult::kInvalidInput;
    }
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const double fraction = static_cast<double>(sample) / static_cast<double>(samples);
      geometry_msgs::msg::PoseStamped interpolated = previous;
      interpolated.pose.position.x += delta_x * fraction;
      interpolated.pose.position.y += delta_y * fraction;
      const double yaw = previous_yaw + yaw_delta * fraction;
      interpolated.pose.orientation.x = 0.0;
      interpolated.pose.orientation.y = 0.0;
      interpolated.pose.orientation.z = std::sin(yaw * 0.5);
      interpolated.pose.orientation.w = std::cos(yaw * 0.5);
      if (diagnostic != nullptr) {
        diagnostic->segment_start_pose_index = index - 1U;
        diagnostic->segment_end_pose_index = index;
        diagnostic->segment_sample_index = sample;
        diagnostic->segment_sample_count = samples;
        diagnostic->segment_fraction = fraction;
      }
      result = check_pose(
        interpolated, accumulated_station_m + segment_length * fraction);
      if (result != CostmapFootprintSweepResult::kClear) {
        return result;
      }
    }
    accumulated_station_m += segment_length;
  }
  return CostmapFootprintSweepResult::kClear;
}

inline LocalCostmapTrackingEnvelopeResult localCostmapTrackingEnvelopeSweep(
  const nav_msgs::msg::Path & path,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options,
  double horizon_m,
  const std::string & lateral_profile,
  double maximum_cross_track_error_m)
{
  LocalCostmapTrackingEnvelopeResult result;
  result.requested_horizon_m = horizon_m;
  if (!localCostmapTrackingFinite(horizon_m) || horizon_m < 0.0 || path.poses.empty()) {
    return result;
  }

  nav_msgs::msg::Path prefix;
  prefix.header = path.header;
  prefix.poses.push_back(path.poses.front());
  double covered = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & first = path.poses[index - 1U];
    const auto & second = path.poses[index];
    const double segment_length = std::hypot(
      second.pose.position.x - first.pose.position.x,
      second.pose.position.y - first.pose.position.y);
    if (!localCostmapTrackingFinite(segment_length) || segment_length <= 0.0) {
      result.sweep_result = CostmapFootprintSweepResult::kInvalidInput;
      return result;
    }
    const double remaining = horizon_m - covered;
    if (remaining <= 1.0e-9) {
      break;
    }
    if (segment_length <= remaining + 1.0e-9) {
      prefix.poses.push_back(second);
      covered += segment_length;
      continue;
    }
    prefix.poses.push_back(localCostmapTrackingInterpolatedPose(
      first, second, std::clamp(remaining / segment_length, 0.0, 1.0)));
    covered = horizon_m;
    break;
  }
  result.covered_horizon_m = covered;
  result.horizon_covered = covered + 1.0e-9 >= horizon_m;
  result.sweep_result = localCostmapTrackingProfiledFootprintPathSweep(
    prefix, costmap, options, lateral_profile, maximum_cross_track_error_m, &result.diagnostic);
  return result;
}

inline LocalCostmapTrackingEnvelopeResult localCostmapTrackingEnvelopeSweep(
  const nav_msgs::msg::Path & path,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options,
  double horizon_m)
{
  LocalCostmapTrackingEnvelopeResult result;
  result.requested_horizon_m = horizon_m;
  if (!localCostmapTrackingFinite(horizon_m) || horizon_m < 0.0 || path.poses.empty()) {
    return result;
  }

  nav_msgs::msg::Path prefix;
  prefix.header = path.header;
  prefix.poses.push_back(path.poses.front());
  double covered = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & first = path.poses[index - 1U];
    const auto & second = path.poses[index];
    const double segment_length = std::hypot(
      second.pose.position.x - first.pose.position.x,
      second.pose.position.y - first.pose.position.y);
    if (!localCostmapTrackingFinite(segment_length) || segment_length <= 0.0) {
      result.sweep_result = CostmapFootprintSweepResult::kInvalidInput;
      return result;
    }
    const double remaining = horizon_m - covered;
    if (remaining <= 1.0e-9) {
      break;
    }
    if (segment_length <= remaining + 1.0e-9) {
      prefix.poses.push_back(second);
      covered += segment_length;
      continue;
    }
    prefix.poses.push_back(localCostmapTrackingInterpolatedPose(
      first, second, std::clamp(remaining / segment_length, 0.0, 1.0)));
    covered = horizon_m;
    break;
  }
  result.covered_horizon_m = covered;
  result.horizon_covered = covered + 1.0e-9 >= horizon_m;
  result.sweep_result = costmapFootprintPathSweep(
    prefix, costmap, options, &result.diagnostic);
  return result;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__LOCAL_COSTMAP_TRACKING_ENVELOPE_HPP_
