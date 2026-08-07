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
