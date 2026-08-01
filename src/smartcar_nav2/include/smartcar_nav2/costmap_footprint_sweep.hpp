#ifndef SMARTCAR_NAV2__COSTMAP_FOOTPRINT_SWEEP_HPP_
#define SMARTCAR_NAV2__COSTMAP_FOOTPRINT_SWEEP_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"

namespace smartcar_nav2
{

// Check the complete padded rectangular vehicle against lethal cells, rather
// than only sampling the global path centreline. The BT supplies the padded
// half extents so the check follows the active Nav2 footprint configuration.
struct CostmapFootprintSweepOptions
{
  double half_length_m{0.30};
  double half_width_m{0.16};
  double sample_spacing_m{0.025};
  std::uint8_t lethal_cost_threshold{253U};
};

enum class CostmapFootprintSweepResult
{
  kClear,
  kInvalidInput,
  kOutOfBounds,
  kLethalOverlap,
};

inline const char * costmapFootprintSweepResultName(CostmapFootprintSweepResult result)
{
  switch (result) {
    case CostmapFootprintSweepResult::kClear:
      return "clear";
    case CostmapFootprintSweepResult::kInvalidInput:
      return "invalid input";
    case CostmapFootprintSweepResult::kOutOfBounds:
      return "footprint leaves costmap bounds";
    case CostmapFootprintSweepResult::kLethalOverlap:
      return "lethal footprint overlap";
  }
  return "unknown result";
}

namespace detail
{

constexpr double kCostmapFootprintSweepTwoPi = 6.28318530717958647692;

inline bool finite(double value)
{
  return std::isfinite(value);
}

inline bool validCostmap(const nav2_msgs::msg::Costmap & costmap)
{
  if (costmap.metadata.resolution <= 0.0f || costmap.metadata.size_x == 0U ||
    costmap.metadata.size_y == 0U)
  {
    return false;
  }
  const auto expected_size = static_cast<std::size_t>(costmap.metadata.size_x) *
    static_cast<std::size_t>(costmap.metadata.size_y);
  return costmap.data.size() >= expected_size;
}

inline bool validOptions(const CostmapFootprintSweepOptions & options)
{
  return finite(options.half_length_m) && finite(options.half_width_m) &&
         finite(options.sample_spacing_m) && options.half_length_m > 0.0 &&
         options.half_width_m > 0.0 && options.sample_spacing_m > 0.0;
}

inline bool worldToMap(
  const nav2_msgs::msg::Costmap & costmap, double world_x, double world_y,
  double & map_x, double & map_y)
{
  if (!finite(world_x) || !finite(world_y)) {
    return false;
  }
  const double origin_yaw = tf2::getYaw(costmap.metadata.origin.orientation);
  const double delta_x = world_x - costmap.metadata.origin.position.x;
  const double delta_y = world_y - costmap.metadata.origin.position.y;
  map_x = std::cos(origin_yaw) * delta_x + std::sin(origin_yaw) * delta_y;
  map_y = -std::sin(origin_yaw) * delta_x + std::cos(origin_yaw) * delta_y;
  return finite(map_x) && finite(map_y);
}

inline void mapToWorld(
  const nav2_msgs::msg::Costmap & costmap, std::size_t map_x, std::size_t map_y,
  double & world_x, double & world_y)
{
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  const double local_x = (static_cast<double>(map_x) + 0.5) * resolution;
  const double local_y = (static_cast<double>(map_y) + 0.5) * resolution;
  const double origin_yaw = tf2::getYaw(costmap.metadata.origin.orientation);
  world_x = costmap.metadata.origin.position.x +
    std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y;
  world_y = costmap.metadata.origin.position.y +
    std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y;
}

enum class FootprintBoundsResult
{
  kInside,
  kOutOfBounds,
  kInvalidInput,
};

inline FootprintBoundsResult footprintInsideCostmap(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options)
{
  const double yaw = tf2::getYaw(pose.pose.orientation);
  if (!finite(yaw) || !finite(pose.pose.position.x) || !finite(pose.pose.position.y)) {
    return FootprintBoundsResult::kInvalidInput;
  }
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double extent_x = options.half_length_m * std::abs(cosine) +
    options.half_width_m * std::abs(sine);
  const double extent_y = options.half_length_m * std::abs(sine) +
    options.half_width_m * std::abs(cosine);
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  const double map_width = resolution * static_cast<double>(costmap.metadata.size_x);
  const double map_height = resolution * static_cast<double>(costmap.metadata.size_y);
  for (const auto & corner : {
      std::pair<double, double>{pose.pose.position.x - extent_x, pose.pose.position.y - extent_y},
      std::pair<double, double>{pose.pose.position.x - extent_x, pose.pose.position.y + extent_y},
      std::pair<double, double>{pose.pose.position.x + extent_x, pose.pose.position.y - extent_y},
      std::pair<double, double>{pose.pose.position.x + extent_x, pose.pose.position.y + extent_y}})
  {
    double local_x = 0.0;
    double local_y = 0.0;
    if (!worldToMap(costmap, corner.first, corner.second, local_x, local_y)) {
      return FootprintBoundsResult::kInvalidInput;
    }
    if (local_x < 0.0 || local_y < 0.0 || local_x >= map_width || local_y >= map_height) {
      return FootprintBoundsResult::kOutOfBounds;
    }
  }
  return FootprintBoundsResult::kInside;
}

inline CostmapFootprintSweepResult footprintPoseIsClear(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options)
{
  const auto bounds_result = footprintInsideCostmap(pose, costmap, options);
  if (bounds_result == FootprintBoundsResult::kInvalidInput) {
    return CostmapFootprintSweepResult::kInvalidInput;
  }
  if (bounds_result == FootprintBoundsResult::kOutOfBounds) {
    return CostmapFootprintSweepResult::kOutOfBounds;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  // A cell's centre can sit outside the vehicle while its area still overlaps
  // it. Inflate by the cell half-diagonal to make the grid test conservative.
  const double cell_radius = resolution * 0.70710678118654752440;
  const double extent_x = options.half_length_m * std::abs(cosine) +
    options.half_width_m * std::abs(sine) + cell_radius;
  const double extent_y = options.half_length_m * std::abs(sine) +
    options.half_width_m * std::abs(cosine) + cell_radius;

  std::array<std::pair<double, double>, 4> corners = {{
      {pose.pose.position.x - extent_x, pose.pose.position.y - extent_y},
      {pose.pose.position.x - extent_x, pose.pose.position.y + extent_y},
      {pose.pose.position.x + extent_x, pose.pose.position.y - extent_y},
      {pose.pose.position.x + extent_x, pose.pose.position.y + extent_y},
    }};
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & corner : corners) {
    double local_x = 0.0;
    double local_y = 0.0;
    if (!worldToMap(costmap, corner.first, corner.second, local_x, local_y)) {
      return CostmapFootprintSweepResult::kInvalidInput;
    }
    min_x = std::min(min_x, local_x);
    max_x = std::max(max_x, local_x);
    min_y = std::min(min_y, local_y);
    max_y = std::max(max_y, local_y);
  }
  const auto first_x = static_cast<std::size_t>(std::max(
    0.0, std::floor(min_x / resolution)));
  const auto first_y = static_cast<std::size_t>(std::max(
    0.0, std::floor(min_y / resolution)));
  const auto last_x = static_cast<std::size_t>(std::min(
    static_cast<double>(costmap.metadata.size_x - 1U), std::floor(max_x / resolution)));
  const auto last_y = static_cast<std::size_t>(std::min(
    static_cast<double>(costmap.metadata.size_y - 1U), std::floor(max_y / resolution)));
  if (first_x > last_x || first_y > last_y) {
    return CostmapFootprintSweepResult::kInvalidInput;
  }

  for (std::size_t map_y = first_y; map_y <= last_y; ++map_y) {
    for (std::size_t map_x = first_x; map_x <= last_x; ++map_x) {
      const std::size_t index = map_y * static_cast<std::size_t>(costmap.metadata.size_x) +
        map_x;
      if (costmap.data[index] < options.lethal_cost_threshold) {
        continue;
      }
      double world_x = 0.0;
      double world_y = 0.0;
      mapToWorld(costmap, map_x, map_y, world_x, world_y);
      const double delta_x = world_x - pose.pose.position.x;
      const double delta_y = world_y - pose.pose.position.y;
      const double vehicle_x = delta_x * cosine + delta_y * sine;
      const double vehicle_y = -delta_x * sine + delta_y * cosine;
      if (std::abs(vehicle_x) <= options.half_length_m + cell_radius &&
        std::abs(vehicle_y) <= options.half_width_m + cell_radius)
      {
        return CostmapFootprintSweepResult::kLethalOverlap;
      }
    }
  }
  return CostmapFootprintSweepResult::kClear;
}

}  // namespace detail

inline CostmapFootprintSweepResult costmapFootprintPathSweep(
  const nav_msgs::msg::Path & path,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options)
{
  if (path.poses.empty() || !detail::validCostmap(costmap) || !detail::validOptions(options)) {
    return CostmapFootprintSweepResult::kInvalidInput;
  }
  const auto first_result = detail::footprintPoseIsClear(path.poses.front(), costmap, options);
  if (first_result != CostmapFootprintSweepResult::kClear) {
    return first_result;
  }

  const double costmap_step = static_cast<double>(costmap.metadata.resolution) * 0.5;
  const double spacing = std::min(options.sample_spacing_m, costmap_step);
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U];
    const auto & current = path.poses[index];
    const double delta_x = current.pose.position.x - previous.pose.position.x;
    const double delta_y = current.pose.position.y - previous.pose.position.y;
    const double length = std::hypot(delta_x, delta_y);
    if (!detail::finite(length) || !detail::finite(spacing) || spacing <= 0.0) {
      return CostmapFootprintSweepResult::kInvalidInput;
    }
    const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / spacing)));
    const double first_yaw = tf2::getYaw(previous.pose.orientation);
    const double yaw_delta = std::remainder(
      tf2::getYaw(current.pose.orientation) - first_yaw,
      detail::kCostmapFootprintSweepTwoPi);
    if (!detail::finite(first_yaw) || !detail::finite(yaw_delta)) {
      return CostmapFootprintSweepResult::kInvalidInput;
    }
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const double fraction = static_cast<double>(sample) / static_cast<double>(samples);
      geometry_msgs::msg::PoseStamped interpolated = previous;
      interpolated.pose.position.x += delta_x * fraction;
      interpolated.pose.position.y += delta_y * fraction;
      const double yaw = first_yaw + yaw_delta * fraction;
      interpolated.pose.orientation.x = 0.0;
      interpolated.pose.orientation.y = 0.0;
      interpolated.pose.orientation.z = std::sin(yaw * 0.5);
      interpolated.pose.orientation.w = std::cos(yaw * 0.5);
      const auto result = detail::footprintPoseIsClear(interpolated, costmap, options);
      if (result != CostmapFootprintSweepResult::kClear) {
        return result;
      }
    }
  }
  return CostmapFootprintSweepResult::kClear;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__COSTMAP_FOOTPRINT_SWEEP_HPP_
