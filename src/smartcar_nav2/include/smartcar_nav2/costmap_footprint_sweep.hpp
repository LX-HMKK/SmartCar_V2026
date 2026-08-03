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

// Check the complete padded rectangular vehicle against physical collision
// cells, rather than only sampling the global path centreline. The BT supplies
// the padded half extents so the check follows the active Nav2 footprint
// configuration.  Nav2's SE2 footprint checker treats 253 as an inflated
// clearance cost, not a physical collision; use 254 here by default so this
// independent continuous check has the same hard-collision semantics.
struct CostmapFootprintSweepOptions
{
  double half_length_m{0.2491};
  double half_width_m{0.095};
  double sample_spacing_m{0.025};
  std::uint8_t lethal_cost_threshold{254U};
};

enum class CostmapFootprintSweepResult
{
  kClear,
  kInvalidInput,
  kOutOfBounds,
  kLethalOverlap,
};

// Captures the first fail-closed sample from a continuous body sweep.  Nav2's
// planner action exposes a Path but no collision provenance, so this record
// keeps the exact interpolation pose and master-costmap cell that caused a
// later candidate rejection observable in the BT log.
struct CostmapFootprintSweepDiagnostic
{
  CostmapFootprintSweepResult result{CostmapFootprintSweepResult::kInvalidInput};
  bool has_sample_pose{false};
  geometry_msgs::msg::PoseStamped sample_pose;
  std::size_t segment_start_pose_index{0U};
  std::size_t segment_end_pose_index{0U};
  std::size_t segment_sample_index{0U};
  std::size_t segment_sample_count{0U};
  double segment_fraction{0.0};
  // Set for an out-of-bounds rejection when the padded body corner which
  // crossed the map boundary is known.
  bool has_boundary_point{false};
  double boundary_world_x{0.0};
  double boundary_world_y{0.0};
  // Set for a lethal or unknown cell overlapped by the padded body.
  bool has_blocking_cell{false};
  std::size_t blocking_cell_x{0U};
  std::size_t blocking_cell_y{0U};
  std::uint8_t blocking_cell_cost{0U};
  double blocking_cell_world_x{0.0};
  double blocking_cell_world_y{0.0};
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

// Values follow nav2_costmap_2d's standard master-grid semantics.  This is a
// value classification only: a nav2_msgs/Costmap is the merged master grid and
// does not retain the individual layer which wrote a cell.
inline const char * costmapFootprintSweepCellCostName(std::uint8_t cost)
{
  if (cost == 255U) {
    return "no_information";
  }
  if (cost == 254U) {
    return "lethal_obstacle";
  }
  if (cost == 253U) {
    return "inscribed_inflated_obstacle";
  }
  if (cost == 0U) {
    return "free_space";
  }
  return "inflation_or_traversal_cost";
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

inline void setDiagnosticResult(
  CostmapFootprintSweepDiagnostic * diagnostic,
  CostmapFootprintSweepResult result)
{
  if (diagnostic != nullptr) {
    diagnostic->result = result;
  }
}

inline void setDiagnosticSamplePose(
  CostmapFootprintSweepDiagnostic * diagnostic,
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (diagnostic != nullptr) {
    diagnostic->has_sample_pose = true;
    diagnostic->sample_pose = pose;
  }
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

inline bool footprintIntersectsCostmapCell(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options,
  std::size_t map_x,
  std::size_t map_y)
{
  const double footprint_yaw = tf2::getYaw(pose.pose.orientation);
  const double costmap_yaw = tf2::getYaw(costmap.metadata.origin.orientation);
  if (!finite(footprint_yaw) || !finite(costmap_yaw)) {
    return true;
  }

  double cell_x = 0.0;
  double cell_y = 0.0;
  mapToWorld(costmap, map_x, map_y, cell_x, cell_y);
  if (!finite(cell_x) || !finite(cell_y)) {
    return true;
  }

  // Test the padded vehicle OBB against the complete occupied cell exactly.
  // A half-cell-diagonal expansion would reject a body that only passes near
  // a cell corner, even though the vehicle and the filled grid cell do not
  // overlap.
  const double footprint_axis_x_x = std::cos(footprint_yaw);
  const double footprint_axis_x_y = std::sin(footprint_yaw);
  const double footprint_axis_y_x = -footprint_axis_x_y;
  const double footprint_axis_y_y = footprint_axis_x_x;
  const double cell_axis_x_x = std::cos(costmap_yaw);
  const double cell_axis_x_y = std::sin(costmap_yaw);
  const double cell_axis_y_x = -cell_axis_x_y;
  const double cell_axis_y_y = cell_axis_x_x;
  const double cell_half_extent = static_cast<double>(costmap.metadata.resolution) * 0.5;
  const double delta_x = cell_x - pose.pose.position.x;
  const double delta_y = cell_y - pose.pose.position.y;
  const std::array<std::pair<double, double>, 4> axes = {{
      {footprint_axis_x_x, footprint_axis_x_y},
      {footprint_axis_y_x, footprint_axis_y_y},
      {cell_axis_x_x, cell_axis_x_y},
      {cell_axis_y_x, cell_axis_y_y},
    }};

  for (const auto & axis : axes) {
    const double footprint_projection =
      options.half_length_m * std::abs(footprint_axis_x_x * axis.first +
      footprint_axis_x_y * axis.second) +
      options.half_width_m * std::abs(footprint_axis_y_x * axis.first +
      footprint_axis_y_y * axis.second);
    const double cell_projection = cell_half_extent * (
      std::abs(cell_axis_x_x * axis.first + cell_axis_x_y * axis.second) +
      std::abs(cell_axis_y_x * axis.first + cell_axis_y_y * axis.second));
    if (std::abs(delta_x * axis.first + delta_y * axis.second) >
      footprint_projection + cell_projection)
    {
      return false;
    }
  }
  return true;
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
  const CostmapFootprintSweepOptions & options,
  CostmapFootprintSweepDiagnostic * diagnostic = nullptr)
{
  const double yaw = tf2::getYaw(pose.pose.orientation);
  if (!finite(yaw) || !finite(pose.pose.position.x) || !finite(pose.pose.position.y)) {
    setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
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
      setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
      return FootprintBoundsResult::kInvalidInput;
    }
    if (local_x < 0.0 || local_y < 0.0 || local_x >= map_width || local_y >= map_height) {
      if (diagnostic != nullptr) {
        diagnostic->has_boundary_point = true;
        diagnostic->boundary_world_x = corner.first;
        diagnostic->boundary_world_y = corner.second;
      }
      setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kOutOfBounds);
      return FootprintBoundsResult::kOutOfBounds;
    }
  }
  return FootprintBoundsResult::kInside;
}

inline CostmapFootprintSweepResult footprintPoseIsClear(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options,
  CostmapFootprintSweepDiagnostic * diagnostic = nullptr)
{
  setDiagnosticSamplePose(diagnostic, pose);
  const auto bounds_result = footprintInsideCostmap(pose, costmap, options, diagnostic);
  if (bounds_result == FootprintBoundsResult::kInvalidInput) {
    setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
    return CostmapFootprintSweepResult::kInvalidInput;
  }
  if (bounds_result == FootprintBoundsResult::kOutOfBounds) {
    setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kOutOfBounds);
    return CostmapFootprintSweepResult::kOutOfBounds;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  // Search the true vehicle AABB and one neighbouring cell in each direction.
  // The extra cells make an edge contact visible to the exact OBB test below;
  // they do not expand the vehicle's collision geometry.
  const double extent_x = options.half_length_m * std::abs(cosine) +
    options.half_width_m * std::abs(sine);
  const double extent_y = options.half_length_m * std::abs(sine) +
    options.half_width_m * std::abs(cosine);

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
      setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
      return CostmapFootprintSweepResult::kInvalidInput;
    }
    min_x = std::min(min_x, local_x);
    max_x = std::max(max_x, local_x);
    min_y = std::min(min_y, local_y);
    max_y = std::max(max_y, local_y);
  }
  auto first_x = static_cast<std::size_t>(std::max(
    0.0, std::floor(min_x / resolution)));
  auto first_y = static_cast<std::size_t>(std::max(
    0.0, std::floor(min_y / resolution)));
  auto last_x = static_cast<std::size_t>(std::min(
    static_cast<double>(costmap.metadata.size_x - 1U), std::floor(max_x / resolution)));
  auto last_y = static_cast<std::size_t>(std::min(
    static_cast<double>(costmap.metadata.size_y - 1U), std::floor(max_y / resolution)));
  if (first_x > 0U) {
    --first_x;
  }
  if (first_y > 0U) {
    --first_y;
  }
  if (last_x + 1U < costmap.metadata.size_x) {
    ++last_x;
  }
  if (last_y + 1U < costmap.metadata.size_y) {
    ++last_y;
  }
  if (first_x > last_x || first_y > last_y) {
    setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
    return CostmapFootprintSweepResult::kInvalidInput;
  }

  for (std::size_t map_y = first_y; map_y <= last_y; ++map_y) {
    for (std::size_t map_x = first_x; map_x <= last_x; ++map_x) {
      const std::size_t index = map_y * static_cast<std::size_t>(costmap.metadata.size_x) +
        map_x;
      if (costmap.data[index] < options.lethal_cost_threshold) {
        continue;
      }
      if (footprintIntersectsCostmapCell(pose, costmap, options, map_x, map_y))
      {
        if (diagnostic != nullptr) {
          double world_x = 0.0;
          double world_y = 0.0;
          mapToWorld(costmap, map_x, map_y, world_x, world_y);
          diagnostic->has_blocking_cell = true;
          diagnostic->blocking_cell_x = map_x;
          diagnostic->blocking_cell_y = map_y;
          diagnostic->blocking_cell_cost = costmap.data[index];
          diagnostic->blocking_cell_world_x = world_x;
          diagnostic->blocking_cell_world_y = world_y;
        }
        setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kLethalOverlap);
        return CostmapFootprintSweepResult::kLethalOverlap;
      }
    }
  }
  setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kClear);
  return CostmapFootprintSweepResult::kClear;
}

}  // namespace detail

inline CostmapFootprintSweepResult costmapFootprintPathSweep(
  const nav_msgs::msg::Path & path,
  const nav2_msgs::msg::Costmap & costmap,
  const CostmapFootprintSweepOptions & options,
  CostmapFootprintSweepDiagnostic * diagnostic = nullptr)
{
  if (diagnostic != nullptr) {
    *diagnostic = CostmapFootprintSweepDiagnostic();
  }
  if (path.poses.empty() || !detail::validCostmap(costmap) || !detail::validOptions(options)) {
    detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
    return CostmapFootprintSweepResult::kInvalidInput;
  }
  if (diagnostic != nullptr) {
    diagnostic->segment_start_pose_index = 0U;
    diagnostic->segment_end_pose_index = 0U;
  }
  const auto first_result = detail::footprintPoseIsClear(
    path.poses.front(), costmap, options, diagnostic);
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
      detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
      return CostmapFootprintSweepResult::kInvalidInput;
    }
    const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / spacing)));
    const double first_yaw = tf2::getYaw(previous.pose.orientation);
    const double yaw_delta = std::remainder(
      tf2::getYaw(current.pose.orientation) - first_yaw,
      detail::kCostmapFootprintSweepTwoPi);
    if (!detail::finite(first_yaw) || !detail::finite(yaw_delta)) {
      detail::setDiagnosticResult(diagnostic, CostmapFootprintSweepResult::kInvalidInput);
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
      if (diagnostic != nullptr) {
        diagnostic->segment_start_pose_index = index - 1U;
        diagnostic->segment_end_pose_index = index;
        diagnostic->segment_sample_index = sample;
        diagnostic->segment_sample_count = samples;
        diagnostic->segment_fraction = fraction;
      }
      const auto result = detail::footprintPoseIsClear(
        interpolated, costmap, options, diagnostic);
      if (result != CostmapFootprintSweepResult::kClear) {
        return result;
      }
    }
  }
  return CostmapFootprintSweepResult::kClear;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__COSTMAP_FOOTPRINT_SWEEP_HPP_
