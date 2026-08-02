#ifndef SMARTCAR_NAV2__FOOTPRINT_SWEEP_COLLISION_SOURCE_HPP_
#define SMARTCAR_NAV2__FOOTPRINT_SWEEP_COLLISION_SOURCE_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"

namespace smartcar_nav2
{

// A nav2_msgs/Costmap is a merged master grid and has no per-cell layer
// provenance.  An optional KeepoutFilter mask can still establish whether a
// reported blocking cell belongs to the static simulation constraint.
enum class KeepoutMaskCellState
{
  kNoMask,
  kFree,
  kOccupied,
  kUnknown,
  kOutOfBounds,
  kWrongFrame,
  kMalformed,
};

inline KeepoutMaskCellState keepoutMaskCellStateAt(
  const nav_msgs::msg::OccupancyGrid * mask,
  const std::string & expected_frame,
  double world_x,
  double world_y)
{
  if (mask == nullptr) {
    return KeepoutMaskCellState::kNoMask;
  }
  if (mask->header.frame_id != expected_frame) {
    return KeepoutMaskCellState::kWrongFrame;
  }
  if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
    mask->info.resolution <= 0.0F || mask->info.width == 0U || mask->info.height == 0U)
  {
    return KeepoutMaskCellState::kMalformed;
  }
  const auto width = static_cast<std::size_t>(mask->info.width);
  const auto height = static_cast<std::size_t>(mask->info.height);
  if (width > std::numeric_limits<std::size_t>::max() / height ||
    mask->data.size() < width * height)
  {
    return KeepoutMaskCellState::kMalformed;
  }
  const auto & orientation = mask->info.origin.orientation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
    !std::isfinite(mask->info.origin.position.x) ||
    !std::isfinite(mask->info.origin.position.y))
  {
    return KeepoutMaskCellState::kMalformed;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    return KeepoutMaskCellState::kMalformed;
  }
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  const double dx = world_x - mask->info.origin.position.x;
  const double dy = world_y - mask->info.origin.position.y;
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  const double resolution = static_cast<double>(mask->info.resolution);
  if (!std::isfinite(local_x) || !std::isfinite(local_y) || local_x < 0.0 ||
    local_y < 0.0 || local_x >= resolution * static_cast<double>(width) ||
    local_y >= resolution * static_cast<double>(height))
  {
    return KeepoutMaskCellState::kOutOfBounds;
  }
  const auto map_x = static_cast<std::size_t>(std::floor(local_x / resolution));
  const auto map_y = static_cast<std::size_t>(std::floor(local_y / resolution));
  const auto occupancy = mask->data[map_y * width + map_x];
  if (occupancy < 0) {
    return KeepoutMaskCellState::kUnknown;
  }
  return occupancy >= 50 ? KeepoutMaskCellState::kOccupied : KeepoutMaskCellState::kFree;
}

inline const char * keepoutCollisionSourceName(KeepoutMaskCellState state)
{
  switch (state) {
    case KeepoutMaskCellState::kNoMask:
      return "merged_raw_costmap_keepout_mask_unavailable";
    case KeepoutMaskCellState::kFree:
      return "non_keepout_merged_raw_costmap";
    case KeepoutMaskCellState::kOccupied:
      return "static_keepout_filter_mask";
    case KeepoutMaskCellState::kUnknown:
      return "keepout_mask_unknown_merged_raw_costmap";
    case KeepoutMaskCellState::kOutOfBounds:
      return "keepout_mask_out_of_bounds_merged_raw_costmap";
    case KeepoutMaskCellState::kWrongFrame:
      return "keepout_mask_wrong_frame_merged_raw_costmap";
    case KeepoutMaskCellState::kMalformed:
      return "keepout_mask_malformed_merged_raw_costmap";
  }
  return "merged_raw_costmap_attribution_unknown";
}

// ``costmap_raw`` intentionally precedes Nav2's filter pipeline, so a raw
// footprint sweep alone cannot prove that a candidate stays inside the
// simulation-only KeepoutFilter mask.  Mirror the mask into a minimal costmap
// and reuse the same continuous padded-body sweep used for live obstacles.
// This also makes the finite mask extent a hard field boundary: an OBB that
// leaves the published map is rejected rather than exploiting unmasked space
// outside the stadium.
enum class StaticKeepoutMaskSweepResult
{
  kNoMask,
  kClear,
  kWrongFrame,
  kMalformed,
  kOutOfBounds,
  kOccupiedOrUnknown,
};

inline const char * staticKeepoutMaskSweepResultName(
  StaticKeepoutMaskSweepResult result)
{
  switch (result) {
    case StaticKeepoutMaskSweepResult::kNoMask:
      return "no mask";
    case StaticKeepoutMaskSweepResult::kClear:
      return "clear";
    case StaticKeepoutMaskSweepResult::kWrongFrame:
      return "wrong frame";
    case StaticKeepoutMaskSweepResult::kMalformed:
      return "malformed";
    case StaticKeepoutMaskSweepResult::kOutOfBounds:
      return "footprint leaves keepout-mask bounds";
    case StaticKeepoutMaskSweepResult::kOccupiedOrUnknown:
      return "occupied or unknown keepout-mask overlap";
  }
  return "unknown";
}

inline StaticKeepoutMaskSweepResult staticKeepoutMaskFootprintPathSweep(
  const nav_msgs::msg::OccupancyGrid * mask,
  const std::string & expected_frame,
  const nav_msgs::msg::Path & path,
  const CostmapFootprintSweepOptions & options,
  CostmapFootprintSweepDiagnostic * diagnostic = nullptr,
  std::int8_t occupied_threshold = 50)
{
  if (mask == nullptr) {
    return StaticKeepoutMaskSweepResult::kNoMask;
  }
  if (mask->header.frame_id != expected_frame) {
    return StaticKeepoutMaskSweepResult::kWrongFrame;
  }
  if (occupied_threshold <= 0 || mask->info.resolution <= 0.0F ||
    mask->info.width == 0U || mask->info.height == 0U ||
    !std::isfinite(mask->info.origin.position.x) ||
    !std::isfinite(mask->info.origin.position.y) ||
    !std::isfinite(mask->info.origin.orientation.x) ||
    !std::isfinite(mask->info.origin.orientation.y) ||
    !std::isfinite(mask->info.origin.orientation.z) ||
    !std::isfinite(mask->info.origin.orientation.w))
  {
    return StaticKeepoutMaskSweepResult::kMalformed;
  }
  const auto width = static_cast<std::size_t>(mask->info.width);
  const auto height = static_cast<std::size_t>(mask->info.height);
  if (width > std::numeric_limits<std::size_t>::max() / height ||
    mask->data.size() < width * height)
  {
    return StaticKeepoutMaskSweepResult::kMalformed;
  }
  const auto & rotation = mask->info.origin.orientation;
  const double norm = std::sqrt(
    rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w);
  // Humble's KeepoutFilter constructs Costmap2D directly from the mask. That
  // implementation supports only an axis-aligned origin, so accepting a
  // rotated or non-unit OccupancyGrid quaternion would make this independent
  // sweep disagree with the filter whose static boundary it is meant to
  // enforce. Treat such a mask as malformed rather than silently projecting it
  // into a different 2D frame.
  constexpr double kOrientationTolerance = 1.0e-6;
  constexpr double kQuaternionNormTolerance = 1.0e-3;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > kQuaternionNormTolerance ||
    std::abs(rotation.x) > kOrientationTolerance ||
    std::abs(rotation.y) > kOrientationTolerance ||
    std::abs(rotation.z) > kOrientationTolerance ||
    std::abs(std::abs(rotation.w) - 1.0) > kQuaternionNormTolerance)
  {
    return StaticKeepoutMaskSweepResult::kMalformed;
  }

  nav2_msgs::msg::Costmap collision_map;
  collision_map.header = mask->header;
  collision_map.metadata.resolution = mask->info.resolution;
  collision_map.metadata.size_x = mask->info.width;
  collision_map.metadata.size_y = mask->info.height;
  collision_map.metadata.origin = mask->info.origin;
  collision_map.metadata.origin.orientation.x = 0.0;
  collision_map.metadata.origin.orientation.y = 0.0;
  collision_map.metadata.origin.orientation.z = 0.0;
  collision_map.metadata.origin.orientation.w = 1.0;
  collision_map.data.reserve(width * height);
  for (std::size_t index = 0U; index < width * height; ++index) {
    const auto occupancy = mask->data[index];
    // KeepoutFilter treats unknown cells as non-traversable.  Preserve that
    // fail-closed interpretation in the independent body sweep.
    collision_map.data.push_back(
      occupancy < 0 ? 255U : (occupancy >= occupied_threshold ? 254U : 0U));
  }

  switch (costmapFootprintPathSweep(path, collision_map, options, diagnostic)) {
    case CostmapFootprintSweepResult::kClear:
      return StaticKeepoutMaskSweepResult::kClear;
    case CostmapFootprintSweepResult::kOutOfBounds:
      return StaticKeepoutMaskSweepResult::kOutOfBounds;
    case CostmapFootprintSweepResult::kLethalOverlap:
      return StaticKeepoutMaskSweepResult::kOccupiedOrUnknown;
    case CostmapFootprintSweepResult::kInvalidInput:
      return StaticKeepoutMaskSweepResult::kMalformed;
  }
  return StaticKeepoutMaskSweepResult::kMalformed;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FOOTPRINT_SWEEP_COLLISION_SOURCE_HPP_
