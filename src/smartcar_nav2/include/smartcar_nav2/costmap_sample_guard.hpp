#ifndef SMARTCAR_NAV2__COSTMAP_SAMPLE_GUARD_HPP_
#define SMARTCAR_NAV2__COSTMAP_SAMPLE_GUARD_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"

namespace smartcar_nav2
{

// The raw costmap is a sensor-derived safety input. A recent DDS delivery is
// insufficient evidence that its contents are recent: transient-local replay
// can deliver an old message just before a recovery. Keep the trusted source
// stamp and a strictly monotonic local sequence together with the receive time.
struct CostmapSample
{
  nav2_msgs::msg::Costmap::SharedPtr costmap;
  std::chrono::steady_clock::time_point received_at;
  std::uint64_t sequence{0U};
  std::int64_t stamp_ns{0};
  // Snapshot of the newest trusted scan at the time this raw-costmap callback
  // arrived. A later map can only be used for retreat when this evidence is
  // newer than the post-clear scan barrier.
  std::uint64_t scan_sequence{0U};
  std::int64_t scan_stamp_ns{0};
};

enum class CostmapSampleFreshness
{
  kFresh,
  kMissing,
  kWrongFrame,
  kMalformed,
  kInvalidStamp,
  kClockUnavailable,
  kSourceStampFuture,
  kSourceStampStale,
  kReceiveStale,
  kBeforeStampBarrier,
  kBeforeSequenceBarrier,
  kMissingScanAssociation,
  kBeforeScanStampBarrier,
  kBeforeScanSequenceBarrier,
  kCostmapBeforeAssociatedScan,
};

// A raw-costmap callback can only be used as sensor-fusion evidence when its
// associated scan existed before the costmap update and is close enough in
// source time to plausibly have been consumed by that update.  The former is
// checked by CostmapSampleFreshness too; this smaller classification gives the
// recovery action a precise reason when a delayed global update is too old.
enum class CostmapScanAssociationFreshness
{
  kFresh,
  kMissing,
  kCostmapBeforeScan,
  kScanTooOld,
};

inline std::optional<std::int64_t> costmapStampNanoseconds(
  const builtin_interfaces::msg::Time & stamp)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  if (stamp.sec < 0 || stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond) ||
    stamp.sec > std::numeric_limits<std::int64_t>::max() / kNanosecondsPerSecond)
  {
    return std::nullopt;
  }
  const auto nanoseconds = static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
    static_cast<std::int64_t>(stamp.nanosec);
  if (nanoseconds <= 0) {
    return std::nullopt;
  }
  return nanoseconds;
}

// Nav2 Humble publishers do not consistently populate the outer header. The
// metadata update time is the costmap's own update clock, so use the newest
// valid value from either field. This remains a source timestamp rather than a
// local receive timestamp and therefore still detects transient-local replay.
inline std::optional<std::int64_t> costmapSourceStampNanoseconds(
  const nav2_msgs::msg::Costmap & costmap)
{
  const auto header_stamp_ns = costmapStampNanoseconds(costmap.header.stamp);
  const auto update_stamp_ns = costmapStampNanoseconds(costmap.metadata.update_time);
  if (!header_stamp_ns.has_value()) {
    return update_stamp_ns;
  }
  if (!update_stamp_ns.has_value()) {
    return header_stamp_ns;
  }
  return *header_stamp_ns >= *update_stamp_ns ? header_stamp_ns : update_stamp_ns;
}

inline bool costmapHasStrictlyNewerSourceStamp(
  const nav2_msgs::msg::Costmap & costmap, std::int64_t previous_stamp_ns)
{
  const auto stamp_ns = costmapSourceStampNanoseconds(costmap);
  return stamp_ns.has_value() && *stamp_ns > previous_stamp_ns;
}

inline CostmapSampleFreshness costmapSampleFreshness(
  const CostmapSample & sample,
  const std::string & expected_frame,
  const std::chrono::milliseconds & maximum_age,
  const rclcpp::Time & ros_now,
  const std::chrono::steady_clock::time_point & steady_now,
  std::int64_t minimum_stamp_ns = 0,
  std::uint64_t minimum_sequence = 0U,
  std::int64_t minimum_scan_stamp_ns = 0,
  std::uint64_t minimum_scan_sequence = 0U,
  bool require_scan_association = false)
{
  if (!sample.costmap || sample.received_at == std::chrono::steady_clock::time_point() ||
    sample.sequence == 0U)
  {
    return CostmapSampleFreshness::kMissing;
  }
  if (sample.costmap->header.frame_id != expected_frame) {
    return CostmapSampleFreshness::kWrongFrame;
  }
  const auto expected_size = static_cast<std::size_t>(sample.costmap->metadata.size_x) *
    static_cast<std::size_t>(sample.costmap->metadata.size_y);
  if (sample.costmap->metadata.resolution <= 0.0F || sample.costmap->metadata.size_x == 0U ||
    sample.costmap->metadata.size_y == 0U || sample.costmap->data.size() < expected_size)
  {
    return CostmapSampleFreshness::kMalformed;
  }
  const auto source_stamp_ns = costmapSourceStampNanoseconds(*sample.costmap);
  if (!source_stamp_ns.has_value() || sample.stamp_ns != *source_stamp_ns) {
    return CostmapSampleFreshness::kInvalidStamp;
  }
  if (minimum_stamp_ns > 0 && sample.stamp_ns <= minimum_stamp_ns) {
    return CostmapSampleFreshness::kBeforeStampBarrier;
  }
  if (minimum_sequence > 0U && sample.sequence <= minimum_sequence) {
    return CostmapSampleFreshness::kBeforeSequenceBarrier;
  }
  const bool scan_association_required = require_scan_association ||
    minimum_scan_stamp_ns > 0 || minimum_scan_sequence > 0U;
  if (scan_association_required) {
    if (sample.scan_sequence == 0U || sample.scan_stamp_ns <= 0) {
      return CostmapSampleFreshness::kMissingScanAssociation;
    }
    if (minimum_scan_stamp_ns > 0 && sample.scan_stamp_ns <= minimum_scan_stamp_ns) {
      return CostmapSampleFreshness::kBeforeScanStampBarrier;
    }
    if (minimum_scan_sequence > 0U && sample.scan_sequence <= minimum_scan_sequence) {
      return CostmapSampleFreshness::kBeforeScanSequenceBarrier;
    }
    if (sample.stamp_ns < sample.scan_stamp_ns) {
      return CostmapSampleFreshness::kCostmapBeforeAssociatedScan;
    }
  }
  if (ros_now.nanoseconds() <= 0) {
    return CostmapSampleFreshness::kClockUnavailable;
  }

  // A small allowance covers /clock and costmap callback ordering, but a map
  // materially newer than the ROS clock is unusable for a physical retreat.
  constexpr std::int64_t kFutureStampToleranceNs = 50000000LL;
  const auto source_age_ns = ros_now.nanoseconds() - sample.stamp_ns;
  if (source_age_ns < -kFutureStampToleranceNs) {
    return CostmapSampleFreshness::kSourceStampFuture;
  }
  const auto maximum_age_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(maximum_age).count();
  if (source_age_ns > maximum_age_ns) {
    return CostmapSampleFreshness::kSourceStampStale;
  }
  if (steady_now < sample.received_at || steady_now - sample.received_at > maximum_age) {
    return CostmapSampleFreshness::kReceiveStale;
  }
  return CostmapSampleFreshness::kFresh;
}

inline const char * costmapSampleFreshnessName(CostmapSampleFreshness freshness)
{
  switch (freshness) {
    case CostmapSampleFreshness::kFresh:
      return "fresh";
    case CostmapSampleFreshness::kMissing:
      return "missing";
    case CostmapSampleFreshness::kWrongFrame:
      return "wrong frame";
    case CostmapSampleFreshness::kMalformed:
      return "malformed";
    case CostmapSampleFreshness::kInvalidStamp:
      return "invalid source stamp";
    case CostmapSampleFreshness::kClockUnavailable:
      return "ROS clock unavailable";
    case CostmapSampleFreshness::kSourceStampFuture:
      return "source stamp is in the future";
    case CostmapSampleFreshness::kSourceStampStale:
      return "source stamp is stale";
    case CostmapSampleFreshness::kReceiveStale:
      return "receive time is stale";
    case CostmapSampleFreshness::kBeforeStampBarrier:
      return "before clear barrier stamp";
    case CostmapSampleFreshness::kBeforeSequenceBarrier:
      return "before clear barrier sequence";
    case CostmapSampleFreshness::kMissingScanAssociation:
      return "missing scan association";
    case CostmapSampleFreshness::kBeforeScanStampBarrier:
      return "before clear scan barrier stamp";
    case CostmapSampleFreshness::kBeforeScanSequenceBarrier:
      return "before clear scan barrier sequence";
    case CostmapSampleFreshness::kCostmapBeforeAssociatedScan:
      return "costmap predates associated scan";
  }
  return "unknown";
}

inline CostmapScanAssociationFreshness costmapScanAssociationFreshness(
  const CostmapSample & sample, const std::chrono::milliseconds & maximum_lag)
{
  if (sample.stamp_ns <= 0 || sample.scan_stamp_ns <= 0 || sample.scan_sequence == 0U ||
    maximum_lag.count() < 0)
  {
    return CostmapScanAssociationFreshness::kMissing;
  }
  if (sample.stamp_ns < sample.scan_stamp_ns) {
    return CostmapScanAssociationFreshness::kCostmapBeforeScan;
  }
  const auto lag_ns = sample.stamp_ns - sample.scan_stamp_ns;
  const auto maximum_lag_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(maximum_lag).count();
  if (lag_ns > maximum_lag_ns) {
    return CostmapScanAssociationFreshness::kScanTooOld;
  }
  return CostmapScanAssociationFreshness::kFresh;
}

inline const char * costmapScanAssociationFreshnessName(
  CostmapScanAssociationFreshness freshness)
{
  switch (freshness) {
    case CostmapScanAssociationFreshness::kFresh:
      return "fresh";
    case CostmapScanAssociationFreshness::kMissing:
      return "missing";
    case CostmapScanAssociationFreshness::kCostmapBeforeScan:
      return "costmap predates scan";
    case CostmapScanAssociationFreshness::kScanTooOld:
      return "scan predates costmap update";
  }
  return "unknown";
}

enum class StaticKeepoutMaskFilterResult
{
  kNoMask,
  kFiltered,
  kWrongFrame,
  kMalformed,
};

// KeepoutFilter cells are static route constraints, not proof that the
// obstacle layer consumed a current scan.  Retain only scan endpoints which
// fall in known-free cells of an optional keepout mask.  Unknown and
// out-of-mask points are deliberately excluded: using either as a witness
// would make a missing or malformed static constraint look like dynamic sensor
// evidence.
inline StaticKeepoutMaskFilterResult filterPointsOutsideStaticKeepoutMask(
  const std::vector<std::pair<double, double>> & points,
  const nav_msgs::msg::OccupancyGrid * mask,
  const std::string & expected_frame,
  std::vector<std::pair<double, double>> & filtered_points,
  std::int8_t occupied_threshold = 50)
{
  filtered_points.clear();
  if (!mask) {
    filtered_points = points;
    return StaticKeepoutMaskFilterResult::kNoMask;
  }
  if (mask->header.frame_id != expected_frame) {
    return StaticKeepoutMaskFilterResult::kWrongFrame;
  }
  if (occupied_threshold <= 0 || mask->info.resolution <= 0.0F ||
    mask->info.width == 0U || mask->info.height == 0U ||
    !std::isfinite(mask->info.origin.position.x) ||
    !std::isfinite(mask->info.origin.position.y))
  {
    return StaticKeepoutMaskFilterResult::kMalformed;
  }
  const auto width = static_cast<std::size_t>(mask->info.width);
  const auto height = static_cast<std::size_t>(mask->info.height);
  if (width > std::numeric_limits<std::size_t>::max() / height ||
    mask->data.size() < width * height)
  {
    return StaticKeepoutMaskFilterResult::kMalformed;
  }

  const auto & rotation = mask->info.origin.orientation;
  if (!std::isfinite(rotation.x) || !std::isfinite(rotation.y) ||
    !std::isfinite(rotation.z) || !std::isfinite(rotation.w))
  {
    return StaticKeepoutMaskFilterResult::kMalformed;
  }
  const double norm = std::sqrt(
    rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    return StaticKeepoutMaskFilterResult::kMalformed;
  }
  const double x = rotation.x / norm;
  const double y = rotation.y / norm;
  const double z = rotation.z / norm;
  const double w = rotation.w / norm;
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  const double resolution = static_cast<double>(mask->info.resolution);

  filtered_points.reserve(points.size());
  for (const auto & point : points) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second)) {
      continue;
    }
    const double dx = point.first - mask->info.origin.position.x;
    const double dy = point.second - mask->info.origin.position.y;
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;
    const int map_x = static_cast<int>(std::floor(local_x / resolution));
    const int map_y = static_cast<int>(std::floor(local_y / resolution));
    if (map_x < 0 || map_y < 0 || map_x >= static_cast<int>(width) ||
      map_y >= static_cast<int>(height))
    {
      continue;
    }
    const auto index = static_cast<std::size_t>(map_y) * width +
      static_cast<std::size_t>(map_x);
    const auto occupancy = mask->data[index];
    if (occupancy < 0 || occupancy >= occupied_threshold) {
      continue;
    }
    filtered_points.push_back(point);
  }
  return StaticKeepoutMaskFilterResult::kFiltered;
}

inline const char * staticKeepoutMaskFilterResultName(StaticKeepoutMaskFilterResult result)
{
  switch (result) {
    case StaticKeepoutMaskFilterResult::kNoMask:
      return "no mask";
    case StaticKeepoutMaskFilterResult::kFiltered:
      return "filtered";
    case StaticKeepoutMaskFilterResult::kWrongFrame:
      return "wrong frame";
    case StaticKeepoutMaskFilterResult::kMalformed:
      return "malformed";
  }
  return "unknown";
}

// A raw costmap can be freshly published immediately after ClearEntireCostmap
// while still containing no sensor observations. Require at least one current
// scan endpoint to be marked lethal in each raw map before using either map as
// physical-retreat evidence. The points must already be expressed in the
// costmap frame.
inline bool costmapHasLethalObservationAtPoints(
  const nav2_msgs::msg::Costmap & costmap,
  const std::vector<std::pair<double, double>> & points,
  std::uint8_t lethal_cost_threshold,
  double match_radius_m)
{
  if (!std::isfinite(match_radius_m) || match_radius_m < 0.0 || points.empty() ||
    costmap.metadata.resolution <= 0.0F || costmap.metadata.size_x == 0U ||
    costmap.metadata.size_y == 0U)
  {
    return false;
  }
  const auto size_x = static_cast<std::size_t>(costmap.metadata.size_x);
  const auto size_y = static_cast<std::size_t>(costmap.metadata.size_y);
  if (costmap.data.size() < size_x * size_y) {
    return false;
  }

  const auto & orientation = costmap.metadata.origin.orientation;
  const double sin_yaw = 2.0 * (
    orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 * (
    orientation.y * orientation.y + orientation.z * orientation.z);
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  const int radius = static_cast<int>(std::ceil(match_radius_m / resolution));
  for (const auto & point : points) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second)) {
      continue;
    }
    const double dx = point.first - costmap.metadata.origin.position.x;
    const double dy = point.second - costmap.metadata.origin.position.y;
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;
    const int map_x = static_cast<int>(std::floor(local_x / resolution));
    const int map_y = static_cast<int>(std::floor(local_y / resolution));
    for (int x = map_x - radius; x <= map_x + radius; ++x) {
      if (x < 0 || x >= static_cast<int>(size_x)) {
        continue;
      }
      for (int y = map_y - radius; y <= map_y + radius; ++y) {
        if (y < 0 || y >= static_cast<int>(size_y)) {
          continue;
        }
        const auto index = static_cast<std::size_t>(y) * size_x +
          static_cast<std::size_t>(x);
        if (costmap.data[index] >= lethal_cost_threshold) {
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__COSTMAP_SAMPLE_GUARD_HPP_
