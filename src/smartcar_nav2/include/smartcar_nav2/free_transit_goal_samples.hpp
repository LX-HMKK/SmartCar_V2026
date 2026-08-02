#ifndef SMARTCAR_NAV2__FREE_TRANSIT_GOAL_SAMPLES_HPP_
#define SMARTCAR_NAV2__FREE_TRANSIT_GOAL_SAMPLES_HPP_

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <optional>
#include <vector>

namespace smartcar_nav2
{

// Position-only transit targets may use a nearby, collision-free endpoint,
// but never leave the action's configured goal acceptance disk.
constexpr double kMaximumFreeTransitGoalOffsetM = 0.10;
constexpr double kFreeTransitGoalSamplesPi = 3.14159265358979323846;
constexpr double kFreeTransitHeadingDedupTolerance = 1.0e-6;
// Keep locked-goal alternatives small even when a caller configures a loose
// controller tolerance.  The authored task heading remains the centre of the
// candidate set and the final semantic check still enforces its full limit.
constexpr double kMaximumLockedGoalYawCandidateOffsetRad = 0.10;

struct FreeTransitGoalOffset
{
  double x{0.0};
  double y{0.0};
};

inline std::vector<FreeTransitGoalOffset> freeTransitGoalOffsets(
  double goal_position_tolerance)
{
  if (!std::isfinite(goal_position_tolerance) || goal_position_tolerance <= 0.0) {
    return {{0.0, 0.0}};
  }

  const double radius = std::min(
    kMaximumFreeTransitGoalOffsetM, goal_position_tolerance);
  // Keep the bounded endpoint search rotationally symmetric. A biased sample
  // set can otherwise miss the only viable side of a narrow obstacle.
  std::vector<FreeTransitGoalOffset> offsets;
  offsets.reserve(7);
  offsets.push_back({0.0, 0.0});
  for (int index = 0; index < 6; ++index) {
    const double angle = kFreeTransitGoalSamplesPi / 2.0 -
      static_cast<double>(index) * kFreeTransitGoalSamplesPi / 3.0;
    offsets.push_back({radius * std::cos(angle), radius * std::sin(angle)});
  }
  return offsets;
}

inline bool isWithinFreeTransitGoalTolerance(
  double x_offset, double y_offset, double goal_position_tolerance)
{
  return std::isfinite(x_offset) && std::isfinite(y_offset) &&
         std::isfinite(goal_position_tolerance) && goal_position_tolerance >= 0.0 &&
         std::hypot(x_offset, y_offset) <= goal_position_tolerance;
}

// A geometric bisector is a useful first attempt, but it must not define the
// phase of every sampled heading. Keep both the reference-relative and the
// world-aligned lattices: the former preserves narrow diagonal routes while
// the latter guarantees cardinal corridor tangents when the reference phase
// falls between them.
inline std::vector<double> freeTransitHeadingHints(
  double reference_yaw,
  const std::vector<double> & tangent_yaws,
  int heading_samples)
{
  std::vector<double> headings;
  if (!std::isfinite(reference_yaw) || heading_samples < 1) {
    return headings;
  }
  const std::size_t maximum_count = static_cast<std::size_t>(heading_samples);
  headings.reserve(maximum_count);

  const auto same_heading = [](double first, double second) {
    return std::abs(std::remainder(
      first - second, 2.0 * kFreeTransitGoalSamplesPi)) <=
           kFreeTransitHeadingDedupTolerance;
  };
  const auto append_unique = [&headings, maximum_count, &same_heading](double yaw) {
    if (headings.size() >= maximum_count) {
      return;
    }
    if (!std::isfinite(yaw)) {
      return;
    }
    const double normalized = std::remainder(
      yaw, 2.0 * kFreeTransitGoalSamplesPi);
    for (const double existing : headings) {
      if (same_heading(existing, normalized)) {
        return;
      }
    }
    headings.push_back(normalized);
  };

  const double heading_step = 2.0 * kFreeTransitGoalSamplesPi /
    static_cast<double>(heading_samples);
  // Preserve the geometric and corridor hints first, then fill the remaining
  // budget from both lattices by largest angular separation. This keeps the
  // finite list useful for a heading on either side of the transit point,
  // rather than consuming every slot in one local arc.
  append_unique(reference_yaw);
  for (const double yaw : tangent_yaws) {
    append_unique(yaw);
  }
  append_unique(reference_yaw + heading_step);
  append_unique(reference_yaw - heading_step);
  for (const double cardinal_yaw : {
      0.0,
      kFreeTransitGoalSamplesPi / 2.0,
      kFreeTransitGoalSamplesPi,
      -kFreeTransitGoalSamplesPi / 2.0})
  {
    append_unique(cardinal_yaw);
  }

  std::vector<double> pool;
  pool.reserve(static_cast<std::size_t>(heading_samples) * 2U + tangent_yaws.size());
  const auto append_pool_unique = [&pool, &same_heading](double yaw) {
    if (!std::isfinite(yaw)) {
      return;
    }
    const double normalized = std::remainder(
      yaw, 2.0 * kFreeTransitGoalSamplesPi);
    for (const double existing : pool) {
      if (same_heading(existing, normalized)) {
        return;
      }
    }
    pool.push_back(normalized);
  };
  append_pool_unique(reference_yaw);
  for (const double yaw : tangent_yaws) {
    append_pool_unique(yaw);
  }
  for (int index = 0; index < heading_samples; ++index) {
    append_pool_unique(reference_yaw + heading_step * static_cast<double>(index));
    append_pool_unique(heading_step * static_cast<double>(index));
  }

  while (headings.size() < maximum_count) {
    std::optional<double> best_heading;
    double best_minimum_separation = -1.0;
    for (const double candidate : pool) {
      bool duplicate = false;
      double minimum_separation = kFreeTransitGoalSamplesPi;
      for (const double existing : headings) {
        const double separation = std::abs(std::remainder(
          existing - candidate, 2.0 * kFreeTransitGoalSamplesPi));
        if (separation <= kFreeTransitHeadingDedupTolerance) {
          duplicate = true;
          break;
        }
        minimum_separation = std::min(minimum_separation, separation);
      }
      if (!duplicate && minimum_separation > best_minimum_separation +
        kFreeTransitHeadingDedupTolerance)
      {
        best_minimum_separation = minimum_separation;
        best_heading = candidate;
      }
    }
    if (!best_heading.has_value()) {
      break;
    }
    append_unique(*best_heading);
  }
  return headings;
}

// A nonzero task quaternion is a real semantic constraint, but the planner's
// discrete heading lattice can make one exact terminal bin collide with a
// nearby obstacle.  Query the authored heading and two bounded alternatives;
// callers must still validate the returned endpoint against the authored yaw.
inline std::vector<double> lockedGoalHeadingHints(
  double authored_yaw,
  double goal_yaw_tolerance)
{
  std::vector<double> headings;
  if (!std::isfinite(authored_yaw) || !std::isfinite(goal_yaw_tolerance) ||
    goal_yaw_tolerance < 0.0)
  {
    return headings;
  }

  const auto normalized = [](double yaw) {
      return std::remainder(yaw, 2.0 * kFreeTransitGoalSamplesPi);
    };
  headings.push_back(normalized(authored_yaw));
  if (goal_yaw_tolerance <= kFreeTransitHeadingDedupTolerance) {
    return headings;
  }

  const double offset = std::min(
    kMaximumLockedGoalYawCandidateOffsetRad,
    goal_yaw_tolerance * (2.0 / 3.0));
  if (offset <= kFreeTransitHeadingDedupTolerance) {
    return headings;
  }
  headings.push_back(normalized(authored_yaw + offset));
  headings.push_back(normalized(authored_yaw - offset));
  return headings;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FREE_TRANSIT_GOAL_SAMPLES_HPP_
