#ifndef SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ODOM_GUARD_HPP_
#define SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ODOM_GUARD_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>

namespace smartcar_nav2
{

struct AckermannReverseRetreatOdomSample
{
  double x{0.0};
  double y{0.0};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence{0U};
};

struct AckermannReverseRetreatOdomLimits
{
  std::chrono::milliseconds maximum_age{500};
  double maximum_step_m{0.05};
  double maximum_travel_m{0.19};
  double maximum_displacement_m{0.19};
};

enum class AckermannReverseRetreatOdomResult
{
  kClear,
  kUnarmed,
  kMalformed,
  kStale,
  kNonMonotonic,
  kStepTooLarge,
  kTravelExceeded,
  kDisplacementExceeded,
};

inline const char * ackermannReverseRetreatOdomResultName(
  AckermannReverseRetreatOdomResult result)
{
  switch (result) {
    case AckermannReverseRetreatOdomResult::kClear:
      return "clear";
    case AckermannReverseRetreatOdomResult::kUnarmed:
      return "unarmed";
    case AckermannReverseRetreatOdomResult::kMalformed:
      return "malformed";
    case AckermannReverseRetreatOdomResult::kStale:
      return "stale";
    case AckermannReverseRetreatOdomResult::kNonMonotonic:
      return "non_monotonic";
    case AckermannReverseRetreatOdomResult::kStepTooLarge:
      return "step_too_large";
    case AckermannReverseRetreatOdomResult::kTravelExceeded:
      return "travel_exceeded";
    case AckermannReverseRetreatOdomResult::kDisplacementExceeded:
      return "displacement_exceeded";
  }
  return "unknown";
}

// Tracks the physical recovery displacement independently from FollowPath's
// goal checker. The guard is deliberately cumulative: a reverse loop cannot
// hide behind a small final displacement.
class AckermannReverseRetreatOdomGuard
{
public:
  bool arm(
    const AckermannReverseRetreatOdomSample & sample,
    const AckermannReverseRetreatOdomLimits & limits,
    const std::chrono::steady_clock::time_point & now)
  {
    if (!validLimits(limits) || sampleStatus(sample, now, limits) !=
      AckermannReverseRetreatOdomResult::kClear)
    {
      reset();
      return false;
    }
    limits_ = limits;
    start_ = sample;
    last_ = sample;
    travelled_m_ = 0.0;
    armed_ = true;
    return true;
  }

  AckermannReverseRetreatOdomResult observe(
    const AckermannReverseRetreatOdomSample & sample,
    const std::chrono::steady_clock::time_point & now)
  {
    if (!armed_) {
      return AckermannReverseRetreatOdomResult::kUnarmed;
    }
    const auto status = sampleStatus(sample, now, limits_);
    if (status != AckermannReverseRetreatOdomResult::kClear) {
      return status;
    }
    if (sample.sequence < last_.sequence) {
      return AckermannReverseRetreatOdomResult::kNonMonotonic;
    }
    if (sample.sequence == last_.sequence) {
      return samePose(sample, last_) ? AckermannReverseRetreatOdomResult::kClear :
             AckermannReverseRetreatOdomResult::kNonMonotonic;
    }

    const double step_m = distance(last_, sample);
    if (!std::isfinite(step_m) || step_m > limits_.maximum_step_m) {
      return AckermannReverseRetreatOdomResult::kStepTooLarge;
    }
    const double travelled_m = travelled_m_ + step_m;
    if (!std::isfinite(travelled_m) || travelled_m >= limits_.maximum_travel_m) {
      return AckermannReverseRetreatOdomResult::kTravelExceeded;
    }
    const double displacement_m = distance(start_, sample);
    if (!std::isfinite(displacement_m) ||
      displacement_m >= limits_.maximum_displacement_m)
    {
      return AckermannReverseRetreatOdomResult::kDisplacementExceeded;
    }

    travelled_m_ = travelled_m;
    last_ = sample;
    return AckermannReverseRetreatOdomResult::kClear;
  }

  void reset()
  {
    armed_ = false;
    start_ = AckermannReverseRetreatOdomSample{};
    last_ = AckermannReverseRetreatOdomSample{};
    travelled_m_ = 0.0;
  }

  bool armed() const { return armed_; }
  double travelled_m() const { return travelled_m_; }
  double displacement_m() const { return armed_ ? distance(start_, last_) : 0.0; }

private:
  static bool validSample(const AckermannReverseRetreatOdomSample & sample)
  {
    return std::isfinite(sample.x) && std::isfinite(sample.y) &&
           sample.sequence > 0U &&
           sample.received_at != std::chrono::steady_clock::time_point();
  }

  static bool validLimits(const AckermannReverseRetreatOdomLimits & limits)
  {
    return limits.maximum_age.count() > 0 && std::isfinite(limits.maximum_step_m) &&
           std::isfinite(limits.maximum_travel_m) &&
           std::isfinite(limits.maximum_displacement_m) &&
           limits.maximum_step_m > 0.0 && limits.maximum_travel_m > 0.0 &&
           limits.maximum_displacement_m > 0.0;
  }

  static AckermannReverseRetreatOdomResult sampleStatus(
    const AckermannReverseRetreatOdomSample & sample,
    const std::chrono::steady_clock::time_point & now,
    const AckermannReverseRetreatOdomLimits & limits)
  {
    if (!validSample(sample) || !validLimits(limits)) {
      return AckermannReverseRetreatOdomResult::kMalformed;
    }
    if (now < sample.received_at || now - sample.received_at > limits.maximum_age) {
      return AckermannReverseRetreatOdomResult::kStale;
    }
    return AckermannReverseRetreatOdomResult::kClear;
  }

  static bool samePose(
    const AckermannReverseRetreatOdomSample & first,
    const AckermannReverseRetreatOdomSample & second)
  {
    return first.x == second.x && first.y == second.y;
  }

  static double distance(
    const AckermannReverseRetreatOdomSample & first,
    const AckermannReverseRetreatOdomSample & second)
  {
    return std::hypot(second.x - first.x, second.y - first.y);
  }

  bool armed_{false};
  AckermannReverseRetreatOdomLimits limits_{};
  AckermannReverseRetreatOdomSample start_{};
  AckermannReverseRetreatOdomSample last_{};
  double travelled_m_{0.0};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__ACKERMANN_REVERSE_RETREAT_ODOM_GUARD_HPP_
