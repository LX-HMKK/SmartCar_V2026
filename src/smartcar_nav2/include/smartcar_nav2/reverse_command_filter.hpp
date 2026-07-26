#ifndef SMARTCAR_NAV2__REVERSE_COMMAND_FILTER_HPP_
#define SMARTCAR_NAV2__REVERSE_COMMAND_FILTER_HPP_

#include <algorithm>
#include <cmath>

namespace smartcar_nav2
{

/// Limits applied to the command emitted by a reverse-only Ackermann controller.
struct ReverseCommandLimits
{
  double vx_min{-0.09};
  double wz_max{0.20};
  double min_turning_radius{0.55};
};

/// ROS-independent representation of the six Twist fields.
struct ReverseCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

enum class ReverseCommandFilterStatus
{
  kAccepted,
  kInvalidLimits,
  kNonFiniteCommand,
  kForwardVelocity
};

struct ReverseCommandFilterResult
{
  ReverseCommand command{};
  ReverseCommandFilterStatus status{ReverseCommandFilterStatus::kInvalidLimits};
  bool limited{false};
};

inline bool finiteValue(double value)
{
  return std::isfinite(value);
}

inline bool validReverseCommandLimits(const ReverseCommandLimits & limits)
{
  return finiteValue(limits.vx_min) && finiteValue(limits.wz_max) &&
         finiteValue(limits.min_turning_radius) && limits.vx_min < 0.0 &&
         limits.wz_max >= 0.0 && limits.min_turning_radius > 0.0;
}

inline bool finiteReverseCommand(const ReverseCommand & command)
{
  return finiteValue(command.linear_x) && finiteValue(command.linear_y) &&
         finiteValue(command.linear_z) && finiteValue(command.angular_x) &&
         finiteValue(command.angular_y) && finiteValue(command.angular_z);
}

inline const char * reverseCommandFilterStatusName(ReverseCommandFilterStatus status)
{
  switch (status) {
    case ReverseCommandFilterStatus::kAccepted:
      return "accepted";
    case ReverseCommandFilterStatus::kInvalidLimits:
      return "invalid_limits";
    case ReverseCommandFilterStatus::kNonFiniteCommand:
      return "non_finite_command";
    case ReverseCommandFilterStatus::kForwardVelocity:
      return "forward_velocity";
  }
  return "unknown";
}

/**
 * Apply the final reverse-only Ackermann command guard.
 *
 * Rejection is fail-closed: every output field is zero. Accepted commands
 * retain only the longitudinal and yaw components supported by Ackermann.
 */
inline ReverseCommandFilterResult enforceReverseCommandLimits(
  const ReverseCommand & input, const ReverseCommandLimits & limits)
{
  ReverseCommandFilterResult result;
  if (!validReverseCommandLimits(limits)) {
    result.status = ReverseCommandFilterStatus::kInvalidLimits;
    return result;
  }
  if (!finiteReverseCommand(input)) {
    result.status = ReverseCommandFilterStatus::kNonFiniteCommand;
    return result;
  }
  if (input.linear_x > 0.0) {
    result.status = ReverseCommandFilterStatus::kForwardVelocity;
    return result;
  }

  result.status = ReverseCommandFilterStatus::kAccepted;
  result.command.linear_x = std::max(input.linear_x, limits.vx_min);
  result.limited = result.command.linear_x != input.linear_x;

  // A zero-velocity command must never carry a rotational command.
  if (result.command.linear_x == 0.0) {
    result.command.angular_z = 0.0;
    result.limited = result.limited || input.angular_z != 0.0 ||
      input.linear_y != 0.0 || input.linear_z != 0.0 ||
      input.angular_x != 0.0 || input.angular_y != 0.0;
    return result;
  }

  const double curvature_limit =
    std::abs(result.command.linear_x) / limits.min_turning_radius;
  const double angular_limit = std::min(limits.wz_max, curvature_limit);
  result.command.angular_z = std::clamp(input.angular_z, -angular_limit, angular_limit);
  result.limited = result.limited || result.command.angular_z != input.angular_z ||
    input.linear_y != 0.0 || input.linear_z != 0.0 ||
    input.angular_x != 0.0 || input.angular_y != 0.0;
  return result;
}

/// Map a positive-forward MPPI command back to the real reverse command domain.
inline ReverseCommand mapVirtualForwardCommandToReverse(
  const ReverseCommand & virtual_command)
{
  ReverseCommand reverse_command = virtual_command;
  reverse_command.linear_x = -virtual_command.linear_x;
  reverse_command.linear_y = -virtual_command.linear_y;
  return reverse_command;
}

/**
 * Translation used before forwarding a speed-limit callback to MPPI.
 *
 * The wrapper exposes reverse speed magnitudes while its delegated MPPI uses
 * virtual positive-forward constraints. Converting absolute values to a
 * percentage keeps both domains on the same dynamic scale. Percentage and
 * no-limit requests retain their original representation.
 */
struct ReverseSpeedLimitTranslation
{
  bool valid{false};
  bool no_limit{false};
  double forwarded_speed_limit{0.0};
  bool forwarded_percentage{false};
  double guard_scale{1.0};
};

inline ReverseSpeedLimitTranslation translateReverseSpeedLimit(
  double speed_limit, bool percentage, double configured_vx_min)
{
  ReverseSpeedLimitTranslation result;
  if (!finiteValue(speed_limit) || speed_limit < 0.0 ||
    !finiteValue(configured_vx_min) || configured_vx_min >= 0.0)
  {
    return result;
  }

  result.valid = true;
  if (speed_limit == 0.0) {
    result.no_limit = true;
    result.forwarded_speed_limit = speed_limit;
    result.forwarded_percentage = percentage;
    result.guard_scale = 1.0;
    return result;
  }

  const double scale = percentage ? speed_limit / 100.0 :
    speed_limit / std::abs(configured_vx_min);
  const double forwarded = percentage ? speed_limit : scale * 100.0;
  if (!finiteValue(scale) || !finiteValue(forwarded)) {
    return ReverseSpeedLimitTranslation{};
  }

  result.forwarded_speed_limit = forwarded;
  result.forwarded_percentage = true;
  // A speed-limit callback may reduce speed, but must never lift the static
  // reverse-only safety envelope above the configured controller limits.
  result.guard_scale = std::min(std::max(scale, 0.0), 1.0);
  return result;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__REVERSE_COMMAND_FILTER_HPP_
