#ifndef SMARTCAR_NAV2__FORWARD_COMMAND_FILTER_HPP_
#define SMARTCAR_NAV2__FORWARD_COMMAND_FILTER_HPP_

#include <algorithm>
#include <cmath>

namespace smartcar_nav2
{

/// Limits applied at the final output of the forward Ackermann controller.
struct ForwardCommandLimits
{
  double vx_max{0.15};
  double wz_max{0.70};
  double min_turning_radius{0.22};
};

/// ROS-independent representation of the six Twist fields.
struct ForwardCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

enum class ForwardCommandFilterStatus
{
  kAccepted,
  kInvalidLimits,
  kNonFiniteCommand,
  kReverseVelocity
};

struct ForwardCommandFilterResult
{
  ForwardCommand command{};
  ForwardCommandFilterStatus status{ForwardCommandFilterStatus::kInvalidLimits};
  bool limited{false};
};

inline bool finiteForwardValue(double value)
{
  return std::isfinite(value);
}

inline bool validForwardCommandLimits(const ForwardCommandLimits & limits)
{
  return finiteForwardValue(limits.vx_max) && finiteForwardValue(limits.wz_max) &&
         finiteForwardValue(limits.min_turning_radius) && limits.vx_max > 0.0 &&
         limits.wz_max >= 0.0 && limits.min_turning_radius > 0.0;
}

inline bool finiteForwardCommand(const ForwardCommand & command)
{
  return finiteForwardValue(command.linear_x) &&
         finiteForwardValue(command.linear_y) &&
         finiteForwardValue(command.linear_z) &&
         finiteForwardValue(command.angular_x) &&
         finiteForwardValue(command.angular_y) &&
         finiteForwardValue(command.angular_z);
}

inline const char * forwardCommandFilterStatusName(ForwardCommandFilterStatus status)
{
  switch (status) {
    case ForwardCommandFilterStatus::kAccepted:
      return "accepted";
    case ForwardCommandFilterStatus::kInvalidLimits:
      return "invalid_limits";
    case ForwardCommandFilterStatus::kNonFiniteCommand:
      return "non_finite_command";
    case ForwardCommandFilterStatus::kReverseVelocity:
      return "reverse_velocity";
  }
  return "unknown";
}

/**
 * Apply a final forward-only Ackermann command guard.
 *
 * MPPI constrains its sampled trajectories, but its final command can still
 * contain a near-zero velocity paired with a large yaw rate after smoothing.
 * This guard is the last point before the velocity smoother and therefore
 * keeps the physical command inside the same minimum-radius envelope.
 */
inline ForwardCommandFilterResult enforceForwardCommandLimits(
  const ForwardCommand & input, const ForwardCommandLimits & limits)
{
  ForwardCommandFilterResult result;
  if (!validForwardCommandLimits(limits)) {
    result.status = ForwardCommandFilterStatus::kInvalidLimits;
    return result;
  }
  if (!finiteForwardCommand(input)) {
    result.status = ForwardCommandFilterStatus::kNonFiniteCommand;
    return result;
  }
  if (input.linear_x < 0.0) {
    result.status = ForwardCommandFilterStatus::kReverseVelocity;
    return result;
  }

  result.status = ForwardCommandFilterStatus::kAccepted;
  result.command = input;
  result.command.linear_x = std::clamp(input.linear_x, 0.0, limits.vx_max);
  result.limited = result.command.linear_x != input.linear_x;

  // A zero-velocity command must never carry a rotational command. This is
  // the condition that otherwise becomes an in-place turn in an Ackermann
  // simulator or on a low-speed physical chassis.
  if (result.command.linear_x == 0.0) {
    result.command.angular_z = 0.0;
    result.limited = result.limited || input.angular_z != 0.0;
  } else {
    const double curvature_limit =
      result.command.linear_x / limits.min_turning_radius;
    const double angular_limit = std::min(limits.wz_max, curvature_limit);
    result.command.angular_z = std::clamp(
      input.angular_z, -angular_limit, angular_limit);
    result.limited = result.limited || result.command.angular_z != input.angular_z;
  }

  result.command.linear_y = 0.0;
  result.command.linear_z = 0.0;
  result.command.angular_x = 0.0;
  result.command.angular_y = 0.0;
  result.limited = result.limited || input.linear_y != 0.0 || input.linear_z != 0.0 ||
    input.angular_x != 0.0 || input.angular_y != 0.0;
  return result;
}

struct ForwardSpeedLimitTranslation
{
  bool valid{false};
  bool no_limit{false};
  double forwarded_speed_limit{0.0};
  bool forwarded_percentage{false};
  double guard_scale{1.0};
};

inline ForwardSpeedLimitTranslation translateForwardSpeedLimit(
  double speed_limit, bool percentage, double configured_vx_max)
{
  ForwardSpeedLimitTranslation result;
  if (!finiteForwardValue(speed_limit) || speed_limit < 0.0 ||
    !finiteForwardValue(configured_vx_max) || configured_vx_max <= 0.0)
  {
    return result;
  }

  result.valid = true;
  if (speed_limit == 0.0) {
    result.no_limit = true;
    result.forwarded_speed_limit = speed_limit;
    result.forwarded_percentage = percentage;
    return result;
  }

  const double scale = percentage ? speed_limit / 100.0 : speed_limit / configured_vx_max;
  const double forwarded = percentage ? speed_limit : scale * 100.0;
  if (!finiteForwardValue(scale) || !finiteForwardValue(forwarded)) {
    return ForwardSpeedLimitTranslation{};
  }

  result.forwarded_speed_limit = forwarded;
  result.forwarded_percentage = true;
  result.guard_scale = std::min(std::max(scale, 0.0), 1.0);
  return result;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__FORWARD_COMMAND_FILTER_HPP_
