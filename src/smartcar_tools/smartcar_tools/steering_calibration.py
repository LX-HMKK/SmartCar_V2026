"""Shared validation and kinematics for steering calibration tools."""

import math


MAX_STEERING_ANGLE_RAD = 0.70
MAX_CIRCLE_SPEED_MPS = 0.15
MAX_CIRCLE_DURATION_SEC = 60.0
MAX_CIRCLE_RATE_HZ = 50.0
MAX_STEERING_HOLD_SEC = 15.0


def _require_finite(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_circle_request(angle, speed, duration, rate, wheelbase):
    """Validate a bounded, forward-only ground-circle calibration request."""
    angle = _require_finite("angle", angle)
    speed = _require_finite("speed", speed)
    duration = _require_finite("duration", duration)
    rate = _require_finite("rate", rate)
    wheelbase = _require_finite("wheelbase", wheelbase)
    if not 0.01 <= abs(angle) <= MAX_STEERING_ANGLE_RAD:
        raise ValueError(
            f"abs(angle) must be in [0.01, {MAX_STEERING_ANGLE_RAD:.2f}] rad")
    if not 0.0 < speed <= MAX_CIRCLE_SPEED_MPS:
        raise ValueError(
            f"speed must be in (0, {MAX_CIRCLE_SPEED_MPS:.2f}] m/s")
    if not 0.0 < duration <= MAX_CIRCLE_DURATION_SEC:
        raise ValueError(
            f"duration must be in (0, {MAX_CIRCLE_DURATION_SEC:.0f}] s")
    if not 0.0 < rate <= MAX_CIRCLE_RATE_HZ:
        raise ValueError(
            f"rate must be in (0, {MAX_CIRCLE_RATE_HZ:.0f}] Hz")
    if wheelbase <= 0.0:
        raise ValueError("wheelbase must be positive")
    return angle, speed, duration, rate, wheelbase


def validate_hold_request(angle, duration):
    """Validate a zero-speed wheel-angle measurement request."""
    angle = _require_finite("angle", angle)
    duration = _require_finite("hold", duration)
    if not 0.0 < abs(angle) <= MAX_STEERING_ANGLE_RAD:
        raise ValueError(
            f"abs(angle) must be in (0, {MAX_STEERING_ANGLE_RAD:.2f}] rad")
    if not 0.0 < duration <= MAX_STEERING_HOLD_SEC:
        raise ValueError(
            f"hold must be in (0, {MAX_STEERING_HOLD_SEC:.0f}] s")
    return angle, duration


def angular_velocity_for_steering(speed, steering_angle, wheelbase):
    """Return the Twist angular.z that maps to a requested Ackermann angle."""
    speed = _require_finite("speed", speed)
    steering_angle = _require_finite("steering_angle", steering_angle)
    wheelbase = _require_finite("wheelbase", wheelbase)
    if speed == 0.0:
        raise ValueError("speed must be nonzero")
    if wheelbase <= 0.0:
        raise ValueError("wheelbase must be positive")
    return speed * math.tan(steering_angle) / wheelbase
