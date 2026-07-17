#!/usr/bin/env python3

import math


_VELOCITY_EPSILON = 1e-9


def _all_finite(*values):
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def steering_angle(
    linear_velocity,
    angular_velocity,
    wheelbase,
    max_steering_angle,
):
    if not _all_finite(
        linear_velocity,
        angular_velocity,
        wheelbase,
        max_steering_angle,
    ):
        return 0.0

    if (
        abs(linear_velocity) <= _VELOCITY_EPSILON
        or abs(angular_velocity) <= _VELOCITY_EPSILON
    ):
        return 0.0

    angle = math.atan(wheelbase * angular_velocity / linear_velocity)
    limit = abs(max_steering_angle)
    return max(-limit, min(limit, angle))


def ackermann_command(
    linear_velocity,
    angular_velocity,
    wheelbase,
    max_steering_angle,
):
    """Return a finite speed/steering pair or a fail-closed zero command."""
    if not _all_finite(
        linear_velocity,
        angular_velocity,
        wheelbase,
        max_steering_angle,
    ):
        return 0.0, 0.0
    return (
        float(linear_velocity),
        steering_angle(
            linear_velocity,
            angular_velocity,
            wheelbase,
            max_steering_angle,
        ),
    )
