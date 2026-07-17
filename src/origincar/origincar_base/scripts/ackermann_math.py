#!/usr/bin/env python3

import math


_VELOCITY_EPSILON = 1e-9


def steering_angle(
    linear_velocity,
    angular_velocity,
    wheelbase,
    max_steering_angle,
):
    if (
        abs(linear_velocity) <= _VELOCITY_EPSILON
        or abs(angular_velocity) <= _VELOCITY_EPSILON
    ):
        return 0.0

    angle = math.atan(wheelbase * angular_velocity / linear_velocity)
    limit = abs(max_steering_angle)
    return max(-limit, min(limit, angle))
