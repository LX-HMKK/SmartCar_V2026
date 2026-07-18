"""Pure velocity-command validation shared by the ROS safety wrapper."""
import math


TWIST_COMPONENT_COUNT = 6
ZERO_TWIST_COMPONENTS = (0.0,) * TWIST_COMPONENT_COUNT


def values_are_finite(values):
    """Return whether every supplied value converts to a finite float."""
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False
    return all(math.isfinite(value) for value in converted)


def sanitize_twist_components(components):
    """Return finite Twist components or a fail-closed all-zero tuple."""
    try:
        values = tuple(float(value) for value in components)
    except (TypeError, ValueError, OverflowError):
        return False, ZERO_TWIST_COMPONENTS

    if len(values) != TWIST_COMPONENT_COUNT:
        return False, ZERO_TWIST_COMPONENTS
    if not values_are_finite(values):
        return False, ZERO_TWIST_COMPONENTS
    return True, values
