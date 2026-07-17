"""Pure velocity-command validation shared by the ROS safety wrapper."""
import math


TWIST_COMPONENT_COUNT = 6
ZERO_TWIST_COMPONENTS = (0.0,) * TWIST_COMPONENT_COUNT


def sanitize_twist_components(components):
    """Return finite Twist components or a fail-closed all-zero tuple."""
    try:
        values = tuple(float(value) for value in components)
    except (TypeError, ValueError, OverflowError):
        return False, ZERO_TWIST_COMPONENTS

    if len(values) != TWIST_COMPONENT_COUNT:
        return False, ZERO_TWIST_COMPONENTS
    if not all(math.isfinite(value) for value in values):
        return False, ZERO_TWIST_COMPONENTS
    return True, values
