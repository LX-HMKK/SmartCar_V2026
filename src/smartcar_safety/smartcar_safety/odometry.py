"""ROS-independent validation for odometry-shaped messages."""
import math


def _values_are_finite(values):
    try:
        return all(math.isfinite(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def odometry_is_finite(message):
    """Check only pose and twist scalars on an already-decoded message."""
    try:
        pose = message.pose.pose
        twist = message.twist.twist
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
    except (AttributeError, TypeError, ValueError):
        return False
    return _values_are_finite(values)
