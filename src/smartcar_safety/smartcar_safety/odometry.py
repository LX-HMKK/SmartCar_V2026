"""ROS-independent validation for odometry-shaped messages."""
import math


def _values_are_finite(values):
    try:
        for value in values:
            if not math.isfinite(value):
                return False
    except (TypeError, ValueError, OverflowError):
        return False
    return True


def odometry_is_finite(message):
    """Return whether all pose, twist, and covariance values are finite."""
    pose_with_covariance = message.pose
    twist_with_covariance = message.twist
    pose = pose_with_covariance.pose
    twist = twist_with_covariance.twist
    position = pose.position
    orientation = pose.orientation
    linear = twist.linear
    angular = twist.angular

    try:
        scalar_values_are_finite = (
            math.isfinite(position.x)
            and math.isfinite(position.y)
            and math.isfinite(position.z)
            and math.isfinite(orientation.x)
            and math.isfinite(orientation.y)
            and math.isfinite(orientation.z)
            and math.isfinite(orientation.w)
            and math.isfinite(linear.x)
            and math.isfinite(linear.y)
            and math.isfinite(linear.z)
            and math.isfinite(angular.x)
            and math.isfinite(angular.y)
            and math.isfinite(angular.z)
        )
    except (TypeError, ValueError, OverflowError):
        return False

    return (
        scalar_values_are_finite
        and _values_are_finite(pose_with_covariance.covariance)
        and _values_are_finite(twist_with_covariance.covariance)
    )
