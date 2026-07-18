"""Pure protocol checks shared by ROS mission adapters and tests."""
import math

from smartcar_task.mission import OperationResult


STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5


def classify_follow_waypoints_result(status, missed_waypoints):
    missed = tuple(int(index) for index in missed_waypoints)
    if int(status) == STATUS_SUCCEEDED and not missed:
        return OperationResult(True, "ok")
    if int(status) == STATUS_CANCELED:
        return OperationResult(False, "navigation_canceled")
    if int(status) == STATUS_SUCCEEDED:
        return OperationResult(
            False,
            "navigation_missed_waypoints:" + ",".join(map(str, missed)),
        )
    return OperationResult(False, f"navigation_status_{int(status)}")


def _finite(values):
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def _yaw_from_quaternion(x, y, z, w):
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def odometry_matches_origin(
    message,
    expected_frame="odom_combined",
    expected_child_frame="base_footprint",
    position_tolerance=0.20,
    yaw_tolerance=0.20,
):
    if message.header.frame_id != expected_frame:
        return False
    if message.child_frame_id != expected_child_frame:
        return False
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    twist = message.twist.twist
    values = [
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    ]
    values.extend(message.pose.covariance)
    values.extend(message.twist.covariance)
    if not _finite(values):
        return False
    norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if abs(norm - 1.0) > 1e-3:
        return False
    yaw = _yaw_from_quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    return (
        abs(position.x) <= float(position_tolerance)
        and abs(position.y) <= float(position_tolerance)
        and abs(yaw) <= float(yaw_tolerance)
    )


def run_reset_sequence(
    navigation_stopped,
    set_pose,
    wait_for_verified_origin,
    clear_localization_fault,
):
    if not navigation_stopped():
        return OperationResult(False, "navigation_not_stopped")
    set_pose_result = set_pose()
    if not set_pose_result.success:
        return set_pose_result
    origin_result = wait_for_verified_origin()
    if not origin_result.success:
        return origin_result
    clear_result = clear_localization_fault()
    if not clear_result.success:
        return clear_result
    return OperationResult(True, "ok")
