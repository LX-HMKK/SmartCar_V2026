"""Pure protocol checks shared by ROS mission adapters and tests."""
from dataclasses import dataclass
import math
import time

from smartcar_task.mission import OperationResult


STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
MOTION_FORWARD = 1
MOTION_FORWARD_RECOVERY = 3


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


def classify_navigate_to_pose_result(status):
    if int(status) == STATUS_SUCCEEDED:
        return OperationResult(True, "ok")
    if int(status) == STATUS_CANCELED:
        return OperationResult(False, "navigation_canceled")
    return OperationResult(False, f"navigation_status_{int(status)}")


def motion_direction():
    """Permit forward tracking plus Nav2's bounded native BackUp recovery."""
    return MOTION_FORWARD_RECOVERY


def navigation_behavior_tree(
    goal_profile,
    precise_behavior_tree,
    transit_behavior_tree,
    heading_locked=True,
):
    profile = str(goal_profile).strip()
    if profile not in {"standard", "precise"}:
        raise ValueError(f"unknown goal profile {profile!r}")
    if profile == "precise":
        value = str(precise_behavior_tree).strip()
        if not value:
            raise ValueError("precise_behavior_tree must not be empty")
        return value
    if not bool(heading_locked):
        value = str(transit_behavior_tree).strip()
        if not value:
            raise ValueError("transit_behavior_tree must not be empty")
        return value
    return ""


@dataclass(frozen=True)
class MotionLease:
    direction: int
    generation: int
    action_uuid: object
    boot_epoch: int = 0
    lease_id: int = 0

    def __post_init__(self):
        if self.direction not in {MOTION_FORWARD, MOTION_FORWARD_RECOVERY}:
            raise ValueError("unknown motion direction")
        if int(self.generation) <= 0:
            raise ValueError("motion generation must be positive")
        if int(self.boot_epoch) < 0 or int(self.lease_id) < 0:
            raise ValueError("motion lease identifiers must be nonnegative")


def _stage_result(stage, result):
    if result.success:
        return result
    return OperationResult(False, f"direction_{stage}:{result.status}")


class MotionDirectionProtocol:
    """Fail-closed lease protocol independent of ROS message transport."""

    def __init__(
        self,
        guard,
        wait_stopped,
        prepare_timeout_sec=1.0,
        prepare_retry_period_sec=0.02,
        monotonic=time.monotonic,
        sleep=time.sleep,
    ):
        self._guard = guard
        self._wait_stopped = wait_stopped
        self._prepare_timeout_sec = float(prepare_timeout_sec)
        self._prepare_retry_period_sec = float(prepare_retry_period_sec)
        if (
            not math.isfinite(self._prepare_timeout_sec)
            or self._prepare_timeout_sec <= 0.0
            or not math.isfinite(self._prepare_retry_period_sec)
            or self._prepare_retry_period_sec <= 0.0
        ):
            raise ValueError("direction prepare timing must be finite and positive")
        self._monotonic = monotonic
        self._sleep = sleep

    @staticmethod
    def provisional(direction, generation, action_uuid):
        return MotionLease(
            direction=int(direction),
            generation=int(generation),
            action_uuid=action_uuid,
        )

    def prepare(self, direction, generation, action_uuid):
        provisional = self.provisional(direction, generation, action_uuid)
        revoked = self.revoke(provisional)
        if not revoked.success:
            return revoked, None
        deadline = self._monotonic() + self._prepare_timeout_sec
        while True:
            result, boot_epoch, lease_id = self._guard.prepare(provisional)
            if result.success:
                break
            if result.status != "stop_barrier_not_ready":
                return _stage_result("prepare", result), None
            now = self._monotonic()
            if now >= deadline:
                return OperationResult(
                    False, "direction_prepare:stop_barrier_timeout"), None
            self._sleep(min(
                self._prepare_retry_period_sec,
                max(0.0, deadline - now),
            ))
        try:
            lease = MotionLease(
                direction=provisional.direction,
                generation=provisional.generation,
                action_uuid=provisional.action_uuid,
                boot_epoch=int(boot_epoch),
                lease_id=int(lease_id),
            )
        except (TypeError, ValueError, OverflowError) as error:
            return OperationResult(
                False,
                f"direction_prepare_invalid:{type(error).__name__}",
            ), None
        if lease.boot_epoch == 0 or lease.lease_id == 0:
            return OperationResult(
                False, "direction_prepare_invalid:zero_identity"), None
        return OperationResult(True, "ok"), lease

    def activate(self, lease):
        return _stage_result("activate", self._guard.activate(lease))

    def renew(self, lease):
        return _stage_result("renew", self._guard.renew(lease))

    def revoke(self, lease):
        return _stage_result("stop", self._guard.stop(lease))

    def settle(self):
        return _stage_result("settle", self._wait_stopped())

    def stop(self, lease):
        return self.revoke(lease)


def _finite(values):
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def twist_is_stopped(twist, linear_tolerance, angular_tolerance):
    linear_limit = float(linear_tolerance)
    angular_limit = float(angular_tolerance)
    linear = (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
    )
    angular = (
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )
    return (
        math.isfinite(linear_limit)
        and math.isfinite(angular_limit)
        and linear_limit >= 0.0
        and angular_limit >= 0.0
        and _finite(linear + angular)
        and all(abs(float(value)) <= linear_limit for value in linear)
        and all(abs(float(value)) <= angular_limit for value in angular)
    )


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
):
    if not navigation_stopped():
        return OperationResult(False, "navigation_not_stopped")
    set_pose_result = set_pose()
    if not set_pose_result.success:
        return set_pose_result
    origin_result = wait_for_verified_origin()
    if not origin_result.success:
        return origin_result
    return OperationResult(True, "ok")
