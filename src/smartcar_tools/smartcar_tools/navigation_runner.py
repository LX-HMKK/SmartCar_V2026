"""Explicitly armed full-course NavigateThroughPoses test runner."""
import json
import math
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from robot_localization.srv import SetPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from smartcar_tools.route_model import load_route


MOTION_GATES = (
    "waypoints_calibrated",
    "extrinsics_calibrated",
    "steering_calibrated",
    "emergency_stop_ready",
    "operator_approved",
)


def _positive_finite(name, value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _nonnegative_finite(name, value):
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return result


def _wait_future(future, timeout_sec):
    event = threading.Event()
    future.add_done_callback(lambda _future: event.set())
    if not event.wait(max(0.0, float(timeout_sec))):
        return False, None, None
    try:
        return True, future.result(), None
    except Exception as error:  # Transport exceptions propagate through futures.
        return True, None, error


def _remove_pending(client, future):
    try:
        client.remove_pending_request(future)
    except (AttributeError, KeyError, RuntimeError):
        pass


def _finite(values):
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def _yaw_from_quaternion(orientation):
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def odometry_matches_origin(
    message,
    expected_frame,
    expected_child_frame,
    position_tolerance,
    yaw_tolerance,
):
    """Return whether a finite, normalized odometry sample is at P origin."""
    if message is None:
        return False
    if message.header.frame_id != expected_frame:
        return False
    if message.child_frame_id != expected_child_frame:
        return False
    pose = message.pose.pose
    twist = message.twist.twist
    values = [
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
        *message.pose.covariance,
        *message.twist.covariance,
    ]
    if not _finite(values):
        return False
    orientation = pose.orientation
    norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if abs(norm - 1.0) > 1.0e-3:
        return False
    return (
        abs(pose.position.x) <= float(position_tolerance)
        and abs(pose.position.y) <= float(position_tolerance)
        and abs(_yaw_from_quaternion(orientation)) <= float(yaw_tolerance)
    )


def odometry_is_finite(message):
    """Return whether all pose/twist values in an odometry sample are finite."""
    if message is None:
        return False
    pose = message.pose.pose
    twist = message.twist.twist
    return _finite([
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
        *message.pose.covariance,
        *message.twist.covariance,
    ])


class NavigationRunner(Node):
    """Run one full route only after prepare and arm services succeed."""

    def __init__(self):
        super().__init__("navigation_test_runner")
        self.declare_parameter("route_file", "")
        self.declare_parameter("behavior_tree", "")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("sensor_timeout_sec", 0.50)
        self.declare_parameter("reset_timeout_sec", 5.0)
        self.declare_parameter("service_timeout_sec", 2.0)
        self.declare_parameter("action_server_timeout_sec", 5.0)
        self.declare_parameter("goal_response_timeout_sec", 5.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)
        self.declare_parameter("navigation_timeout_sec", 180.0)
        self.declare_parameter("arm_timeout_sec", 30.0)
        self.declare_parameter("origin_position_tolerance", 0.20)
        self.declare_parameter("origin_yaw_tolerance", 0.20)
        self.declare_parameter("use_laser_odometry", False)
        self.declare_parameter("laser_odometry_calibrated", False)
        self.declare_parameter("laser_odom_topic", "/odom_laser")
        self.declare_parameter(
            "laser_reset_service",
            "/smartcar/localization/reset_laser_odometry",
        )
        for gate in MOTION_GATES:
            self.declare_parameter(gate, False)

        route_file = str(self.get_parameter("route_file").value).strip()
        if not route_file:
            raise ValueError("route_file must be provided")
        self._route_file = route_file
        self._route = load_route(route_file)
        self._behavior_tree = str(
            self.get_parameter("behavior_tree").value
        ).strip()
        self._base_frame = str(self.get_parameter("base_frame").value).strip()
        if not self._base_frame:
            raise ValueError("base_frame must not be empty")
        self._sensor_timeout_sec = _positive_finite(
            "sensor_timeout_sec",
            self.get_parameter("sensor_timeout_sec").value,
        )
        self._reset_timeout_sec = _positive_finite(
            "reset_timeout_sec",
            self.get_parameter("reset_timeout_sec").value,
        )
        self._service_timeout_sec = _positive_finite(
            "service_timeout_sec",
            self.get_parameter("service_timeout_sec").value,
        )
        self._action_server_timeout_sec = _positive_finite(
            "action_server_timeout_sec",
            self.get_parameter("action_server_timeout_sec").value,
        )
        self._goal_response_timeout_sec = _positive_finite(
            "goal_response_timeout_sec",
            self.get_parameter("goal_response_timeout_sec").value,
        )
        self._cancel_timeout_sec = _positive_finite(
            "cancel_timeout_sec",
            self.get_parameter("cancel_timeout_sec").value,
        )
        self._navigation_timeout_sec = _positive_finite(
            "navigation_timeout_sec",
            self.get_parameter("navigation_timeout_sec").value,
        )
        self._arm_timeout_sec = _positive_finite(
            "arm_timeout_sec",
            self.get_parameter("arm_timeout_sec").value,
        )
        self._origin_position_tolerance = _nonnegative_finite(
            "origin_position_tolerance",
            self.get_parameter("origin_position_tolerance").value,
        )
        self._origin_yaw_tolerance = _nonnegative_finite(
            "origin_yaw_tolerance",
            self.get_parameter("origin_yaw_tolerance").value,
        )
        self._motion_gates = {
            gate: bool(self.get_parameter(gate).value)
            for gate in MOTION_GATES
        }
        self._use_laser_odometry = bool(
            self.get_parameter("use_laser_odometry").value
        )
        self._laser_odometry_calibrated = bool(
            self.get_parameter("laser_odometry_calibrated").value
        )
        self._laser_odom_topic = str(
            self.get_parameter("laser_odom_topic").value
        ).strip()
        self._laser_reset_service = str(
            self.get_parameter("laser_reset_service").value
        ).strip()
        if self._use_laser_odometry and not self._laser_odom_topic:
            raise ValueError(
                "laser_odom_topic must not be empty when laser odometry is enabled"
            )

        self._condition = threading.Condition(threading.RLock())
        self._state = "locking_estop"
        self._failure_reason = ""
        self._prepared = False
        self._estop_latched = False
        self._estop_retry_required = True
        self._pending_terminal = None
        self._armed_deadline = None
        self._started_at = None
        self._finished_elapsed_sec = 0.0
        self._current_index = 0
        self._remaining_points = len(self._route.waypoints)
        self._distance_remaining_m = 0.0
        self._last_odom = None
        self._last_odom_received_at = None
        self._last_scan_received_at = None
        self._last_laser_odom = None
        self._last_laser_odom_received_at = None
        self._odom_sequence = 0
        self._goal_generation = 0
        self._terminalized_generation = None
        self._pending_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_requested = False
        self._forced_failure_reason = ""
        self._last_status_publish_at = 0.0

        self._io_group = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String,
            "/smartcar/test/navigation/status",
            status_qos,
        )
        self.create_subscription(
            Odometry,
            "/odom_combined",
            self._on_odom,
            10,
            callback_group=self._io_group,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
            callback_group=self._io_group,
        )
        if self._use_laser_odometry:
            self.create_subscription(
                Odometry,
                self._laser_odom_topic,
                self._on_laser_odom,
                10,
                callback_group=self._io_group,
            )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._estop_client = self.create_client(
            SetBool,
            "/smartcar/safety/emergency_stop",
            callback_group=self._io_group,
        )
        self._set_pose_client = self.create_client(
            SetPose,
            "/set_pose",
            callback_group=self._io_group,
        )
        self._clear_fault_client = self.create_client(
            Trigger,
            "/smartcar/safety/clear_localization_fault",
            callback_group=self._io_group,
        )
        self._laser_reset_client = None
        if self._use_laser_odometry and self._laser_reset_service:
            self._laser_reset_client = self.create_client(
                Trigger,
                self._laser_reset_service,
                callback_group=self._io_group,
            )
        self._action_client = ActionClient(
            self,
            NavigateThroughPoses,
            "/navigate_through_poses",
            callback_group=self._io_group,
        )

        self.create_service(
            Trigger,
            "/smartcar/test/navigation/prepare",
            self._on_prepare,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            "/smartcar/test/navigation/arm",
            self._on_arm,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            "/smartcar/test/navigation/start",
            self._on_start,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            "/smartcar/test/navigation/stop",
            self._on_stop,
            callback_group=self._service_group,
        )
        self.create_timer(
            0.20,
            self._on_housekeeping,
            callback_group=self._service_group,
        )
        self._publish_status(force=True)
        self.get_logger().warning(
            "Navigation test runner loaded an unstarted route with "
            f"{len(self._route.waypoints)} points from {route_file}"
        )

    def _now_monotonic(self):
        return time.monotonic()

    def _navigation_active_locked(self):
        return (
            self._pending_goal_future is not None
            or self._goal_handle is not None
        )

    def _elapsed_locked(self):
        if self._started_at is None:
            return self._finished_elapsed_sec
        return max(0.0, self._now_monotonic() - self._started_at)

    def _snapshot_locked(self):
        waypoint_count = len(self._route.waypoints)
        current_point = ""
        if waypoint_count:
            index = min(max(0, self._current_index), waypoint_count - 1)
            current_point = str(self._route.waypoints[index].id)
        arm_remaining = 0.0
        if self._armed_deadline is not None:
            arm_remaining = max(
                0.0,
                self._armed_deadline - self._now_monotonic(),
            )
        return {
            "state": self._state,
            "route_file": str(Path(self._route_file).name),
            "route_calibrated": bool(self._route.calibrated),
            "laser_odometry_enabled": bool(self._use_laser_odometry),
            "laser_odometry_calibrated": bool(
                self._laser_odometry_calibrated
            ),
            "estop_latched": bool(self._estop_latched),
            "current_point": current_point,
            "current_index": int(self._current_index),
            "total_points": waypoint_count,
            "remaining_points": int(self._remaining_points),
            "distance_remaining_m": round(
                float(self._distance_remaining_m), 3
            ),
            "elapsed_sec": round(self._elapsed_locked(), 2),
            "arm_remaining_sec": round(arm_remaining, 2),
            "failure_reason": self._failure_reason,
        }

    def _publish_status(self, force=False):
        now = self._now_monotonic()
        if not force and now - self._last_status_publish_at < 0.10:
            return
        with self._condition:
            payload = self._snapshot_locked()
        self._status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True))
        )
        self._last_status_publish_at = now

    def _set_state(self, state, failure_reason=""):
        with self._condition:
            self._state = str(state)
            self._failure_reason = str(failure_reason)
            self._condition.notify_all()
        self._publish_status(force=True)

    def _on_odom(self, message):
        with self._condition:
            self._last_odom = message
            self._last_odom_received_at = self._now_monotonic()
            self._odom_sequence += 1
            self._condition.notify_all()

    def _on_scan(self, _message):
        with self._condition:
            self._last_scan_received_at = self._now_monotonic()

    def _on_laser_odom(self, message):
        with self._condition:
            self._last_laser_odom = message
            self._last_laser_odom_received_at = self._now_monotonic()

    def _call_estop(self, latch, timeout_sec=None):
        if latch:
            # Treat an unconfirmed latch as unsafe until the service proves it.
            with self._condition:
                self._estop_latched = False
                self._estop_retry_required = True
        timeout = (
            self._service_timeout_sec
            if timeout_sec is None
            else max(0.0, float(timeout_sec))
        )
        started = self._now_monotonic()
        if not self._estop_client.wait_for_service(timeout_sec=timeout):
            return False, "emergency_stop_service_unavailable"
        remaining = max(0.0, timeout - (self._now_monotonic() - started))
        request = SetBool.Request()
        request.data = bool(latch)
        future = self._estop_client.call_async(request)
        completed, response, error = _wait_future(future, remaining)
        if not completed:
            _remove_pending(self._estop_client, future)
            return False, "emergency_stop_timeout"
        if error is not None or response is None:
            return False, f"emergency_stop_error:{type(error).__name__}"
        if not response.success:
            return False, str(response.message) or "emergency_stop_rejected"
        with self._condition:
            self._estop_latched = bool(latch)
            self._estop_retry_required = False
        return True, str(response.message)

    def _reload_route(self):
        route = load_route(self._route_file)
        with self._condition:
            self._route = route
            self._current_index = 0
            self._remaining_points = len(route.waypoints)
            self._distance_remaining_m = 0.0

    def _reset_origin(self):
        deadline = self._now_monotonic() + self._reset_timeout_sec
        reset_clients = [self._set_pose_client, self._clear_fault_client]
        if self._laser_reset_client is not None:
            reset_clients.insert(0, self._laser_reset_client)
        for client in reset_clients:
            remaining = deadline - self._now_monotonic()
            if remaining <= 0.0 or not client.wait_for_service(
                timeout_sec=remaining
            ):
                return False, "reset_service_unavailable"

        if self._laser_reset_client is not None:
            future = self._laser_reset_client.call_async(Trigger.Request())
            completed, response, error = _wait_future(
                future,
                max(0.0, deadline - self._now_monotonic()),
            )
            if not completed:
                _remove_pending(self._laser_reset_client, future)
                return False, "laser_odometry_reset_timeout"
            if error is not None or response is None:
                return (
                    False,
                    f"laser_odometry_reset_error:{type(error).__name__}",
                )
            if not response.success:
                return (
                    False,
                    str(response.message) or "laser_odometry_reset_rejected",
                )

        request = SetPose.Request()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._route.frame_id
        pose.pose.pose.orientation.w = 1.0
        diagonal = (0.25, 0.25, 1.0e6, 1.0e6, 1.0e6, 0.50)
        for index, value in enumerate(diagonal):
            pose.pose.covariance[index * 6 + index] = value
        request.pose = pose
        future = self._set_pose_client.call_async(request)
        completed, response, error = _wait_future(
            future,
            max(0.0, deadline - self._now_monotonic()),
        )
        if not completed:
            _remove_pending(self._set_pose_client, future)
            return False, "set_pose_timeout"
        if error is not None or response is None:
            return False, f"set_pose_error:{type(error).__name__}"

        with self._condition:
            verified_after_sequence = self._odom_sequence
            while self._now_monotonic() < deadline:
                if (
                    self._odom_sequence > verified_after_sequence
                    and odometry_matches_origin(
                        self._last_odom,
                        expected_frame=self._route.frame_id,
                        expected_child_frame=self._base_frame,
                        position_tolerance=self._origin_position_tolerance,
                        yaw_tolerance=self._origin_yaw_tolerance,
                    )
                ):
                    break
                self._condition.wait(
                    timeout=max(0.0, deadline - self._now_monotonic())
                )
            else:
                return False, "origin_verification_timeout"

        last_message = "clear_localization_fault_rejected"
        while self._now_monotonic() < deadline:
            future = self._clear_fault_client.call_async(Trigger.Request())
            completed, response, error = _wait_future(
                future,
                max(0.0, deadline - self._now_monotonic()),
            )
            if not completed:
                _remove_pending(self._clear_fault_client, future)
                return False, "clear_localization_fault_timeout"
            if error is not None or response is None:
                return (
                    False,
                    f"clear_localization_fault_error:{type(error).__name__}",
                )
            if response.success:
                return True, "prepared"
            last_message = str(response.message) or last_message
            time.sleep(min(0.10, max(0.0, deadline - self._now_monotonic())))
        return False, last_message

    def _on_prepare(self, _request, response):
        with self._condition:
            active = self._navigation_active_locked()
            self._prepared = False
            self._armed_deadline = None
            self._pending_terminal = None
            self._estop_retry_required = True
        self._set_state("locking_estop")
        latched, message = self._call_estop(True)
        if not latched:
            self._set_state("fault", message)
            response.success = False
            response.message = message
            return response

        if active:
            self._set_state("stopping")
            cancel_future = self._request_cancel()
            if cancel_future is not None:
                _wait_future(cancel_future, self._cancel_timeout_sec)
            deadline = self._now_monotonic() + self._cancel_timeout_sec
            with self._condition:
                while self._now_monotonic() < deadline:
                    navigation_done = not self._navigation_active_locked()
                    terminal_done = self._state not in (
                        "starting",
                        "running",
                        "stopping",
                        "latching_estop",
                    )
                    if navigation_done and terminal_done:
                        break
                    self._condition.wait(
                        timeout=max(0.0, deadline - self._now_monotonic())
                    )
                else:
                    response.success = False
                    response.message = "navigation_cancel_timeout"
                    self._set_state("locked", response.message)
                    return response
            # A terminal callback also latches, but prepare owns the final
            # latch immediately before resetting either odometry source.
            latched, message = self._call_estop(True)
            if not latched:
                self._set_state("fault", message)
                response.success = False
                response.message = message
                return response

        try:
            self._reload_route()
        except (OSError, ValueError) as error:
            message = f"route_reload_failed:{type(error).__name__}:{error}"
            self._set_state("locked", message)
            response.success = False
            response.message = message
            return response

        self._set_state("preparing")
        success, message = self._reset_origin()
        if not success:
            self._set_state("locked", message)
            response.success = False
            response.message = message
            return response
        with self._condition:
            self._prepared = True
            self._finished_elapsed_sec = 0.0
            self._started_at = None
            self._forced_failure_reason = ""
        self._set_state("prepared")
        response.success = True
        response.message = "origin reset and verified; emergency stop remains latched"
        return response

    def _sensor_readiness_error(self):
        now = self._now_monotonic()
        with self._condition:
            odom_at = self._last_odom_received_at
            scan_at = self._last_scan_received_at
            odom = self._last_odom
            laser_odom_at = self._last_laser_odom_received_at
            laser_odom = self._last_laser_odom
        if odom_at is None or now - odom_at > self._sensor_timeout_sec:
            return "odom_stale"
        if not odometry_matches_origin(
            odom,
            expected_frame=self._route.frame_id,
            expected_child_frame=self._base_frame,
            position_tolerance=self._origin_position_tolerance,
            yaw_tolerance=self._origin_yaw_tolerance,
        ):
            return "odom_not_at_origin"
        if scan_at is None or now - scan_at > self._sensor_timeout_sec:
            return "scan_stale"
        if self._use_laser_odometry:
            if (
                laser_odom_at is None
                or now - laser_odom_at > self._sensor_timeout_sec
            ):
                return "laser_odom_stale"
            if not odometry_matches_origin(
                laser_odom,
                expected_frame=self._route.frame_id,
                expected_child_frame=self._base_frame,
                position_tolerance=self._origin_position_tolerance,
                yaw_tolerance=self._origin_yaw_tolerance,
            ):
                return "laser_odom_invalid_or_not_at_origin"
        try:
            transform = self._tf_buffer.lookup_transform(
                self._route.frame_id,
                self._base_frame,
                Time(),
            )
        except TransformException:
            return "tf_unavailable"
        transform_values = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        if not _finite(transform_values):
            return "tf_invalid"
        rotation = transform.transform.rotation
        rotation_norm = math.sqrt(
            rotation.x * rotation.x
            + rotation.y * rotation.y
            + rotation.z * rotation.z
            + rotation.w * rotation.w
        )
        if abs(rotation_norm - 1.0) > 1.0e-3:
            return "tf_invalid"
        stamp = Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds <= 0:
            return "tf_timestamp_invalid"
        age_sec = (
            self.get_clock().now().nanoseconds - stamp.nanoseconds
        ) / 1.0e9
        if age_sec < -self._sensor_timeout_sec:
            return "tf_timestamp_in_future"
        if age_sec > self._sensor_timeout_sec:
            return "tf_stale"
        return ""

    def _on_arm(self, _request, response):
        with self._condition:
            if not self._prepared or self._state != "prepared":
                response.success = False
                response.message = "prepare must succeed before arm"
                return response
            missing_gates = [
                name for name, ready in self._motion_gates.items()
                if not ready
            ]
        if not self._route.calibrated:
            missing_gates.insert(0, "route_file_calibrated")
        if (
            self._use_laser_odometry
            and not self._laser_odometry_calibrated
        ):
            missing_gates.append("laser_odometry_calibrated")
        if missing_gates:
            response.success = False
            response.message = "motion gates not satisfied: " + ",".join(
                missing_gates
            )
            return response
        sensor_error = self._sensor_readiness_error()
        if sensor_error:
            response.success = False
            response.message = sensor_error
            return response
        if not self._action_client.wait_for_server(
            timeout_sec=self._action_server_timeout_sec
        ):
            response.success = False
            response.message = "navigate_through_poses_server_unavailable"
            return response
        cleared, message = self._call_estop(False)
        if not cleared:
            self._set_state("prepared", message)
            response.success = False
            response.message = message
            return response
        sensor_error = self._sensor_readiness_error()
        if sensor_error:
            latched, latch_message = self._call_estop(True)
            self._set_state(
                "prepared" if latched else "fault",
                sensor_error if latched else f"{sensor_error};{latch_message}",
            )
            response.success = False
            response.message = sensor_error
            return response
        with self._condition:
            self._armed_deadline = (
                self._now_monotonic() + self._arm_timeout_sec
            )
        self._set_state("armed")
        response.success = True
        response.message = (
            f"armed for {self._arm_timeout_sec:.1f}s; call start explicitly"
        )
        return response

    def _make_goal(self):
        goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for point in self._route.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self._route.frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = float(point.x)
            pose.pose.position.y = float(point.y)
            yaw = math.radians(float(point.yaw_deg))
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.append(pose)
        if self._behavior_tree:
            goal.behavior_tree = self._behavior_tree
        return goal

    def _on_start(self, _request, response):
        arm_expired = False
        with self._condition:
            if self._state != "armed" or self._armed_deadline is None:
                response.success = False
                response.message = "navigation is not armed"
                return response
            if self._now_monotonic() >= self._armed_deadline:
                self._armed_deadline = None
                self._prepared = True
                self._estop_retry_required = True
                arm_expired = True
            elif self._navigation_active_locked():
                response.success = False
                response.message = "navigation goal is already active"
                return response
            if not arm_expired:
                self._goal_generation += 1
                generation = self._goal_generation
                self._terminalized_generation = None
                self._cancel_requested = False
                self._forced_failure_reason = ""
                self._armed_deadline = None
                self._current_index = 0
                self._remaining_points = len(self._route.waypoints)
                self._distance_remaining_m = 0.0
        if arm_expired:
            latched, message = self._call_estop(True)
            self._set_state(
                "prepared" if latched else "fault",
                "arm_timeout" if latched else f"arm_timeout;{message}",
            )
            response.success = False
            response.message = "arm authorization expired"
            return response
        sensor_error = self._sensor_readiness_error()
        if sensor_error:
            self._finish_terminal(generation, "failed", sensor_error)
            response.success = False
            response.message = sensor_error
            return response
        self._set_state("starting")

        if not self._action_client.server_is_ready():
            message = "navigate_through_poses_server_unavailable"
            self._finish_terminal(generation, "failed", message)
            response.success = False
            response.message = message
            return response
        try:
            future = self._action_client.send_goal_async(
                self._make_goal(),
                feedback_callback=self._on_feedback,
            )
        except Exception as error:
            message = f"navigation_send_error:{type(error).__name__}"
            self._finish_terminal(generation, "failed", message)
            response.success = False
            response.message = message
            return response
        with self._condition:
            self._pending_goal_future = future
        completed, goal_handle, error = _wait_future(
            future,
            self._goal_response_timeout_sec,
        )
        if not completed:
            with self._condition:
                self._cancel_requested = True
            future.add_done_callback(
                lambda completed_future: self._on_late_goal_response(
                    generation,
                    completed_future,
                )
            )
            message = "navigation_goal_response_timeout_unconfirmed"
            self._finish_terminal(generation, "failed", message)
            response.success = False
            response.message = message
            return response
        with self._condition:
            self._pending_goal_future = None
            self._condition.notify_all()
        if error is not None or goal_handle is None:
            message = f"navigation_goal_error:{type(error).__name__}"
            self._finish_terminal(generation, "failed", message)
            response.success = False
            response.message = message
            return response
        if not goal_handle.accepted:
            message = "navigation_goal_rejected"
            self._finish_terminal(generation, "failed", message)
            response.success = False
            response.message = message
            return response

        try:
            result_future = goal_handle.get_result_async()
        except Exception as error:
            message = f"navigation_result_request_error:{type(error).__name__}"
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._finish_terminal(generation, "failed", message)
            response.success = False
            response.message = message
            return response
        with self._condition:
            self._goal_handle = goal_handle
            self._result_future = result_future
            self._started_at = self._now_monotonic()
            self._finished_elapsed_sec = 0.0
        self._set_state("running")
        result_future.add_done_callback(
            lambda completed_future: self._on_navigation_result(
                generation,
                completed_future,
            )
        )
        response.success = True
        response.message = (
            f"navigation started with {len(self._route.waypoints)} poses"
        )
        return response

    def _on_late_goal_response(self, generation, future):
        try:
            goal_handle = future.result()
        except Exception:
            goal_handle = None
        with self._condition:
            if generation != self._goal_generation:
                return
            self._pending_goal_future = None
            if goal_handle is not None and goal_handle.accepted:
                self._goal_handle = goal_handle
            self._condition.notify_all()
        if goal_handle is not None and goal_handle.accepted:
            try:
                result_future = goal_handle.get_result_async()
                with self._condition:
                    self._result_future = result_future
                result_future.add_done_callback(
                    lambda completed_future: self._on_late_goal_result(
                        generation,
                        completed_future,
                    )
                )
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _future: None)
            except Exception:
                pass

    def _on_late_goal_result(self, generation, _future):
        with self._condition:
            if generation != self._goal_generation:
                return
            self._goal_handle = None
            self._result_future = None
            self._condition.notify_all()

    def _on_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        with self._condition:
            if self._state not in ("running", "stopping"):
                return
            remaining = int(getattr(
                feedback,
                "number_of_poses_remaining",
                self._remaining_points,
            ))
            total = len(self._route.waypoints)
            self._remaining_points = min(max(0, remaining), total)
            self._current_index = min(
                max(0, total - self._remaining_points),
                max(0, total - 1),
            )
            distance = float(getattr(
                feedback,
                "distance_remaining",
                self._distance_remaining_m,
            ))
            if math.isfinite(distance) and distance >= 0.0:
                self._distance_remaining_m = distance
        self._publish_status()

    def _on_navigation_result(self, generation, future):
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
            result = wrapped_result.result
        except Exception as error:
            status = GoalStatus.STATUS_UNKNOWN
            result = None
            transport_error = f"navigation_result_error:{type(error).__name__}"
        else:
            transport_error = ""
        with self._condition:
            if generation != self._goal_generation:
                return
            self._goal_handle = None
            self._result_future = None
            forced_failure = self._forced_failure_reason
            cancel_requested = self._cancel_requested
            self._condition.notify_all()
        if forced_failure:
            terminal_state = "failed"
            reason = forced_failure
        elif transport_error:
            terminal_state = "failed"
            reason = transport_error
        elif cancel_requested:
            terminal_state = "canceled"
            reason = "navigation_canceled"
        elif status == GoalStatus.STATUS_SUCCEEDED:
            terminal_state = "succeeded"
            reason = ""
            with self._condition:
                self._remaining_points = 0
                self._current_index = max(0, len(self._route.waypoints) - 1)
                self._distance_remaining_m = 0.0
        elif status == GoalStatus.STATUS_CANCELED:
            terminal_state = "canceled"
            reason = "navigation_canceled"
        else:
            terminal_state = "failed"
            error_code = getattr(result, "error_code", status)
            error_msg = str(getattr(result, "error_msg", "")).strip()
            reason = f"navigation_status_{status}:error_code_{error_code}"
            if error_msg:
                reason += f":{error_msg}"
        self._finish_terminal(generation, terminal_state, reason)

    def _finish_terminal(self, generation, terminal_state, reason):
        with self._condition:
            if self._terminalized_generation == generation:
                return
            self._terminalized_generation = generation
            if self._started_at is not None:
                self._finished_elapsed_sec = self._elapsed_locked()
            self._started_at = None
            self._armed_deadline = None
            self._prepared = False
            self._pending_terminal = (str(terminal_state), str(reason))
            self._estop_retry_required = True
            self._state = "latching_estop"
            self._failure_reason = str(reason)
            self._condition.notify_all()
        self._publish_status(force=True)
        latched, latch_message = self._call_estop(True)
        with self._condition:
            if latched:
                self._state = str(terminal_state)
                self._failure_reason = str(reason)
                self._pending_terminal = None
            else:
                self._state = "fault"
                self._failure_reason = (
                    f"{reason};{latch_message}" if reason else latch_message
                )
                self._estop_retry_required = True
            self._condition.notify_all()
        self._publish_status(force=True)

    def _request_cancel(self):
        with self._condition:
            goal_handle = self._goal_handle
            self._cancel_requested = True
        if goal_handle is None:
            return None
        try:
            return goal_handle.cancel_goal_async()
        except Exception:
            return None

    def _on_stop(self, _request, response):
        latched, latch_message = self._call_estop(True)
        with self._condition:
            active = self._navigation_active_locked()
            self._prepared = False
            self._armed_deadline = None
            if active:
                self._state = "stopping"
                self._failure_reason = ""
            elif latched:
                self._state = "locked"
                self._failure_reason = ""
            else:
                self._state = "fault"
                self._failure_reason = latch_message
        self._publish_status(force=True)
        cancel_future = self._request_cancel() if active else None
        if cancel_future is not None:
            _wait_future(cancel_future, self._cancel_timeout_sec)
        deadline = self._now_monotonic() + self._cancel_timeout_sec
        with self._condition:
            while self._navigation_active_locked() and (
                self._now_monotonic() < deadline
            ):
                self._condition.wait(
                    timeout=max(0.0, deadline - self._now_monotonic())
                )
            terminal = not self._navigation_active_locked()
        self._publish_status(force=True)
        response.success = bool(latched and terminal)
        if not latched:
            response.message = latch_message
        elif terminal:
            response.message = "navigation stopped; emergency stop latched"
        else:
            response.message = "stop requested; terminal confirmation pending"
        return response

    def _on_housekeeping(self):
        now = self._now_monotonic()
        with self._condition:
            state = self._state
            armed_deadline = self._armed_deadline
            started_at = self._started_at
            retry_estop = self._estop_retry_required
            pending_terminal = self._pending_terminal

        if state == "armed" and armed_deadline is not None and now >= armed_deadline:
            with self._condition:
                self._armed_deadline = None
                self._prepared = True
                self._estop_retry_required = True
            latched, message = self._call_estop(True)
            self._set_state(
                "prepared" if latched else "fault",
                "arm_timeout" if latched else f"arm_timeout;{message}",
            )
            return

        if (
            state == "running"
            and started_at is not None
            and now - started_at >= self._navigation_timeout_sec
        ):
            with self._condition:
                self._forced_failure_reason = "navigation_timeout"
                self._state = "stopping"
                self._estop_retry_required = True
            self._call_estop(True)
            self._request_cancel()
            self._publish_status(force=True)
            return

        if retry_estop and state not in ("armed", "running"):
            latched, _message = self._call_estop(
                True,
                timeout_sec=min(0.50, self._service_timeout_sec),
            )
            if latched:
                with self._condition:
                    if pending_terminal is not None:
                        self._state, self._failure_reason = pending_terminal
                        self._pending_terminal = None
                    elif self._state in ("locking_estop", "fault"):
                        self._state = "locked"
                        self._failure_reason = ""
                    self._condition.notify_all()
                self._publish_status(force=True)
                return

        if now - self._last_status_publish_at >= 1.0:
            self._publish_status(force=True)

    def latch_for_shutdown(self, executor):
        """Best-effort final latch while the executor can still service replies."""
        if not self._estop_client.wait_for_service(timeout_sec=0.25):
            return False
        request = SetBool.Request()
        request.data = True
        future = self._estop_client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=0.75)
        try:
            return bool(future.done() and future.result().success)
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = NavigationRunner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.latch_for_shutdown(executor)
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown(timeout_sec=2.0)
        if rclpy.ok():
            rclpy.shutdown()
