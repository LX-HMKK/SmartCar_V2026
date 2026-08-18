"""ROS-facing adapters for semantic waypoint missions."""

import math
import subprocess
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_localization.srv import SetPose
from smartcar_interfaces.srv import (
    ActivateMotion,
    DescribeScene,
    PrepareMotion,
    ReadQr,
    RenewMotion,
    StopMotion,
)
from std_msgs.msg import String

from smartcar_task.mission import OperationResult
from smartcar_task.navigation_runtime import RosNavigator
from smartcar_task.protocols import (
    odometry_matches_origin,
    run_reset_sequence,
    twist_is_stopped,
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
    except Exception as error:  # rclpy futures propagate transport errors here.
        return True, None, error


def _remove_pending(client, future):
    try:
        client.remove_pending_request(future)
    except (AttributeError, KeyError, RuntimeError):
        pass


def _time_from_nanoseconds(nanoseconds):
    value = int(nanoseconds)
    if value < 0:
        raise ValueError("ROS timestamp must be nonnegative")
    from builtin_interfaces.msg import Time

    return Time(
        sec=value // 1_000_000_000,
        nanosec=value % 1_000_000_000,
    )


class SystemClock:
    def __init__(self, node):
        self._node = node

    def now_ns(self):
        return self._node.get_clock().now().nanoseconds

    @staticmethod
    def sleep(seconds):
        time.sleep(max(0.0, float(seconds)))


class RosOutput:
    def __init__(self, node):
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = node.create_publisher(
            String, "/smartcar/task/state", state_qos)
        self._text_publisher = node.create_publisher(
            String, "/smartcar/output/text", 10)
        self._qr_publisher = node.create_publisher(
            String, "/smartcar/output/qr", 10)
        self._vlm_publisher = node.create_publisher(
            String, "/smartcar/output/vlm", 10)
        self._c_zone_direction_publisher = node.create_publisher(
            String, "/smartcar/output/c_zone_direction", 10)

    def publish_state(self, state):
        self._state_publisher.publish(String(data=str(state)))

    def publish_text(self, value):
        self._text_publisher.publish(String(data=str(value)))

    def publish_qr(self, value):
        self._qr_publisher.publish(String(data=str(value)))

    def publish_vlm(self, value):
        self._vlm_publisher.publish(String(data=str(value)))

    def publish_c_zone_direction(self, value):
        self._c_zone_direction_publisher.publish(String(data=str(value)))


class RosDirectionGuard:
    """ROS transport plus a scoped local odometry stop confirmation."""

    def __init__(
        self,
        node,
        callback_group,
        service_timeout_sec,
        stop_timeout_sec,
        stop_dwell_sec,
        linear_tolerance,
        angular_tolerance,
        odom_stale_timeout_sec,
    ):
        self._node = node
        self._callback_group = callback_group
        self._service_timeout_sec = _positive_finite(
            "direction_service_timeout_sec", service_timeout_sec)
        self._stop_timeout_sec = _positive_finite(
            "direction_stop_timeout_sec", stop_timeout_sec)
        self._stop_dwell_sec = _positive_finite(
            "direction_stop_dwell_sec", stop_dwell_sec)
        self._linear_tolerance = _nonnegative_finite(
            "direction_stop_linear_tolerance", linear_tolerance)
        self._angular_tolerance = _nonnegative_finite(
            "direction_stop_angular_tolerance", angular_tolerance)
        self._odom_stale_timeout_sec = _positive_finite(
            "direction_odom_stale_timeout_sec", odom_stale_timeout_sec)
        self._prepare_client = node.create_client(
            PrepareMotion,
            "/smartcar/direction_guard/prepare",
            callback_group=callback_group,
        )
        self._activate_client = node.create_client(
            ActivateMotion,
            "/smartcar/direction_guard/activate",
            callback_group=callback_group,
        )
        self._renew_client = node.create_client(
            RenewMotion,
            "/smartcar/direction_guard/renew",
            callback_group=callback_group,
        )
        self._stop_client = node.create_client(
            StopMotion,
            "/smartcar/direction_guard/stop",
            callback_group=callback_group,
        )
        self._clients = (
            self._prepare_client,
            self._activate_client,
            self._renew_client,
            self._stop_client,
        )
        self._odom_condition = threading.Condition(threading.RLock())
        self._odom_sequence = 0
        self._barrier_sequence = 0
        self._zero_since = None
        self._last_zero = None
        self._odom_subscription = None

    def wait_ready(self, timeout_sec):
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        poll_interval = 0.5
        for client in self._clients:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                if client.wait_for_service(timeout_sec=min(poll_interval, remaining)):
                    break
        return True

    def _on_odom(self, message):
        twist = message.twist.twist
        now = time.monotonic()
        zero = twist_is_stopped(
            twist,
            self._linear_tolerance,
            self._angular_tolerance,
        )
        with self._odom_condition:
            self._odom_sequence += 1
            if self._odom_sequence <= self._barrier_sequence or not zero:
                self._zero_since = None
                self._last_zero = None
            else:
                if self._zero_since is None:
                    self._zero_since = now
                self._last_zero = now
            self._odom_condition.notify_all()

    def _start_stop_barrier(self):
        with self._odom_condition:
            self._barrier_sequence = self._odom_sequence
            self._zero_since = None
            self._last_zero = None

    def _start_odom_observation(self):
        with self._odom_condition:
            if self._odom_subscription is not None:
                return
            # Keep one subscription for the node lifetime; queued executor
            # callbacks make per-wait destruction unsafe.
            self._odom_subscription = self._node.create_subscription(
                Odometry,
                "/odom",
                self._on_odom,
                10,
                callback_group=self._callback_group,
            )

    def wait_stopped(self):
        self._start_odom_observation()
        deadline = time.monotonic() + self._stop_timeout_sec
        self._start_stop_barrier()
        with self._odom_condition:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if (
                    self._zero_since is not None
                    and self._last_zero is not None
                    and self._last_zero - self._zero_since
                    >= self._stop_dwell_sec
                    and now - self._last_zero
                    <= self._odom_stale_timeout_sec
                ):
                    return OperationResult(True, "stopped")
                self._odom_condition.wait(
                    timeout=max(0.0, deadline - now))
        return OperationResult(False, "odom_stop_timeout")

    @staticmethod
    def _set_identity(request, lease):
        request.boot_epoch = int(lease.boot_epoch)
        request.lease_id = int(lease.lease_id)
        request.generation = int(lease.generation)
        request.action_uuid = lease.action_uuid

    def _call(self, client, request, operation):
        try:
            future = client.call_async(request)
        except Exception as error:
            return OperationResult(
                False, f"{operation}_error:{type(error).__name__}"), None
        completed, response, error = _wait_future(
            future, self._service_timeout_sec)
        if not completed:
            _remove_pending(client, future)
            return OperationResult(False, f"{operation}_timeout"), None
        if error is not None or response is None:
            return OperationResult(
                False, f"{operation}_error:{type(error).__name__}"), None
        status = str(response.status).strip() or operation
        return OperationResult(bool(response.success), status), response

    def prepare(self, lease):
        request = PrepareMotion.Request()
        request.direction = int(lease.direction)
        request.generation = int(lease.generation)
        request.action_uuid = lease.action_uuid
        result, response = self._call(
            self._prepare_client, request, "prepare")
        if response is None:
            return result, 0, 0
        return result, response.boot_epoch, response.lease_id

    def activate(self, lease):
        request = ActivateMotion.Request()
        self._set_identity(request, lease)
        return self._call(
            self._activate_client, request, "activate")[0]

    def renew(self, lease):
        request = RenewMotion.Request()
        self._set_identity(request, lease)
        return self._call(self._renew_client, request, "renew")[0]

    def stop(self, lease):
        request = StopMotion.Request()
        self._set_identity(request, lease)
        return self._call(self._stop_client, request, "stop")[0]


class RosVision:
    def __init__(self, node, callback_group):
        self._node = node
        self._read_qr_client = node.create_client(
            ReadQr,
            "/smartcar/vision/read_qr",
            callback_group=callback_group,
        )
        self._describe_client = node.create_client(
            DescribeScene,
            "/smartcar/vision/describe_scene",
            callback_group=callback_group,
        )
        self._reader_process = None
        self._reader_preloaded = bool(node.get_parameter(
            "qr_reader_preloaded").value)
        reader_startup = node.get_parameter("qr_reader_startup_sec").value
        self._reader_startup_sec = float(reader_startup)

    def _ensure_reader(self):
        if self._reader_preloaded:
            return
        if self._reader_process is not None and self._reader_process.poll() is None:
            return
        # If the old process exited but was never waited on, reap it first
        # to avoid zombie processes (Python 3.9+ does not auto-reap).
        if self._reader_process is not None:
            try:
                self._reader_process.wait(timeout=0.0)
            except subprocess.TimeoutExpired:
                pass  # should not happen — poll() already confirmed exit
            except Exception:
                pass
        image_topic = str(self._node.get_parameter(
            "barcode_reader_image_topic").value).strip()
        if not image_topic:
            raise ValueError("barcode_reader_image_topic must be nonempty")
        self._node.get_logger().info("Starting barcode_reader on demand")
        cmd = [
            "ros2", "run", "zbar_ros", "barcode_reader", "--ros-args",
            "-r", f"image:={image_topic}",
            "-r", "barcode:=/barcode",
            "-p", "throttle_repeated_barcodes:=0.0",
        ]
        self._reader_process = subprocess.Popen(cmd)
        time.sleep(self._reader_startup_sec)

    def _stop_reader(self):
        if self._reader_preloaded:
            return
        if self._reader_process is None:
            return
        self._node.get_logger().info("Stopping barcode_reader")
        try:
            self._reader_process.terminate()
            self._reader_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                self._reader_process.kill()
                # Blocking wait — SIGKILL is near-instant on Linux.
                self._reader_process.wait()
            except subprocess.TimeoutExpired:
                self._node.get_logger().error(
                    "barcode_reader did not respond to SIGKILL")
            except ProcessLookupError:
                pass
        except ProcessLookupError:
            pass
        finally:
            self._reader_process = None

    def shutdown(self):
        self._stop_reader()

    def wait_ready(self, require_qr, require_vlm, timeout_sec):
        deadline = time.monotonic() + float(timeout_sec)
        if require_qr and not self._read_qr_client.wait_for_service(
            timeout_sec=max(0.0, deadline - time.monotonic())
        ):
            return False
        if require_vlm and not self._describe_client.wait_for_service(
            timeout_sec=max(0.0, deadline - time.monotonic())
        ):
            return False
        return True

    def read_qr(self, not_before_ns, timeout_sec):
        self._ensure_reader()
        try:
            request = ReadQr.Request()
            request.not_before = _time_from_nanoseconds(not_before_ns)
            request.timeout_sec = float(timeout_sec)
            future = self._read_qr_client.call_async(request)
            completed, response, error = _wait_future(
                future, max(0.0, float(timeout_sec)))
            if not completed:
                _remove_pending(self._read_qr_client, future)
                return OperationResult(False, "read_qr_transport_timeout")
            if error is not None or response is None:
                return OperationResult(
                    False,
                    f"read_qr_transport_error:{type(error).__name__}",
                )
            return OperationResult(
                bool(response.success),
                str(response.status),
                str(response.content),
            )
        finally:
            self._stop_reader()

    def describe_scene(self, not_before_ns, timeout_sec):
        request = DescribeScene.Request()
        request.not_before = _time_from_nanoseconds(not_before_ns)
        request.timeout_sec = float(timeout_sec)
        future = self._describe_client.call_async(request)
        # The service owns the full VLM timeout budget; leave transport time
        # for ROS response delivery so a completed request is not misreported.
        completed, response, error = _wait_future(
            future, max(0.0, float(timeout_sec)) + 1.0)
        if not completed:
            _remove_pending(self._describe_client, future)
            return OperationResult(False, "describe_scene_transport_timeout")
        if error is not None or response is None:
            return OperationResult(
                False,
                f"describe_scene_transport_error:{type(error).__name__}",
            )
        return OperationResult(
            bool(response.success),
            str(response.status),
            str(response.description),
        )


class RosLocalization:
    def __init__(
        self,
        node,
        navigator,
        callback_group,
        reset_timeout_sec,
        position_tolerance,
        yaw_tolerance,
    ):
        self._node = node
        self._callback_group = callback_group
        self._navigator = navigator
        self._reset_timeout_sec = _positive_finite(
            "reset_timeout_sec", reset_timeout_sec)
        self._position_tolerance = _nonnegative_finite(
            "origin_position_tolerance", position_tolerance)
        self._yaw_tolerance = _nonnegative_finite(
            "origin_yaw_tolerance", yaw_tolerance)
        self._set_pose_client = node.create_client(
            SetPose,
            "/set_pose",
            callback_group=callback_group,
        )
        self._condition = threading.Condition()
        self._odom_sequence = 0
        self._latest_odom = None
        self._verified_after_sequence = 0
        self._deadline = 0.0
        self._odom_subscription = None

    def reset_origin(self):
        self._start_odometry_observation()
        self._deadline = time.monotonic() + self._reset_timeout_sec
        remaining = self._deadline - time.monotonic()
        if (
            remaining <= 0.0
            or not self._set_pose_client.wait_for_service(
                timeout_sec=remaining
            )
        ):
            return OperationResult(False, "reset_service_unavailable")
        return run_reset_sequence(
            lambda: not self._navigator.is_active(),
            self._call_set_pose,
            self._wait_for_verified_origin,
        )

    def _start_odometry_observation(self):
        with self._condition:
            if self._odom_subscription is None:
                # Keep one subscription for the node lifetime; reset sequence
                # numbers define each verification boundary.
                self._odom_subscription = self._node.create_subscription(
                    Odometry,
                    "/odom_combined",
                    self._on_odom,
                    10,
                    callback_group=self._callback_group,
                )

    def _call_set_pose(self):
        request = SetPose.Request()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = "odom_combined"
        pose.pose.pose.orientation.w = 1.0
        diagonal = (0.25, 0.25, 1e6, 1e6, 1e6, 0.50)
        for index, value in enumerate(diagonal):
            pose.pose.covariance[index * 6 + index] = value
        request.pose = pose
        future = self._set_pose_client.call_async(request)
        remaining = self._deadline - time.monotonic()
        completed, response, error = _wait_future(future, remaining)
        if not completed:
            _remove_pending(self._set_pose_client, future)
            return OperationResult(False, "set_pose_timeout")
        if error is not None or response is None:
            return OperationResult(
                False,
                f"set_pose_error:{type(error).__name__}",
            )
        with self._condition:
            self._verified_after_sequence = self._odom_sequence
        return OperationResult(True, "ok")

    def _wait_for_verified_origin(self):
        with self._condition:
            while time.monotonic() < self._deadline:
                if (
                    self._odom_sequence > self._verified_after_sequence
                    and self._latest_odom is not None
                    and odometry_matches_origin(
                        self._latest_odom,
                        position_tolerance=self._position_tolerance,
                        yaw_tolerance=self._yaw_tolerance,
                    )
                ):
                    return OperationResult(True, "ok")
                self._condition.wait(
                    timeout=max(0.0, self._deadline - time.monotonic()))
        return OperationResult(False, "origin_verification_timeout")

    def _on_odom(self, message):
        with self._condition:
            self._odom_sequence += 1
            self._latest_odom = message
            self._condition.notify_all()
