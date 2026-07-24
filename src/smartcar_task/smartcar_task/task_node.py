"""ROS 2 adapters and services for semantic waypoint missions."""
import math
import shlex
import subprocess
import threading
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_localization.srv import SetPose
from smartcar_interfaces.srv import DescribeScene, ReadQr
from std_msgs.msg import String
from std_srvs.srv import Trigger

from smartcar_task.mission import Mission, MissionConfig, OperationResult
from smartcar_task.protocols import (
    classify_follow_waypoints_result,
    odometry_matches_origin,
    run_reset_sequence,
)
from smartcar_task.waypoints import load_waypoints


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
        self._speech_publisher = node.create_publisher(
            String, "/smartcar/output/speech", 10)

    def publish_state(self, state):
        self._state_publisher.publish(String(data=str(state)))

    def publish_text(self, value):
        self._text_publisher.publish(String(data=str(value)))

    def publish_speech(self, value):
        self._speech_publisher.publish(String(data=str(value)))


class RosNavigator:
    """Synchronous worker-thread facade over a FollowWaypoints ActionClient."""

    def __init__(
        self,
        node,
        callback_group,
        navigation_timeout_sec,
        goal_response_timeout_sec,
        cancel_timeout_sec,
    ):
        self._node = node
        self._client = ActionClient(
            node, FollowWaypoints, "/follow_waypoints",
            callback_group=callback_group,
        )
        self._navigation_timeout_sec = _positive_finite(
            "navigation_timeout_sec", navigation_timeout_sec)
        self._goal_response_timeout_sec = _positive_finite(
            "goal_response_timeout_sec", goal_response_timeout_sec)
        self._cancel_timeout_sec = _positive_finite(
            "cancel_timeout_sec", cancel_timeout_sec)
        self._condition = threading.Condition(threading.RLock())
        self._goal_generation = 0
        self._pending_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._goal_error = None
        self._poisoned = False
        self._cancel_error = None
        self._terminal_generation = None

    def wait_ready(self, timeout_sec):
        with self._condition:
            if self._poisoned:
                return False
        return self._client.wait_for_server(timeout_sec=float(timeout_sec))

    def navigate(self, waypoints, reverse_direction=False):
        # reverse_direction is computed by mission.py from the waypoint
        # direction field.  It is currently unused (the planner selects
        # reverse motion via REEDS_SHEPP cost optimisation and the
        # controller obeys via allow_reversing).  Reserved for future
        # per-segment planner parameter switching.
        waypoints = tuple(waypoints)
        if not waypoints:
            return OperationResult(False, "navigation_segment_empty")

        goal = FollowWaypoints.Goal()
        stamp = self._node.get_clock().now().to_msg()
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = waypoint.frame_id
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ) = waypoint.position
            qx, qy, qz, qw = waypoint.orientation
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = (qx, qy, qz, qw)
            goal.poses.append(pose)

        with self._condition:
            if self._active_locked():
                status = (
                    "navigation_goal_unconfirmed"
                    if self._poisoned
                    else "navigation_goal_already_active"
                )
                return OperationResult(False, status)
            self._goal_generation += 1
            generation = self._goal_generation
            self._cancel_requested = False
            self._goal_error = None
            self._cancel_error = None
            self._terminal_generation = None
            try:
                future = self._client.send_goal_async(goal)
            except Exception as error:
                return OperationResult(
                    False,
                    f"navigation_send_error:{type(error).__name__}",
                )
            self._pending_goal_future = future
            future.add_done_callback(
                lambda completed: self._on_goal_response(
                    generation, completed))

        response_deadline = (
            time.monotonic() + self._goal_response_timeout_sec)
        with self._condition:
            while (
                generation == self._goal_generation
                and self._pending_goal_future is not None
                and time.monotonic() < response_deadline
            ):
                self._condition.wait(
                    timeout=max(0.0, response_deadline - time.monotonic()))
            if self._pending_goal_future is not None:
                self._cancel_requested = True
                self._poisoned = True
                return OperationResult(
                    False, "navigation_goal_response_timeout_unconfirmed")
            if self._goal_error is not None:
                return OperationResult(False, self._goal_error)
            goal_handle = self._goal_handle
            result_future = self._result_future

        if goal_handle is None or not goal_handle.accepted:
            return OperationResult(False, "navigation_goal_rejected")
        if result_future is None:
            return OperationResult(False, "navigation_result_unavailable")

        completed, response, error = _wait_future(
            result_future, self._navigation_timeout_sec)
        if not completed:
            terminal = self.cancel()
            status = (
                "navigation_timeout"
                if terminal
                else "navigation_timeout_cancel_unconfirmed"
            )
            return OperationResult(False, status)
        if error is not None or response is None:
            return OperationResult(
                False,
                f"navigation_result_error:{type(error).__name__}",
            )
        return classify_follow_waypoints_result(
            response.status,
            response.result.missed_waypoints,
        )

    def cancel(self):
        deadline = time.monotonic() + self._cancel_timeout_sec
        with self._condition:
            self._cancel_requested = True
            while (
                self._pending_goal_future is not None
                and self._goal_handle is None
                and time.monotonic() < deadline
            ):
                self._condition.wait(
                    timeout=max(0.0, deadline - time.monotonic()))

            if self._goal_handle is not None:
                self._request_cancel_locked()
            while self._active_locked() and time.monotonic() < deadline:
                self._condition.wait(
                    timeout=max(0.0, deadline - time.monotonic()))
            return not self._active_locked()

    def is_active(self):
        with self._condition:
            return self._active_locked()

    def _active_locked(self):
        return (
            self._poisoned
            or
            self._pending_goal_future is not None
            or self._goal_handle is not None
            or self._result_future is not None
        )

    def _on_goal_response(self, generation, future):
        with self._condition:
            if generation != self._goal_generation:
                return
            self._pending_goal_future = None
            try:
                goal_handle = future.result()
            except Exception as error:
                self._poisoned = True
                self._goal_error = (
                    f"navigation_goal_response_error:{type(error).__name__}")
                self._condition.notify_all()
                return
            if goal_handle is None or not goal_handle.accepted:
                # A definitive rejection proves that no server-side goal exists.
                self._poisoned = False
                self._goal_error = "navigation_goal_rejected"
                self._condition.notify_all()
                return
            self._goal_handle = goal_handle
            try:
                result_future = goal_handle.get_result_async()
                self._result_future = result_future
                result_future.add_done_callback(
                    lambda completed: self._on_goal_result(
                        generation, completed))
            except Exception as error:
                self._poisoned = True
                self._goal_error = (
                    f"navigation_result_request_error:{type(error).__name__}")
                self._condition.notify_all()
                return
            if self._cancel_requested:
                self._request_cancel_locked(generation)
            self._condition.notify_all()

    def _request_cancel_locked(self, generation=None):
        if self._goal_handle is None or self._cancel_future is not None:
            return
        generation = (
            self._goal_generation if generation is None else generation)
        try:
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(
                lambda future: self._on_cancel_response(generation, future))
        except Exception as error:
            self._poisoned = True
            self._goal_error = (
                f"navigation_cancel_error:{type(error).__name__}")

    def _on_cancel_response(self, generation, future):
        with self._condition:
            if (
                generation != self._goal_generation
                or self._terminal_generation == generation
            ):
                return
            try:
                response = future.result()
                if response.return_code != 0 or not response.goals_canceling:
                    self._cancel_error = (
                        f"navigation_cancel_rejected:{response.return_code}")
                    self._poisoned = True
            except Exception as error:
                self._cancel_error = (
                    f"navigation_cancel_error:{type(error).__name__}")
                self._poisoned = True
            self._condition.notify_all()

    def _on_goal_result(self, generation, future):
        with self._condition:
            if generation != self._goal_generation:
                return
            try:
                future.result()
            except Exception as error:
                # A transport exception is not proof that Nav2 stopped.
                self._poisoned = True
                self._goal_error = (
                    f"navigation_result_error:{type(error).__name__}")
                self._condition.notify_all()
                return
            # A completed action result is a terminal proof, even when its
            # status is ABORTED or CANCELED; the mission may now retry safely.
            self._terminal_generation = generation
            self._poisoned = False
            self._pending_goal_future = None
            self._goal_handle = None
            self._result_future = None
            self._cancel_future = None
            self._condition.notify_all()


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
        reader_startup = node.get_parameter("qr_reader_startup_sec").value
        self._reader_startup_sec = float(reader_startup)

    def _ensure_reader(self):
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
        cmd_str = str(self._node.get_parameter("barcode_reader_cmd").value)
        self._node.get_logger().info("Starting barcode_reader on demand")
        cmd = shlex.split(cmd_str)
        self._reader_process = subprocess.Popen(cmd)
        time.sleep(self._reader_startup_sec)

    def _stop_reader(self):
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

    def describe_scene(self, not_before_ns, timeout_sec, prompt):
        request = DescribeScene.Request()
        request.not_before = _time_from_nanoseconds(not_before_ns)
        request.timeout_sec = float(timeout_sec)
        request.prompt = str(prompt)
        future = self._describe_client.call_async(request)
        completed, response, error = _wait_future(
            future, max(0.0, float(timeout_sec)))
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
            bool(response.fallback_used),
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
        node.create_subscription(
            Odometry,
            "/odom_combined",
            self._on_odom,
            10,
            callback_group=callback_group,
        )

    def reset_origin(self):
        self._deadline = time.monotonic() + self._reset_timeout_sec
        if not self._wait_for_reset_services():
            return OperationResult(False, "reset_service_unavailable")
        return run_reset_sequence(
            lambda: not self._navigator.is_active(),
            self._call_set_pose,
            self._wait_for_verified_origin,
        )

    def _wait_for_reset_services(self):
        remaining = self._deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        return self._set_pose_client.wait_for_service(
            timeout_sec=remaining)

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


class TaskNode(Node):
    def __init__(self):
        super().__init__("task_node")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("waypoints_calibrated", False)
        self.declare_parameter("extrinsics_calibrated", False)
        self.declare_parameter("steering_calibrated", False)
        self.declare_parameter("emergency_stop_ready", False)
        self.declare_parameter("operator_approved", False)
        self.declare_parameter("use_laser_odometry", False)
        self.declare_parameter("laser_odometry_calibrated", False)
        self.declare_parameter("autostart_mission", False)
        self.declare_parameter("server_wait_timeout_sec", 5.0)
        self.declare_parameter("navigation_timeout_sec", 120.0)
        self.declare_parameter("goal_response_timeout_sec", 5.0)
        self.declare_parameter("cancel_timeout_sec", 3.0)
        self.declare_parameter("stop_timeout_sec", 5.0)
        self.declare_parameter("navigation_retries", 1)
        self.declare_parameter("navigation_retry_delay_sec", 0.25)
        self.declare_parameter("qr_settle_sec", 2.0)
        self.declare_parameter("qr_timeout_sec", 3.0)
        self.declare_parameter("qr_retries", 1)
        self.declare_parameter("qr_retry_delay_sec", 0.25)
        self.declare_parameter("vlm_timeout_sec", 8.0)
        self.declare_parameter(
            "vlm_prompt", "请描述图中人物立牌的外观和动作。")
        self.declare_parameter("reset_timeout_sec", 5.0)
        self.declare_parameter("origin_position_tolerance", 0.20)
        self.declare_parameter("origin_yaw_tolerance", 0.20)
        self.declare_parameter("qr_reader_startup_sec", 2.0)
        self.declare_parameter(
            "barcode_reader_cmd",
            "ros2 run zbar_ros barcode_reader --ros-args "
            "-r image:=/aurora/rgb/image_raw "
            "-r barcode:=/barcode "
            "-p throttle_repeated_barcodes:=0.0",
        )

        waypoints_file = str(
            self.get_parameter("waypoints_file").value).strip()
        if not waypoints_file:
            raise ValueError("waypoints_file must be provided")
        self._waypoints = load_waypoints(waypoints_file)
        self._motion_gates = {
            name: bool(self.get_parameter(name).value)
            for name in (
                "waypoints_calibrated",
                "extrinsics_calibrated",
                "steering_calibrated",
                "emergency_stop_ready",
                "operator_approved",
            )
        }
        if bool(self.get_parameter("use_laser_odometry").value):
            self._motion_gates["laser_odometry_calibrated"] = bool(
                self.get_parameter("laser_odometry_calibrated").value)
        self._stop_timeout_sec = _positive_finite(
            "stop_timeout_sec",
            self.get_parameter("stop_timeout_sec").value,
        )

        self._io_group = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._output = RosOutput(self)
        self._navigator = RosNavigator(
            self,
            self._io_group,
            self.get_parameter("navigation_timeout_sec").value,
            self.get_parameter("goal_response_timeout_sec").value,
            self.get_parameter("cancel_timeout_sec").value,
        )
        self._vision = RosVision(self, self._io_group)
        self._localization = RosLocalization(
            self,
            self._navigator,
            self._io_group,
            self.get_parameter("reset_timeout_sec").value,
            self.get_parameter("origin_position_tolerance").value,
            self.get_parameter("origin_yaw_tolerance").value,
        )
        config = MissionConfig(
            server_wait_timeout_sec=self.get_parameter(
                "server_wait_timeout_sec").value,
            navigation_retries=self.get_parameter(
                "navigation_retries").value,
            navigation_retry_delay_sec=self.get_parameter(
                "navigation_retry_delay_sec").value,
            qr_settle_sec=self.get_parameter("qr_settle_sec").value,
            qr_timeout_sec=self.get_parameter("qr_timeout_sec").value,
            qr_retries=self.get_parameter("qr_retries").value,
            qr_retry_delay_sec=self.get_parameter(
                "qr_retry_delay_sec").value,
            vlm_timeout_sec=self.get_parameter("vlm_timeout_sec").value,
            vlm_prompt=self.get_parameter("vlm_prompt").value,
        )
        self._mission = Mission(
            navigator=self._navigator,
            vision=self._vision,
            localization=self._localization,
            clock=SystemClock(self),
            output=self._output,
            config=config,
        )
        self._worker_lock = threading.RLock()
        self._worker = None

        self.create_service(
            Trigger,
            "/smartcar/task/start",
            self._on_start,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            "/smartcar/task/stop",
            self._on_stop,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            "/smartcar/task/reset",
            self._on_reset,
            callback_group=self._service_group,
        )

        self._autostart_timer = None
        if bool(self.get_parameter("autostart_mission").value):
            self._autostart_timer = self.create_timer(
                0.5,
                self._on_autostart,
                callback_group=self._service_group,
            )
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} semantic waypoints")

    def _start_worker(self):
        with self._worker_lock:
            missing_gates = [
                name for name, ready in self._motion_gates.items()
                if not ready
            ]
            if missing_gates:
                return (
                    False,
                    "motion gates not satisfied: " + ",".join(missing_gates),
                )
            if self._worker is not None and self._worker.is_alive():
                return False, "mission worker already running"
            generation = self._mission.reserve_start()
            if generation is None:
                return False, f"mission state is {self._mission.state.value}"
            self._worker = threading.Thread(
                target=self._run_mission,
                args=(generation,),
                name="smartcar-mission",
                daemon=True,
            )
            self._worker.start()
        return True, "mission started"

    def _run_mission(self, generation):
        result = self._mission.run_reserved(generation, self._waypoints)
        if result.success:
            self.get_logger().info(result.status)
        else:
            self.get_logger().warning(result.status)

    def _on_start(self, _request, response):
        response.success, response.message = self._start_worker()
        return response

    def _on_stop(self, _request, response):
        accepted = self._mission.request_stop()
        if not accepted:
            response.success = False
            response.message = "no mission is running"
            return response
        with self._worker_lock:
            worker = self._worker
        if worker is not None:
            worker.join(timeout=self._stop_timeout_sec)
        terminal = (
            (worker is None or not worker.is_alive())
            and not self._navigator.is_active()
        )
        response.success = terminal
        response.message = (
            "mission stopped"
            if terminal
            else "stop requested; terminal confirmation pending"
        )
        return response

    def _on_reset(self, _request, response):
        with self._worker_lock:
            worker = self._worker
        if worker is not None and worker.is_alive():
            response.success = False
            response.message = "mission worker has not stopped"
            return response
        result = self._mission.reset()
        response.success = result.success
        response.message = result.status
        return response

    def _on_autostart(self):
        if self._autostart_timer is not None:
            self.destroy_timer(self._autostart_timer)
            self._autostart_timer = None
        success, message = self._start_worker()
        if not success:
            self.get_logger().error(f"Mission autostart failed: {message}")

    def stop_for_shutdown(self):
        accepted = self._mission.request_stop()
        if not accepted and self._navigator.is_active():
            self._navigator.cancel()
        with self._worker_lock:
            worker = self._worker
        if worker is not None:
            worker.join(timeout=self._stop_timeout_sec)
        self._vision.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_for_shutdown()
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
