"""ROS 2 adapters and services for semantic waypoint missions."""
import math
import subprocess
import threading
import time
from uuid import uuid4

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
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
from smartcar_interfaces.srv import (
    ActivateMotion,
    DescribeScene,
    PrepareMotion,
    ReadQr,
    RenewMotion,
    StopMotion,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID

from smartcar_task.mission import (
    Mission,
    MissionConfig,
    OperationResult,
)
from smartcar_task.planning_segments import (
    PlanningSegmentError,
    allows_reverse_handoff_through_poses,
    load_planning_segments,
    materialize_mission_route,
    materialize_navigation_segments,
)
from smartcar_task.protocols import (
    MotionDirectionProtocol,
    classify_navigate_to_pose_result,
    motion_direction,
    navigation_behavior_tree,
    odometry_matches_origin,
    run_reset_sequence,
    twist_is_stopped,
)
from smartcar_task.route_geometry import RouteGeometryError, materialize_free_yaws
from smartcar_task.waypoints import is_heading_locked, load_waypoint_document


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


class RosDirectionGuard:
    """ROS transport plus an independent odometry stop barrier."""

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
        self._odom_subscription = node.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            10,
            callback_group=callback_group,
        )

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

    def wait_stopped(self):
        deadline = time.monotonic() + self._stop_timeout_sec
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
        self._start_stop_barrier()
        request = StopMotion.Request()
        self._set_identity(request, lease)
        return self._call(self._stop_client, request, "stop")[0]


class RosNavigator:
    """One guarded Nav2 action at a time, with terminal proof."""

    def __init__(
        self,
        node,
        callback_group,
        direction_guard,
        reverse_behavior_tree,
        reverse_handoff_behavior_tree,
        precise_forward_behavior_tree,
        navigation_timeout_sec,
        goal_response_timeout_sec,
        cancel_timeout_sec,
        direction_renew_period_sec,
        direction_prepare_timeout_sec,
        direction_prepare_retry_period_sec,
        through_poses_behavior_tree="",
        reverse_through_poses_behavior_tree="",
        reverse_locked_through_poses_behavior_tree="",
        reverse_return_through_poses_behavior_tree="",
    ):
        self._node = node
        self._client = ActionClient(
            node,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=callback_group,
        )
        self._through_client = ActionClient(
            node,
            NavigateThroughPoses,
            "/navigate_through_poses",
            callback_group=callback_group,
        )
        self._direction_guard = direction_guard
        self._motion_protocol = MotionDirectionProtocol(
            direction_guard,
            direction_guard.wait_stopped,
            prepare_timeout_sec=direction_prepare_timeout_sec,
            prepare_retry_period_sec=direction_prepare_retry_period_sec,
        )
        self._reverse_behavior_tree = str(reverse_behavior_tree).strip()
        navigation_behavior_tree(True, self._reverse_behavior_tree)
        self._reverse_handoff_behavior_tree = str(
            reverse_handoff_behavior_tree).strip()
        navigation_behavior_tree(
            True,
            self._reverse_behavior_tree,
            goal_profile="reverse_handoff",
            reverse_handoff_behavior_tree=(
                self._reverse_handoff_behavior_tree),
        )
        self._precise_forward_behavior_tree = str(
            precise_forward_behavior_tree).strip()
        navigation_behavior_tree(
            False,
            self._reverse_behavior_tree,
            goal_profile="precise",
            precise_forward_behavior_tree=self._precise_forward_behavior_tree,
        )
        self._through_poses_behavior_tree = str(
            through_poses_behavior_tree).strip()
        self._reverse_through_poses_behavior_tree = str(
            reverse_through_poses_behavior_tree).strip()
        self._reverse_locked_through_poses_behavior_tree = str(
            reverse_locked_through_poses_behavior_tree).strip()
        self._reverse_return_through_poses_behavior_tree = str(
            reverse_return_through_poses_behavior_tree).strip()
        self._navigation_timeout_sec = _positive_finite(
            "navigation_timeout_sec", navigation_timeout_sec)
        self._goal_response_timeout_sec = _positive_finite(
            "goal_response_timeout_sec", goal_response_timeout_sec)
        self._cancel_timeout_sec = _positive_finite(
            "cancel_timeout_sec", cancel_timeout_sec)
        self._renew_period_sec = _positive_finite(
            "direction_renew_period_sec", direction_renew_period_sec)
        self._condition = threading.Condition(threading.RLock())
        self._guard_call_lock = threading.Lock()
        self._goal_generation = 0
        self._operation_active = False
        self._navigate_attached = False
        self._current_identity = None
        self._guard_stop_attempted_generation = None
        self._guard_revoked_generation = None
        self._guard_stopped_generation = None
        self._pending_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._goal_error = None
        self._cancel_error = None
        self._terminal_generation = None
        self._terminal_response = None
        self._poisoned = False

    def wait_ready(self, timeout_sec):
        with self._condition:
            if self._poisoned:
                return False
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        if not self._direction_guard.wait_ready(
            max(0.0, deadline - time.monotonic())
        ):
            return False
        poll_interval = 0.5
        clients = [self._client]
        if (
            self._through_poses_behavior_tree
            or self._reverse_through_poses_behavior_tree
            or self._reverse_locked_through_poses_behavior_tree
            or self._reverse_return_through_poses_behavior_tree
        ):
            clients.append(self._through_client)
        for client in clients:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                if client.wait_for_server(
                    timeout_sec=min(poll_interval, remaining)
                ):
                    break
        return True

    def navigate(self, waypoint, reverse_direction=False):
        try:
            behavior_tree = navigation_behavior_tree(
                reverse_direction,
                self._reverse_behavior_tree,
                goal_profile=waypoint.goal_profile,
                precise_forward_behavior_tree=(
                    self._precise_forward_behavior_tree),
                reverse_handoff_behavior_tree=(
                    self._reverse_handoff_behavior_tree),
            )
        except ValueError as error:
            return OperationResult(False, f"navigation_config:{error}")

        try:
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_stamped(waypoint)
            goal.behavior_tree = behavior_tree
        except (TypeError, ValueError) as error:
            return OperationResult(False, f"navigation_config:{error}")
        return self._navigate_goal(goal, self._client, reverse_direction)

    def navigate_through(self, waypoints, reverse_direction=False):
        """Run one constant-direction segment without stopping at through goals."""
        goals = tuple(waypoints)
        if len(goals) < 2:
            return OperationResult(
                False, "navigation_through_requires_multiple_goals")
        if any(
            waypoint.direction != goals[0].direction for waypoint in goals
        ):
            return OperationResult(False, "navigation_through_direction_mismatch")
        nonstandard = [
            waypoint.id or str(index)
            for index, waypoint in enumerate(goals)
            if waypoint.goal_profile != "standard"
        ]
        if nonstandard and not (
            reverse_direction
            and allows_reverse_handoff_through_poses(goals)
        ):
            return OperationResult(
                False,
                "navigation_through_nonstandard_goal_profile:"
                + ",".join(nonstandard),
            )
        try:
            behavior_tree = self._through_behavior_tree(
                reverse_direction,
                is_heading_locked(goals[-1]),
                goals[-1].task == "return",
            )
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self._pose_stamped(waypoint) for waypoint in goals]
            goal.behavior_tree = behavior_tree
        except (TypeError, ValueError) as error:
            return OperationResult(False, f"navigation_config:{error}")
        return self._navigate_goal(goal, self._through_client, reverse_direction)

    def _through_behavior_tree(
        self, reverse_direction, terminal_heading_locked, terminal_is_return=False
    ):
        if reverse_direction and terminal_heading_locked and terminal_is_return:
            behavior_tree = self._reverse_return_through_poses_behavior_tree
            direction = "reverse_return"
        elif reverse_direction and terminal_heading_locked:
            behavior_tree = self._reverse_locked_through_poses_behavior_tree
            direction = "reverse_locked"
        elif reverse_direction:
            behavior_tree = self._reverse_through_poses_behavior_tree
            direction = "reverse"
        else:
            behavior_tree = self._through_poses_behavior_tree
            direction = "forward"
        if not behavior_tree:
            raise ValueError(
                f"{direction}_through_poses_behavior_tree must not be empty"
            )
        return behavior_tree

    def _pose_stamped(self, waypoint):
        qx, qy, qz, qw = waypoint.orientation
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or (
            norm > 1.0e-3 and abs(norm - 1.0) > 1.0e-3
        ):
            raise ValueError(
                "navigation goal orientation must be a unit quaternion or "
                "the free-heading zero sentinel"
            )
        pose = PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = waypoint.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            waypoint.position
        )
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        return pose

    def _navigate_goal(self, goal, action_client, reverse_direction=False):
        action_uuid = UUID(uuid=list(uuid4().bytes))
        direction = motion_direction(reverse_direction)
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
            self._operation_active = True
            self._navigate_attached = True
            self._cancel_requested = False
            self._goal_error = None
            self._cancel_error = None
            self._terminal_generation = None
            self._terminal_response = None
            self._guard_stop_attempted_generation = None
            self._guard_revoked_generation = None
            self._guard_stopped_generation = None
            self._current_identity = self._motion_protocol.provisional(
                direction, generation, action_uuid)

        prepared, lease = self._prepare_motion(
            direction, generation, action_uuid)
        if not prepared.success:
            self._clear_without_server_goal(generation, poison=False)
            return prepared

        with self._condition:
            canceled_before_send = self._cancel_requested
        if canceled_before_send:
            stopped = self._stop_motion(generation)
            self._clear_without_server_goal(
                generation, poison=not stopped.success)
            return OperationResult(False, "navigation_canceled")

        with self._condition:
            try:
                future = action_client.send_goal_async(
                    goal, goal_uuid=action_uuid)
            except Exception as error:
                send_error = OperationResult(
                    False,
                    f"navigation_send_error:{type(error).__name__}",
                )
            else:
                send_error = None
                self._pending_goal_future = future
                future.add_done_callback(
                    lambda completed: self._on_goal_response(
                        generation, completed))
        if send_error is not None:
            stopped = self._stop_motion(generation)
            self._clear_without_server_goal(
                generation, poison=not stopped.success)
            return stopped if not stopped.success else send_error

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
            response_timed_out = self._pending_goal_future is not None
            goal_error = self._goal_error
            goal_handle = self._goal_handle

        if response_timed_out:
            terminal = self.cancel()
            if terminal:
                stopped, _response = self._consume_terminal(generation)
                if not stopped.success:
                    return stopped
                return OperationResult(False, "navigation_timeout")
            self._detach_navigation(generation)
            return OperationResult(
                False, "navigation_goal_response_timeout_unconfirmed")

        if goal_error == "navigation_goal_rejected":
            stopped = self._stop_motion(generation)
            self._clear_without_server_goal(
                generation, poison=not stopped.success)
            return stopped if not stopped.success else OperationResult(
                False, goal_error)

        if goal_error is not None:
            if goal_handle is not None:
                terminal = self.cancel()
                if terminal:
                    stopped, _response = self._consume_terminal(generation)
                    if not stopped.success:
                        return stopped
                    return OperationResult(False, goal_error)
            else:
                self._stop_motion(generation)
            self._detach_navigation(generation)
            return OperationResult(False, f"{goal_error}_unconfirmed")

        with self._condition:
            terminal_before_activate = (
                self._terminal_generation == generation)
            cancel_before_activate = self._cancel_requested
        if terminal_before_activate:
            stopped, response = self._consume_terminal(generation)
            if not stopped.success:
                return stopped
            return classify_navigate_to_pose_result(response.status)
        if cancel_before_activate:
            terminal = self.cancel()
            if not terminal:
                self._detach_navigation(generation)
                return OperationResult(
                    False, "navigation_cancel_unconfirmed")
            stopped, response = self._consume_terminal(generation)
            if not stopped.success:
                return stopped
            return classify_navigate_to_pose_result(response.status)

        activated = self._activate_motion(generation)
        if not activated.success:
            terminal = self.cancel()
            if terminal:
                stopped, _response = self._consume_terminal(generation)
                if not stopped.success:
                    return stopped
                return activated
            self._detach_navigation(generation)
            return OperationResult(
                False, f"{activated.status}:cancel_unconfirmed")

        deadline = time.monotonic() + self._navigation_timeout_sec
        next_renew = time.monotonic() + self._renew_period_sec
        while True:
            goal_error = None
            cancel_error = None
            with self._condition:
                if self._terminal_generation == generation:
                    break
                if self._goal_error is not None:
                    goal_error = self._goal_error
                elif self._cancel_error is not None and self._poisoned:
                    cancel_error = self._cancel_error
                else:
                    cancel_requested = self._cancel_requested
                    now = time.monotonic()
                    wake_at = deadline if cancel_requested else min(
                        deadline, next_renew)
                    if now < wake_at:
                        self._condition.wait(timeout=wake_at - now)
                        continue

            if goal_error is not None:
                terminal = self.cancel()
                if terminal:
                    stopped, _response = self._consume_terminal(generation)
                    if not stopped.success:
                        return stopped
                    return OperationResult(False, goal_error)
                self._detach_navigation(generation)
                return OperationResult(False, f"{goal_error}_unconfirmed")
            if cancel_error is not None:
                self._stop_motion(generation)
                self._detach_navigation(generation)
                return OperationResult(False, cancel_error)

            now = time.monotonic()
            if cancel_requested:
                if not self.cancel():
                    self._detach_navigation(generation)
                    return OperationResult(
                        False, "navigation_cancel_unconfirmed")
                continue
            if now >= deadline:
                terminal = self.cancel()
                if terminal:
                    stopped, _response = self._consume_terminal(generation)
                    if not stopped.success:
                        return stopped
                    return OperationResult(False, "navigation_timeout")
                self._detach_navigation(generation)
                return OperationResult(
                    False, "navigation_timeout_cancel_unconfirmed")
            renewed = self._renew_motion(generation)
            if not renewed.success:
                terminal = self.cancel()
                if terminal:
                    stopped, _response = self._consume_terminal(generation)
                    if not stopped.success:
                        return stopped
                    return renewed
                self._detach_navigation(generation)
                return OperationResult(
                    False, f"{renewed.status}:cancel_unconfirmed")
            next_renew = time.monotonic() + self._renew_period_sec

        stopped, response = self._consume_terminal(generation)
        if not stopped.success:
            return stopped
        return classify_navigate_to_pose_result(response.status)

    def cancel(self):
        with self._condition:
            if not self._active_locked():
                return True
            generation = self._goal_generation
            self._cancel_requested = True
            self._condition.notify_all()

        self._revoke_motion(generation)
        deadline = time.monotonic() + self._cancel_timeout_sec
        with self._condition:
            while (
                generation == self._goal_generation
                and self._operation_active
                and self._pending_goal_future is not None
                and self._goal_handle is None
                and time.monotonic() < deadline
            ):
                self._condition.wait(
                    timeout=max(0.0, deadline - time.monotonic()))

            if (
                generation == self._goal_generation
                and self._goal_handle is not None
            ):
                self._request_cancel_locked(generation)
            while (
                generation == self._goal_generation
                and self._operation_active
                and self._terminal_generation != generation
                and time.monotonic() < deadline
            ):
                self._condition.wait(
                    timeout=max(0.0, deadline - time.monotonic()))

            terminal = self._terminal_generation == generation
            cleared = not self._operation_active
            if not terminal and not cleared:
                self._poisoned = True
                if self._cancel_error is None:
                    self._cancel_error = "navigation_cancel_unconfirmed"
        if terminal or cleared:
            settled = self._settle_motion(generation)
            if settled.success and cleared:
                with self._condition:
                    self._clear_operation_locked(generation, poison=False)
            return settled.success
        return False

    def is_active(self):
        with self._condition:
            return self._active_locked()

    def _active_locked(self):
        return self._poisoned or self._operation_active

    def _prepare_motion(self, direction, generation, action_uuid):
        with self._guard_call_lock:
            result, lease = self._motion_protocol.prepare(
                direction, generation, action_uuid)
            with self._condition:
                if generation == self._goal_generation and result.success:
                    self._current_identity = lease
                    self._guard_revoked_generation = None
                    self._guard_stopped_generation = None
                self._condition.notify_all()
            return result, lease

    def _activate_motion(self, generation):
        with self._guard_call_lock:
            with self._condition:
                if generation != self._goal_generation:
                    return OperationResult(False, "direction_stale_generation")
                lease = self._current_identity
            result = self._motion_protocol.activate(lease)
            if not result.success:
                with self._condition:
                    self._poisoned = True
            return result

    def _renew_motion(self, generation):
        with self._guard_call_lock:
            with self._condition:
                if generation != self._goal_generation:
                    return OperationResult(False, "direction_stale_generation")
                lease = self._current_identity
            result = self._motion_protocol.renew(lease)
            if not result.success:
                with self._condition:
                    self._poisoned = True
            return result

    def _revoke_motion(self, generation):
        with self._guard_call_lock:
            with self._condition:
                if generation != self._goal_generation:
                    return OperationResult(False, "direction_stale_generation")
                if self._guard_revoked_generation == generation:
                    return OperationResult(True, "revoked")
                identity = self._current_identity
                if identity is None:
                    identity = self._motion_protocol.provisional(
                        motion_direction(False), generation, UUID())
                self._guard_stop_attempted_generation = generation
            result = self._motion_protocol.revoke(identity)
            with self._condition:
                if generation == self._goal_generation:
                    if result.success:
                        self._guard_revoked_generation = generation
                    else:
                        self._poisoned = True
                    self._condition.notify_all()
            return result

    def _settle_motion(self, generation):
        revoked = self._revoke_motion(generation)
        if not revoked.success:
            return revoked
        with self._guard_call_lock:
            with self._condition:
                if generation != self._goal_generation:
                    return OperationResult(False, "direction_stale_generation")
                if self._guard_stopped_generation == generation:
                    return OperationResult(True, "stopped")
            result = self._motion_protocol.settle()
            with self._condition:
                if generation == self._goal_generation:
                    if result.success:
                        self._guard_stopped_generation = generation
                    else:
                        self._poisoned = True
                    self._condition.notify_all()
            return result

    def _stop_motion(self, generation):
        revoked = self._revoke_motion(generation)
        if not revoked.success:
            return revoked
        return self._settle_motion(generation)

    def _consume_terminal(self, generation):
        stopped = self._stop_motion(generation)
        with self._condition:
            response = self._terminal_response
            terminal = self._terminal_generation == generation
            if terminal:
                self._clear_operation_locked(
                    generation, poison=not stopped.success)
            if response is None:
                return OperationResult(
                    False, "navigation_result_unavailable"), None
        return stopped, response

    def _clear_without_server_goal(self, generation, poison):
        with self._condition:
            self._clear_operation_locked(generation, poison=poison)

    def _clear_operation_locked(self, generation, poison):
        if generation != self._goal_generation:
            return
        self._operation_active = False
        self._navigate_attached = False
        if not poison:
            self._current_identity = None
        self._pending_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._poisoned = bool(poison)
        self._condition.notify_all()

    def _detach_navigation(self, generation):
        with self._condition:
            self._detach_navigation_locked(generation)

    def _detach_navigation_locked(self, generation):
        if generation == self._goal_generation:
            self._navigate_attached = False

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
            if (
                self._cancel_requested
                and self._guard_stop_attempted_generation == generation
            ):
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
            self._cancel_error = (
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
                response = future.result()
            except Exception as error:
                self._poisoned = True
                self._goal_error = (
                    f"navigation_result_error:{type(error).__name__}")
                self._condition.notify_all()
                return
            self._terminal_generation = generation
            self._terminal_response = response
            if (
                not self._navigate_attached
                and self._guard_stopped_generation == generation
            ):
                self._clear_operation_locked(generation, poison=False)
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
        self.declare_parameter("server_wait_timeout_sec", 30.0)
        self.declare_parameter("navigation_timeout_sec", 120.0)
        self.declare_parameter("goal_response_timeout_sec", 2.0)
        self.declare_parameter("cancel_timeout_sec", 3.0)
        self.declare_parameter("stop_timeout_sec", 5.0)
        self.declare_parameter(
            "reverse_behavior_tree",
            "/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/"
            "config/behavior_trees/"
            "navigate_to_pose_reverse_w_replanning_and_recovery.xml",
        )
        self.declare_parameter(
            "reverse_handoff_behavior_tree",
            "/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/"
            "config/behavior_trees/"
            "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml",
        )
        self.declare_parameter(
            "precise_forward_behavior_tree",
            "/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/"
            "config/behavior_trees/"
            "navigate_to_pose_precise_w_replanning_and_recovery.xml",
        )
        self.declare_parameter(
            "through_poses_behavior_tree",
            "/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/"
            "config/behavior_trees/"
            "navigate_through_poses_w_replanning_and_recovery.xml",
        )
        self.declare_parameter(
            "reverse_through_poses_behavior_tree",
            "/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/"
            "config/behavior_trees/"
            "navigate_through_poses_reverse_w_replanning_and_recovery.xml",
        )
        self.declare_parameter(
            "reverse_locked_through_poses_behavior_tree",
            "/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/"
            "config/behavior_trees/"
            "navigate_through_poses_reverse_locked_w_replanning_and_recovery.xml",
        )
        self.declare_parameter(
            "reverse_return_through_poses_behavior_tree",
            "/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/"
            "config/behavior_trees/"
            "navigate_through_poses_reverse_return_w_replanning_and_recovery.xml",
        )
        self.declare_parameter("direction_service_timeout_sec", 0.08)
        self.declare_parameter("direction_lease_timeout_sec", 0.25)
        self.declare_parameter("direction_prepare_timeout_sec", 1.0)
        self.declare_parameter("direction_prepare_retry_period_sec", 0.02)
        self.declare_parameter("direction_renew_period_sec", 0.10)
        self.declare_parameter("direction_stop_timeout_sec", 2.0)
        self.declare_parameter("direction_stop_dwell_sec", 0.25)
        self.declare_parameter("direction_stop_linear_tolerance", 0.01)
        self.declare_parameter("direction_stop_angular_tolerance", 0.05)
        self.declare_parameter("direction_odom_stale_timeout_sec", 0.25)
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
        self.declare_parameter("barcode_reader_image_topic", "/image")

        waypoints_file = str(
            self.get_parameter("waypoints_file").value).strip()
        if not waypoints_file:
            raise ValueError("waypoints_file must be provided")
        try:
            waypoint_document, authored_waypoints = load_waypoint_document(
                waypoints_file
            )
            planning_segments = load_planning_segments(
                waypoint_document,
                authored_waypoints,
            )
            ordered_waypoints = materialize_mission_route(
                authored_waypoints,
                planning_segments,
            )
            self._waypoints = materialize_free_yaws(ordered_waypoints)
            materialized_by_id = {
                waypoint.id: waypoint for waypoint in self._waypoints
            }
            authored_navigation_segments = materialize_navigation_segments(
                authored_waypoints,
                planning_segments,
            )
            self._navigation_segments = tuple(
                tuple(materialized_by_id[waypoint.id] for waypoint in segment)
                for segment in authored_navigation_segments
            )
        except (PlanningSegmentError, RouteGeometryError, ValueError) as error:
            raise ValueError(f"invalid mission route: {error}") from error
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
        direction_lease_timeout = _positive_finite(
            "direction_lease_timeout_sec",
            self.get_parameter("direction_lease_timeout_sec").value,
        )
        direction_service_timeout = _positive_finite(
            "direction_service_timeout_sec",
            self.get_parameter("direction_service_timeout_sec").value,
        )
        direction_renew_period = _positive_finite(
            "direction_renew_period_sec",
            self.get_parameter("direction_renew_period_sec").value,
        )
        if direction_service_timeout >= direction_lease_timeout:
            raise ValueError(
                "direction_service_timeout_sec must be below lease timeout")
        if direction_renew_period >= direction_lease_timeout:
            raise ValueError(
                "direction_renew_period_sec must be below lease timeout")
        self._direction_guard = RosDirectionGuard(
            self,
            self._io_group,
            direction_service_timeout,
            self.get_parameter("direction_stop_timeout_sec").value,
            self.get_parameter("direction_stop_dwell_sec").value,
            self.get_parameter("direction_stop_linear_tolerance").value,
            self.get_parameter("direction_stop_angular_tolerance").value,
            self.get_parameter("direction_odom_stale_timeout_sec").value,
        )
        self._navigator = RosNavigator(
            self,
            self._io_group,
            self._direction_guard,
            self.get_parameter("reverse_behavior_tree").value,
            self.get_parameter("reverse_handoff_behavior_tree").value,
            self.get_parameter("precise_forward_behavior_tree").value,
            self.get_parameter("navigation_timeout_sec").value,
            self.get_parameter("goal_response_timeout_sec").value,
            self.get_parameter("cancel_timeout_sec").value,
            direction_renew_period,
            self.get_parameter("direction_prepare_timeout_sec").value,
            self.get_parameter("direction_prepare_retry_period_sec").value,
            self.get_parameter("through_poses_behavior_tree").value,
            self.get_parameter("reverse_through_poses_behavior_tree").value,
            self.get_parameter(
                "reverse_locked_through_poses_behavior_tree").value,
            self.get_parameter(
                "reverse_return_through_poses_behavior_tree").value,
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
        self._autostart_retries = 0
        self._autostart_max_retries = 60
        if bool(self.get_parameter("autostart_mission").value):
            self._autostart_timer = self.create_timer(
                3.0,
                self._on_autostart,
                callback_group=self._service_group,
            )
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} semantic waypoints in "
            f"{len(self._navigation_segments)} planning segments")

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
        result = self._mission.run_reserved(
            generation,
            self._waypoints,
            self._navigation_segments,
        )
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
        self._autostart_retries += 1
        success, message = self._start_worker()
        if success:
            if self._autostart_timer is not None:
                self.destroy_timer(self._autostart_timer)
                self._autostart_timer = None
            self.get_logger().info("Mission autostarted")
            return
        if self._autostart_retries >= self._autostart_max_retries:
            if self._autostart_timer is not None:
                self.destroy_timer(self._autostart_timer)
                self._autostart_timer = None
            self.get_logger().error(
                f"Mission autostart failed after "
                f"{self._autostart_retries} attempts: {message}")
        else:
            self.get_logger().warn(
                f"Mission autostart attempt {self._autostart_retries}/"
                f"{self._autostart_max_retries}: {message}")

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
