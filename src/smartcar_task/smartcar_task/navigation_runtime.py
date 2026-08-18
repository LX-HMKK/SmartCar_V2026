"""Native Nav2 action and direction-lease lifecycle adapter."""

import math
import threading
import time
from uuid import uuid4

from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from unique_identifier_msgs.msg import UUID

from smartcar_task.mission import OperationResult
from smartcar_task.navigation_goals import Nav2GoalFactory
from smartcar_task.protocols import (
    MotionDirectionProtocol,
    classify_navigate_to_pose_result,
    motion_direction,
)


def _positive_finite(name, value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


class _Nav2OperationLifecycle:
    """Run one Nav2 goal through dispatch, terminal proof, and cancellation.

    The state remains on RosNavigator so action callbacks and direction-lease
    operations use the same condition variable and generation boundary.
    """

    def _navigate_goal(self, goal, action_client):
        action_uuid = UUID(uuid=list(uuid4().bytes))
        direction = motion_direction()
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
                self._warn_renewal_failure(renewed)
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


class _DirectionLeaseLifecycle:
    """Own the direction-guard lease transitions for an active Nav2 goal."""

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
            return result

    def _warn_renewal_failure(self, result):
        """Record renewal transport failures without revoking Nav2 motion."""
        now = time.monotonic()
        status = str(result.status)
        if (
            status == self._renew_warning_status
            and self._renew_warning_at is not None
            and now - self._renew_warning_at < 5.0
        ):
            return
        self._node.get_logger().warning(
            f"direction renewal unavailable ({status}); continuing under "
            "candidate-command watchdog",
        )
        self._renew_warning_status = status
        self._renew_warning_at = now

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
                        motion_direction(), generation, UUID())
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
        settled = self._motion_protocol.settle()
        if not settled.success:
            with self._condition:
                if generation == self._goal_generation:
                    self._poisoned = True
            return settled
        with self._guard_call_lock:
            with self._condition:
                if generation != self._goal_generation:
                    return OperationResult(False, "direction_stale_generation")
                if self._guard_stopped_generation == generation:
                    return settled
                # Stop revokes the direction lease and makes the command gate
                # output zero immediately. EKF velocity is not a reliable
                # terminal condition for the next navigation action.
                self._guard_stopped_generation = generation
                self._condition.notify_all()
        return settled

    def _stop_motion(self, generation):
        revoked = self._revoke_motion(generation)
        if not revoked.success:
            return revoked
        return self._settle_motion(generation)


class _Nav2ActionCallbacks:
    """Maintain Nav2 action callback state for the current generation."""

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



class RosNavigator(
    _Nav2OperationLifecycle,
    _Nav2ActionCallbacks,
    _DirectionLeaseLifecycle,
):
    """One guarded Nav2 action at a time, with terminal proof."""

    def __init__(
        self,
        node,
        callback_group,
        direction_guard,
        precise_behavior_tree,
        transit_behavior_tree,
        navigation_timeout_sec,
        goal_response_timeout_sec,
        cancel_timeout_sec,
        direction_renew_period_sec,
        direction_prepare_timeout_sec,
        direction_prepare_retry_period_sec,
        through_poses_behavior_tree="",
        transit_through_poses_behavior_tree="",
        precise_through_poses_behavior_tree="",
        return_through_poses_behavior_tree="",
    ):
        self._node = node
        self._callback_group = callback_group
        self._client = None
        self._through_client = None
        self._direction_guard = direction_guard
        self._motion_protocol = MotionDirectionProtocol(
            direction_guard,
            direction_guard.wait_stopped,
            prepare_timeout_sec=direction_prepare_timeout_sec,
            prepare_retry_period_sec=direction_prepare_retry_period_sec,
        )
        self._goal_factory = Nav2GoalFactory(
            node,
            precise_behavior_tree,
            transit_behavior_tree,
            through_poses_behavior_tree,
            transit_through_poses_behavior_tree,
            precise_through_poses_behavior_tree,
            return_through_poses_behavior_tree,
        )
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
        self._renew_warning_status = None
        self._renew_warning_at = None

    def wait_ready(self, timeout_sec):
        with self._condition:
            if self._poisoned:
                return False
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        if not self._direction_guard.wait_ready(
            max(0.0, deadline - time.monotonic())
        ):
            return False
        return True

    def _action_client(self, through_poses):
        attribute = "_through_client" if through_poses else "_client"
        with self._condition:
            client = getattr(self, attribute)
            if client is not None:
                return client
            if through_poses:
                client = ActionClient(
                    self._node,
                    NavigateThroughPoses,
                    "/navigate_through_poses",
                    callback_group=self._callback_group,
                )
            else:
                client = ActionClient(
                    self._node,
                    NavigateToPose,
                    "/navigate_to_pose",
                    callback_group=self._callback_group,
                )
            setattr(self, attribute, client)
            return client

    def prewarm_action_clients(self):
        """Start DDS discovery before the operator releases the e-stop."""
        self._action_client(through_poses=False)
        self._action_client(through_poses=True)

    def _wait_for_action_server(self, client, timeout_sec):
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        poll_interval = 0.5
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            if client.wait_for_server(timeout_sec=min(poll_interval, remaining)):
                return True

    def navigate(self, waypoint):
        try:
            goal = self._goal_factory.navigate_goal(waypoint)
        except (TypeError, ValueError) as error:
            return OperationResult(False, f"navigation_config:{error}")
        client = self._action_client(through_poses=False)
        if not self._wait_for_action_server(
            client, self._goal_response_timeout_sec
        ):
            return OperationResult(False, "navigation_server_unavailable")
        return self._navigate_goal(goal, client)

    def navigate_through(self, waypoints):
        """Run one constant-direction segment without stopping at through goals."""
        try:
            goal = self._goal_factory.navigate_through_goal(waypoints)
        except (TypeError, ValueError) as error:
            if str(error).startswith("navigation_through_"):
                return OperationResult(False, str(error))
            return OperationResult(False, f"navigation_config:{error}")
        client = self._action_client(through_poses=True)
        if not self._wait_for_action_server(
            client, self._goal_response_timeout_sec
        ):
            return OperationResult(False, "navigation_server_unavailable")
        return self._navigate_goal(goal, client)
