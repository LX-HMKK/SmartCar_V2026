#!/usr/bin/env python3
"""Start the Gazebo Nav2 lifecycle stack after DDS service warm-up.

Nav2 Humble creates its lifecycle service clients just after the lifecycle
manager process starts.  Starting the manager immediately can race Fast DDS
service reply-reader discovery when a controller has a non-trivial configure
step (such as loading MPPI critics).  This process deliberately waits until
both the manager and every managed node's state service are discoverable,
then lets those readers settle before issuing exactly one STARTUP request.
If that request's response is lost, it performs only bounded read-only state
probes and accepts the startup only when every managed node is already ACTIVE.

It publishes no motion command.  A zero exit status means every managed Nav2
node reported ACTIVE after a successful STARTUP response or after bounded
state probes proved that only that response was lost.
"""

from __future__ import annotations

import math
import sys
import time
from collections.abc import Mapping

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


MANAGED_NODES = (
    "controller_server",
    "planner_server",
    "bt_navigator",
    "velocity_smoother",
)
STARTUP_STATE_PROBE_INTERVAL_SEC = 2.0


class Nav2LifecycleStartup(Node):
    """Request one verified Nav2 startup after lifecycle DDS warm-up."""

    def __init__(self) -> None:
        super().__init__("nav2_lifecycle_startup")
        self.declare_parameter(
            "manager_service", "/lifecycle_manager_navigation/manage_nodes")
        self.declare_parameter("warmup_sec", 4.0)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("startup_timeout_sec", 90.0)
        self.declare_parameter("state_timeout_sec", 15.0)

        self._warmup_sec = self._positive_finite("warmup_sec")
        self._service_timeout_sec = self._positive_finite("service_timeout_sec")
        self._startup_timeout_sec = self._positive_finite("startup_timeout_sec")
        self._state_timeout_sec = self._positive_finite("state_timeout_sec")
        manager_service = str(self.get_parameter("manager_service").value)
        if not manager_service.startswith("/"):
            raise ValueError("manager_service must be absolute")

        self._manager_client = self.create_client(
            ManageLifecycleNodes, manager_service)
        self._state_clients = {
            node_name: self.create_client(
                GetState, f"/{node_name}/get_state")
            for node_name in MANAGED_NODES
        }

    def _positive_finite(self, parameter_name: str) -> float:
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{parameter_name} must be a positive finite number")
        return value

    def _wait_for_services(self) -> bool:
        services = {
            "lifecycle manager": self._manager_client,
            **{
                f"{node_name} state": client
                for node_name, client in self._state_clients.items()
            },
        }
        deadline = time.monotonic() + self._service_timeout_sec
        pending = set(services)
        while rclpy.ok() and pending and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            for label in tuple(pending):
                if services[label].wait_for_service(timeout_sec=min(0.25, remaining)):
                    pending.remove(label)
            rclpy.spin_once(self, timeout_sec=0.0)
        if pending:
            self.get_logger().error(
                "Nav2 lifecycle services did not become ready: "
                + ", ".join(sorted(pending)))
            return False
        return True

    def _wait_for_futures(
        self,
        futures: Mapping[str, object],
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and any(not future.done() for future in futures.values()):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.10, remaining))
        pending = [label for label, future in futures.items() if not future.done()]
        if pending:
            self.get_logger().error(
                "Timed out waiting for Nav2 lifecycle response: "
                + ", ".join(pending))
            return False
        return True

    def _active_states(
        self, timeout_sec: float | None = None
    ) -> tuple[bool, dict[str, int]]:
        """Query all managed nodes without assuming the manager reply arrived."""
        response_timeout_sec = (
            self._state_timeout_sec if timeout_sec is None else timeout_sec)
        if not math.isfinite(response_timeout_sec) or response_timeout_sec <= 0.0:
            return False, {}
        request = GetState.Request()
        futures = {
            node_name: client.call_async(request)
            for node_name, client in self._state_clients.items()
        }
        if not self._wait_for_futures(futures, response_timeout_sec):
            return False, {}

        states: dict[str, int] = {}
        for node_name, future in futures.items():
            try:
                response = future.result()
            except Exception as error:  # rclpy reports transport errors here.
                self.get_logger().error(
                    f"Could not query {node_name} lifecycle state: {error}")
                return False, states
            if response is None:
                self.get_logger().error(
                    f"{node_name} lifecycle state query returned no response")
                return False, states
            states[node_name] = int(response.current_state.id)
        return all(
            state == State.PRIMARY_STATE_ACTIVE for state in states.values()), states

    def _wait_for_startup_or_active(self, future: object) -> bool:
        """Accept one STARTUP only after state probes prove its actual result.

        Fast DDS can lose the manager's service response while delivering the
        lifecycle state traffic.  Polling the managed nodes during the bounded
        wait avoids needlessly stalling a healthy Nav2 stack for the full
        service timeout.  It never sends a second STARTUP request.
        """
        deadline = time.monotonic() + self._startup_timeout_sec
        next_probe = time.monotonic()
        startup_acknowledged = False
        last_states: dict[str, int] = {}
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self, timeout_sec=min(0.10, remaining))

            if future.done() and not startup_acknowledged:
                try:
                    response = future.result()
                except Exception as error:  # rclpy reports transport errors here.
                    self.get_logger().error(
                        f"Nav2 lifecycle STARTUP request failed: {error}")
                    return False
                if response is None or not response.success:
                    self.get_logger().error("Nav2 lifecycle manager rejected STARTUP")
                    return False
                startup_acknowledged = True
                self.get_logger().info(
                    "Nav2 lifecycle manager acknowledged STARTUP; verifying "
                    "managed node states")

            now = time.monotonic()
            if now < next_probe:
                continue
            # Do not let one lost GetState response consume the complete
            # STARTUP budget.  A later probe can still prove that activation
            # completed, while an unresponsive stack remains fail-closed.
            probe_timeout = min(
                self._state_timeout_sec,
                max(0.10, deadline - now),
            )
            active, states = self._active_states(probe_timeout)
            if states:
                last_states = states
            if active:
                if startup_acknowledged:
                    self.get_logger().info(
                        "Nav2 lifecycle startup verified ACTIVE: "
                        + ", ".join(MANAGED_NODES))
                else:
                    self.get_logger().warn(
                        "Nav2 lifecycle STARTUP response is pending, but "
                        "managed nodes are verified ACTIVE")
                return True
            self.get_logger().info(
                "Nav2 lifecycle STARTUP pending activation: "
                f"acknowledged={startup_acknowledged}, states={states}")
            next_probe = time.monotonic() + STARTUP_STATE_PROBE_INTERVAL_SEC

        if not rclpy.ok():
            return False
        self.get_logger().error(
            "Nav2 lifecycle STARTUP did not verify every managed node ACTIVE; "
            f"acknowledged={startup_acknowledged}, states={last_states}")
        return False

    def start(self) -> bool:
        if not self._wait_for_services():
            return False

        self.get_logger().info(
            "Nav2 lifecycle services discovered; warming Fast DDS reply readers "
            f"for {self._warmup_sec:.1f}s")
        deadline = time.monotonic() + self._warmup_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.10, deadline - time.monotonic()))
        if not rclpy.ok():
            return False

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self._manager_client.call_async(request)
        return self._wait_for_startup_or_active(future)


def main() -> int:
    rclpy.init()
    node = Nav2LifecycleStartup()
    try:
        return 0 if node.start() else 2
    except (KeyboardInterrupt, ExternalShutdownException):
        return 2
    except Exception as error:
        node.get_logger().error(f"Nav2 lifecycle startup failed: {error}")
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
