#!/usr/bin/env python3
"""Collect physical navigation startup health in one bounded ROS process."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
import os
import time


LIFECYCLE_NODES = (
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
)
TASK_SERVICES = ("start", "reset")
ACTIVE_STATE_ID = 3
DEPTH_MIN_FRAMES = 2
MAX_DEPTH_CAPTURE_AGE_SEC = 0.25
MAX_DEPTH_FUTURE_SKEW_SEC = 0.05
DEPTH_POINTS_FRAME = "depth_camera_link_1"
DEPTH_SCAN_FRAME = "base_footprint"


@dataclass
class SafetyRequirements:
    """Runtime safety limits queried from the active safety node."""

    require_scan: bool | None = None
    scan_timeout_sec: float | None = None
    require_odom: bool | None = None
    odom_timeout_sec: float | None = None
    require_raw_odom: bool | None = None
    raw_odom_timeout_sec: float | None = None
    require_depth_points: bool | None = None
    depth_points_timeout_sec: float | None = None
    minimum_voltage: float | None = None
    voltage_timeout_sec: float | None = None

    def is_complete(self) -> bool:
        return all(value is not None for value in (
            self.require_scan,
            self.scan_timeout_sec,
            self.require_odom,
            self.odom_timeout_sec,
            self.require_raw_odom,
            self.raw_odom_timeout_sec,
            self.require_depth_points,
            self.depth_points_timeout_sec,
            self.minimum_voltage,
            self.voltage_timeout_sec,
        ))


@dataclass
class StartupSnapshot:
    lifecycle: dict[str, int] = field(default_factory=dict)
    safety: str = ""
    direction_guard: str = ""
    task: str = ""
    task_services: dict[str, bool] = field(default_factory=dict)
    depth_status: str = ""
    depth_points: int = 0
    depth_scan: int = 0
    depth_frame: str = ""
    scan_frame: str = ""
    local_costmap: int = 0
    global_costmap: int = 0
    costmap_sources: dict[str, str] = field(default_factory=dict)
    costmap_topics: dict[str, str] = field(default_factory=dict)
    capture_age_sec: float | None = None
    odom_received_at: float | None = None
    raw_odom_received_at: float | None = None
    voltage_received_at: float | None = None
    voltage: float | None = None
    scan_received_at: float | None = None
    depth_points_received_at: float | None = None
    depth_scan_received_at: float | None = None
    safety_requirements: SafetyRequirements = field(
        default_factory=SafetyRequirements)
    rviz_required: bool = False
    rviz_running: bool | None = None


def _is_fresh(received_at: float | None, timeout_sec: float, now_sec: float) -> bool:
    return received_at is not None and now_sec - received_at <= timeout_sec


def _safety_input_missing(snapshot: StartupSnapshot, now_sec: float) -> list[str]:
    requirements = snapshot.safety_requirements
    if not requirements.is_complete():
        return ["safety_parameters"]

    missing = []
    if requirements.require_scan and not _is_fresh(
            snapshot.scan_received_at, float(requirements.scan_timeout_sec), now_sec):
        missing.append("scan_fresh")
    if requirements.require_odom and not _is_fresh(
            snapshot.odom_received_at, float(requirements.odom_timeout_sec), now_sec):
        missing.append("odom_fresh")
    if requirements.require_raw_odom and not _is_fresh(
            snapshot.raw_odom_received_at,
            float(requirements.raw_odom_timeout_sec), now_sec):
        missing.append("raw_odom_fresh")
    if requirements.require_depth_points and not _is_fresh(
            snapshot.depth_points_received_at,
            float(requirements.depth_points_timeout_sec), now_sec):
        missing.append("depth_points_fresh")
    if float(requirements.minimum_voltage) > 0.0:
        if not _is_fresh(
                snapshot.voltage_received_at,
                float(requirements.voltage_timeout_sec), now_sec):
            missing.append("voltage_fresh")
        elif snapshot.voltage is None or snapshot.voltage < float(
                requirements.minimum_voltage):
            missing.append("voltage_minimum")
    return missing


def missing_ready_items(
    snapshot: StartupSnapshot,
    require_depth: bool,
    now_sec: float = 0.0,
) -> list[str]:
    missing = [
        name for name in LIFECYCLE_NODES
        if snapshot.lifecycle.get(name) != ACTIVE_STATE_ID
    ]
    if snapshot.safety != "emergency_stop":
        missing.append("safety=emergency_stop")
    if snapshot.direction_guard != "stopped":
        missing.append("direction_guard=stopped")
    if snapshot.task != "IDLE":
        missing.append("task=IDLE")
    for service in TASK_SERVICES:
        if not snapshot.task_services.get(service, False):
            missing.append(f"task_service={service}")
    missing.extend(_safety_input_missing(snapshot, now_sec))

    expected_source = "depth_scan" if require_depth else "scan"
    expected_topic = "/smartcar/depth/scan" if require_depth else "/scan"
    for scope, frame_count in (
            ("local", snapshot.local_costmap),
            ("global", snapshot.global_costmap)):
        if frame_count < 1:
            missing.append(f"{scope}_costmap_after_observation")
        if snapshot.costmap_sources.get(scope) != expected_source:
            missing.append(f"{scope}_costmap_source")
        if snapshot.costmap_topics.get(scope) != expected_topic:
            missing.append(f"{scope}_costmap_topic")

    if require_depth:
        if snapshot.depth_status != "depth_points_active":
            missing.append("depth_status")
        if snapshot.depth_points < DEPTH_MIN_FRAMES:
            missing.append("depth_points")
        if snapshot.depth_scan < DEPTH_MIN_FRAMES:
            missing.append("depth_scan")
        if snapshot.depth_frame != DEPTH_POINTS_FRAME:
            missing.append("depth_points_frame")
        if snapshot.scan_frame != DEPTH_SCAN_FRAME:
            missing.append("depth_scan_frame")
        if snapshot.capture_age_sec is None or not (
                -MAX_DEPTH_FUTURE_SKEW_SEC <= snapshot.capture_age_sec
                <= MAX_DEPTH_CAPTURE_AGE_SEC):
            missing.append("depth_capture_age")
        if not _is_fresh(
                snapshot.depth_scan_received_at,
                float(snapshot.safety_requirements.depth_points_timeout_sec or 0.0),
                now_sec):
            missing.append("depth_scan_fresh")

    if snapshot.rviz_required and snapshot.rviz_running is not True:
        missing.append("rviz")
    return missing


def _age_text(received_at: float | None, now_sec: float | None) -> str:
    if received_at is None or now_sec is None:
        return "-"
    return f"{max(0.0, now_sec - received_at):.2f}s"


def summary(snapshot: StartupSnapshot, now_sec: float | None = None) -> str:
    active_count = sum(
        snapshot.lifecycle.get(name) == ACTIVE_STATE_ID
        for name in LIFECYCLE_NODES)
    task_services = "+".join(
        name for name in TASK_SERVICES
        if snapshot.task_services.get(name, False)) or "waiting"
    depth = (
        f"{snapshot.depth_status or 'waiting'} "
        f"points={snapshot.depth_points} scan={snapshot.depth_scan} "
        f"frames={snapshot.depth_frame or '-'}/{snapshot.scan_frame or '-'}"
    )
    capture_age = "-" if snapshot.capture_age_sec is None else (
        f"{snapshot.capture_age_sec:.3f}s")
    rviz = "not_requested"
    if snapshot.rviz_required:
        rviz = "running" if snapshot.rviz_running else "dead"
    sources = "/".join(
        snapshot.costmap_sources.get(scope, "-")
        for scope in ("local", "global"))
    return (
        f"nav2={active_count}/{len(LIFECYCLE_NODES)} "
        f"safety={snapshot.safety or 'waiting'} "
        f"guard={snapshot.direction_guard or 'waiting'} "
        f"task={snapshot.task or 'waiting'} services={task_services} "
        f"costmaps={snapshot.local_costmap}/{snapshot.global_costmap} "
        f"sources={sources} depth=[{depth}] capture_age={capture_age} "
        f"inputs=odom:{_age_text(snapshot.odom_received_at, now_sec)},"
        f"raw:{_age_text(snapshot.raw_odom_received_at, now_sec)},"
        f"voltage:{_age_text(snapshot.voltage_received_at, now_sec)},"
        f"scan:{_age_text(snapshot.scan_received_at, now_sec)} rviz={rviz}"
    )


def _process_is_running(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except (OSError, ProcessLookupError):
        return False
    return True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Wait for one complete navigation startup snapshot")
    parser.add_argument("--depth-camera", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--launch-pid", type=int, default=0)
    parser.add_argument("--rviz-pid", type=int, default=0)
    return parser.parse_args(argv)


def run(args) -> int:
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise ValueError("timeout must be finite and positive")
    if args.launch_pid < 0 or args.rviz_pid < 0:
        raise ValueError("process IDs must be non-negative")

    from lifecycle_msgs.srv import GetState
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2
    from std_msgs.msg import Float32, String
    from std_srvs.srv import Trigger

    status_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    sensor_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    reliable_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    costmap_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    safety_parameter_names = (
        "require_scan",
        "scan_timeout_sec",
        "require_odom",
        "odom_timeout_sec",
        "require_raw_odom",
        "raw_odom_timeout_sec",
        "require_depth_points",
        "depth_points_timeout_sec",
        "minimum_voltage",
        "voltage_timeout_sec",
    )
    costmap_parameter_names = (
        "obstacle_layer.observation_sources",
        "obstacle_layer.depth_scan.topic" if args.depth_camera
        else "obstacle_layer.scan.topic",
    )

    class StartupStatusNode(Node):
        def __init__(self):
            super().__init__("nav_startup_status")
            self.snapshot = StartupSnapshot(rviz_required=args.rviz_pid > 0)
            self.deadline = time.monotonic() + args.timeout
            self.exit_code: int | None = None
            self._last_report = ""
            self._last_report_at = 0.0
            self._last_lifecycle_poll_at = 0.0
            self._last_parameter_poll_at = 0.0
            self._lifecycle_futures = {}
            self._parameter_futures = {}
            self._lifecycle_clients = {
                name: self.create_client(GetState, f"/{name}/get_state")
                for name in LIFECYCLE_NODES
            }
            self._task_clients = {
                name: self.create_client(Trigger, f"/smartcar/task/{name}")
                for name in TASK_SERVICES
            }
            self._safety_parameter_client = self.create_client(
                GetParameters, "/safety_node/get_parameters")
            self._costmap_parameter_clients = {
                scope: self.create_client(
                    GetParameters,
                    f"/{scope}_costmap/{scope}_costmap/get_parameters",
                )
                for scope in ("local", "global")
            }
            self.create_subscription(
                String, "/smartcar/safety/status", self._on_safety, status_qos)
            self.create_subscription(
                String, "/smartcar/direction_guard/status",
                self._on_direction_guard, status_qos)
            self.create_subscription(
                String, "/smartcar/task/state", self._on_task, status_qos)
            self.create_subscription(
                Odometry, "/odom_combined", self._on_odom, reliable_qos)
            self.create_subscription(
                Odometry, "/odom", self._on_raw_odom, reliable_qos)
            self.create_subscription(
                Float32, "/PowerVoltage", self._on_voltage, reliable_qos)
            self.create_subscription(
                LaserScan, "/scan", self._on_scan, sensor_qos)
            self.create_subscription(
                Costmap, "/local_costmap/costmap_raw",
                self._on_local_costmap, costmap_qos)
            self.create_subscription(
                Costmap, "/global_costmap/costmap_raw",
                self._on_global_costmap, costmap_qos)
            if args.depth_camera:
                self.create_subscription(
                    String, "/smartcar/depth_obstacles/status",
                    self._on_depth_status, status_qos)
                self.create_subscription(
                    PointCloud2, "/smartcar/depth/points",
                    self._on_depth_points, sensor_qos)
                self.create_subscription(
                    LaserScan, "/smartcar/depth/scan",
                    self._on_depth_scan, sensor_qos)
            self.create_timer(0.2, self._tick)

        def _on_safety(self, message):
            self.snapshot.safety = message.data.strip()

        def _on_direction_guard(self, message):
            self.snapshot.direction_guard = message.data.strip()

        def _on_task(self, message):
            self.snapshot.task = message.data.strip()

        def _on_odom(self, _message):
            self.snapshot.odom_received_at = time.monotonic()

        def _on_raw_odom(self, _message):
            self.snapshot.raw_odom_received_at = time.monotonic()

        def _on_voltage(self, message):
            self.snapshot.voltage_received_at = time.monotonic()
            self.snapshot.voltage = float(message.data)

        def _on_scan(self, _message):
            self.snapshot.scan_received_at = time.monotonic()

        def _on_depth_status(self, message):
            self.snapshot.depth_status = message.data.strip()

        def _on_depth_points(self, message):
            self.snapshot.depth_points += 1
            self.snapshot.depth_points_received_at = time.monotonic()
            self.snapshot.depth_frame = message.header.frame_id
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            if stamp_ns > 0:
                self.snapshot.capture_age_sec = (
                    self.get_clock().now().nanoseconds - stamp_ns) / 1e9

        def _on_depth_scan(self, message):
            self.snapshot.depth_scan += 1
            self.snapshot.depth_scan_received_at = time.monotonic()
            self.snapshot.scan_frame = message.header.frame_id

        def _on_local_costmap(self, _message):
            if self._observation_has_arrived():
                self.snapshot.local_costmap += 1

        def _on_global_costmap(self, _message):
            if self._observation_has_arrived():
                self.snapshot.global_costmap += 1

        def _observation_has_arrived(self) -> bool:
            if args.depth_camera:
                return self.snapshot.depth_scan_received_at is not None
            return self.snapshot.scan_received_at is not None

        def _store_lifecycle(self, name, future):
            try:
                self.snapshot.lifecycle[name] = future.result().current_state.id
            except Exception:
                self.snapshot.lifecycle.pop(name, None)

        def _store_safety_parameters(self, future):
            try:
                values = future.result().values
                if len(values) != len(safety_parameter_names) or any(
                        value.type == ParameterType.PARAMETER_NOT_SET
                        for value in values):
                    return
                self.snapshot.safety_requirements = SafetyRequirements(
                    require_scan=values[0].bool_value,
                    scan_timeout_sec=values[1].double_value,
                    require_odom=values[2].bool_value,
                    odom_timeout_sec=values[3].double_value,
                    require_raw_odom=values[4].bool_value,
                    raw_odom_timeout_sec=values[5].double_value,
                    require_depth_points=values[6].bool_value,
                    depth_points_timeout_sec=values[7].double_value,
                    minimum_voltage=values[8].double_value,
                    voltage_timeout_sec=values[9].double_value,
                )
            except Exception:
                return

        def _store_costmap_parameters(self, scope, future):
            try:
                values = future.result().values
                if len(values) != len(costmap_parameter_names) or any(
                        value.type == ParameterType.PARAMETER_NOT_SET
                        for value in values):
                    return
                self.snapshot.costmap_sources[scope] = values[0].string_value.strip()
                self.snapshot.costmap_topics[scope] = values[1].string_value.strip()
            except Exception:
                return

        def _poll_lifecycle(self, now_sec):
            if now_sec - self._last_lifecycle_poll_at < 1.0:
                return
            self._last_lifecycle_poll_at = now_sec
            for name, client in self._lifecycle_clients.items():
                if self.snapshot.lifecycle.get(name) == ACTIVE_STATE_ID:
                    continue
                future = self._lifecycle_futures.get(name)
                if future is not None and not future.done():
                    continue
                if not client.service_is_ready():
                    continue
                future = client.call_async(GetState.Request())
                future.add_done_callback(
                    lambda completed, node_name=name:
                    self._store_lifecycle(node_name, completed))
                self._lifecycle_futures[name] = future

        def _poll_parameters(self, now_sec):
            if now_sec - self._last_parameter_poll_at < 1.0:
                return
            self._last_parameter_poll_at = now_sec
            safety_future = self._parameter_futures.get("safety")
            if (not self.snapshot.safety_requirements.is_complete()
                    and (safety_future is None or safety_future.done())
                    and self._safety_parameter_client.service_is_ready()):
                request = GetParameters.Request()
                request.names = list(safety_parameter_names)
                safety_future = self._safety_parameter_client.call_async(request)
                safety_future.add_done_callback(self._store_safety_parameters)
                self._parameter_futures["safety"] = safety_future
            for scope, client in self._costmap_parameter_clients.items():
                future = self._parameter_futures.get(scope)
                if (scope in self.snapshot.costmap_sources
                        and scope in self.snapshot.costmap_topics):
                    continue
                if future is not None and not future.done():
                    continue
                if not client.service_is_ready():
                    continue
                request = GetParameters.Request()
                request.names = list(costmap_parameter_names)
                future = client.call_async(request)
                future.add_done_callback(
                    lambda completed, costmap_scope=scope:
                    self._store_costmap_parameters(costmap_scope, completed))
                self._parameter_futures[scope] = future

        def _poll_task_services(self):
            self.snapshot.task_services = {
                name: client.service_is_ready()
                for name, client in self._task_clients.items()
            }

        def _tick(self):
            now_sec = time.monotonic()
            if args.launch_pid and not _process_is_running(args.launch_pid):
                print("NOT_READY launch_exited", flush=True)
                self.exit_code = 1
                return
            if self.snapshot.rviz_required:
                self.snapshot.rviz_running = _process_is_running(args.rviz_pid)
            self._poll_lifecycle(now_sec)
            self._poll_parameters(now_sec)
            self._poll_task_services()
            missing = missing_ready_items(
                self.snapshot, args.depth_camera, now_sec)
            report = summary(self.snapshot, now_sec)
            if (report != self._last_report
                    and now_sec - self._last_report_at >= 1.0):
                elapsed = args.timeout - max(0.0, self.deadline - now_sec)
                print(f"[startup {elapsed:.1f}s] {report}", flush=True)
                self._last_report = report
                self._last_report_at = now_sec
            if not missing:
                print(f"READY {report}", flush=True)
                self.exit_code = 0
                return
            if now_sec >= self.deadline:
                print(
                    f"NOT_READY missing={','.join(missing)} {report}",
                    flush=True)
                self.exit_code = 1

    rclpy.init()
    node = StartupStatusNode()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.25)
        return node.exit_code if node.exit_code is not None else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    try:
        return run(parse_args(argv))
    except ValueError as error:
        print(f"invalid startup status arguments: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
