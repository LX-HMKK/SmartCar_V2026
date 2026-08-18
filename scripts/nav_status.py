#!/usr/bin/env python3
"""Collect physical navigation startup health in one bounded ROS process."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
import os
from pathlib import Path
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
VISION_SERVICES = ("qr", "vlm")
ACTIVE_STATE_ID = 3
POLL_INTERVAL_SEC = 0.2
RPC_TIMEOUT_SEC = 1.0
PREARM_RPC_TIMEOUT_SEC = 12.0
DEPTH_MIN_FRAMES = 2
MAX_DEPTH_CAPTURE_AGE_SEC = 0.25
MAX_DEPTH_FUTURE_SKEW_SEC = 0.05
DEPTH_POINTS_FRAME = "depth_camera_link_1"
DEPTH_SCAN_FRAME = "base_footprint"

LIFECYCLE_STATE_NAMES = {
    0: "unknown",
    1: "unconfigured",
    2: "inactive",
    3: "active",
    4: "finalized",
    10: "configuring",
    11: "cleaningup",
    12: "shuttingdown",
    13: "activating",
    14: "deactivating",
    15: "errorprocessing",
}


@dataclass
class PendingRpc:
    """One outstanding asynchronous service request and its monotonic start."""

    future: object
    started_at: float


def lifecycle_state_name(state_id: int | None) -> str:
    if state_id is None:
        return "missing"
    return LIFECYCLE_STATE_NAMES.get(state_id, f"state_{state_id}")


def inactive_lifecycle_nodes(snapshot: "StartupSnapshot") -> list[str]:
    return [
        f"{name}={lifecycle_state_name(snapshot.lifecycle.get(name))}"
        for name in LIFECYCLE_NODES
        if snapshot.lifecycle.get(name) != ACTIVE_STATE_ID
    ]


def take_current_rpc(
    pending_calls: dict[str, PendingRpc], key: str, future: object,
) -> PendingRpc | None:
    """Remove and return only the request still current for ``key``.

    A timed-out request can complete after its replacement was sent.  Its
    callback must not overwrite the newer response or clear the newer request.
    """

    pending = pending_calls.get(key)
    if pending is None or pending.future is not future:
        return None
    pending_calls.pop(key, None)
    return pending


def discard_timed_out_rpc(
    pending_calls: dict[str, PendingRpc], key: str, now_sec: float,
) -> PendingRpc | None:
    """Cancel and discard one expired RPC, returning it for diagnostics."""

    pending = pending_calls.get(key)
    if pending is None or pending.future.done():
        return None
    if now_sec - pending.started_at < RPC_TIMEOUT_SEC:
        return None

    # Drop it before cancellation because cancel() may synchronously invoke its
    # callback.  A late callback is then ignored by take_current_rpc().
    pending_calls.pop(key, None)
    try:
        pending.future.cancel()
    except Exception:
        pass
    return pending


def service_response_succeeded(response: object) -> bool:
    """Return whether a std_srvs response explicitly accepted the request."""

    return bool(getattr(response, "success", False))


class StartupTimelineLogger:
    """Emit startup diagnostics to stdout and, optionally, a private append log."""

    def __init__(self, log_path: str | None):
        self._path = Path(log_path) if log_path else None
        self._write_failed = False

    def emit(self, message: str) -> None:
        # ROS status strings are external input. Keep a malformed value from
        # injecting extra lines into the persistent diagnostic log.
        line = message.replace("\r", "\\r").replace("\n", "\\n")
        print(line, flush=True)
        if self._path is None or self._write_failed:
            return

        try:
            self._path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT
            if hasattr(os, "O_CLOEXEC"):
                flags |= os.O_CLOEXEC
            if hasattr(os, "O_NOFOLLOW"):
                flags |= os.O_NOFOLLOW
            descriptor = os.open(self._path, flags, 0o600)
            try:
                entry = (
                    f"{time.strftime('%Y-%m-%dT%H:%M:%S%z')} {line}\n"
                ).encode("utf-8", "replace")
                if os.write(descriptor, entry) != len(entry):
                    raise OSError("short startup timeline write")
            finally:
                os.close(descriptor)
        except OSError:
            self._write_failed = True
            print("WARNING startup timeline log append unavailable", flush=True)


@dataclass
class SafetyRequirements:
    """Runtime safety limits queried from the active safety node."""

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
    vision_services: dict[str, bool] = field(default_factory=dict)
    vision_required: bool = False
    qr_reader_ready: bool | None = None
    qr_reader_required: bool = False
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
    require_vision: bool = False,
    require_qr_reader: bool = False,
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
    if require_vision:
        for service in VISION_SERVICES:
            if not snapshot.vision_services.get(service, False):
                missing.append(f"vision_service={service}")
    if require_qr_reader and snapshot.qr_reader_ready is not True:
        missing.append("qr_reader")
    missing.extend(_safety_input_missing(snapshot, now_sec))

    expected_source = "depth_scan"
    expected_topic = "/smartcar/depth/scan"
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
    lifecycle_waiting = ",".join(inactive_lifecycle_nodes(snapshot)) or "none"
    nav2_state = "active" if active_count == len(LIFECYCLE_NODES) else (
        f"waiting[{lifecycle_waiting}]")
    task_services = "+".join(
        name for name in TASK_SERVICES
        if snapshot.task_services.get(name, False)) or "waiting"
    vision = "not_requested"
    if snapshot.vision_required:
        vision = "+".join(
            name for name in VISION_SERVICES
            if snapshot.vision_services.get(name, False)) or "waiting"
    qr_reader = "not_requested"
    if snapshot.qr_reader_required:
        qr_reader = "ready" if snapshot.qr_reader_ready else "waiting"
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
        f"nav2={nav2_state} "
        f"safety={snapshot.safety or 'waiting'} "
        f"guard={snapshot.direction_guard or 'waiting'} "
        f"task={snapshot.task or 'waiting'} services={task_services} "
        f"vision={vision} qr_reader={qr_reader} "
        f"costmaps={snapshot.local_costmap}/{snapshot.global_costmap} "
        f"sources={sources} depth=[{depth}] capture_age={capture_age} "
        f"inputs=odom:{_age_text(snapshot.odom_received_at, now_sec)},"
        f"raw:{_age_text(snapshot.raw_odom_received_at, now_sec)},"
        f"voltage:{_age_text(snapshot.voltage_received_at, now_sec)} rviz={rviz}"
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
    parser.add_argument(
        "--vision-services",
        action="store_true",
        help="Require the QR and VLM services to be available",
    )
    parser.add_argument(
        "--preloaded-qr-reader",
        action="store_true",
        help="Require a launch-managed zbar publisher on /barcode",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--launch-pid", type=int, default=0)
    parser.add_argument("--rviz-pid", type=int, default=0)
    parser.add_argument(
        "--prearm",
        action="store_true",
        help=(
            "After the health snapshot, confirm the software stop and reset "
            "the task origin in this same ROS process"
        ),
    )
    parser.add_argument(
        "--timeline-log",
        help="Append startup progress and final diagnostics to this local log file",
    )
    return parser.parse_args(argv)


def run(args) -> int:
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise ValueError("timeout must be finite and positive")
    if args.launch_pid < 0 or args.rviz_pid < 0:
        raise ValueError("process IDs must be non-negative")
    if args.preloaded_qr_reader and not args.vision_services:
        raise ValueError("--preloaded-qr-reader requires --vision-services")
    timeline_log = getattr(args, "timeline_log", None)
    prearm = getattr(args, "prearm", False)

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
    from std_srvs.srv import SetBool, Trigger
    if args.vision_services:
        from smartcar_interfaces.srv import DescribeScene, ReadQr

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
        "obstacle_layer.depth_scan.topic",
    )

    class StartupStatusNode(Node):
        def __init__(self):
            super().__init__("nav_startup_status")
            self.snapshot = StartupSnapshot(
                rviz_required=args.rviz_pid > 0,
                vision_required=args.vision_services,
                qr_reader_required=args.preloaded_qr_reader,
            )
            self.started_at = time.monotonic()
            self.deadline = self.started_at + args.timeout
            self.exit_code: int | None = None
            self._timeline = StartupTimelineLogger(timeline_log)
            self._last_report = ""
            self._last_report_at = self.started_at - 1.0
            self._last_lifecycle_poll_at = 0.0
            self._last_parameter_poll_at = 0.0
            self._lifecycle_futures: dict[str, PendingRpc] = {}
            self._parameter_futures: dict[str, PendingRpc] = {}
            self._prearm_futures: dict[str, PendingRpc] = {}
            self._prearm_phase = "health" if prearm else "disabled"
            self._prearm_error = ""
            self._lifecycle_clients = {
                name: self.create_client(GetState, f"/{name}/get_state")
                for name in LIFECYCLE_NODES
            }
            self._task_clients = {
                name: self.create_client(Trigger, f"/smartcar/task/{name}")
                for name in TASK_SERVICES
            }
            self._emergency_stop_client = None
            if prearm:
                self._emergency_stop_client = self.create_client(
                    SetBool, "/smartcar/safety/emergency_stop")
            self._vision_clients = {}
            if args.vision_services:
                self._vision_clients = {
                    "qr": self.create_client(
                        ReadQr, "/smartcar/vision/read_qr"),
                    "vlm": self.create_client(
                        DescribeScene, "/smartcar/vision/describe_scene"),
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
            self._emit(
                "BEGIN "
                f"timeout={args.timeout:.1f}s launch_pid={args.launch_pid or '-'} "
                f"depth_camera={args.depth_camera} "
                f"vision_services={args.vision_services} prearm={prearm}",
                self.started_at,
            )

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
            return self.snapshot.depth_scan_received_at is not None

        def _emit(self, message: str, now_sec: float | None = None):
            if now_sec is None:
                now_sec = time.monotonic()
            elapsed = max(0.0, now_sec - self.started_at)
            self._timeline.emit(f"[startup {elapsed:.1f}s] {message}")

        def _discard_expired_rpc(
            self,
            pending_calls: dict[str, PendingRpc],
            key: str,
            target: str,
            now_sec: float,
        ) -> bool:
            expired = discard_timed_out_rpc(pending_calls, key, now_sec)
            if expired is None:
                return False
            self._emit(
                f"rpc_timeout target={target} age="
                f"{now_sec - expired.started_at:.2f}s retrying",
                now_sec,
            )
            return True

        def _start_rpc(
            self,
            pending_calls: dict[str, PendingRpc],
            key: str,
            target: str,
            client,
            request,
            callback,
        ) -> bool:
            started_at = time.monotonic()
            try:
                future = client.call_async(request)
            except Exception:
                self._emit(f"rpc_request_failed target={target}", started_at)
                return False
            pending_calls[key] = PendingRpc(future=future, started_at=started_at)
            future.add_done_callback(callback)
            return True

        def _fail_prearm(self, reason: str, now_sec: float):
            if self._prearm_error:
                return
            self._prearm_error = reason
            self._emit(f"PREARM_FAILED reason={reason}", now_sec)

        def _discard_expired_prearm_rpc(
            self, key: str, target: str, now_sec: float,
        ) -> bool:
            pending = self._prearm_futures.get(key)
            if pending is None or pending.future.done():
                return False
            age_sec = now_sec - pending.started_at
            if age_sec < PREARM_RPC_TIMEOUT_SEC:
                return False

            # A reset may already have reached the task node when its response
            # times out.  Do not issue it again; retain the startup stop and
            # require an explicit operator re-arm instead.
            self._prearm_futures.pop(key, None)
            try:
                pending.future.cancel()
            except Exception:
                pass
            self._fail_prearm(f"{target}_timeout", now_sec)
            return True

        def _store_prearm_stop_latch(self, future):
            if take_current_rpc(
                    self._prearm_futures, "emergency_stop", future) is None:
                return
            try:
                response = future.result()
            except Exception:
                self._fail_prearm(
                    "emergency_stop_response_error", time.monotonic())
                return
            if not service_response_succeeded(response):
                self._fail_prearm(
                    "emergency_stop_rejected", time.monotonic())
                return
            self._prearm_phase = "reset"
            self._emit("PREARM_STOP_LATCHED")

        def _store_prearm_reset(self, future):
            if take_current_rpc(self._prearm_futures, "reset", future) is None:
                return
            try:
                response = future.result()
            except Exception:
                self._fail_prearm("origin_reset_response_error", time.monotonic())
                return
            if not service_response_succeeded(response):
                self._fail_prearm("origin_reset_rejected", time.monotonic())
                return
            self._prearm_phase = "complete"
            self._emit("PREARM_ORIGIN_RESET_COMPLETE")

        def _advance_prearm(self, now_sec: float):
            if self._prearm_error or self._prearm_phase == "complete":
                return
            if self._prearm_phase == "health":
                if not self._emergency_stop_client.service_is_ready():
                    return
                request = SetBool.Request()
                request.data = True
                if not self._start_rpc(
                        self._prearm_futures,
                        "emergency_stop",
                        "prearm:emergency_stop",
                        self._emergency_stop_client,
                        request,
                        self._store_prearm_stop_latch,
                ):
                    self._fail_prearm("emergency_stop_request_failed", now_sec)
                    return
                self._prearm_phase = "latching_stop"
                self._emit("PREARM_STOP_LATCHING", now_sec)
                return
            if self._prearm_phase == "latching_stop":
                pending = self._prearm_futures.get("emergency_stop")
                if pending is not None and pending.future.done():
                    self._store_prearm_stop_latch(pending.future)
                else:
                    self._discard_expired_prearm_rpc(
                        "emergency_stop", "emergency_stop", now_sec)
                return
            if self._prearm_phase == "reset":
                reset_client = self._task_clients["reset"]
                if not reset_client.service_is_ready():
                    return
                if not self._start_rpc(
                        self._prearm_futures,
                        "reset",
                        "prearm:origin_reset",
                        reset_client,
                        Trigger.Request(),
                        self._store_prearm_reset,
                ):
                    self._fail_prearm("origin_reset_request_failed", now_sec)
                    return
                self._prearm_phase = "resetting_origin"
                self._emit("PREARM_ORIGIN_RESETTING", now_sec)
                return
            if self._prearm_phase == "resetting_origin":
                pending = self._prearm_futures.get("reset")
                if pending is not None and pending.future.done():
                    self._store_prearm_reset(pending.future)
                else:
                    self._discard_expired_prearm_rpc(
                        "reset", "origin_reset", now_sec)
                return

        def _store_lifecycle(self, name, future):
            if take_current_rpc(self._lifecycle_futures, name, future) is None:
                return
            try:
                state_id = future.result().current_state.id
            except Exception:
                if name in self.snapshot.lifecycle:
                    self.snapshot.lifecycle.pop(name, None)
                self._emit(f"lifecycle_rpc_error node={name}")
                return
            previous_state = self.snapshot.lifecycle.get(name)
            self.snapshot.lifecycle[name] = state_id
            if previous_state != state_id:
                self._emit(
                    f"lifecycle node={name} "
                    f"state={lifecycle_state_name(state_id)}")

        def _store_safety_parameters(self, future):
            if take_current_rpc(self._parameter_futures, "safety", future) is None:
                return
            try:
                values = future.result().values
                if len(values) != len(safety_parameter_names) or any(
                        value.type == ParameterType.PARAMETER_NOT_SET
                        for value in values):
                    return
                self.snapshot.safety_requirements = SafetyRequirements(
                    require_odom=values[0].bool_value,
                    odom_timeout_sec=values[1].double_value,
                    require_raw_odom=values[2].bool_value,
                    raw_odom_timeout_sec=values[3].double_value,
                    require_depth_points=values[4].bool_value,
                    depth_points_timeout_sec=values[5].double_value,
                    minimum_voltage=values[6].double_value,
                    voltage_timeout_sec=values[7].double_value,
                )
            except Exception:
                self._emit("parameters_rpc_error target=safety")
                return

        def _store_costmap_parameters(self, scope, future):
            if take_current_rpc(
                    self._parameter_futures, scope, future) is None:
                return
            try:
                values = future.result().values
                if len(values) != len(costmap_parameter_names) or any(
                        value.type == ParameterType.PARAMETER_NOT_SET
                        for value in values):
                    return
                self.snapshot.costmap_sources[scope] = values[0].string_value.strip()
                self.snapshot.costmap_topics[scope] = values[1].string_value.strip()
            except Exception:
                self._emit(f"parameters_rpc_error target={scope}_costmap")
                return

        def _poll_lifecycle(self, now_sec):
            if now_sec - self._last_lifecycle_poll_at < POLL_INTERVAL_SEC:
                return
            self._last_lifecycle_poll_at = now_sec
            for name, client in self._lifecycle_clients.items():
                if self.snapshot.lifecycle.get(name) == ACTIVE_STATE_ID:
                    continue
                pending = self._lifecycle_futures.get(name)
                if pending is not None:
                    if pending.future.done():
                        self._store_lifecycle(name, pending.future)
                    elif not self._discard_expired_rpc(
                            self._lifecycle_futures,
                            name,
                            f"lifecycle:{name}",
                            now_sec):
                        continue
                if self.snapshot.lifecycle.get(name) == ACTIVE_STATE_ID:
                    continue
                if not client.service_is_ready():
                    continue
                self._start_rpc(
                    self._lifecycle_futures,
                    name,
                    f"lifecycle:{name}",
                    client,
                    GetState.Request(),
                    lambda completed, node_name=name:
                    self._store_lifecycle(node_name, completed),
                )

        def _poll_parameters(self, now_sec):
            if now_sec - self._last_parameter_poll_at < POLL_INTERVAL_SEC:
                return
            self._last_parameter_poll_at = now_sec
            safety_pending = self._parameter_futures.get("safety")
            if safety_pending is not None:
                if safety_pending.future.done():
                    self._store_safety_parameters(safety_pending.future)
                else:
                    self._discard_expired_rpc(
                        self._parameter_futures,
                        "safety",
                        "parameters:safety",
                        now_sec,
                    )
            if (not self.snapshot.safety_requirements.is_complete()
                    and "safety" not in self._parameter_futures
                    and self._safety_parameter_client.service_is_ready()):
                request = GetParameters.Request()
                request.names = list(safety_parameter_names)
                self._start_rpc(
                    self._parameter_futures,
                    "safety",
                    "parameters:safety",
                    self._safety_parameter_client,
                    request,
                    self._store_safety_parameters,
                )
            for scope, client in self._costmap_parameter_clients.items():
                pending = self._parameter_futures.get(scope)
                if pending is not None:
                    if pending.future.done():
                        self._store_costmap_parameters(scope, pending.future)
                    else:
                        self._discard_expired_rpc(
                            self._parameter_futures,
                            scope,
                            f"parameters:{scope}_costmap",
                            now_sec,
                        )
                if (scope in self.snapshot.costmap_sources
                        and scope in self.snapshot.costmap_topics):
                    continue
                if scope in self._parameter_futures:
                    continue
                if not client.service_is_ready():
                    continue
                request = GetParameters.Request()
                request.names = list(costmap_parameter_names)
                self._start_rpc(
                    self._parameter_futures,
                    scope,
                    f"parameters:{scope}_costmap",
                    client,
                    request,
                    lambda completed, costmap_scope=scope:
                    self._store_costmap_parameters(costmap_scope, completed),
                )

        def _poll_task_services(self):
            self.snapshot.task_services = {
                name: client.service_is_ready()
                for name, client in self._task_clients.items()
            }

        def _poll_vision_services(self):
            if not self._vision_clients:
                return
            self.snapshot.vision_services = {
                name: client.service_is_ready()
                for name, client in self._vision_clients.items()
            }
            if args.preloaded_qr_reader:
                self.snapshot.qr_reader_ready = self.count_publishers(
                    "/barcode") > 0

        def _tick(self):
            now_sec = time.monotonic()
            if args.launch_pid and not _process_is_running(args.launch_pid):
                report = summary(self.snapshot, now_sec)
                self._emit(
                    f"NOT_READY missing=launch_exited {report}", now_sec)
                self.exit_code = 1
                return
            if self.snapshot.rviz_required:
                self.snapshot.rviz_running = _process_is_running(args.rviz_pid)
            self._poll_lifecycle(now_sec)
            self._poll_parameters(now_sec)
            self._poll_task_services()
            self._poll_vision_services()
            missing = missing_ready_items(
                self.snapshot,
                args.depth_camera,
                now_sec,
                require_vision=args.vision_services,
                require_qr_reader=args.preloaded_qr_reader,
            )
            report = summary(self.snapshot, now_sec)
            progress = f"{report} missing={','.join(missing) or 'none'}"
            if (progress != self._last_report
                    and now_sec - self._last_report_at >= 1.0):
                self._emit(progress, now_sec)
                self._last_report = progress
                self._last_report_at = now_sec
            if prearm:
                if self._prearm_phase == "health":
                    if not missing:
                        self._advance_prearm(now_sec)
                else:
                    self._advance_prearm(now_sec)
                if self._prearm_error:
                    self._emit(
                        f"NOT_READY prearm={self._prearm_error} {report}",
                        now_sec)
                    self.exit_code = 1
                    return
                if self._prearm_phase == "complete":
                    self._emit(f"PREARM_READY {report} missing=none", now_sec)
                    self.exit_code = 0
                    return
            elif not missing:
                self._emit(f"READY {report} missing=none", now_sec)
                self.exit_code = 0
                return
            if now_sec >= self.deadline:
                if prearm and self._prearm_phase != "health":
                    self._fail_prearm(
                        f"{self._prearm_phase}_deadline", now_sec)
                    self._emit(
                        f"NOT_READY prearm={self._prearm_error} {report}",
                        now_sec)
                    self.exit_code = 1
                    return
                self._emit(
                    f"NOT_READY missing={','.join(missing)} {report}",
                    now_sec)
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
