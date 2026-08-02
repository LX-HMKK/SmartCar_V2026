#!/usr/bin/env python3
"""Fail-closed Gazebo sensor preflight used before Nav2 starts.

This process is deliberately short-lived and publishes no motion command.  It
waits for a progressing simulation clock, finite Gazebo odometry relayed into
``/odom_combined``, and a non-self LaserScan that is temporally aligned with
that odometry.  A headless Ogre failure otherwise looks like a valid ROS graph
while every scan range is the sensor minimum, which lets Nav2 start without
the obstacles visible in Gazebo.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


NSEC_PER_SEC = 1_000_000_000


def stamp_ns(stamp: object) -> int:
    """Convert a ROS builtin timestamp to nanoseconds."""
    return int(stamp.sec) * NSEC_PER_SEC + int(stamp.nanosec)


def valid_scan_beam_count(
    scan: LaserScan, minimum_range: float, maximum_range: float
) -> int:
    """Count obstacle returns that cannot be the lidar's self-observation."""
    lower = max(minimum_range, float(scan.range_min))
    upper = min(maximum_range, float(scan.range_max))
    return sum(
        1
        for value in scan.ranges
        if math.isfinite(value) and lower < value < upper
    )


class SimSensorPreflight(Node):
    """Observe the minimum sensor chain required before Nav2 may start."""

    def __init__(self) -> None:
        super().__init__("sim_sensor_preflight")
        self.declare_parameter("timeout_sec", 35.0)
        self.declare_parameter("min_obstacle_range_m", 0.25)
        self.declare_parameter("max_obstacle_range_m", 2.5)
        self.declare_parameter("min_valid_beams", 5)
        self.declare_parameter("max_scan_odom_skew_sec", 0.075)
        self.declare_parameter("max_sensor_age_sec", 0.35)
        # A single synchronized message can be a stale bridge sample just
        # before Gazebo's sensor systems settle. Require a sustained stream so
        # Nav2 never starts on a false-positive preflight.
        self.declare_parameter("min_sensor_stream_span_sec", 1.0)
        self.declare_parameter("min_odom_samples", 15)
        self.declare_parameter("min_scan_samples", 5)
        # The first preflight proves Gazebo is publishing sensor data. A second
        # invocation runs after static sensor TF is available and requires this
        # exact scan-time lookup before Nav2 can create obstacle layers.
        self.declare_parameter("require_scan_time_tf", False)
        self.declare_parameter("tf_target_frame", "odom_combined")

        self._timeout_sec = self._positive_float("timeout_sec")
        self._minimum_range = self._positive_float("min_obstacle_range_m")
        self._maximum_range = self._positive_float("max_obstacle_range_m")
        if self._maximum_range <= self._minimum_range:
            raise ValueError(
                "max_obstacle_range_m must exceed min_obstacle_range_m")
        self._min_valid_beams = int(
            self.get_parameter("min_valid_beams").value)
        if self._min_valid_beams <= 0:
            raise ValueError("min_valid_beams must be positive")
        self._max_scan_odom_skew_ns = int(
            self._positive_float("max_scan_odom_skew_sec") * NSEC_PER_SEC)
        self._max_sensor_age_ns = int(
            self._positive_float("max_sensor_age_sec") * NSEC_PER_SEC)
        self._min_stream_span_ns = int(
            self._positive_float("min_sensor_stream_span_sec") * NSEC_PER_SEC)
        self._min_odom_samples = int(self.get_parameter("min_odom_samples").value)
        self._min_scan_samples = int(self.get_parameter("min_scan_samples").value)
        self._require_scan_time_tf = bool(
            self.get_parameter("require_scan_time_tf").value)
        self._tf_target_frame = str(
            self.get_parameter("tf_target_frame").value).strip()
        if self._min_odom_samples <= 0 or self._min_scan_samples <= 0:
            raise ValueError("minimum sensor sample counts must be positive")
        if self._require_scan_time_tf and not self._tf_target_frame:
            raise ValueError("tf_target_frame must be non-empty when scan TF is required")

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # ROS 2's /clock publisher uses the clock/sensor best-effort profile
        # in Humble. A reliable subscriber is incompatible with that writer
        # and would make the preflight fail even when Gazebo is healthy.
        self.create_subscription(
            Clock, "/clock", self._clock_callback, sensor_qos)
        self.create_subscription(
            Odometry, "/odom_combined", self._odom_callback, reliable_qos)
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, sensor_qos)

        self._last_clock_ns: int | None = None
        self._clock_advanced = False
        self._odom_stamp_ns: int | None = None
        self._scan_stamp_ns: int | None = None
        self._scan_frame_id: str | None = None
        self._valid_beams = 0
        self._first_odom_stamp_ns: int | None = None
        self._first_scan_stamp_ns: int | None = None
        self._odom_samples = 0
        self._scan_samples = 0
        self._tf_buffer: Buffer | None = None
        self._tf_listener: TransformListener | None = None
        if self._require_scan_time_tf:
            self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self._tf_listener = TransformListener(self._tf_buffer, self)

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a positive finite number")
        return value

    def _clock_callback(self, msg: Clock) -> None:
        current_ns = stamp_ns(msg.clock)
        if current_ns <= 0:
            return
        if self._last_clock_ns is not None and current_ns > self._last_clock_ns:
            self._clock_advanced = True
        self._last_clock_ns = current_ns

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if norm <= 1.0e-6:
            return
        received_stamp_ns = stamp_ns(msg.header.stamp)
        if received_stamp_ns > 0:
            if (
                self._odom_stamp_ns is None
                or received_stamp_ns > self._odom_stamp_ns
            ):
                if self._first_odom_stamp_ns is None:
                    self._first_odom_stamp_ns = received_stamp_ns
                self._odom_stamp_ns = received_stamp_ns
                self._odom_samples += 1

    def _scan_callback(self, msg: LaserScan) -> None:
        received_stamp_ns = stamp_ns(msg.header.stamp)
        if received_stamp_ns <= 0:
            return
        if self._scan_stamp_ns is None or received_stamp_ns > self._scan_stamp_ns:
            if self._first_scan_stamp_ns is None:
                self._first_scan_stamp_ns = received_stamp_ns
            self._scan_stamp_ns = received_stamp_ns
            self._scan_frame_id = str(msg.header.frame_id).strip()
            self._scan_samples += 1
            self._valid_beams = valid_scan_beam_count(
                msg, self._minimum_range, self._maximum_range)

    def _scan_time_tf_available(self) -> bool:
        """Return whether the latest scan frame transforms at its source time."""
        if not self._require_scan_time_tf:
            return True
        if (
            self._tf_buffer is None
            or self._scan_stamp_ns is None
            or not self._scan_frame_id
        ):
            return False
        try:
            self._tf_buffer.lookup_transform(
                self._tf_target_frame,
                self._scan_frame_id,
                Time(nanoseconds=self._scan_stamp_ns),
                timeout=Duration(seconds=0.0),
            )
        except (TransformException, ValueError):
            return False
        return True

    def _status(self) -> tuple[bool, str]:
        now_ns = self._last_clock_ns
        if not self._clock_advanced or now_ns is None:
            return False, "clock_not_advancing"
        if self._odom_stamp_ns is None:
            return False, "odom_missing"
        if self._scan_stamp_ns is None:
            return False, "scan_missing"
        if self._odom_samples < self._min_odom_samples:
            return False, "odom_stream_short"
        if self._scan_samples < self._min_scan_samples:
            return False, "scan_stream_short"
        if (
            self._first_odom_stamp_ns is None
            or self._first_scan_stamp_ns is None
            or self._odom_stamp_ns - self._first_odom_stamp_ns
            < self._min_stream_span_ns
            or self._scan_stamp_ns - self._first_scan_stamp_ns
            < self._min_stream_span_ns
        ):
            return False, "sensor_stream_short"
        if self._valid_beams < self._min_valid_beams:
            return False, "scan_has_no_nonself_returns"
        if (
            abs(self._scan_stamp_ns - self._odom_stamp_ns)
            > self._max_scan_odom_skew_ns
        ):
            return False, "scan_odom_skew"
        if now_ns < self._odom_stamp_ns or now_ns < self._scan_stamp_ns:
            return False, "sensor_stamp_ahead_of_clock"
        if now_ns - self._odom_stamp_ns > self._max_sensor_age_ns:
            return False, "odom_stale"
        if now_ns - self._scan_stamp_ns > self._max_sensor_age_ns:
            return False, "scan_stale"
        if not self._scan_time_tf_available():
            return False, "scan_time_tf_missing"
        return True, "ready"

    def wait(self) -> bool:
        deadline = time.monotonic() + self._timeout_sec
        last_reason = "waiting_for_topics"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready, reason = self._status()
            if ready:
                self.get_logger().info(
                    "sim sensor preflight READY "
                    f"(valid_beams={self._valid_beams}, "
                    f"clock={self._last_clock_ns}, "
                    f"odom={self._odom_stamp_ns}, scan={self._scan_stamp_ns}, "
                    f"scan_frame={self._scan_frame_id}, "
                    f"odom_samples={self._odom_samples}, "
                    f"scan_samples={self._scan_samples})"
                )
                return True
            last_reason = reason
        self.get_logger().error(
            "sim sensor preflight FAILED "
            f"({last_reason}, valid_beams={self._valid_beams}, "
            f"clock={self._last_clock_ns}, odom={self._odom_stamp_ns}, "
            f"scan={self._scan_stamp_ns}, scan_frame={self._scan_frame_id}, "
            f"odom_samples={self._odom_samples}, "
            f"scan_samples={self._scan_samples})"
        )
        return False


def main() -> int:
    rclpy.init()
    node = SimSensorPreflight()
    try:
        return 0 if node.wait() else 2
    except (KeyboardInterrupt, ExternalShutdownException):
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
