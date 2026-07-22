#!/usr/bin/env python3
"""Monitor odometry pipeline health — rates, latency, and EKF diagnostics.

Usage (on RDK):
  ros2 run smartcar_tools odom_diag
  ros2 run smartcar_tools odom_diag --ros-args -p duration_sec:=30.0
"""

import math
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class OdomDiagNode(Node):
    """Accumulate per-topic statistics and print a summary on shutdown."""

    def __init__(self):
        super().__init__("odom_diag")
        self.declare_parameter("duration_sec", 15.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_combined_topic", "/odom_combined")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_laser_topic", "/odom_laser")
        self.declare_parameter("diagnostics_topic", "/diagnostics")

        self._duration = max(2.0, float(self.get_parameter("duration_sec").value))

        # per-topic counters: {topic: [timestamps]}
        self._arrivals = {}

        for name, topic in [
            ("odom", self.get_parameter("odom_topic").value),
            ("odom_combined", self.get_parameter("odom_combined_topic").value),
            ("scan", self.get_parameter("scan_topic").value),
            ("odom_laser", self.get_parameter("odom_laser_topic").value),
        ]:
            self._arrivals[topic] = []
            if name == "scan":
                self.create_subscription(
                    LaserScan,
                    topic,
                    lambda msg, t=topic: self._record(t),
                    qos_profile_sensor_data,
                )
            else:
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda msg, t=topic: self._record(t),
                    10,
                )

        self._diag_msgs = []
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            self._on_diagnostics,
            10,
        )

        self._start = time.monotonic()
        self._shutdown_timer = self.create_timer(self._duration, self._finish)
        self.get_logger().info(
            f"Monitoring {len(self._arrivals)} topics for {self._duration:.0f}s..."
        )

    def _record(self, topic):
        self._arrivals[topic].append(time.monotonic())

    def _on_diagnostics(self, msg):
        self._diag_msgs.append(msg)

    def _finish(self):
        elapsed = time.monotonic() - self._start
        print(f"\n{'='*60}")
        print(f" Odometry Pipeline Diagnostics  ({elapsed:.1f}s sample)")
        print(f"{'='*60}")

        for topic, stamps in sorted(self._arrivals.items()):
            count = len(stamps)
            if count < 2:
                print(f"\n  {topic}: {count} msgs (insufficient data)")
                continue

            rate = count / elapsed
            gaps = [stamps[i] - stamps[i - 1] for i in range(1, count)]
            min_gap = min(gaps) * 1000
            max_gap = max(gaps) * 1000
            mean_gap = sum(gaps) / len(gaps) * 1000

            # Compute stddev
            variance = sum((g * 1000 - mean_gap) ** 2 for g in gaps) / len(gaps)
            stddev = math.sqrt(variance)

            status = "OK"
            if max_gap > 500:
                status = "WARN: gaps >500ms"
            elif max_gap > 250:
                status = "NOTE: gaps >250ms"

            print(f"\n  {topic}:")
            print(f"    Count:  {count}  |  Rate:  {rate:.1f} Hz")
            print(f"    Gap:    min={min_gap:.1f}  max={max_gap:.1f}  "
                  f"mean={mean_gap:.1f}  σ={stddev:.1f} ms")
            print(f"    Status: {status}")

        # ── EKF diagnostics ──────────────────────────────────────
        ekf_warnings = []
        for diag in self._diag_msgs:
            for status in diag.status:
                if "ekf" in status.name.lower():
                    if status.level == DiagnosticStatus.WARN:
                        ekf_warnings.append(f"{status.name}: {status.message}")
                    elif status.level == DiagnosticStatus.ERROR:
                        ekf_warnings.append(f"ERROR {status.name}: {status.message}")
                    elif status.level == DiagnosticStatus.STALE:
                        ekf_warnings.append(f"STALE {status.name}: {status.message}")

        if ekf_warnings:
            print(f"\n  EKF Diagnostics ({len(ekf_warnings)} issues):")
            for w in ekf_warnings:
                print(f"    ⚠ {w}")
        else:
            print(f"\n  EKF Diagnostics: no warnings/errors observed")

        # ── Summary judgment ──────────────────────────────────────
        print(f"\n{'─'*60}")
        odom_rate = len(self._arrivals.get("/odom", [])) / elapsed if elapsed > 0 else 0
        ekf_rate = len(self._arrivals.get("/odom_combined", [])) / elapsed if elapsed > 0 else 0
        scan_rate = len(self._arrivals.get("/scan", [])) / elapsed if elapsed > 0 else 0

        print(f"  Summary: odom={odom_rate:.1f}Hz  ekf={ekf_rate:.1f}Hz  "
              f"scan={scan_rate:.1f}Hz  ekf_warnings={len(ekf_warnings)}")
        print(f"{'─'*60}")

        if ekf_warnings:
            print(f"\n  ⚠  EKF has warnings — check above for details.")
            print(f"  Likely cause: CPU contention at higher speed OR")
            print(f"  serial frame gaps exceeding EKF sensor_timeout (0.25s).")
        elif odom_rate < 20:
            print(f"\n  ⚠  /odom rate ({odom_rate:.1f} Hz) is below 20 Hz.")
            print(f"  STM32 may be sending frames too slowly.")
        else:
            print(f"\n  ✅ Pipeline looks healthy at current speed.")

        print()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OdomDiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
