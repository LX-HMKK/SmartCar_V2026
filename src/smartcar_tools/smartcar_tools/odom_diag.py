#!/usr/bin/env python3
"""Measure odometry-pipeline arrival rates and EKF diagnostics."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class OdomDiagNode(Node):
    """Collect bounded-duration topic statistics without stopping from a callback."""

    def __init__(self):
        super().__init__("odom_diag")
        self.declare_parameter("duration_sec", 15.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("odom_combined_topic", "/odom_combined")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_laser_topic", "/odom_laser")
        self.declare_parameter("diagnostics_topic", "/diagnostics")

        self._duration = max(2.0, float(self.get_parameter("duration_sec").value))
        self._topics = {
            "odom": str(self.get_parameter("odom_topic").value),
            "imu": str(self.get_parameter("imu_topic").value),
            "odom_combined": str(
                self.get_parameter("odom_combined_topic").value
            ),
            "scan": str(self.get_parameter("scan_topic").value),
            "odom_laser": str(self.get_parameter("odom_laser_topic").value),
        }
        self._arrivals = {name: [] for name in self._topics}
        self._ekf_warnings = set()
        self.finished = False
        self._start = time.monotonic()

        message_types = {
            "odom": Odometry,
            "imu": Imu,
            "odom_combined": Odometry,
            "scan": LaserScan,
            "odom_laser": Odometry,
        }
        for name, topic in self._topics.items():
            qos = qos_profile_sensor_data if name in {"imu", "scan"} else 10
            self.create_subscription(
                message_types[name],
                topic,
                lambda _message, key=name: self._record(key),
                qos,
            )

        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            self._on_diagnostics,
            10,
        )
        self._finish_timer = self.create_timer(self._duration, self.finish)
        self.get_logger().info(
            f"Monitoring {len(self._topics)} topics for {self._duration:.0f}s..."
        )

    def _record(self, name):
        now = time.monotonic()
        self._arrivals[name].append(now)
        # A busy subscription queue must not be able to starve completion.
        if now - self._start >= self._duration:
            self.finish()

    def _on_diagnostics(self, message):
        for status in message.status:
            if "ekf" not in status.name.lower():
                continue
            if status.level == DiagnosticStatus.WARN:
                self._ekf_warnings.add(f"{status.name}: {status.message}")
            elif status.level == DiagnosticStatus.ERROR:
                self._ekf_warnings.add(f"ERROR {status.name}: {status.message}")
            elif status.level == DiagnosticStatus.STALE:
                self._ekf_warnings.add(f"STALE {status.name}: {status.message}")

    @staticmethod
    def _rate(stamps):
        if len(stamps) < 2 or stamps[-1] <= stamps[0]:
            return 0.0
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])

    def finish(self):
        if self.finished:
            return

        elapsed = time.monotonic() - self._start
        print("\n" + "=" * 60, flush=True)
        print(f" Odometry Pipeline Diagnostics ({elapsed:.1f}s sample)", flush=True)
        print("=" * 60, flush=True)

        for name in sorted(self._arrivals):
            topic = self._topics[name]
            stamps = self._arrivals[name]
            count = len(stamps)
            if count < 2:
                print(f"\n  {topic}: {count} msgs (insufficient data)", flush=True)
                continue

            gaps_ms = [
                (current - previous) * 1000.0
                for previous, current in zip(stamps, stamps[1:])
            ]
            mean_gap = sum(gaps_ms) / len(gaps_ms)
            variance = sum(
                (gap - mean_gap) ** 2 for gap in gaps_ms
            ) / len(gaps_ms)
            max_gap = max(gaps_ms)
            status = "OK"
            if max_gap > 500.0:
                status = "WARN: gaps >500ms"
            elif max_gap > 250.0:
                status = "NOTE: gaps >250ms"

            print(f"\n  {topic}:", flush=True)
            print(
                f"    Count: {count} | Rate: {self._rate(stamps):.1f} Hz",
                flush=True,
            )
            print(
                "    Gap: "
                f"min={min(gaps_ms):.1f} max={max_gap:.1f} "
                f"mean={mean_gap:.1f} stddev={math.sqrt(variance):.1f} ms",
                flush=True,
            )
            print(f"    Status: {status}", flush=True)

        if self._ekf_warnings:
            print(
                f"\n  EKF diagnostics ({len(self._ekf_warnings)} unique issues):",
                flush=True,
            )
            for warning in sorted(self._ekf_warnings):
                print(f"    WARN: {warning}", flush=True)
        else:
            print("\n  EKF diagnostics: no warnings/errors observed", flush=True)

        rates = {
            name: self._rate(stamps) for name, stamps in self._arrivals.items()
        }
        print("\n" + "-" * 60, flush=True)
        print(
            "  Summary: "
            f"odom={rates['odom']:.1f}Hz "
            f"imu={rates['imu']:.1f}Hz "
            f"ekf={rates['odom_combined']:.1f}Hz "
            f"scan={rates['scan']:.1f}Hz "
            f"laser_odom={rates['odom_laser']:.1f}Hz "
            f"ekf_warnings={len(self._ekf_warnings)}",
            flush=True,
        )
        print("-" * 60, flush=True)

        self._finish_timer.cancel()
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = OdomDiagNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
