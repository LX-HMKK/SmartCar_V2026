"""Unit tests for the ROS-independent velocity safety decision logic."""
import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_safety.guard import SafetyGuard


class SafetyGuardTests(unittest.TestCase):
    def make_guard(self, **overrides):
        options = {
            "command_timeout_sec": 0.30,
            "scan_timeout_sec": 0.35,
            "odom_timeout_sec": 0.35,
            "minimum_voltage": 0.0,
            "require_scan": True,
            "require_odom": True,
        }
        options.update(overrides)
        return SafetyGuard(**options)

    def make_healthy(self, guard, now=10.0, voltage=12.0):
        guard.mark_command(now)
        guard.mark_scan(now)
        guard.mark_odom(now)
        guard.mark_voltage(voltage, now)

    def test_startup_fails_closed(self):
        result = self.make_guard().evaluate(0.0)
        self.assertEqual(result, {"allowed": False, "reason": "command_missing"})

    def test_healthy_inputs_allow_passthrough(self):
        guard = self.make_guard()
        self.make_healthy(guard)
        self.assertEqual(guard.evaluate(10.20), {"allowed": True, "reason": "ok"})

    def test_stale_command_blocks_motion(self):
        guard = self.make_guard()
        self.make_healthy(guard)
        self.assertEqual(
            guard.evaluate(10.31),
            {"allowed": False, "reason": "command_stale"},
        )

    def test_stale_scan_blocks_motion(self):
        guard = self.make_guard()
        self.make_healthy(guard)
        guard.mark_command(10.20)
        guard.mark_odom(10.20)
        self.assertEqual(
            guard.evaluate(10.36),
            {"allowed": False, "reason": "scan_stale"},
        )

    def test_stale_odom_blocks_motion(self):
        guard = self.make_guard()
        self.make_healthy(guard)
        guard.mark_command(10.20)
        guard.mark_scan(10.20)
        self.assertEqual(
            guard.evaluate(10.36),
            {"allowed": False, "reason": "odom_stale"},
        )

    def test_emergency_stop_latches_until_explicitly_cleared(self):
        guard = self.make_guard()
        self.make_healthy(guard)
        guard.set_emergency_stop(True)
        self.assertEqual(
            guard.evaluate(10.10),
            {"allowed": False, "reason": "emergency_stop"},
        )
        guard.set_emergency_stop(False)
        self.assertEqual(guard.evaluate(10.10), {"allowed": True, "reason": "ok"})

    def test_disabled_sensor_requirements_do_not_block(self):
        guard = self.make_guard(require_scan=False, require_odom=False)
        guard.mark_command(10.0)
        self.assertEqual(guard.evaluate(10.20), {"allowed": True, "reason": "ok"})

    def test_low_voltage_blocks_motion_when_threshold_enabled(self):
        guard = self.make_guard(minimum_voltage=11.0)
        self.make_healthy(guard, voltage=10.9)
        self.assertEqual(
            guard.evaluate(10.10),
            {"allowed": False, "reason": "voltage_low"},
        )

    def test_non_finite_voltage_blocks_motion_when_threshold_enabled(self):
        for voltage in (math.nan, math.inf, -math.inf):
            with self.subTest(voltage=voltage):
                guard = self.make_guard(minimum_voltage=11.0)
                self.make_healthy(guard, voltage=voltage)
                self.assertEqual(
                    guard.evaluate(10.10),
                    {"allowed": False, "reason": "voltage_invalid"},
                )

    def test_non_finite_minimum_voltage_is_rejected(self):
        for threshold in (math.nan, math.inf, -math.inf):
            with self.subTest(threshold=threshold):
                with self.assertRaises(ValueError):
                    self.make_guard(minimum_voltage=threshold)

    def test_non_finite_timeouts_are_rejected(self):
        for field in (
            "command_timeout_sec",
            "scan_timeout_sec",
            "odom_timeout_sec",
        ):
            for timeout in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field, timeout=timeout):
                    with self.assertRaises(ValueError):
                        self.make_guard(**{field: timeout})


if __name__ == "__main__":
    unittest.main()
