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
            "odom_timeout_sec": 0.35,
            "raw_odom_timeout_sec": 0.25,
            "minimum_voltage": 0.0,
            "voltage_timeout_sec": 1.0,
            "max_linear_speed_mps": 0.30,
            "require_odom": True,
            "require_raw_odom": True,
            "depth_points_timeout_sec": 0.50,
            "require_depth_points": False,
        }
        options.update(overrides)
        return SafetyGuard(**options)

    def make_healthy(self, guard, now=10.0, voltage=12.0):
        self.assertTrue(guard.mark_command(now, 0.0))
        guard.mark_odom(now)
        guard.mark_raw_odom(now)
        if guard.require_depth_points:
            guard.mark_depth_points(now)
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

    def test_stale_odom_blocks_motion(self):
        guard = self.make_guard()
        self.make_healthy(guard)
        guard.mark_command(10.20, 0.0)
        self.assertEqual(
            guard.evaluate(10.36),
            {"allowed": False, "reason": "odom_stale"},
        )

    def test_raw_odom_is_required_by_default(self):
        guard = SafetyGuard(require_odom=False)
        guard.mark_command(10.0, 0.0)

        self.assertEqual(guard.raw_odom_timeout_sec, 0.25)
        self.assertEqual(
            guard.evaluate(10.1),
            {"allowed": False, "reason": "raw_odom_missing"},
        )

    def test_fresh_raw_odom_allows_motion(self):
        guard = self.make_guard(require_raw_odom=True)
        self.make_healthy(guard)
        guard.mark_raw_odom(10.0)

        self.assertEqual(guard.evaluate(10.20), {"allowed": True, "reason": "ok"})

    def test_stale_raw_odom_blocks_motion(self):
        guard = self.make_guard(require_raw_odom=True)
        self.make_healthy(guard)
        guard.mark_raw_odom(10.0)
        guard.mark_command(10.20, 0.0)
        guard.mark_odom(10.20)

        self.assertEqual(
            guard.evaluate(10.26),
            {"allowed": False, "reason": "raw_odom_stale"},
        )

    def test_depth_points_are_fail_closed_when_required(self):
        guard = self.make_guard(
            command_timeout_sec=1.0,
            odom_timeout_sec=1.0,
            raw_odom_timeout_sec=1.0,
            depth_points_timeout_sec=1.0,
            require_depth_points=True,
        )
        self.make_healthy(guard)
        self.assertEqual(guard.evaluate(10.20), {"allowed": True, "reason": "ok"})

        guard.mark_command(11.10, 0.0)
        guard.mark_odom(11.10)
        guard.mark_raw_odom(11.10)
        self.assertEqual(
            guard.evaluate(11.10),
            {"allowed": False, "reason": "depth_points_stale"},
        )

        guard = self.make_guard(require_depth_points=True)
        self.assertTrue(guard.mark_command(10.0, 0.0))
        guard.mark_odom(10.0)
        guard.mark_raw_odom(10.0)
        self.assertEqual(
            guard.evaluate(10.10),
            {"allowed": False, "reason": "depth_points_missing"},
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
        guard = self.make_guard(
            require_odom=False,
            require_raw_odom=False,
        )
        guard.mark_command(10.0, 0.0)
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

    def test_stale_voltage_blocks_motion_when_threshold_enabled(self):
        guard = self.make_guard(
            command_timeout_sec=1.0,
            odom_timeout_sec=1.0,
            raw_odom_timeout_sec=1.0,
            minimum_voltage=11.0,
            voltage_timeout_sec=0.50,
        )
        self.make_healthy(guard, now=10.0, voltage=12.0)
        guard.mark_command(10.60, 0.0)
        guard.mark_odom(10.60)
        guard.mark_raw_odom(10.60)

        self.assertEqual(
            guard.evaluate(10.60),
            {"allowed": False, "reason": "voltage_stale"},
        )

    def test_speed_limit_blocks_both_directions_until_explicitly_cleared(self):
        guard = self.make_guard(
            require_odom=False,
            require_raw_odom=False,
        )
        for speed in (0.300001, -0.300001):
            with self.subTest(speed=speed):
                self.assertFalse(guard.mark_command(10.0, speed))
                self.assertEqual(
                    guard.evaluate(10.01),
                    {
                        "allowed": False,
                        "reason": "command_speed_limit_exceeded",
                    },
                )
                self.assertFalse(
                    guard.mark_command(10.02, speed / abs(speed) * 0.30))
                guard.clear_command_speed_limit_fault()
                self.assertTrue(
                    guard.mark_command(10.03, speed / abs(speed) * 0.30))
                self.assertEqual(
                    guard.evaluate(10.04),
                    {"allowed": True, "reason": "ok"},
                )

    def test_non_finite_minimum_voltage_is_rejected(self):
        for threshold in (math.nan, math.inf, -math.inf):
            with self.subTest(threshold=threshold):
                with self.assertRaises(ValueError):
                    self.make_guard(minimum_voltage=threshold)

    def test_non_finite_timeouts_are_rejected(self):
        for field in (
            "command_timeout_sec",
            "odom_timeout_sec",
            "raw_odom_timeout_sec",
            "depth_points_timeout_sec",
            "voltage_timeout_sec",
        ):
            for timeout in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field, timeout=timeout):
                    with self.assertRaises(ValueError):
                        self.make_guard(**{field: timeout})

    def test_zero_and_negative_timeouts_are_rejected(self):
        for field in (
            "command_timeout_sec",
            "odom_timeout_sec",
            "raw_odom_timeout_sec",
            "depth_points_timeout_sec",
            "voltage_timeout_sec",
        ):
            for timeout in (0.0, -0.01):
                with self.subTest(field=field, timeout=timeout):
                    with self.assertRaises(ValueError):
                        self.make_guard(**{field: timeout})

    def test_negative_minimum_voltage_is_rejected(self):
        with self.assertRaises(ValueError):
            self.make_guard(minimum_voltage=-0.01)

    def test_non_positive_or_non_finite_speed_limit_is_rejected(self):
        for limit in (0.0, -0.01, math.nan, math.inf, -math.inf):
            with self.subTest(limit=limit):
                with self.assertRaises(ValueError):
                    self.make_guard(max_linear_speed_mps=limit)

    def test_zero_minimum_voltage_remains_disabled(self):
        guard = self.make_guard(
            minimum_voltage=0.0,
            require_odom=False,
            require_raw_odom=False,
        )
        guard.mark_command(10.0, 0.0)
        self.assertEqual(guard.evaluate(10.1), {"allowed": True, "reason": "ok"})

    def test_invalid_command_blocks_without_refreshing_last_valid_timestamp(self):
        guard = self.make_guard()
        self.make_healthy(guard)
        guard.mark_command_invalid()

        self.assertEqual(guard.command_received_at, 10.0)
        self.assertEqual(
            guard.evaluate(10.1),
            {"allowed": False, "reason": "command_invalid"},
        )

        guard.mark_command(10.2, 0.0)
        self.assertEqual(guard.evaluate(10.21), {"allowed": True, "reason": "ok"})


if __name__ == "__main__":
    unittest.main()
