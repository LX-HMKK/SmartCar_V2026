import math
from pathlib import Path
import sys
import unittest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.short_drive_test import (
    runtime_mode_errors,
    outcome_passed,
    route_profile_spec,
    validate_test_limits,
)


class ShortDriveLimitTests(unittest.TestCase):
    def test_accepts_bounded_ground_test_values(self):
        self.assertEqual(
            validate_test_limits(0.05, 30.0),
            {"speed_mps": 0.05, "timeout_sec": 30.0},
        )

    def test_accepts_the_fixed_p_to_c1_prefix_timeout(self):
        limits = validate_test_limits(0.05, 240.0, "p_to_c1")
        self.assertEqual(limits["timeout_sec"], 240.0)

    def test_rejects_speed_above_release_test_cap(self):
        with self.assertRaisesRegex(ValueError, "speed_mps exceeds"):
            validate_test_limits(0.31, 10.0)

    def test_rejects_non_finite_or_non_positive_values(self):
        for value in (0.0, -1.0, math.inf, math.nan):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite and positive"):
                    validate_test_limits(value, 10.0)

    def test_rejects_timeout_above_caps(self):
        with self.assertRaisesRegex(ValueError, "timeout_sec exceeds"):
            validate_test_limits(0.05, 120.1)
        with self.assertRaisesRegex(ValueError, "timeout_sec exceeds"):
            validate_test_limits(0.05, 240.1, "p_to_c1")

    def test_rejects_unknown_route_profile(self):
        with self.assertRaisesRegex(ValueError, "route_profile must be one of"):
            route_profile_spec("full_route")

    def test_only_nav2_completion_is_a_successful_outcome(self):
        self.assertTrue(outcome_passed("mission_completed"))
        for reason in (
            "timeout",
            "raw_odom_stale",
            "unexpected_reverse:-0.031m",
            "ackermann_speed_limit:0.060m/s",
            "distance_limit:3.501m",
        ):
            with self.subTest(reason=reason):
                self.assertFalse(outcome_passed(reason))

    def test_fused_odom_watchdog_failure_is_not_a_pass(self):
        self.assertFalse(outcome_passed("fused_odom_stale"))

    def test_runtime_mode_rejects_normal_navigation_settings(self):
        safety = {
            "max_linear_speed_mps": 0.30,
            "require_depth_points": False,
            "emergency_stop_on_start": False,
        }
        task = {
            "supervised_p_to_a_only": False,
            "supervised_p_to_c1_only": False,
            "navigation_test_end_segment_id": "",
            "autostart_mission": False,
            "steering_calibrated": False,
            "emergency_stop_ready": False,
            "operator_approved": False,
        }
        errors = runtime_mode_errors(safety, task, 0.05)
        self.assertTrue(errors)
        self.assertIn("safety.require_depth_points must be True", errors)
        self.assertIn("task.supervised_p_to_a_only must be True", errors)

    def test_runtime_mode_accepts_the_dedicated_depth_short_drive(self):
        safety = {
            "max_linear_speed_mps": 0.05,
            "require_depth_points": True,
            "emergency_stop_on_start": True,
        }
        task = {
            "supervised_p_to_a_only": True,
            "supervised_p_to_c1_only": False,
            "navigation_test_end_segment_id": "p_to_qr",
            "autostart_mission": False,
            "steering_calibrated": True,
            "emergency_stop_ready": True,
            "operator_approved": True,
        }
        self.assertEqual(runtime_mode_errors(safety, task, 0.05), [])

    def test_runtime_mode_accepts_the_fixed_depth_p_to_c1_prefix(self):
        safety = {
            "max_linear_speed_mps": 0.05,
            "require_depth_points": True,
            "emergency_stop_on_start": True,
        }
        task = {
            "supervised_p_to_a_only": False,
            "supervised_p_to_c1_only": True,
            "navigation_test_end_segment_id": "qr_to_vlm",
            "autostart_mission": False,
            "steering_calibrated": True,
            "emergency_stop_ready": True,
            "operator_approved": True,
        }
        self.assertEqual(runtime_mode_errors(safety, task, 0.05, "p_to_c1"), [])


if __name__ == "__main__":
    unittest.main()
