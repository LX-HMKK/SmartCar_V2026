import math
from pathlib import Path
import sys
import unittest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.short_drive_test import (
    completed_distance_reason,
    runtime_mode_errors,
    validate_test_limits,
)


class ShortDriveLimitTests(unittest.TestCase):
    def test_accepts_bounded_ground_test_values(self):
        self.assertEqual(
            validate_test_limits(0.75, 0.05, 30.0),
            {"distance_m": 0.75, "speed_mps": 0.05, "timeout_sec": 30.0},
        )

    def test_rejects_speed_above_release_test_cap(self):
        with self.assertRaisesRegex(ValueError, "speed_mps exceeds"):
            validate_test_limits(0.25, 0.31, 10.0)

    def test_rejects_non_finite_or_non_positive_values(self):
        for value in (0.0, -1.0, math.inf, math.nan):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite and positive"):
                    validate_test_limits(value, 0.05, 10.0)

    def test_rejects_distance_and_timeout_above_caps(self):
        with self.assertRaisesRegex(ValueError, "distance_m exceeds"):
            validate_test_limits(3.01, 0.05, 10.0)
        with self.assertRaisesRegex(ValueError, "timeout_sec exceeds"):
            validate_test_limits(0.25, 0.05, 120.1)

    def test_only_the_distance_limit_is_a_successful_outcome(self):
        self.assertTrue(completed_distance_reason("distance_limit:0.250m"))
        for reason in (
            "timeout",
            "raw_odom_stale",
            "unexpected_reverse:-0.031m",
            "ackermann_speed_limit:0.060m/s",
        ):
            with self.subTest(reason=reason):
                self.assertFalse(completed_distance_reason(reason))

    def test_runtime_mode_rejects_normal_navigation_settings(self):
        safety = {
            "max_linear_speed_mps": 0.30,
            "require_depth_points": False,
            "require_scan": True,
            "emergency_stop_on_start": False,
        }
        task = {
            "use_depth_camera": False,
            "depth_camera_calibrated": False,
            "supervised_p_to_a_only": False,
            "supervised_p_to_c1_only": False,
            "navigation_test_end_segment_id": "",
            "autostart_mission": False,
            "waypoints_calibrated": False,
            "extrinsics_calibrated": False,
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
            "require_scan": False,
            "emergency_stop_on_start": True,
        }
        task = {
            "use_depth_camera": True,
            "depth_camera_calibrated": True,
            "supervised_p_to_a_only": True,
            "supervised_p_to_c1_only": False,
            "navigation_test_end_segment_id": "p_to_qr",
            "autostart_mission": False,
            "waypoints_calibrated": True,
            "extrinsics_calibrated": True,
            "steering_calibrated": True,
            "emergency_stop_ready": True,
            "operator_approved": True,
        }
        self.assertEqual(runtime_mode_errors(safety, task, 0.05), [])


if __name__ == "__main__":
    unittest.main()
