"""Contracts for bounded steering-calibration command-line tools."""

import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.steering_calibration import (  # noqa: E402
    MAX_CIRCLE_SPEED_MPS,
    MAX_STEERING_ANGLE_RAD,
    angular_velocity_for_steering,
    validate_circle_request,
    validate_hold_request,
)


DRIVE = PACKAGE_ROOT / "smartcar_tools" / "steering_circle_drive.py"
HOLD = PACKAGE_ROOT / "smartcar_tools" / "steering_hold.py"
SETUP = PACKAGE_ROOT / "setup.py"
MANIFEST = PACKAGE_ROOT / "package.xml"


class SteeringCalibrationMathTests(unittest.TestCase):
    def test_circle_request_is_bounded_to_the_verified_release_limits(self):
        self.assertEqual(
            validate_circle_request(0.30, 0.15, 20.0, 20.0, 0.189),
            (0.30, 0.15, 20.0, 20.0, 0.189),
        )
        for angle, speed, duration, rate, wheelbase in (
            (0.0, 0.15, 20.0, 20.0, 0.189),
            (MAX_STEERING_ANGLE_RAD + 0.01, 0.15, 20.0, 20.0, 0.189),
            (0.30, MAX_CIRCLE_SPEED_MPS + 0.01, 20.0, 20.0, 0.189),
            (0.30, 0.15, 0.0, 20.0, 0.189),
            (0.30, 0.15, 20.0, 0.0, 0.189),
            (0.30, 0.15, 20.0, 20.0, 0.0),
        ):
            with self.subTest(angle=angle, speed=speed):
                with self.assertRaises(ValueError):
                    validate_circle_request(
                        angle, speed, duration, rate, wheelbase)

    def test_hold_request_is_angle_limited_and_requires_a_finite_duration(
            self):
        self.assertEqual(validate_hold_request(-0.30, 12.0), (-0.30, 12.0))
        for angle, duration in ((0.0, 12.0), (0.71, 12.0), (0.3, 0.0)):
            with self.subTest(angle=angle, duration=duration):
                with self.assertRaises(ValueError):
                    validate_hold_request(angle, duration)

    def test_twist_curvature_converts_back_to_the_requested_steering(
            self):
        angular = angular_velocity_for_steering(0.15, 0.30, 0.189)
        self.assertAlmostEqual(math.atan(0.189 * angular / 0.15), 0.30)
        with self.assertRaises(ValueError):
            angular_velocity_for_steering(0.0, 0.30, 0.189)


class SteeringCalibrationCommandContracts(unittest.TestCase):
    def test_ground_circle_uses_the_guarded_velocity_chain_and_full_lease(
            self):
        source = DRIVE.read_text(encoding="utf-8")
        self.assertIn('"/cmd_vel_nav"', source)
        self.assertNotIn('"/ackermann_cmd"', source)
        for service in ("prepare", "activate", "renew", "stop"):
            with self.subTest(service=service):
                self.assertIn(f'"/smartcar/direction_guard/{service}"', source)
        self.assertIn("MOTION_FORWARD = 1", source)
        self.assertIn("--yes", source)
        self.assertIn("finally:\n            self._stop_motion()", source)
        self.assertIn("self._publish_zero()", source)

    def test_static_hold_is_safety_owned_and_cannot_request_speed(self):
        source = HOLD.read_text(encoding="utf-8")
        self.assertIn('"/smartcar/safety/steering_calibration_hold"', source)
        self.assertNotIn('"/ackermann_cmd"', source)
        self.assertNotIn('"--speed"', source)
        self.assertIn("--yes", source)
        self.assertIn("self._call(0.0, 0.0)", source)

    def test_all_three_tools_are_installed_and_manifest_has_uuid_dependency(
            self):
        setup = SETUP.read_text(encoding="utf-8")
        for entry in (
            "steering_circle_analyze",
            "steering_circle_drive",
            "steering_hold",
        ):
            with self.subTest(entry=entry):
                self.assertIn(entry, setup)
        root = ElementTree.parse(MANIFEST).getroot()
        dependencies = {item.text for item in root.findall("exec_depend")}
        self.assertIn("unique_identifier_msgs", dependencies)


if __name__ == "__main__":
    unittest.main()
