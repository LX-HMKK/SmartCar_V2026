import math
from pathlib import Path
import sys
import unittest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.short_drive_test import validate_test_limits


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


if __name__ == "__main__":
    unittest.main()
