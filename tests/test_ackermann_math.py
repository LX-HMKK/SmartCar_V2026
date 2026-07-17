import math
import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "origincar"
    / "origincar_base"
    / "scripts"
)
sys.path.insert(0, str(SCRIPTS_DIR))

from ackermann_math import steering_angle


class SteeringAngleTest(unittest.TestCase):
    def test_straight_motion_returns_zero(self):
        self.assertEqual(steering_angle(1.0, 0.0, 0.189, 0.45), 0.0)
        self.assertEqual(steering_angle(1.0, 1e-12, 0.189, 0.45), 0.0)

    def test_turn_direction_follows_angular_velocity(self):
        self.assertGreater(steering_angle(1.0, 0.8, 0.189, 0.45), 0.0)
        self.assertLess(steering_angle(1.0, -0.8, 0.189, 0.45), 0.0)

    def test_steering_angle_is_saturated(self):
        self.assertEqual(steering_angle(0.1, 10.0, 0.189, 0.45), 0.45)
        self.assertEqual(steering_angle(0.1, -10.0, 0.189, 0.45), -0.45)

    def test_reverse_motion_reverses_steering_sign(self):
        expected = math.atan(0.189 * 0.8 / -1.0)
        self.assertAlmostEqual(
            steering_angle(-1.0, 0.8, 0.189, 0.45), expected
        )

    def test_effectively_zero_linear_velocity_returns_zero(self):
        self.assertEqual(steering_angle(1e-12, 1.0, 0.189, 0.45), 0.0)


if __name__ == "__main__":
    unittest.main()
