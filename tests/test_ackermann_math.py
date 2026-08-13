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

from ackermann_math import ackermann_command, steering_angle


ADAPTER = SCRIPTS_DIR / "cmd_vel_to_ackermann_drive.py"


class SteeringAngleTest(unittest.TestCase):
    def test_straight_motion_returns_zero(self):
        self.assertEqual(steering_angle(1.0, 0.0, 0.144, 0.45), 0.0)
        self.assertEqual(steering_angle(1.0, 1e-12, 0.144, 0.45), 0.0)

    def test_turn_direction_follows_angular_velocity(self):
        self.assertGreater(steering_angle(1.0, 0.8, 0.144, 0.45), 0.0)
        self.assertLess(steering_angle(1.0, -0.8, 0.144, 0.45), 0.0)

    def test_steering_angle_is_saturated(self):
        self.assertEqual(steering_angle(0.1, 10.0, 0.144, 0.45), 0.45)
        self.assertEqual(steering_angle(0.1, -10.0, 0.144, 0.45), -0.45)

    def test_reverse_motion_reverses_steering_sign(self):
        expected = math.atan(0.144 * 0.8 / -1.0)
        self.assertAlmostEqual(
            steering_angle(-1.0, 0.8, 0.144, 0.45), expected
        )

    def test_effectively_zero_linear_velocity_returns_zero(self):
        self.assertEqual(steering_angle(1e-12, 1.0, 0.144, 0.45), 0.0)

    def test_non_finite_ackermann_inputs_fail_closed_to_zero(self):
        valid = [1.0, 0.8, 0.144, 0.45]
        field_names = (
            "linear_velocity",
            "angular_velocity",
            "wheelbase",
            "max_steering_angle",
        )
        for index, field_name in enumerate(field_names):
            for invalid in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field_name, invalid=invalid):
                    values = valid.copy()
                    values[index] = invalid
                    self.assertEqual(ackermann_command(*values), (0.0, 0.0))

    def test_finite_ackermann_command_preserves_speed_and_steering(self):
        speed, steering = ackermann_command(1.0, 0.8, 0.144, 0.45)
        self.assertEqual(speed, 1.0)
        self.assertEqual(steering, steering_angle(1.0, 0.8, 0.144, 0.45))

    def test_adapter_uses_sanitized_speed_and_steering(self):
        source = ADAPTER.read_text(encoding="utf-8")
        self.assertIn("from ackermann_math import ackermann_command", source)
        self.assertIn("speed, steering = ackermann_command(", source)
        self.assertIn("msg.drive.speed = speed", source)


if __name__ == "__main__":
    unittest.main()
