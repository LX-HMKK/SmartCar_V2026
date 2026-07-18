"""Pure tests for sanitizing Twist components before ROS publication."""
import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_safety.velocity import (
    ZERO_TWIST_COMPONENTS,
    sanitize_twist_components,
    values_are_finite,
)


class TwistSanitizerTests(unittest.TestCase):
    def test_generic_finite_value_check(self):
        self.assertTrue(values_are_finite([0, 1.5, -2]))
        for invalid in (math.nan, math.inf, -math.inf, "not-a-number"):
            with self.subTest(invalid=invalid):
                self.assertFalse(values_are_finite([0.0, invalid]))

    def test_finite_components_are_preserved(self):
        components = (0.5, -0.1, 0.2, -0.3, 0.4, -0.5)
        valid, sanitized = sanitize_twist_components(components)
        self.assertTrue(valid)
        self.assertEqual(sanitized, components)

    def test_each_non_finite_component_fails_closed_to_zero(self):
        field_names = (
            "linear.x",
            "linear.y",
            "linear.z",
            "angular.x",
            "angular.y",
            "angular.z",
        )
        for index, field_name in enumerate(field_names):
            for invalid in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field_name, invalid=invalid):
                    components = [0.1] * 6
                    components[index] = invalid
                    valid, sanitized = sanitize_twist_components(components)
                    self.assertFalse(valid)
                    self.assertEqual(sanitized, ZERO_TWIST_COMPONENTS)

    def test_wrong_component_count_fails_closed_to_zero(self):
        for components in ((), (0.0,) * 5, (0.0,) * 7):
            with self.subTest(component_count=len(components)):
                valid, sanitized = sanitize_twist_components(components)
                self.assertFalse(valid)
                self.assertEqual(sanitized, ZERO_TWIST_COMPONENTS)


if __name__ == "__main__":
    unittest.main()
