"""Pure tests for ROS node parameter validation."""
import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_safety.guard import validate_publish_frequency


class PublishFrequencyTests(unittest.TestCase):
    def test_positive_finite_frequency_is_accepted(self):
        self.assertEqual(validate_publish_frequency(20.0), 20.0)

    def test_non_positive_or_non_finite_frequency_is_rejected(self):
        for frequency in (0.0, -1.0, math.nan, math.inf, -math.inf):
            with self.subTest(frequency=frequency):
                with self.assertRaises(ValueError):
                    validate_publish_frequency(frequency)


if __name__ == "__main__":
    unittest.main()
