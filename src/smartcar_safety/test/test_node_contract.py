"""ROS-runtime-independent contract for safety_node command sanitization."""
from pathlib import Path
import unittest


NODE_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "smartcar_safety"
    / "safety_node.py"
)


class SafetyNodeCommandContractTests(unittest.TestCase):
    def test_all_twist_fields_are_sanitized_before_caching(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        for field in (
            "message.linear.x",
            "message.linear.y",
            "message.linear.z",
            "message.angular.x",
            "message.angular.y",
            "message.angular.z",
        ):
            self.assertIn(field, source)
        self.assertIn("sanitize_twist_components((", source)

    def test_invalid_command_is_latched_and_publishes_zero_immediately_and_later(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("self._last_command_components = None", source)
        self.assertIn("self.guard.mark_command_invalid()", source)
        self.assertGreaterEqual(
            source.count("twist_from_components(ZERO_TWIST_COMPONENTS)"),
            2,
        )


if __name__ == "__main__":
    unittest.main()
