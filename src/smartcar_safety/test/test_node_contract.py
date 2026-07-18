"""ROS-runtime-independent contract for safety_node command sanitization."""
from pathlib import Path
import unittest


NODE_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "smartcar_safety"
    / "safety_node.py"
)


class SafetyNodeCommandContractTests(unittest.TestCase):
    def test_scan_subscription_uses_sensor_data_qos(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("from rclpy.qos import qos_profile_sensor_data", source)
        self.assertIn(
            'LaserScan, "/scan", self._on_scan, qos_profile_sensor_data',
            source,
        )

    def test_raw_odom_has_an_independent_required_subscription(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn(
            'self.declare_parameter("raw_odom_timeout_sec", 0.25)',
            source,
        )
        self.assertIn(
            'self.declare_parameter("require_raw_odom", True)',
            source,
        )
        self.assertIn(
            'Odometry, "/odom", self._on_raw_odom, 10',
            source,
        )
        self.assertIn(
            'raw_odom_timeout_sec=self.get_parameter("raw_odom_timeout_sec").value',
            source,
        )
        self.assertIn(
            'require_raw_odom=self.get_parameter("require_raw_odom").value',
            source,
        )
        self.assertIn("self.guard.mark_raw_odom(now_sec)", source)

    def test_odometry_must_be_finite_and_localization_fault_clear_is_explicit(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("def odometry_is_finite(message):", source)
        self.assertIn("self.guard.mark_odom_invalid()", source)
        self.assertIn("self.guard.mark_raw_odom_invalid(now_sec)", source)
        self.assertIn(
            '"/smartcar/safety/clear_localization_fault"', source)
        self.assertIn("self.guard.clear_localization_fault(now_sec)", source)
        self.assertIn("fresh_nonzero_command", source)

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
