"""ROS-runtime-independent contract for safety_node command sanitization."""
import ast
from pathlib import Path
import unittest


NODE_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "smartcar_safety"
    / "safety_node.py"
)
CPP_NODE_SOURCE = Path(__file__).resolve().parents[1] / "src" / "safety_node.cpp"
CONFIG_SOURCE = Path(__file__).resolve().parents[1] / "config" / "safety.yaml"


class SafetyNodeCommandContractTests(unittest.TestCase):
    def test_safety_inputs_keep_only_the_latest_sample(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("LATEST_RELIABLE_QOS = QoSProfile(depth=1)", source)
        self.assertIn("LATEST_SENSOR_QOS = QoSProfile(", source)
        self.assertIn("depth=1", source)
        self.assertIn('Twist, "/cmd_vel", self._on_command, LATEST_RELIABLE_QOS', source)
        self.assertIn(
            'LaserScan, "/scan", self._on_scan, LATEST_SENSOR_QOS, raw=True',
            source,
        )
        self.assertIn(
            'Odometry, "/odom_combined", self._on_odom, LATEST_RELIABLE_QOS,',
            source,
        )
        self.assertIn(
            'Odometry, "/odom", self._on_raw_odom, LATEST_RELIABLE_QOS,',
            source,
        )
        self.assertEqual(source.count("raw=True"), 3)

    def test_raw_odom_has_an_independent_required_subscription(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn(
            'self.declare_parameter("require_raw_odom", True)',
            source,
        )
        self.assertIn(
            'self.declare_parameter("raw_odom_timeout_sec", 0.25)',
            source,
        )
        self.assertIn(
            'Odometry, "/odom", self._on_raw_odom, LATEST_RELIABLE_QOS',
            source,
        )
        self.assertIn(
            'require_raw_odom=self.get_parameter("require_raw_odom").value',
            source,
        )
        self.assertIn(
            'raw_odom_timeout_sec=self.get_parameter("raw_odom_timeout_sec").value',
            source,
        )
        self.assertIn("self.guard.mark_raw_odom(", source)

    def test_odom_callbacks_are_lightweight_heartbeats(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("self.guard.mark_odom(", source)
        self.assertIn("self.guard.mark_raw_odom(", source)
        self.assertIn("self._last_odom_processed_at", source)
        self.assertIn("self._last_raw_odom_processed_at", source)
        self.assertIn("self._odom_throttle_interval", source)
        self.assertNotIn("serialized_odometry_is_finite", source)
        self.assertNotIn("mark_odom_invalid", source)
        self.assertNotIn("mark_raw_odom_invalid", source)
        self.assertNotIn("clear_localization_fault", source)

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
        self.assertIn(
            "self._zero_command = twist_from_components(ZERO_TWIST_COMPONENTS)",
            source,
        )
        self.assertIn("self._last_command_components = None", source)
        self.assertIn("self._last_command_message = None", source)
        self.assertIn("self.guard.mark_command_invalid()", source)
        self.assertIn("self._safe_publisher.publish(self._zero_command)", source)
        self.assertIn("self._publish_ackermann(self._zero_command)", source)
        self.assertIn("command = self._zero_command", source)

    def test_final_linear_speed_cap_is_present_in_both_implementations(self):
        for node in (NODE_SOURCE, CPP_NODE_SOURCE):
            with self.subTest(node=node.name):
                source = node.read_text(encoding="utf-8")
                self.assertIn("max_linear_speed_mps", source)
                self.assertIn("mark_command", source)
                self.assertIn("publish_zero_command", source)
                self.assertIn("clear_command_speed_limit_fault", source)
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        self.assertIn("max_linear_speed_mps: 0.30", config)

        cpp_source = CPP_NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("publish_ackermann(zero_command_)", cpp_source)

    def test_voltage_freshness_is_configured_in_both_implementations(self):
        for node in (NODE_SOURCE, CPP_NODE_SOURCE):
            with self.subTest(node=node.name):
                self.assertIn(
                    "voltage_timeout_sec", node.read_text(encoding="utf-8"))
        self.assertIn(
            "voltage_timeout_sec: 1.0",
            CONFIG_SOURCE.read_text(encoding="utf-8"),
        )

    def test_startup_emergency_stop_is_explicit_and_fail_closed(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn(
            'self.declare_parameter("emergency_stop_on_start", False)',
            source,
        )
        self.assertIn(
            'self.get_parameter("emergency_stop_on_start").value',
            source,
        )
        self.assertIn("self.guard.set_emergency_stop(True)", source)

    def test_status_is_transient_local_and_only_changes_are_published(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("STATUS_QOS = QoSProfile(", source)
        self.assertIn("durability=DurabilityPolicy.TRANSIENT_LOCAL", source)
        self.assertIn("String, \"/smartcar/safety/status\", STATUS_QOS", source)
        self.assertIn("if reason == self._last_status_reason:", source)
        self.assertNotIn("_last_blocked_status_at", source)

    def test_watchdog_uses_a_monotonic_arrival_clock(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("return time.monotonic()", source)
        # get_clock().now() is only used for ackermann message header stamps,
        # NOT for watchdog interval comparisons (which use time.monotonic())
        self.assertEqual(source.count("self.get_clock().now()"), 1)


if __name__ == "__main__":
    unittest.main()
