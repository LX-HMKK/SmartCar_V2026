"""Host-side contracts for the C++ command-direction guard."""
from pathlib import Path
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE = PACKAGE_ROOT / "src" / "direction_guard_node.cpp"
CORE = PACKAGE_ROOT / "src" / "direction_guard.cpp"
CONFIG = PACKAGE_ROOT / "config" / "direction_guard.yaml"
LAUNCH = PACKAGE_ROOT / "launch" / "direction_guard.launch.py"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"


class DirectionGuardContractTests(unittest.TestCase):
    def test_node_owns_the_post_smoother_command_boundary(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn('"/cmd_vel_candidate"', source)
        self.assertIn('create_publisher<geometry_msgs::msg::Twist>("/cmd_vel"', source)
        self.assertNotIn("/ackermann_cmd", source)

    def test_all_lease_services_are_private_to_the_guard_namespace(self):
        source = NODE.read_text(encoding="utf-8")
        for operation in ("prepare", "activate", "renew", "stop"):
            self.assertIn(
                f'"/smartcar/direction_guard/{operation}"', source)

    def test_fail_closed_conditions_are_latched_by_the_core(self):
        source = CORE.read_text(encoding="utf-8")
        for reason in (
            "fault_candidate_invalid",
            "fault_unsupported_twist",
            "fault_wrong_direction",
            "fault_candidate_timeout",
        ):
            self.assertIn(reason, source)
        self.assertIn("phase_ = DirectionGuardPhase::Faulted", source)
        self.assertIn("direction_ = MotionDirection::Stop", source)

    def test_defaults_disable_lease_expiry_but_keep_command_freshness(self):
        parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
            "direction_guard"
        ]["ros__parameters"]
        self.assertEqual(parameters["publish_frequency_hz"], 20.0)
        self.assertEqual(parameters["candidate_timeout_sec"], 0.40)
        self.assertEqual(parameters["permit_timeout_sec"], 0.0)
        self.assertEqual(parameters["forward_recovery_max_reverse_speed"], 0.25)
        self.assertNotIn("forward_recovery_max_reverse_duration_sec", parameters)
        self.assertGreater(parameters["stop_settle_sec"], 0.0)
        source = CORE.read_text(encoding="utf-8")
        self.assertNotIn('latch_fault("fault_permit_timeout")', source)
        self.assertIn('latch_fault("fault_candidate_timeout")', source)

    def test_forward_recovery_is_capped_at_the_guard_boundary(self):
        header = (PACKAGE_ROOT / "include" / "smartcar_safety" /
                  "direction_guard.hpp").read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        self.assertIn("ForwardRecovery = 3", header)
        self.assertIn("kForwardRecoveryMaxReverseSpeed{0.25}", header)
        self.assertIn("forward_recovery_max_reverse_speed", header)
        self.assertIn("exceeds native BackUp cap", core)
        self.assertIn("MotionDirection::ForwardRecovery", core)
        self.assertIn("warning_recovery_reverse_speed_rejected", core)
        self.assertIn("warning_recovery_reverse_turn_rejected", core)
        self.assertNotIn("fault_recovery_reverse_replayed", core)
        self.assertNotIn("fault_recovery_reverse_timeout", core)
        self.assertNotIn("forward_recovery_max_reverse_duration_sec", core)
        node = NODE.read_text(encoding="utf-8")
        self.assertIn('declare_parameter("forward_recovery_max_reverse_speed"', node)
        self.assertIn("config.forward_recovery_max_reverse_speed", node)

    def test_node_logs_the_latched_fault_reason_before_later_stop_handling(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn('status.rfind("fault_", 0) == 0', source)
        self.assertIn("RCLCPP_ERROR(get_logger()", source)
        self.assertIn("direction guard latched %s", source)

    def test_node_logs_recovery_command_rejections_without_latching(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn('status.rfind("warning_", 0) == 0', source)
        self.assertIn("RCLCPP_WARN(get_logger()", source)
        self.assertIn("forward mission lease remains active", source)

    def test_launch_starts_only_the_cpp_guard(self):
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('executable="direction_guard_node"', source)
        self.assertIn('"direction_guard.yaml"', source)

    def test_default_colcon_tests_are_narrow_and_direction_specific(self):
        source = CMAKE.read_text(encoding="utf-8")
        self.assertNotIn("ament_lint_auto_find_test_dependencies", source)
        self.assertIn(
            "ament_add_gtest(test_direction_guard_cpp", source)
        self.assertIn(
            "ament_add_pytest_test(test_direction_guard_contract_py", source)


if __name__ == "__main__":
    unittest.main()
