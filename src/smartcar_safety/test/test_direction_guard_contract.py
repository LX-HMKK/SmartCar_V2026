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
            "fault_permit_timeout",
        ):
            self.assertIn(reason, source)
        self.assertIn("phase_ = DirectionGuardPhase::Faulted", source)
        self.assertIn("direction_ = MotionDirection::Stop", source)

    def test_defaults_leave_renewal_timing_margin(self):
        parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
            "direction_guard"
        ]["ros__parameters"]
        self.assertEqual(parameters["publish_frequency_hz"], 20.0)
        self.assertLess(
            parameters["candidate_timeout_sec"],
            parameters["permit_timeout_sec"],
        )
        self.assertLessEqual(parameters["permit_timeout_sec"], 0.30)
        self.assertGreater(parameters["stop_settle_sec"], 0.0)

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
