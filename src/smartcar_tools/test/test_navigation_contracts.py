"""Package-local navigation test contracts run by colcon/pytest on RDK."""
import ast
from pathlib import Path
import unittest
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAV2_ROOT = PACKAGE_ROOT.parent / "smartcar_nav2"
RUNNER = PACKAGE_ROOT / "smartcar_tools" / "navigation_runner.py"
LAUNCH = PACKAGE_ROOT / "launch" / "navigation_test.launch.py"
PARAMS = NAV2_ROOT / "config" / "field_test_nav2_params.yaml"
BT = (
    NAV2_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_field_test.xml"
)


class NavigationContracts(unittest.TestCase):
    def test_navigation_runner_is_explicitly_armed_and_fail_closed(self):
        source = RUNNER.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn("NavigateThroughPoses", source)
        self.assertNotIn("FollowWaypoints", source)
        self.assertIn('self.declare_parameter(gate, False)', source)
        self.assertIn('self.declare_parameter("arm_timeout_sec", 30.0)', source)
        self.assertIn('self._state = "locking_estop"', source)
        self.assertGreaterEqual(source.count("self._sensor_readiness_error()"), 3)
        self.assertIn("cancel_requested = self._cancel_requested", source)
        for interface in ("prepare", "arm", "start", "stop", "status"):
            self.assertIn(f"/smartcar/test/navigation/{interface}", source)

    def test_navigation_launch_isolated_and_latches_safety(self):
        source = LAUNCH.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('"safety_emergency_stop_on_start": "true"', source)
        self.assertIn('"use_laser_odometry"', source)
        self.assertIn('"laser_odometry_calibrated"', source)
        self.assertIn("/smartcar/localization/reset_laser_odometry", source)
        self.assertIn('"arm_timeout_sec": 30.0', source)
        self.assertNotIn('"arm_timeout_sec": "30.0"', source)
        for excluded in ("smartcar_task", "smartcar_vision", "smartcar_speech"):
            self.assertNotIn(excluded, source)

    def test_field_profile_is_low_speed_ackermann_navigation(self):
        params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
        controller = params["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        smoother = params["velocity_smoother"]["ros__parameters"]
        planner = params["planner_server"]["ros__parameters"]["GridBased"]
        self.assertEqual(controller["desired_linear_vel"], 0.15)
        self.assertEqual(smoother["max_velocity"][0], 0.15)
        self.assertEqual(smoother["min_velocity"][0], 0.0)
        self.assertEqual(planner["motion_model_for_search"], "DUBIN")
        self.assertIs(controller["use_rotate_to_heading"], False)
        self.assertIs(controller["allow_reversing"], False)
        self.assertNotIn(
            "backup",
            params["behavior_server"]["ros__parameters"]["behavior_plugins"],
        )

    def test_field_tree_has_tight_pass_radius_and_no_spin(self):
        root = ElementTree.parse(BT).getroot()
        tags = [element.tag for element in root.iter()]
        self.assertNotIn("Spin", tags)
        self.assertNotIn("BackUp", tags)
        self.assertIn("ComputePathThroughPoses", tags)
        remove_passed = next(root.iter("RemovePassedGoals"))
        self.assertEqual(float(remove_passed.attrib["radius"]), 0.20)
        self.assertEqual(remove_passed.attrib["global_frame"], "odom_combined")
        self.assertEqual(remove_passed.attrib["robot_base_frame"], "base_footprint")


if __name__ == "__main__":
    unittest.main()
