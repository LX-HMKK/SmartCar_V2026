"""Contracts for the isolated, fail-closed full-course navigation test."""
import ast
from pathlib import Path
import unittest
from xml.etree import ElementTree

import yaml


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "src" / "smartcar_tools"
NAV2 = ROOT / "src" / "smartcar_nav2"
RUNNER = TOOLS / "smartcar_tools" / "navigation_runner.py"
LAUNCH = TOOLS / "launch" / "navigation_test.launch.py"
PARAMS = NAV2 / "config" / "field_test_nav2_params.yaml"
BT = (
    NAV2
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_field_test.xml"
)
TO_POSE_BT = (
    NAV2
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_field_test.xml"
)


class NavigationTestContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner_source = RUNNER.read_text(encoding="utf-8")
        cls.launch_source = LAUNCH.read_text(encoding="utf-8")
        cls.params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))

    def test_runner_is_valid_python_and_exposes_explicit_services(self):
        ast.parse(self.runner_source)
        for service in (
            "/smartcar/test/navigation/prepare",
            "/smartcar/test/navigation/arm",
            "/smartcar/test/navigation/start",
            "/smartcar/test/navigation/stop",
            "/smartcar/test/navigation/status",
        ):
            self.assertIn(service, self.runner_source)
        self.assertIn("NavigateThroughPoses", self.runner_source)
        self.assertIn('"/navigate_through_poses"', self.runner_source)
        self.assertNotIn("FollowWaypoints", self.runner_source)

    def test_runner_is_fail_closed_and_has_no_autostart_path(self):
        self.assertIn('self._state = "locking_estop"', self.runner_source)
        self.assertIn('self.declare_parameter(gate, False)', self.runner_source)
        self.assertIn('self.declare_parameter("arm_timeout_sec", 30.0)', self.runner_source)
        self.assertNotIn("autostart_mission", self.runner_source)
        self.assertNotIn("autostart_navigation", self.runner_source)
        self.assertIn("_finish_terminal", self.runner_source)
        self.assertIn("_call_estop(True)", self.runner_source)
        self.assertGreaterEqual(
            self.runner_source.count("self._sensor_readiness_error()"),
            3,
        )
        self.assertIn(
            "cancel_requested = self._cancel_requested",
            self.runner_source,
        )
        self.assertIn(
            'self.declare_parameter("route_end_id", "")',
            self.runner_source,
        )
        self.assertIn("self._waypoints = self._select_waypoints", self.runner_source)
        self.assertIn("ClearEntireCostmap", self.runner_source)
        self.assertIn("post_clear_scan_timeout", self.runner_source)
        self.assertIn(
            "required_scan_sequence = self._scan_sequence + 2",
            self.runner_source,
        )

    def test_optional_laser_odometry_is_disabled_and_gated_by_default(self):
        self.assertIn(
            'self.declare_parameter("use_laser_odometry", False)',
            self.runner_source,
        )
        self.assertIn(
            'self.declare_parameter("laser_odometry_calibrated", False)',
            self.runner_source,
        )
        self.assertIn("laser_odom_stale", self.runner_source)
        self.assertIn("laser_odometry_reset_timeout", self.runner_source)
        self.assertIn(
            'DeclareLaunchArgument(\n'
            '            "use_laser_odometry",\n'
            '            default_value="false"',
            self.launch_source,
        )
        self.assertIn(
            '"laser_odometry_calibrated",\n            default_value="false"',
            self.launch_source,
        )

    def test_launch_is_navigation_only_and_latches_safety_on_start(self):
        ast.parse(self.launch_source)
        self.assertIn('"safety_emergency_stop_on_start": "true"', self.launch_source)
        self.assertIn('executable="navigation_runner"', self.launch_source)
        self.assertIn("field_test_nav2_params.yaml", self.launch_source)
        self.assertIn("navigate_through_poses_field_test.xml", self.launch_source)
        self.assertIn("navigate_to_pose_field_test.xml", self.launch_source)
        self.assertIn(
            '"route_end_id": LaunchConfiguration("route_end_id")',
            self.launch_source,
        )
        for forbidden_package in (
            "smartcar_task",
            "smartcar_vision",
            "smartcar_speech",
        ):
            self.assertNotIn(forbidden_package, self.launch_source)

    def test_field_profile_limits_supervised_motion_to_point_15_mps(self):
        controller = self.params["controller_server"]["ros__parameters"]
        smoother = self.params["velocity_smoother"]["ros__parameters"]
        planner = self.params["planner_server"]["ros__parameters"]["GridBased"]
        self.assertEqual(controller["FollowPath"]["desired_linear_vel"], 0.15)
        self.assertEqual(smoother["max_velocity"][0], 0.15)
        self.assertLessEqual(
            abs(smoother["max_velocity"][2]),
            smoother["max_velocity"][0] / planner["minimum_turning_radius"]
            + 1.0e-9,
        )
        self.assertFalse(controller["FollowPath"]["use_rotate_to_heading"])
        self.assertFalse(controller["FollowPath"]["allow_reversing"])
        self.assertEqual(smoother["min_velocity"][0], 0.0)
        self.assertNotIn(
            "backup",
            self.params["behavior_server"]["ros__parameters"][
                "behavior_plugins"
            ],
        )
        self.assertEqual(planner["motion_model_for_search"], "DUBIN")

    def test_field_tree_tightens_pass_radius_without_spin(self):
        root = ElementTree.parse(BT).getroot()
        tags = [element.tag for element in root.iter()]
        self.assertNotIn("Spin", tags)
        self.assertNotIn("BackUp", tags)
        self.assertIn("ComputePathThroughPoses", tags)
        self.assertIn("FollowPath", tags)
        remove_passed = next(root.iter("RemovePassedGoals"))
        self.assertAlmostEqual(float(remove_passed.attrib["radius"]), 0.20)
        self.assertEqual(remove_passed.attrib["global_frame"], "odom_combined")
        self.assertEqual(remove_passed.attrib["robot_base_frame"], "base_footprint")

    def test_default_single_pose_tree_cannot_request_reverse(self):
        root = ElementTree.parse(TO_POSE_BT).getroot()
        tags = [element.tag for element in root.iter()]
        self.assertNotIn("Spin", tags)
        self.assertNotIn("BackUp", tags)
        self.assertIn("ComputePathToPose", tags)


if __name__ == "__main__":
    unittest.main()
