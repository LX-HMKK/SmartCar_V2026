import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_FILE = PACKAGE_ROOT / "config" / "nav2_params.yaml"
FORWARD_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_w_replanning_and_recovery.xml"
)
PRECISE_FORWARD_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_precise_w_replanning_and_recovery.xml"
)
REVERSE_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_w_replanning_and_recovery.xml"
)
REVERSE_HANDOFF_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml"
)


class TestReverseNavigationContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))

    def test_one_fixed_dubin_planner_serves_both_directions(self):
        planner_server = self.params["planner_server"]["ros__parameters"]
        self.assertEqual(planner_server["planner_plugins"], ["GridBased"])
        planner = planner_server["GridBased"]
        self.assertEqual(planner["motion_model_for_search"], "DUBIN")
        self.assertNotIn("reverse_penalty", planner)
        self.assertAlmostEqual(planner["minimum_turning_radius"], 0.55)
        self.assertEqual(planner["tolerance"], 0.0)

        forward_tags = {
            element.tag for element in ElementTree.parse(FORWARD_BT_FILE).iter()
        }
        self.assertIn("ComputePathToPose", forward_tags)
        self.assertNotIn("ComputeReversePathToPose", forward_tags)

    def test_reverse_tree_replans_and_validates_before_following(self):
        root = ElementTree.parse(REVERSE_BT_FILE).getroot()
        tags = [element.tag for element in root.iter()]
        self.assertNotIn("RateController", tags)
        self.assertNotIn("PipelineSequence", tags)
        self.assertIn("NavigateReverseWithFailureReplanning", {
            element.attrib.get("name") for element in root.iter()
        })
        self.assertIn("ComputeReversePathToPose", tags)
        self.assertIn("IsPathValid", tags)
        self.assertIn("FollowPath", tags)
        self.assertNotIn("ComputePathToPose", tags)
        for forbidden in ("Spin", "BackUp", "Wait"):
            self.assertNotIn(forbidden, tags)

        reverse_compute = root.find(".//ComputeReversePathToPose")
        self.assertIsNotNone(reverse_compute)
        self.assertEqual(reverse_compute.attrib["planner_id"], "GridBased")
        planner_radius = self.params["planner_server"]["ros__parameters"][
            "GridBased"
        ]["minimum_turning_radius"]
        reverse_radius = float(
            reverse_compute.attrib["minimum_turning_radius"]
        )
        self.assertAlmostEqual(reverse_radius, planner_radius)
        self.assertAlmostEqual(reverse_radius, 0.55)
        self.assertAlmostEqual(
            float(reverse_compute.attrib["curvature_tolerance"]), 0.20
        )
        self.assertLessEqual(
            float(reverse_compute.attrib["goal_position_tolerance"]), 0.12
        )

        follow = root.find(".//FollowPath")
        self.assertEqual(follow.attrib["controller_id"], "FollowPath")
        self.assertEqual(
            follow.attrib["goal_checker_id"], "reverse_goal_checker"
        )

    def test_reverse_goal_checker_is_stricter_than_forward(self):
        controller = self.params["controller_server"]["ros__parameters"]
        self.assertIn("reverse_goal_checker", controller["goal_checker_plugins"])
        forward = controller["goal_checker"]
        reverse = controller["reverse_goal_checker"]
        self.assertLess(
            reverse["xy_goal_tolerance"], forward["xy_goal_tolerance"]
        )
        self.assertLess(
            reverse["yaw_goal_tolerance"], forward["yaw_goal_tolerance"]
        )
        self.assertIs(reverse["stateful"], False)

    def test_reverse_handoff_uses_dedicated_mppi_ackermann_controller(self):
        regular_root = ElementTree.parse(REVERSE_BT_FILE).getroot()
        handoff_root = ElementTree.parse(REVERSE_HANDOFF_BT_FILE).getroot()
        handoff_tags = [element.tag for element in handoff_root.iter()]
        self.assertIn("ComputeReversePathToPose", handoff_tags)
        self.assertIn("IsPathValid", handoff_tags)
        self.assertNotIn("ComputePathToPose", handoff_tags)
        for forbidden in ("RateController", "PipelineSequence", "Spin", "BackUp", "Wait"):
            self.assertNotIn(forbidden, handoff_tags)

        regular_compute = regular_root.find(".//ComputeReversePathToPose")
        handoff_compute = handoff_root.find(".//ComputeReversePathToPose")
        self.assertIsNotNone(regular_compute)
        self.assertIsNotNone(handoff_compute)
        for attribute in (
            "planner_id",
            "minimum_turning_radius",
            "curvature_tolerance",
            "maximum_direction_error",
            "start_position_tolerance",
            "start_yaw_tolerance",
            "goal_position_tolerance",
            "goal_yaw_tolerance",
            "minimum_segment_length",
        ):
            self.assertEqual(
                handoff_compute.attrib[attribute],
                regular_compute.attrib[attribute],
            )

        follow = handoff_root.find(".//FollowPath")
        self.assertIsNotNone(follow)
        self.assertEqual(follow.attrib["controller_id"], "ReverseHandoff")
        self.assertEqual(
            follow.attrib["goal_checker_id"], "reverse_goal_checker")

        controller = self.params["controller_server"]["ros__parameters"]
        self.assertEqual(
            controller["controller_plugins"],
            ["FollowPath", "ReverseHandoff"],
        )
        regular = controller["FollowPath"]
        handoff = controller["ReverseHandoff"]
        self.assertEqual(
            regular["plugin"],
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        )
        self.assertEqual(
            handoff["plugin"], "smartcar_nav2::ReverseOnlyMPPIController"
        )
        self.assertEqual(handoff["motion_model"], "Ackermann")
        planner_radius = self.params["planner_server"]["ros__parameters"][
            "GridBased"
        ]["minimum_turning_radius"]
        self.assertAlmostEqual(
            handoff["AckermannConstraints"]["min_turning_r"], planner_radius
        )
        self.assertAlmostEqual(handoff["vx_min"], 0.02)
        self.assertAlmostEqual(handoff["vx_max"], 0.09)
        self.assertGreater(handoff["vx_min"], 0.0)
        self.assertLess(handoff["vx_min"], handoff["vx_max"])
        self.assertAlmostEqual(
            handoff["model_dt"], 1.0 / controller["controller_frequency"]
        )
        self.assertEqual(handoff["iteration_count"], 2)
        self.assertLessEqual(handoff["batch_size"], 1000)
        self.assertIs(handoff["visualize"], False)

        critics = handoff["critics"]
        self.assertIn("GoalAngleCritic", critics)
        self.assertNotIn("PreferForwardCritic", critics)
        goal_angle = handoff["GoalAngleCritic"]
        self.assertIs(goal_angle["enabled"], True)
        self.assertGreater(
            goal_angle["cost_weight"], handoff["GoalCritic"]["cost_weight"]
        )
        self.assertGreaterEqual(
            handoff["ConstraintCritic"]["cost_weight"],
            goal_angle["cost_weight"],
        )
        self.assertGreater(
            goal_angle["threshold_to_consider"],
            controller["reverse_goal_checker"]["xy_goal_tolerance"],
        )
        self.assertIs(
            handoff["PathAngleCritic"]["forward_preference"], True
        )
        self.assertIs(
            handoff["PathAlignCritic"]["use_path_orientations"], True
        )

        projected_heading = (
            handoff["vx_max"]
            * handoff["time_steps"]
            * handoff["model_dt"]
            / planner_radius
        )
        self.assertGreater(
            projected_heading,
            controller["reverse_goal_checker"]["yaw_goal_tolerance"],
        )
        smoother = self.params["velocity_smoother"]["ros__parameters"]
        self.assertIs(smoother["scale_velocities"], True)

    def test_precise_forward_tree_uses_registered_goal_checker(self):
        root = ElementTree.parse(PRECISE_FORWARD_BT_FILE).getroot()
        tags = [element.tag for element in root.iter()]
        self.assertIn("RateController", tags)
        self.assertIn("ComputePathToPose", tags)
        self.assertIn("FollowPath", tags)
        self.assertNotIn("ComputeReversePathToPose", tags)
        for forbidden in ("Spin", "BackUp", "Wait"):
            self.assertNotIn(forbidden, tags)

        compute = root.find(".//ComputePathToPose")
        self.assertIsNotNone(compute)
        planner_id = compute.attrib["planner_id"]
        self.assertEqual(planner_id, "GridBased")
        planner = self.params["planner_server"]["ros__parameters"][planner_id]
        self.assertEqual(planner["tolerance"], 0.0)

        follow = root.find(".//FollowPath")
        self.assertIsNotNone(follow)
        self.assertEqual(follow.attrib["controller_id"], "FollowPath")
        checker_id = follow.attrib["goal_checker_id"]
        self.assertEqual(checker_id, "precise_goal_checker")

        controller = self.params["controller_server"]["ros__parameters"]
        self.assertIn(checker_id, controller["goal_checker_plugins"])
        precise = controller[checker_id]
        self.assertEqual(
            precise["plugin"], "nav2_controller::SimpleGoalChecker"
        )
        self.assertAlmostEqual(precise["xy_goal_tolerance"], 0.12)
        self.assertAlmostEqual(precise["yaw_goal_tolerance"], 0.15)
        self.assertIs(precise["stateful"], False)

    def test_bt_plugin_list_retains_required_nav2_nodes(self):
        plugins = set(
            self.params["bt_navigator"]["ros__parameters"][
                "plugin_lib_names"
            ]
        )
        self.assertIn(
            "smartcar_compute_reverse_path_to_pose_action_bt_node", plugins
        )
        self.assertTrue(
            {
                "nav2_compute_path_to_pose_action_bt_node",
                "nav2_follow_path_action_bt_node",
                "nav2_is_path_valid_condition_bt_node",
                "nav2_pipeline_sequence_bt_node",
                "nav2_get_pose_from_path_action_bt_node",
                "nav2_path_expiring_timer_condition",
                "nav2_progress_checker_selector_bt_node",
                "nav2_rate_controller_bt_node",
                "nav2_recovery_node_bt_node",
                "nav2_smoother_selector_bt_node",
            }.issubset(plugins)
        )

    def test_cancelled_reverse_plan_fails_closed(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_reverse_path_to_pose_action.cpp"
        ).read_text(encoding="utf-8")
        cancelled_body = source.split(
            "ComputeReversePathToPoseAction::on_cancelled()", 1
        )[1].split("}", 1)[0]
        self.assertIn("clearPathOutput()", cancelled_body)
        self.assertIn("BT::NodeStatus::FAILURE", cancelled_body)

    def test_virtual_heading_is_safe_for_configured_footprints(self):
        local = ast.literal_eval(
            self.params["local_costmap"]["local_costmap"]["ros__parameters"][
                "footprint"
            ]
        )
        global_footprint = ast.literal_eval(
            self.params["global_costmap"]["global_costmap"]["ros__parameters"][
                "footprint"
            ]
        )
        self.assertEqual(local, global_footprint)
        for footprint in (local, global_footprint):
            vertices = {(round(x, 9), round(y, 9)) for x, y in footprint}
            for x, y in vertices:
                self.assertIn((-x, -y), vertices)

    def test_build_installs_reverse_bt_library_and_tests(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn(
            "add_library(smartcar_compute_reverse_path_to_pose_action_bt_node",
            cmake,
        )
        self.assertIn(
            "target_compile_definitions("
            "smartcar_compute_reverse_path_to_pose_action_bt_node PRIVATE",
            cmake,
        )
        self.assertIn("BT_PLUGIN_EXPORT", cmake)
        self.assertIn("test_reverse_path_utils", cmake)
        self.assertIn("test_reverse_command_filter", cmake)
        self.assertIn("test_reverse_navigation_contracts.py", cmake)
        self.assertIn(
            "add_library(smartcar_reverse_only_mppi_controller", cmake
        )
        self.assertIn("pluginlib_export_plugin_description_file(", cmake)
        self.assertIn("reverse_only_mppi_controller_plugin.xml", cmake)
        self.assertIn("configure_file(", cmake)
        self.assertIn("@ONLY", cmake)
        self.assertTrue(PRECISE_FORWARD_BT_FILE.is_file())
        self.assertTrue(REVERSE_HANDOFF_BT_FILE.is_file())
        self.assertIn("SMARTCAR_NAV2_BEHAVIOR_TREE_FILES", cmake)
        self.assertIn(REVERSE_HANDOFF_BT_FILE.name, cmake)
        self.assertIn("install(DIRECTORY launch config maps", cmake)
        self.assertNotIn(
            f'PATTERN "{PRECISE_FORWARD_BT_FILE.name}" EXCLUDE', cmake
        )
        params_source = PARAMS_FILE.read_text(encoding="utf-8")
        self.assertIn("@SMARTCAR_NAV2_SHARE_DIR@", params_source)
        self.assertNotIn("/root/ros2_ws", params_source)


if __name__ == "__main__":
    unittest.main()
