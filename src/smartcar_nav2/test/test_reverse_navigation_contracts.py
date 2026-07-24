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
REVERSE_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_w_replanning_and_recovery.xml"
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

        forward_tags = {
            element.tag for element in ElementTree.parse(FORWARD_BT_FILE).iter()
        }
        self.assertIn("ComputePathToPose", forward_tags)
        self.assertNotIn("ComputeReversePathToPose", forward_tags)

    def test_reverse_tree_replans_and_validates_before_following(self):
        root = ElementTree.parse(REVERSE_BT_FILE).getroot()
        tags = [element.tag for element in root.iter()]
        self.assertIn("RateController", tags)
        self.assertIn("ComputeReversePathToPose", tags)
        self.assertIn("IsPathValid", tags)
        self.assertIn("FollowPath", tags)
        self.assertNotIn("ComputePathToPose", tags)
        for forbidden in ("Spin", "BackUp", "Wait"):
            self.assertNotIn(forbidden, tags)

        reverse_compute = root.find(".//ComputeReversePathToPose")
        self.assertIsNotNone(reverse_compute)
        self.assertEqual(reverse_compute.attrib["planner_id"], "GridBased")
        self.assertEqual(
            float(reverse_compute.attrib["minimum_turning_radius"]), 0.55
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
        vertices = {(round(x, 9), round(y, 9)) for x, y in global_footprint}
        for x, y in vertices:
            self.assertIn((-x, -y), vertices)
        self.assertGreaterEqual(
            max(x for x, _y in global_footprint), max(x for x, _y in local))
        self.assertLessEqual(
            min(x for x, _y in global_footprint), min(x for x, _y in local))

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
        self.assertIn("test_reverse_navigation_contracts.py", cmake)


if __name__ == "__main__":
    unittest.main()
