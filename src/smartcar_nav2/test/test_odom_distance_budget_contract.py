"""Contracts for the runtime odometry anti-detour fuse."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_FILE = PACKAGE_ROOT / "config" / "nav2_params.yaml"
SOURCE_FILE = PACKAGE_ROOT / "src" / "odom_distance_budget_action.cpp"
HEADER_FILE = PACKAGE_ROOT / "include" / "smartcar_nav2" / "odom_distance_budget_action.hpp"
CMAKE_FILE = PACKAGE_ROOT / "CMakeLists.txt"
ACTIVE_ROUTE_TREES = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_precise_w_replanning_and_recovery.xml",
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml",
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_w_replanning_and_recovery.xml",
)

STRICT_PRECISE_TREE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_precise_w_replanning_and_recovery.xml"
)


class TestOdomDistanceBudgetContracts(unittest.TestCase):
    def test_active_route_trees_have_a_top_level_measured_travel_fuse(self):
        for tree_file in ACTIVE_ROUTE_TREES:
            with self.subTest(tree=tree_file.name):
                root = ElementTree.parse(tree_file).getroot()
                behavior_tree = root.find("BehaviorTree")
                self.assertIsNotNone(behavior_tree)
                self.assertEqual(len(behavior_tree), 1)
                budget = behavior_tree[0]
                self.assertEqual(budget.tag, "OdomDistanceBudget")
                self.assertEqual(budget.attrib["goal"], "{goal}")
                self.assertEqual(budget.attrib["odom_topic"], "/odom_combined")
                self.assertEqual(budget.attrib["odom_frame"], "odom_combined")
                self.assertEqual(
                    budget.attrib["robot_base_frame"], "base_footprint"
                )
                if tree_file == STRICT_PRECISE_TREE:
                    self.assertEqual(budget.attrib["max_distance_ratio"], "1.40")
                    self.assertEqual(budget.attrib["distance_slack_m"], "0.40")
                else:
                    self.assertEqual(budget.attrib["max_distance_ratio"], "2.00")
                    self.assertEqual(budget.attrib["distance_slack_m"], "0.80")
                self.assertEqual(budget.attrib["max_odom_step_m"], "0.50")
                self.assertEqual(budget.attrib["odom_timeout_sec"], "0.50")
                self.assertEqual(budget.attrib["initial_odom_wait_sec"], "1.00")
                self.assertEqual(len(budget), 1)

    def test_budget_plugin_is_loaded_by_each_bt_navigator(self):
        document = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))
        plugin_name = "smartcar_odom_distance_budget_action_bt_node"
        for node_name in (
            "bt_navigator",
            "bt_navigator_navigate_through_poses_rclcpp_node",
        ):
            with self.subTest(node=node_name):
                plugins = document[node_name]["ros__parameters"][
                    "plugin_lib_names"
                ]
                self.assertIn(plugin_name, plugins)

    def test_guard_halts_the_navigation_child_without_a_command_publisher(self):
        source = SOURCE_FILE.read_text(encoding="utf-8")
        self.assertIn("haltChild();", source)
        self.assertIn("return BT::NodeStatus::FAILURE;", source)
        self.assertIn("travel_budget_exceeded", source)
        self.assertIn("odom_stale", source)
        self.assertIn("kWaitingForOdom", source)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn(".publish(", source)
        self.assertNotIn("geometry_msgs/msg/twist", source)

    def test_guard_explicitly_spins_its_private_odom_callback_group(self):
        source = SOURCE_FILE.read_text(encoding="utf-8")
        header = HEADER_FILE.read_text(encoding="utf-8")
        self.assertIn("rclcpp::CallbackGroup::SharedPtr callback_group_", header)
        self.assertIn(
            "rclcpp::executors::SingleThreadedExecutor callback_group_executor_",
            header,
        )
        self.assertIn("rclcpp::CallbackGroupType::MutuallyExclusive, false", source)
        self.assertIn(
            "callback_group_executor_.add_callback_group(", source
        )
        self.assertIn(
            "odometry_options.callback_group = callback_group_;", source
        )
        self.assertIn("}, odometry_options);", source)

        tick_source = source[source.index("BT::NodeStatus OdomDistanceBudgetAction::tick()"):]
        self.assertLess(
            tick_source.index("callback_group_executor_.spin_some();"),
            tick_source.index("beginNavigation()"),
        )

    def test_build_installs_the_plugin_and_tracker_test(self):
        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        self.assertIn(
            "add_library(smartcar_odom_distance_budget_action_bt_node", cmake
        )
        self.assertIn("src/odom_distance_budget_action.cpp", cmake)
        self.assertIn("smartcar_odom_distance_budget_action_bt_node", cmake)
        self.assertIn("test_odom_distance_budget_tracker", cmake)


if __name__ == "__main__":
    unittest.main()
