import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "src" / "smartcar_nav2"
PARAMS_FILE = PACKAGE_ROOT / "config" / "nav2_params.yaml"
BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_w_replanning_and_recovery.xml"
)
BT_THROUGH_POSES_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_w_replanning_and_recovery.xml"
)
WAYPOINTS_FILE = (
    PACKAGE_ROOT / "config" / "waypoints" / "default_waypoints.yaml"
)
NAVIGATION_LAUNCH_FILE = PACKAGE_ROOT / "launch" / "navigation_launch.py"
NAV2_BRINGUP_LAUNCH_FILE = PACKAGE_ROOT / "launch" / "nav2_bringup.launch.py"
SAFE_OUTPUT_ZERO_DEADLINE_SEC = 0.40

EXPECTED_FOOTPRINT = [
    [0.168, 0.112],
    [0.168, -0.112],
    [-0.168, -0.112],
    [-0.168, 0.112],
]
UNSUPPORTED_HUMBLE_RPP_KEYS = {
    "max_linear_vel",
    "min_linear_vel",
    "use_approach_linear_velocity_scaling",
    "use_fixed_curvature_lookahead",
    "curvature_lookahead_dist",
    "max_angular_vel",
    "max_lateral_accel",
}


def ros_parameters(config, node_name):
    return config[node_name]["ros__parameters"]


def launch_calls_by_package(source):
    calls = {}
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        function_name = getattr(node.func, "id", None) or getattr(
            node.func, "attr", None
        )
        if function_name not in {"Node", "ComposableNode"}:
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        package = keywords.get("package")
        if not isinstance(package, ast.Constant) or not isinstance(
            package.value, str
        ):
            continue
        remappings = set()
        for candidate in ast.walk(keywords.get("remappings", ast.List())):
            if not isinstance(candidate, ast.Tuple) or len(candidate.elts) != 2:
                continue
            values = []
            for element in candidate.elts:
                if isinstance(element, ast.Constant) and isinstance(
                    element.value, str
                ):
                    values.append(element.value)
            if len(values) == 2:
                remappings.add(tuple(values))
        calls.setdefault(package.value, []).append(remappings)
    return calls


class TestNav2Contracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))
        cls.waypoints = yaml.safe_load(
            WAYPOINTS_FILE.read_text(encoding="utf-8")
        )["waypoints"]

    def test_behavior_server_uses_ackermann_safe_plugins(self):
        self.assertNotIn("recoveries_server", self.params)
        behavior = ros_parameters(self.params, "behavior_server")
        self.assertEqual(
            behavior["behavior_plugins"],
            ["backup", "drive_on_heading", "wait"],
        )
        self.assertEqual(behavior["backup"]["plugin"], "nav2_behaviors/BackUp")
        self.assertEqual(
            behavior["drive_on_heading"]["plugin"],
            "nav2_behaviors/DriveOnHeading",
        )
        self.assertEqual(behavior["wait"]["plugin"], "nav2_behaviors/Wait")
        self.assertNotIn("spin", behavior)
        self.assertNotIn("nav2_recoveries", yaml.safe_dump(self.params))

    def test_behavior_tree_has_no_in_place_rotation(self):
        for behavior_tree in (BT_FILE, BT_THROUGH_POSES_FILE):
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                tags = [element.tag for element in root.iter()]
                self.assertNotIn("Spin", tags)
                self.assertIn("Wait", tags)
                self.assertIn("BackUp", tags)
                self.assertIn("ClearEntireCostmap", tags)

        through_tags = [
            element.tag
            for element in ElementTree.parse(BT_THROUGH_POSES_FILE).getroot().iter()
        ]
        self.assertIn("ComputePathThroughPoses", through_tags)
        self.assertIn("RemovePassedGoals", through_tags)

    def test_bt_navigator_only_overrides_matching_tree(self):
        bt_navigator = ros_parameters(self.params, "bt_navigator")
        self.assertEqual(
            bt_navigator["default_nav_to_pose_bt_xml"], "<bt_xml_file>"
        )
        self.assertEqual(
            bt_navigator["default_nav_through_poses_bt_xml"],
            "<bt_through_poses_xml_file>",
        )
        self.assertNotEqual(
            bt_navigator["default_nav_to_pose_bt_xml"],
            bt_navigator["default_nav_through_poses_bt_xml"],
        )
        self.assertNotIn("navigators", bt_navigator)
        self.assertNotIn("navigate_to_pose", bt_navigator)
        self.assertNotIn("navigate_through_poses", bt_navigator)

    def test_controller_uses_humble_rpp_contract(self):
        controller = ros_parameters(self.params, "controller_server")
        self.assertEqual(controller["goal_checker_plugins"], ["goal_checker"])
        self.assertNotIn("goal_checker_plugin", controller)

        rpp = controller["FollowPath"]
        self.assertEqual(
            rpp["plugin"],
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        )
        self.assertIs(rpp["use_rotate_to_heading"], False)
        self.assertIs(rpp["allow_reversing"], True)
        for key in UNSUPPORTED_HUMBLE_RPP_KEYS:
            self.assertNotIn(key, rpp)

    def test_smac_hybrid_obeys_ackermann_kinematics(self):
        planner = ros_parameters(self.params, "planner_server")["GridBased"]
        self.assertEqual(
            planner["plugin"], "nav2_smac_planner/SmacPlannerHybrid"
        )
        self.assertEqual(planner["motion_model_for_search"], "DUBIN")
        self.assertAlmostEqual(planner["minimum_turning_radius"], 0.40)

    def test_costmaps_use_the_exact_polygon_footprint(self):
        for costmap_name in ("local_costmap", "global_costmap"):
            costmap = self.params[costmap_name][costmap_name]["ros__parameters"]
            self.assertNotIn("robot_radius", costmap)
            self.assertIsInstance(costmap["footprint"], str)
            self.assertEqual(ast.literal_eval(costmap["footprint"]), EXPECTED_FOOTPRINT)

    def test_rpp_inflation_model_matches_both_costmaps(self):
        rpp = ros_parameters(self.params, "controller_server")["FollowPath"]
        cost_scaling_factors = []
        inflation_radii = []
        for costmap_name in ("local_costmap", "global_costmap"):
            costmap = self.params[costmap_name][costmap_name]["ros__parameters"]
            inflation = costmap["inflation_layer"]
            cost_scaling_factors.append(inflation["cost_scaling_factor"])
            inflation_radii.append(inflation["inflation_radius"])

        self.assertTrue(
            all(
                factor == rpp["inflation_cost_scaling_factor"]
                for factor in cost_scaling_factors
            )
        )
        self.assertLessEqual(rpp["cost_scaling_dist"], min(inflation_radii))

    def test_waypoint_follower_defers_semantic_waits_to_mission(self):
        follower = ros_parameters(self.params, "waypoint_follower")
        self.assertIs(follower["stop_on_failure"], True)
        self.assertEqual(
            follower["wait_at_waypoint"]["waypoint_pause_duration"], 100
        )

    def test_velocity_smoother_respects_safety_and_curvature_limits(self):
        smoother = ros_parameters(self.params, "velocity_smoother")
        self.assertGreater(smoother["velocity_timeout"], 0.0)
        self.assertLessEqual(smoother["velocity_timeout"], 0.30)

        minimum_turning_radius = ros_parameters(
            self.params, "planner_server"
        )["GridBased"]["minimum_turning_radius"]
        max_linear = max(
            abs(smoother["max_velocity"][0]),
            abs(smoother["min_velocity"][0]),
        )
        max_angular = max(
            abs(smoother["max_velocity"][2]),
            abs(smoother["min_velocity"][2]),
        )
        self.assertLessEqual(
            max_angular,
            max_linear / minimum_turning_radius + 1.0e-9,
        )

        longitudinal_deceleration = abs(smoother["max_decel"][0])
        self.assertGreater(longitudinal_deceleration, 0.0)
        stop_bound = (
            smoother["velocity_timeout"]
            + max_linear / longitudinal_deceleration
            + 2.0 / smoother["smoothing_frequency"]
        )
        self.assertLessEqual(stop_bound, SAFE_OUTPUT_ZERO_DEADLINE_SEC)

    def test_navigation_launch_has_one_smoothed_command_path(self):
        source = NAVIGATION_LAUNCH_FILE.read_text(encoding="utf-8")
        calls = launch_calls_by_package(source)
        input_remapping = ("cmd_vel", "cmd_vel_nav")
        output_remapping = ("cmd_vel_smoothed", "cmd_vel")

        for package in ("nav2_controller", "nav2_behaviors"):
            self.assertEqual(len(calls[package]), 2)
            self.assertTrue(
                all(input_remapping in remappings for remappings in calls[package])
            )

        self.assertEqual(len(calls["nav2_velocity_smoother"]), 2)
        self.assertTrue(
            all(
                {input_remapping, output_remapping}.issubset(remappings)
                for remappings in calls["nav2_velocity_smoother"]
            )
        )

        wrapper_source = NAV2_BRINGUP_LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertNotIn("nav2_bringup_dir", wrapper_source)
        self.assertIn("'navigation_launch.py'", wrapper_source)

    def test_waypoints_are_valid_and_fit_the_rolling_global_costmap(self):
        global_costmap = self.params["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        footprint = ast.literal_eval(global_costmap["footprint"])
        inflation_radius = global_costmap["inflation_layer"]["inflation_radius"]
        margin_x = max(abs(point[0]) for point in footprint) + inflation_radius
        margin_y = max(abs(point[1]) for point in footprint) + inflation_radius
        margin_x += global_costmap["resolution"]
        margin_y += global_costmap["resolution"]

        for index, waypoint in enumerate(self.waypoints):
            with self.subTest(waypoint=index):
                self.assertEqual(waypoint["frame_id"], "odom_combined")
                pose = waypoint["pose"]
                values = [*pose["position"].values(), *pose["orientation"].values()]
                self.assertTrue(all(math.isfinite(value) for value in values))
                quaternion_norm = math.sqrt(
                    sum(value * value for value in pose["orientation"].values())
                )
                self.assertAlmostEqual(quaternion_norm, 1.0, delta=1.0e-3)

        for index, (start, goal) in enumerate(
            zip(self.waypoints, self.waypoints[1:])
        ):
            with self.subTest(segment=index):
                start_position = start["pose"]["position"]
                goal_position = goal["pose"]["position"]
                self.assertLessEqual(
                    abs(goal_position["x"] - start_position["x"]),
                    global_costmap["width"] / 2.0 - margin_x,
                )
                self.assertLessEqual(
                    abs(goal_position["y"] - start_position["y"]),
                    global_costmap["height"] / 2.0 - margin_y,
                )

    def test_package_declares_direct_runtime_and_test_dependencies(self):
        root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
        exec_dependencies = {element.text for element in root.findall("exec_depend")}
        test_dependencies = {element.text for element in root.findall("test_depend")}
        self.assertTrue(
            {
                "nav2_behaviors",
                "nav2_bt_navigator",
                "nav2_controller",
                "nav2_lifecycle_manager",
                "nav2_planner",
                "nav2_smac_planner",
                "nav2_regulated_pure_pursuit_controller",
                "nav2_smoother",
            }.issubset(exec_dependencies)
        )
        self.assertTrue(
            {
                "launch_testing_ament_cmake",
                "rclpy",
                "lifecycle_msgs",
                "nav_msgs",
                "sensor_msgs",
                "geometry_msgs",
                "tf2_ros",
                "smartcar_safety",
            }.issubset(test_dependencies)
        )


if __name__ == "__main__":
    unittest.main()
