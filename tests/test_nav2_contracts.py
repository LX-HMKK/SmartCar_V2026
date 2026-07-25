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
BT_REVERSE_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_w_replanning_and_recovery.xml"
)
BT_PRECISE_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_precise_w_replanning_and_recovery.xml"
)
WAYPOINTS_FILE = (
    PACKAGE_ROOT / "config" / "waypoints" / "default_waypoints.yaml"
)
NAV_ONLY_WAYPOINTS_FILE = (
    PACKAGE_ROOT / "config" / "waypoints" / "nav_only.yaml"
)
BRINGUP_COORD_FILE = (
    ROOT / "src" / "smartcar_bringup" / "config" / "bringup_coord.yaml"
)
NAVIGATION_LAUNCH_FILE = PACKAGE_ROOT / "launch" / "navigation_launch.py"
NAV2_BRINGUP_LAUNCH_FILE = PACKAGE_ROOT / "launch" / "nav2_bringup.launch.py"
SAFE_OUTPUT_ZERO_DEADLINE_SEC = 0.40

EXPECTED_LOCAL_FOOTPRINT = [
    [0.27, 0.13],
    [0.27, -0.13],
    [-0.10, -0.13],
    [-0.10, 0.13],
]
EXPECTED_GLOBAL_FOOTPRINT = [
    [0.27, 0.13],
    [0.27, -0.13],
    [-0.27, -0.13],
    [-0.27, 0.13],
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


def planar_yaw(orientation):
    return math.atan2(
        2.0
        * (
            orientation["w"] * orientation["z"]
            + orientation["x"] * orientation["y"]
        ),
        1.0
        - 2.0
        * (
            orientation["y"] * orientation["y"]
            + orientation["z"] * orientation["z"]
        ),
    )


def waypoint_by_id(document, waypoint_id):
    return next(
        waypoint
        for waypoint in document["waypoints"]
        if waypoint["id"] == waypoint_id
    )


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
        cls.default_waypoint_document = yaml.safe_load(
            WAYPOINTS_FILE.read_text(encoding="utf-8")
        )
        cls.nav_only_waypoint_document = yaml.safe_load(
            NAV_ONLY_WAYPOINTS_FILE.read_text(encoding="utf-8")
        )
        cls.bringup_coord = yaml.safe_load(
            BRINGUP_COORD_FILE.read_text(encoding="utf-8")
        )

    def test_unused_motion_servers_are_removed(self):
        self.assertNotIn("recoveries_server", self.params)
        self.assertNotIn("behavior_server", self.params)
        self.assertNotIn("waypoint_follower", self.params)
        self.assertNotIn("nav2_recoveries", yaml.safe_dump(self.params))

    def test_behavior_tree_has_no_in_place_rotation(self):
        for behavior_tree in (
            BT_FILE,
            BT_PRECISE_FILE,
            BT_THROUGH_POSES_FILE,
            BT_REVERSE_FILE,
        ):
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                tags = [element.tag for element in root.iter()]
                self.assertNotIn("Spin", tags)
                # Wait and BackUp removed for Ackermann chassis — cannot
                # rotate in place or reverse as recovery.
                self.assertIn("ClearEntireCostmap", tags)

        through_tags = [
            element.tag
            for element in ElementTree.parse(BT_THROUGH_POSES_FILE).getroot().iter()
        ]
        self.assertIn("ComputePathThroughPoses", through_tags)
        self.assertIn("RemovePassedGoals", through_tags)

    def test_bt_navigator_only_overrides_matching_tree(self):
        bt_navigator = ros_parameters(self.params, "bt_navigator")
        self.assertEqual(bt_navigator["bt_loop_duration"], 20)
        self.assertTrue(
            bt_navigator["default_nav_to_pose_bt_xml"].endswith(
                "navigate_to_pose_w_replanning_and_recovery.xml"
            )
        )
        self.assertTrue(
            bt_navigator["default_nav_through_poses_bt_xml"].endswith(
                "navigate_through_poses_w_replanning_and_recovery.xml"
            )
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
        self.assertEqual(
            controller["goal_checker_plugins"],
            ["goal_checker", "precise_goal_checker", "reverse_goal_checker"],
        )
        precise = controller["precise_goal_checker"]
        self.assertEqual(
            precise["plugin"], "nav2_controller::SimpleGoalChecker"
        )
        self.assertLess(
            precise["xy_goal_tolerance"],
            controller["goal_checker"]["xy_goal_tolerance"],
        )
        self.assertLess(
            precise["yaw_goal_tolerance"],
            controller["goal_checker"]["yaw_goal_tolerance"],
        )
        self.assertAlmostEqual(precise["xy_goal_tolerance"], 0.12)
        self.assertAlmostEqual(precise["yaw_goal_tolerance"], 0.15)
        self.assertIs(precise["stateful"], False)
        self.assertLessEqual(
            controller["reverse_goal_checker"]["xy_goal_tolerance"], 0.12)
        self.assertLessEqual(
            controller["reverse_goal_checker"]["yaw_goal_tolerance"], 0.25)
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
        self.assertNotIn("reverse_penalty", planner)
        # Controller goal checkers target path.poses.back(), so a nonzero
        # planner fallback could bypass the precise QR waypoint envelope.
        self.assertEqual(planner["tolerance"], 0.0)
        minimum_turning_radius = planner["minimum_turning_radius"]
        self.assertAlmostEqual(minimum_turning_radius, 0.55)

        calibration = self.bringup_coord["calibration"]
        wheelbase = float(calibration["wheelbase"])
        max_steering_angle = float(calibration["max_steering_angle"])
        required_steering_angle = math.atan(
            wheelbase / minimum_turning_radius
        )
        self.assertLessEqual(required_steering_angle, max_steering_angle)

    def test_costmaps_use_real_and_virtual_yaw_safe_footprints(self):
        local = self.params["local_costmap"]["local_costmap"]["ros__parameters"]
        global_costmap = self.params["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        self.assertEqual(
            ast.literal_eval(local["footprint"]), EXPECTED_LOCAL_FOOTPRINT)
        self.assertEqual(
            ast.literal_eval(global_costmap["footprint"]),
            EXPECTED_GLOBAL_FOOTPRINT,
        )
        self.assertEqual(local["footprint_padding"], 0.03)
        self.assertEqual(global_costmap["footprint_padding"], 0.03)

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
        # DUBIN handles kinematic constraints in both real and virtual poses;
        # velocity_smoother limits are platform-level caps, not derived from
        # the planner's minimum turning radius.
        self.assertGreater(max_angular, 0.0)
        self.assertLessEqual(max_angular, 1.0)

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
        output_remapping = ("cmd_vel_smoothed", "cmd_vel_candidate")

        self.assertEqual(len(calls["nav2_controller"]), 1)
        self.assertTrue(
            all(
                input_remapping in remappings
                for remappings in calls["nav2_controller"]
            )
        )
        self.assertNotIn("nav2_behaviors", calls)
        self.assertNotIn("nav2_waypoint_follower", calls)

        self.assertEqual(len(calls["nav2_velocity_smoother"]), 1)
        self.assertTrue(
            all(
                {input_remapping, output_remapping}.issubset(remappings)
                for remappings in calls["nav2_velocity_smoother"]
            )
        )

        wrapper_source = NAV2_BRINGUP_LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertNotIn("nav2_bringup_dir", wrapper_source)
        self.assertIn("'navigation_launch.py'", wrapper_source)

    def test_navigation_launch_omits_unused_path_smoother(self):
        source = NAVIGATION_LAUNCH_FILE.read_text(encoding="utf-8")
        calls = launch_calls_by_package(source)
        self.assertNotIn("nav2_smoother", calls)
        self.assertNotIn("smoother_server", source)
        self.assertNotIn("use_waypoint_follower", source)
        self.assertNotIn("use_composition", source)

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

    def test_precise_reverse_handoff_geometry_matches_both_routes(self):
        default = self.default_waypoint_document
        nav_only = self.nav_only_waypoint_document
        self.assertEqual(
            [waypoint["id"] for waypoint in default["waypoints"]],
            [waypoint["id"] for waypoint in nav_only["waypoints"]],
        )
        for default_waypoint, nav_waypoint in zip(
            default["waypoints"], nav_only["waypoints"]
        ):
            with self.subTest(waypoint=default_waypoint["id"]):
                self.assertEqual(
                    default_waypoint["pose"], nav_waypoint["pose"]
                )
                self.assertEqual(
                    default_waypoint["direction"],
                    nav_waypoint["direction"],
                )
                self.assertEqual(
                    default_waypoint.get("goal_profile", "standard"),
                    nav_waypoint.get("goal_profile", "standard"),
                )

        precise_ids = [
            waypoint["id"]
            for waypoint in default["waypoints"]
            if waypoint.get("goal_profile", "standard") == "precise"
        ]
        self.assertEqual(precise_ids, ["a_task_observe"])
        qr = waypoint_by_id(default, "a_task_observe")
        corridor_enter = waypoint_by_id(default, "b_corridor_enter")
        corridor_out = waypoint_by_id(default, "b_corridor_out")
        self.assertEqual(qr["direction"], "forward")
        self.assertEqual(corridor_enter["direction"], "reverse")
        self.assertEqual(corridor_out["direction"], "reverse")

        qr_yaw = planar_yaw(qr["pose"]["orientation"])
        corridor_yaw = planar_yaw(
            corridor_enter["pose"]["orientation"]
        )
        self.assertAlmostEqual(qr_yaw, math.radians(30.0), delta=1.0e-6)
        self.assertAlmostEqual(
            corridor_yaw, math.radians(-70.0), delta=1.0e-6
        )
        self.assertAlmostEqual(
            math.remainder(corridor_yaw - qr_yaw, 2.0 * math.pi),
            math.radians(-100.0),
            delta=1.0e-6,
        )

        corridor_position = corridor_enter["pose"]["position"]
        corridor_out_position = corridor_out["pose"]["position"]
        motion_yaw = math.atan2(
            corridor_out_position["y"] - corridor_position["y"],
            corridor_out_position["x"] - corridor_position["x"],
        )
        reverse_body_tangent = math.remainder(
            motion_yaw - math.pi, 2.0 * math.pi
        )
        tangent_error = abs(
            math.remainder(
                corridor_yaw - reverse_body_tangent, 2.0 * math.pi
            )
        )
        self.assertLess(tangent_error, math.radians(0.1))

        radius = ros_parameters(self.params, "planner_server")["GridBased"][
            "minimum_turning_radius"
        ]
        qr_position = qr["pose"]["position"]
        virtual_qr_yaw = qr_yaw + math.pi
        virtual_corridor_yaw = corridor_yaw + math.pi
        goal_left_center = (
            corridor_position["x"]
            - radius * math.sin(virtual_corridor_yaw),
            corridor_position["y"]
            + radius * math.cos(virtual_corridor_yaw),
        )
        precise = ros_parameters(self.params, "controller_server")[
            "precise_goal_checker"
        ]
        yaw_tolerance = precise["yaw_goal_tolerance"]
        yaw_lower = virtual_qr_yaw - yaw_tolerance
        yaw_upper = virtual_qr_yaw + yaw_tolerance
        candidate_yaws = [yaw_lower, yaw_upper]

        # The start-circle distance can have an interior extremum. Include
        # every stationary angle in the accepted yaw interval, not just its
        # endpoints, so future waypoint edits cannot evade this contract.
        center_dx = goal_left_center[0] - qr_position["x"]
        center_dy = goal_left_center[1] - qr_position["y"]
        stationary_yaw = math.atan2(center_dx, -center_dy)
        for base_yaw in (stationary_yaw, stationary_yaw + math.pi):
            first_turn = math.ceil(
                (yaw_lower - base_yaw) / (2.0 * math.pi)
            )
            last_turn = math.floor(
                (yaw_upper - base_yaw) / (2.0 * math.pi)
            )
            candidate_yaws.extend(
                base_yaw + turn * 2.0 * math.pi
                for turn in range(first_turn, last_turn + 1)
            )

        center_distances = []
        for start_yaw in candidate_yaws:
            start_right_center = (
                qr_position["x"] + radius * math.sin(start_yaw),
                qr_position["y"] - radius * math.cos(start_yaw),
            )
            center_distances.append(
                math.hypot(
                    goal_left_center[0] - start_right_center[0],
                    goal_left_center[1] - start_right_center[1],
                )
            )

        # Reverse triangle inequality covers any accepted XY error vector.
        # This only locks a conservative RSL tangent margin at the configured
        # yaw interval; it is not a complete Dubins shortest-path proof.
        conservative_center_distance = (
            min(center_distances) - precise["xy_goal_tolerance"]
        )
        self.assertGreater(conservative_center_distance, 2.0 * radius)

    def test_package_declares_direct_runtime_and_test_dependencies(self):
        root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
        exec_dependencies = {element.text for element in root.findall("exec_depend")}
        direct_dependencies = {element.text for element in root.findall("depend")}
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
            }.issubset(test_dependencies | direct_dependencies)
        )


if __name__ == "__main__":
    unittest.main()
