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
BT_REVERSE_THROUGH_POSES_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_w_replanning_and_recovery.xml"
)
BT_REVERSE_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_w_replanning_and_recovery.xml"
)
BT_REVERSE_HANDOFF_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml"
)
BT_REVERSE_LOCKED_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_locked_w_replanning_and_recovery.xml"
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
    [-0.27, -0.13],
    [-0.27, 0.13],
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
DEFAULT_HEADING_LOCKED_TASKS = frozenset({"start", "qr", "vlm", "return"})


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


def effective_heading_mode(waypoint):
    """Mirror the YAML contract without treating every ``nav`` point as free."""
    return waypoint.get(
        "heading_mode",
        "locked" if waypoint["task"] in DEFAULT_HEADING_LOCKED_TASKS else "free",
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
            BT_REVERSE_HANDOFF_FILE,
            BT_REVERSE_LOCKED_FILE,
        ):
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                tags = [element.tag for element in root.iter()]
                self.assertNotIn("Spin", tags)
                # Stock behaviors require behavior_server and do not preserve
                # the task direction lease. The project recovery remains a
                # RecordFollowPath still dispatches through controller_server.
                self.assertIn("ClearEntireCostmap", tags)
                self.assertNotIn("BackUp", tags)
                self.assertNotIn("DriveOnHeading", tags)
                self.assertNotIn("Wait", tags)
                self.assertNotIn("FollowPath", tags)
                for follow_path in root.iter("RecordFollowPath"):
                    self.assertNotIn("allow_reversing", follow_path.attrib)

        through_tags = [
            element.tag
            for element in ElementTree.parse(BT_THROUGH_POSES_FILE).getroot().iter()
        ]
        reverse_through_tags = [
            element.tag
            for element in ElementTree.parse(
                BT_REVERSE_THROUGH_POSES_FILE
            ).getroot().iter()
        ]
        self.assertIn("ComputeFreeHeadingPathThroughPoses", through_tags)
        self.assertIn(
            "ComputeReverseFreeHeadingPathThroughPoses", reverse_through_tags
        )
        self.assertIn("RemovePassedGoals", through_tags)
        for behavior_tree in (
            BT_THROUGH_POSES_FILE,
            BT_REVERSE_THROUGH_POSES_FILE,
        ):
            remove_passed_goals = list(
                ElementTree.parse(behavior_tree).getroot().iter(
                    "RemovePassedGoals")
            )
            self.assertEqual(len(remove_passed_goals), 1)
            self.assertEqual(
                remove_passed_goals[0].attrib.get("robot_base_frame"),
                "base_footprint",
            )

    def test_record_follow_path_publishes_only_acknowledged_goals(self):
        source = (
            PACKAGE_ROOT / "src" / "record_follow_path_action.cpp"
        ).read_text(encoding="utf-8")
        header = (
            PACKAGE_ROOT / "include" / "smartcar_nav2"
            / "record_follow_path_action.hpp"
        ).read_text(encoding="utf-8")
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

        self.assertIn("RecordFollowPathAction", header)
        self.assertIn('"/smartcar/accepted_global_plan"', source)
        self.assertIn("rclcpp::QoS(1).reliable().transient_local()", source)
        self.assertIn("send_goal_options.goal_response_callback", source)
        self.assertIn("handleGoalResponse(generation, accepted_path", source)
        self.assertIn("cancelled_dispatch_generation_ >= generation", source)
        self.assertIn("async_cancel_goal(goal_handle)", source)
        self.assertIn("void RecordFollowPathAction::halt()", source)
        self.assertIn("cancelOutstandingDispatch();", source)
        self.assertIn("goalAcknowledgementTimeout", source)
        self.assertIn("kMinimumGoalAcknowledgementTimeout{500}", source)
        self.assertIn("startLateGoalAcknowledgementGuard();", source)
        self.assertIn("late_goal_acknowledgement_pending_", source)
        self.assertIn("callback_group_executor_.spin_until_future_complete", source)
        self.assertNotIn(
            "action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>",
            source,
        )
        self.assertIn("publishAcceptedPath(path);", source)
        self.assertIn('"RecordFollowPath"', source)
        self.assertIn(
            "smartcar_record_follow_path_action_bt_node", cmake)

        active_trees = (
            BT_FILE,
            BT_PRECISE_FILE,
            BT_THROUGH_POSES_FILE,
            BT_REVERSE_FILE,
            BT_REVERSE_HANDOFF_FILE,
            BT_REVERSE_THROUGH_POSES_FILE,
            BT_REVERSE_LOCKED_FILE,
        )
        for behavior_tree in active_trees:
            with self.subTest(behavior_tree=behavior_tree.name):
                tags = [
                    element.tag
                    for element in ElementTree.parse(behavior_tree).getroot().iter()
                ]
                self.assertEqual(tags.count("RecordFollowPath"), 1)
                self.assertNotIn("FollowPath", tags)

        for navigator in (
            "bt_navigator",
            "bt_navigator_navigate_through_poses_rclcpp_node",
        ):
            with self.subTest(navigator=navigator):
                self.assertIn(
                    "smartcar_record_follow_path_action_bt_node",
                    ros_parameters(self.params, navigator)["plugin_lib_names"],
                )

        retreat_source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn('"/smartcar/accepted_global_plan"', retreat_source)
        self.assertIn("[this, generation, accepted_path]", retreat_source)
        self.assertIn("accepted_path_publisher_->publish(accepted_path);", retreat_source)

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

    def test_bt_service_timeout_allows_costmap_recovery_round_trip(self):
        primary = ros_parameters(self.params, "bt_navigator")
        through_poses = ros_parameters(
            self.params, "bt_navigator_navigate_through_poses_rclcpp_node"
        )
        self.assertEqual(primary["default_server_timeout"], 100)
        self.assertEqual(
            through_poses["default_server_timeout"],
            primary["default_server_timeout"],
        )
        self.assertGreaterEqual(
            primary["default_server_timeout"],
            5 * primary["bt_loop_duration"],
        )

    def test_controller_uses_forward_rpp_and_mppi_handoff_contracts(self):
        controller = ros_parameters(self.params, "controller_server")
        self.assertEqual(controller["odom_topic"], "/odom_combined")
        self.assertEqual(
            controller["goal_checker_plugins"],
            [
                "goal_checker",
                "precise_goal_checker",
                "reverse_goal_checker",
                "transit_goal_checker",
                "recovery_goal_checker",
            ],
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
            controller["reverse_goal_checker"]["xy_goal_tolerance"], 0.30)
        self.assertLessEqual(
            controller["reverse_goal_checker"]["yaw_goal_tolerance"], 0.25)
        transit = controller["transit_goal_checker"]
        self.assertEqual(
            transit["plugin"], "nav2_controller::PositionGoalChecker"
        )
        self.assertAlmostEqual(transit["xy_goal_tolerance"], 0.50)
        self.assertNotIn("yaw_goal_tolerance", transit)
        self.assertIs(transit["stateful"], False)
        self.assertNotIn("goal_checker_plugin", controller)
        self.assertEqual(
            controller["controller_plugins"],
            [
                "FollowPath",
                "ForwardAvoidance",
                "ForwardHandoff",
                "ReverseHandoff",
                "ReverseRecovery",
            ],
        )

        rpp = controller["FollowPath"]
        self.assertEqual(
            rpp["plugin"],
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        )
        self.assertIs(rpp["use_rotate_to_heading"], False)
        self.assertIs(rpp["allow_reversing"], True)
        for key in UNSUPPORTED_HUMBLE_RPP_KEYS:
            self.assertNotIn(key, rpp)

        forward = controller["ForwardAvoidance"]
        self.assertEqual(
            forward["plugin"], "smartcar_nav2::ForwardOnlyRPPController"
        )
        self.assertAlmostEqual(forward["desired_linear_vel"], 0.15)
        self.assertGreater(forward["forward_max_angular_velocity"], 0.0)
        self.assertIs(forward["allow_reversing"], False)
        self.assertIs(forward["use_rotate_to_heading"], False)
        self.assertIs(forward["use_collision_detection"], True)

        planner_radius = ros_parameters(self.params, "planner_server")[
            "GridBased"
        ]["minimum_turning_radius"]
        forward_handoff = controller["ForwardHandoff"]
        self.assertEqual(
            forward_handoff["plugin"], "smartcar_nav2::ForwardOnlyMPPIController"
        )
        self.assertEqual(forward_handoff["motion_model"], "Ackermann")
        self.assertAlmostEqual(forward_handoff["vx_min"], 0.02)
        self.assertAlmostEqual(forward_handoff["vx_max"], 0.15)
        self.assertGreater(forward_handoff["vx_min"], 0.0)
        self.assertLess(forward_handoff["vx_min"], forward_handoff["vx_max"])
        self.assertAlmostEqual(
            forward_handoff["AckermannConstraints"]["min_turning_r"],
            planner_radius,
        )
        self.assertGreaterEqual(
            forward_handoff["wz_max"],
            forward_handoff["vx_max"] / planner_radius,
        )
        self.assertIs(forward_handoff["CostCritic"]["consider_footprint"], True)
        self.assertIn("GoalAngleCritic", forward_handoff["critics"])
        self.assertIn("PathAngleCritic", forward_handoff["critics"])

        handoff = controller["ReverseHandoff"]
        self.assertEqual(
            handoff["plugin"], "smartcar_nav2::ReverseOnlyMPPIController"
        )
        self.assertEqual(handoff["motion_model"], "Ackermann")
        self.assertAlmostEqual(
            handoff["AckermannConstraints"]["min_turning_r"], planner_radius
        )
        self.assertAlmostEqual(
            forward["forward_min_turning_radius"], planner_radius
        )
        self.assertLessEqual(
            forward["desired_linear_vel"] /
            forward["forward_max_angular_velocity"], planner_radius + 0.01
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
        self.assertIs(handoff["regenerate_noises"], False)

        critics = handoff["critics"]
        self.assertIn("ConstraintCritic", critics)
        self.assertIn("CostCritic", critics)
        self.assertIn("GoalCritic", critics)
        self.assertIn("GoalAngleCritic", critics)
        self.assertIn("PathAngleCritic", critics)
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
        self.assertIs(handoff["CostCritic"]["consider_footprint"], True)

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

        recovery = controller["ReverseRecovery"]
        self.assertEqual(
            recovery["plugin"], "smartcar_nav2::ReverseOnlyMPPIController"
        )
        self.assertEqual(recovery["motion_model"], "Ackermann")
        self.assertAlmostEqual(recovery["vx_min"], 0.015)
        self.assertAlmostEqual(recovery["vx_max"], 0.05)
        self.assertAlmostEqual(recovery["wz_max"], 0.0)
        self.assertLess(recovery["vx_min"], recovery["vx_max"])
        self.assertEqual(
            recovery["AckermannConstraints"]["min_turning_r"], planner_radius
        )
        recovery_goal = controller["recovery_goal_checker"]
        self.assertEqual(
            recovery_goal["plugin"], "nav2_controller::SimpleGoalChecker"
        )
        self.assertLess(recovery_goal["xy_goal_tolerance"], 0.15)
        self.assertAlmostEqual(recovery_goal["yaw_goal_tolerance"], 0.10)
        self.assertIs(recovery_goal["stateful"], False)

        smoother = ros_parameters(self.params, "velocity_smoother")
        self.assertIs(smoother["scale_velocities"], True)

    def test_reverse_handoff_tree_keeps_strict_planning_and_goal_checks(self):
        regular = ElementTree.parse(BT_REVERSE_FILE).getroot()
        handoff = ElementTree.parse(BT_REVERSE_HANDOFF_FILE).getroot()
        regular_compute = regular.find(".//ComputeReverseFreeHeadingPathToPose")
        handoff_compute = handoff.find(
            ".//ComputeReverseFreeHeadingPathToPose"
        )
        self.assertIsNotNone(regular_compute)
        self.assertIsNotNone(handoff_compute)
        # Ordinary reverse waypoints carry a zero quaternion and resolve their
        # terminal heading in the live costmap. The VLM handoff is heading
        # locked, so the shared reverse node retains its single authored-yaw
        # candidate while also providing the same safe retreat barrier.
        self.assertIn("heading_samples", regular_compute.attrib)
        self.assertIn("heading_samples", handoff_compute.attrib)
        for field in (
            "goal",
            "path",
            "planner_id",
            "curvature_tolerance",
            "maximum_direction_error",
            "start_position_tolerance",
            "start_yaw_tolerance",
            "goal_position_tolerance",
            "goal_yaw_tolerance",
            "minimum_segment_length",
        ):
            self.assertEqual(
                handoff_compute.attrib[field], regular_compute.attrib[field]
            )
        self.assertNotIn("minimum_turning_radius", regular_compute.attrib)
        self.assertNotIn("minimum_turning_radius", handoff_compute.attrib)

        follow = handoff.find(".//RecordFollowPath")
        self.assertIsNotNone(follow)
        self.assertEqual(follow.attrib["controller_id"], "ReverseHandoff")
        self.assertEqual(
            follow.attrib["goal_checker_id"], "reverse_goal_checker")

    def test_reverse_retreat_is_planner_only_and_single_shot(self):
        forward_trees = (BT_FILE, BT_PRECISE_FILE, BT_THROUGH_POSES_FILE)
        for behavior_tree in forward_trees:
            with self.subTest(behavior_tree=behavior_tree.name):
                tags = {
                    element.tag
                    for element in ElementTree.parse(behavior_tree).getroot().iter()
                }
                self.assertNotIn("AckermannReverseRetreat", tags)

        reverse_trees = (
            BT_REVERSE_FILE,
            BT_REVERSE_HANDOFF_FILE,
            BT_REVERSE_THROUGH_POSES_FILE,
            BT_REVERSE_LOCKED_FILE,
        )
        for behavior_tree in reverse_trees:
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                retreats = list(root.iter("AckermannReverseRetreat"))
                self.assertEqual(len(retreats), 1)
                retreat = retreats[0]
                self.assertEqual(retreat.attrib["controller_id"], "ReverseRecovery")
                self.assertEqual(
                    retreat.attrib["goal_checker_id"], "recovery_goal_checker"
                )
                self.assertEqual(retreat.attrib["retreat_used"], "{reverse_retreat_used}")
                self.assertAlmostEqual(float(retreat.attrib["retreat_distance_m"]), 0.15)
                self.assertEqual(retreat.attrib["follow_path_result_timeout_ms"], "12000")
                self.assertEqual(retreat.attrib["scan_min_obstacle_range_m"], "0.25")
                self.assertEqual(retreat.attrib["scan_max_obstacle_range_m"], "2.50")
                self.assertEqual(retreat.attrib["scan_costmap_match_radius_m"], "0.12")
                self.assertEqual(retreat.attrib["retreat_odom_max_age_ms"], "500")
                self.assertEqual(retreat.attrib["retreat_odom_max_step_m"], "0.05")
                self.assertEqual(retreat.attrib["retreat_odom_max_travel_m"], "0.19")
                self.assertEqual(retreat.attrib["retreat_odom_max_displacement_m"], "0.19")
                self.assertNotIn("static_keepout_mask_topic", retreat.attrib)
                self.assertEqual(len(list(root.iter("SetBlackboard"))), 1)
                self.assertNotIn("BackUp", {element.tag for element in root.iter()})
                self.assertNotIn("DriveOnHeading", {element.tag for element in root.iter()})

        primary_plugins = ros_parameters(self.params, "bt_navigator")["plugin_lib_names"]
        through_plugins = ros_parameters(
            self.params, "bt_navigator_navigate_through_poses_rclcpp_node"
        )["plugin_lib_names"]
        self.assertIn(
            "smartcar_ackermann_reverse_retreat_action_bt_node", primary_plugins
        )
        self.assertIn(
            "smartcar_ackermann_reverse_retreat_action_bt_node", through_plugins
        )

    def test_smac_hybrid_obeys_ackermann_kinematics(self):
        planner = ros_parameters(self.params, "planner_server")["GridBased"]
        self.assertEqual(
            planner["plugin"], "nav2_smac_planner/SmacPlannerHybrid"
        )
        self.assertEqual(planner["motion_model_for_search"], "DUBIN")
        self.assertNotIn("reverse_penalty", planner)
        # Controller goal checkers target path.poses.back().  With 144-bin
        # angle quantization the planner has sufficient resolution to land
        # within a 5 cm envelope, which is well inside the precise goal
        # checker xy tolerance (0.12 m).  Zero tolerance forces winding
        # approach paths that the Ackermann chassis cannot track.
        self.assertLessEqual(planner["tolerance"], 0.05)
        minimum_turning_radius = planner["minimum_turning_radius"]
        self.assertAlmostEqual(minimum_turning_radius, 0.22)
        # The Humble Smac smoother does not preserve the Hybrid-A* curvature
        # constraint. A reverse path must retain the DUBIN lattice geometry.
        self.assertIs(planner["smooth_path"], False)

        calibration = self.bringup_coord["calibration"]
        wheelbase = float(calibration["wheelbase"])
        max_steering_angle = float(calibration["max_steering_angle"])
        # 0.70 rad is the direct-pass-through command proven on the vehicle.
        # The planner keeps a 2 cm radius margin above the measured 0.20 m
        # ground circle. The ideal bicycle estimate is only a sanity check:
        # its small residual exceeds the measured command limit by < 0.01 rad.
        required_steering_angle = math.atan(wheelbase / minimum_turning_radius)
        self.assertAlmostEqual(max_steering_angle, 0.70)
        self.assertLessEqual(
            required_steering_angle,
            max_steering_angle + 0.01,
        )

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

    def test_reverse_mppi_wrapper_is_built_and_uses_portable_bt_paths(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        params_source = PARAMS_FILE.read_text(encoding="utf-8")
        plugin_xml = (
            PACKAGE_ROOT / "reverse_only_mppi_controller_plugin.xml"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "add_library(smartcar_reverse_only_mppi_controller", cmake
        )
        self.assertIn("pluginlib_export_plugin_description_file(", cmake)
        self.assertIn("reverse_only_mppi_controller_plugin.xml", cmake)
        self.assertIn("test_reverse_command_filter", cmake)
        self.assertIn("forward_only_mppi_controller_plugin.xml", cmake)
        self.assertIn("forward_only_rpp_controller_plugin.xml", cmake)
        self.assertIn("test_forward_command_filter", cmake)
        self.assertIn("test_forward_path_tracking_guard", cmake)
        self.assertIn("add_library(smartcar_forward_only_mppi_controller", cmake)
        self.assertIn("add_library(smartcar_forward_only_rpp_controller", cmake)
        self.assertIn("configure_file(", cmake)
        self.assertIn("@ONLY", cmake)
        self.assertIn("smartcar_nav2::ReverseOnlyMPPIController", plugin_xml)
        self.assertIn("@SMARTCAR_NAV2_SHARE_DIR@", params_source)
        self.assertNotIn("/root/ros2_ws", params_source)

    def test_forward_controllers_keep_costmap_collision_contracts(self):
        forward = ros_parameters(self.params, "controller_server")[
            "ForwardAvoidance"
        ]
        forward_handoff = ros_parameters(self.params, "controller_server")[
            "ForwardHandoff"
        ]
        inflation_radii = []
        for costmap_name in ("local_costmap", "global_costmap"):
            costmap = self.params[costmap_name][costmap_name]["ros__parameters"]
            inflation = costmap["inflation_layer"]
            inflation_radii.append(inflation["inflation_radius"])

        self.assertTrue(all(radius > 0.0 for radius in inflation_radii))
        self.assertTrue(forward["use_collision_detection"])
        self.assertGreater(
            forward["max_allowed_time_to_collision_up_to_carrot"], 0.0)
        self.assertTrue(forward["use_cost_regulated_linear_velocity_scaling"])
        self.assertTrue(forward_handoff["CostCritic"]["consider_footprint"])
        rpp_source = (PACKAGE_ROOT / "src" / "forward_only_rpp_controller.cpp").read_text(
            encoding="utf-8")
        self.assertIn("void ForwardOnlyRPPController::setPlan", rpp_source)
        self.assertIn("projectForwardPathTrackingPose", rpp_source)
        self.assertIn("forward_path_max_cross_track_error", rpp_source)
        self.assertIn("forwardPathTrackingTerminalLookaheadActive", rpp_source)
        self.assertIn("forward_terminal_lookahead_m", rpp_source)
        self.assertIn("inCollision(", rpp_source)
        self.assertIn("nav2_core::PlannerException", rpp_source)

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

    def test_navigation_launch_uses_the_requested_resolved_parameter_files(self):
        source = NAVIGATION_LAUNCH_FILE.read_text(encoding="utf-8")
        wrapper = NAV2_BRINGUP_LAUNCH_FILE.read_text(encoding="utf-8")

        self.assertIn("params_file.perform(context)", source)
        self.assertIn("params_overlay_file.perform(context)", source)
        self.assertIn("nav2_params_fixed.yaml", source)
        self.assertIn("params_overlay_file", wrapper)
        self.assertIn("'params_overlay_file': params_overlay_file", wrapper)

    def test_lifecycle_manager_startup_delay_defaults_to_zero_and_is_forwarded(self):
        navigation = NAVIGATION_LAUNCH_FILE.read_text(encoding="utf-8")
        bringup = NAV2_BRINGUP_LAUNCH_FILE.read_text(encoding="utf-8")
        wrapper = (PACKAGE_ROOT / "launch" / "smartcar_nav2.launch.py").read_text(
            encoding="utf-8"
        )

        for source in (navigation, bringup, wrapper):
            self.assertIn("lifecycle_manager_delay_sec", source)
            self.assertIn("default_value='0.0'", source)
        self.assertIn("TimerAction(period=delay_sec", navigation)
        self.assertIn(
            "'lifecycle_manager_delay_sec': lifecycle_manager_delay_sec", bringup
        )
        self.assertIn(
            "'lifecycle_manager_delay_sec': lifecycle_manager_delay_sec", wrapper
        )

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
                self.assertTrue(
                    all(math.isfinite(value) for value in pose["position"].values())
                )
                heading_mode = effective_heading_mode(waypoint)
                self.assertIn(heading_mode, {"free", "locked"})
                if heading_mode == "locked":
                    orientation = pose["orientation"]
                    self.assertTrue(all(math.isfinite(value) for value in orientation.values()))
                    quaternion_norm = math.sqrt(
                        sum(value * value for value in orientation.values())
                    )
                    self.assertLessEqual(abs(quaternion_norm - 1.0), 1.0e-3)
                else:
                    # An omitted YAML orientation is parsed as the explicit
                    # all-zero position-only sentinel before Nav2 receives it.
                    self.assertNotIn("orientation", pose)
                    self.assertEqual(
                        pose.get(
                            "orientation",
                            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
                        ),
                        {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
                    )

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

    def test_nav_qr_substitute_can_explicitly_lock_its_heading(self):
        substitute = waypoint_by_id(
            self.nav_only_waypoint_document, "a_task_observe"
        )

        self.assertEqual(substitute["task"], "nav")
        self.assertEqual(substitute["heading_mode"], "locked")
        self.assertEqual(effective_heading_mode(substitute), "locked")
        orientation = substitute["pose"]["orientation"]
        quaternion_norm = math.sqrt(sum(value * value for value in orientation.values()))
        self.assertLessEqual(abs(quaternion_norm - 1.0), 1.0e-3)

    def test_simulation_route_preserves_protected_semantic_targets(self):
        default = self.default_waypoint_document
        nav_only = self.nav_only_waypoint_document
        semantic_ids = (
            "p_start", "a_task_observe", "c_corner_1", "p_finish",
        )
        default_by_id = {
            waypoint["id"]: waypoint for waypoint in default["waypoints"]
        }
        nav_by_id = {
            waypoint["id"]: waypoint for waypoint in nav_only["waypoints"]
        }

        self.assertEqual(
            [waypoint["id"] for waypoint in default["waypoints"]],
            list(semantic_ids),
        )
        self.assertEqual(
            [waypoint["id"] for waypoint in nav_only["waypoints"]],
            list(semantic_ids),
        )
        self.assertEqual(
            [waypoint["task"] for waypoint in default["waypoints"]],
            ["start", "qr", "vlm", "return"],
        )
        self.assertEqual(
            [waypoint["task"] for waypoint in nav_only["waypoints"]],
            ["start", "nav", "nav", "return"],
        )

        # nav_only suppresses media requests, but it must use the exact
        # protected physical targets and locked headings of the full mission.
        for waypoint_id in semantic_ids:
            with self.subTest(waypoint=waypoint_id):
                self.assertEqual(
                    default_by_id[waypoint_id]["pose"]["position"],
                    nav_by_id[waypoint_id]["pose"]["position"],
                )
                self.assertEqual(
                    default_by_id[waypoint_id]["direction"],
                    nav_by_id[waypoint_id]["direction"],
                )
                self.assertEqual(
                    default_by_id[waypoint_id].get("goal_profile", "standard"),
                    nav_by_id[waypoint_id].get("goal_profile", "standard"),
                )
                self.assertEqual(
                    default_by_id[waypoint_id]["pose"]["orientation"],
                    nav_by_id[waypoint_id]["pose"]["orientation"],
                )
                self.assertEqual(
                    effective_heading_mode(default_by_id[waypoint_id]), "locked"
                )
                self.assertEqual(
                    effective_heading_mode(nav_by_id[waypoint_id]), "locked"
                )
                quaternion_norm = math.sqrt(sum(
                    value * value
                    for value in nav_by_id[waypoint_id]["pose"]["orientation"].values()
                ))
                self.assertLessEqual(abs(quaternion_norm - 1.0), 1.0e-3)

        self.assertEqual(nav_by_id["a_task_observe"]["task"], "nav")
        self.assertEqual(nav_by_id["c_corner_1"]["task"], "nav")
        self.assertEqual(nav_by_id["c_corner_1"]["direction"], "reverse")
        self.assertEqual(
            nav_by_id["c_corner_1"].get("goal_profile", "standard"),
            "reverse_handoff",
        )
        self.assertEqual(nav_by_id["p_finish"]["direction"], "forward")
        self.assertFalse(
            {"via", "corridor", "loop"}
            & {waypoint["task"] for waypoint in default["waypoints"]}
        )
        self.assertFalse(
            {
                "b_corridor_gate", "b_corridor_enter", "c_entry_west",
                "c_corner_2", "c_corner_3", "c_corner_4",
                "b_corridor_return_enter", "b_corridor_return_drop",
                "b_corridor_return",
            }
            & set(nav_by_id)
        )

        precise_ids = [
            waypoint["id"]
            for waypoint in default["waypoints"]
            if waypoint.get("goal_profile", "standard") == "precise"
        ]
        self.assertEqual(precise_ids, ["a_task_observe"])
        reverse_handoff_ids = [
            waypoint["id"]
            for waypoint in default["waypoints"]
            if waypoint.get("goal_profile", "standard")
            == "reverse_handoff"
        ]
        self.assertEqual(reverse_handoff_ids, ["c_corner_1"])

        qr = waypoint_by_id(default, "a_task_observe")
        c_corner_1 = waypoint_by_id(default, "c_corner_1")
        self.assertEqual(qr["direction"], "forward")

        # Terminal waypoints retain fixed orientation
        qr_yaw = planar_yaw(qr["pose"]["orientation"])
        start = waypoint_by_id(default, "p_start")
        start_position = start["pose"]["position"]
        qr_position = qr["pose"]["position"]
        direct_arrival_yaw = math.atan2(
            qr_position["y"] - start_position["y"],
            qr_position["x"] - start_position["x"],
        )
        self.assertAlmostEqual(qr_yaw, direct_arrival_yaw, delta=1.0e-6)
        self.assertAlmostEqual(
            planar_yaw(c_corner_1["pose"]["orientation"]),
            math.radians(60.0),
            delta=1.0e-6,
        )

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
            }.issubset(exec_dependencies)
        )
        self.assertTrue(
            {
                "nav2_core",
                "nav2_costmap_2d",
                "nav2_mppi_controller",
                "nav2_regulated_pure_pursuit_controller",
                "pluginlib",
                "rclcpp_lifecycle",
            }.issubset(direct_dependencies)
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
