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
SIM_REVERSE_HANDOFF_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_handoff_sim_w_replanning_and_recovery.xml"
)
THROUGH_POSES_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_w_replanning_and_recovery.xml"
)
REVERSE_THROUGH_POSES_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_w_replanning_and_recovery.xml"
)
REVERSE_LOCKED_THROUGH_POSES_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_locked_w_replanning_and_recovery.xml"
)
REVERSE_RETURN_THROUGH_POSES_BT_FILE = (
    PACKAGE_ROOT
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_return_w_replanning_and_recovery.xml"
)


class TestReverseNavigationContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))

    def assert_reverse_retreat_contract(self, root, compute_tag, eligibility_key):
        retreats = list(root.iter("AckermannReverseRetreat"))
        self.assertEqual(len(retreats), 1)
        retreat = retreats[0]
        self.assertEqual(retreat.attrib["allow_retreat"], "{" + eligibility_key + "}")
        self.assertEqual(retreat.attrib["retreat_used"], "{reverse_retreat_used}")
        self.assertEqual(retreat.attrib["controller_id"], "ReverseRecovery")
        self.assertEqual(retreat.attrib["goal_checker_id"], "recovery_goal_checker")
        self.assertAlmostEqual(float(retreat.attrib["retreat_distance_m"]), 0.15)
        self.assertGreaterEqual(int(retreat.attrib["costmap_max_age_ms"]), 1000)
        self.assertLessEqual(int(retreat.attrib["costmap_max_age_ms"]), 2000)
        self.assertGreaterEqual(
            int(retreat.attrib["perception_wait_timeout_ms"]), 2500
        )
        self.assertLessEqual(
            int(retreat.attrib["perception_wait_timeout_ms"]), 5000
        )
        self.assertEqual(retreat.attrib["follow_path_goal_timeout_ms"], "1000")
        self.assertEqual(retreat.attrib["follow_path_result_timeout_ms"], "12000")
        self.assertEqual(retreat.attrib["scan_costmap_fusion_lag_ms"], "1250")
        self.assertNotIn("static_keepout_mask_topic", retreat.attrib)
        self.assertEqual(retreat.attrib["footprint_lethal_cost"], "253")
        self.assertEqual(retreat.attrib["scan_min_obstacle_range_m"], "0.25")
        self.assertEqual(retreat.attrib["scan_max_obstacle_range_m"], "2.50")
        self.assertEqual(retreat.attrib["scan_costmap_match_radius_m"], "0.12")
        self.assertEqual(retreat.attrib["retreat_odom_max_age_ms"], "500")
        self.assertEqual(retreat.attrib["retreat_odom_max_step_m"], "0.05")
        self.assertEqual(retreat.attrib["retreat_odom_max_travel_m"], "0.19")
        self.assertEqual(retreat.attrib["retreat_odom_max_displacement_m"], "0.19")

        compute = root.find(f".//{compute_tag}")
        self.assertIsNotNone(compute)
        self.assertEqual(compute.attrib["recovery_eligible"], "{" + eligibility_key + "}")

        initializers = list(root.iter("SetBlackboard"))
        self.assertEqual(len(initializers), 1)
        self.assertEqual(initializers[0].attrib, {
            "value": "false", "output_key": "reverse_retreat_used"})
        self.assertEqual(len(list(root.iter("AckermannReverseRetreat"))), 1)

    def test_one_fixed_dubin_planner_serves_both_directions(self):
        planner_server = self.params["planner_server"]["ros__parameters"]
        self.assertEqual(planner_server["planner_plugins"], ["GridBased"])
        planner = planner_server["GridBased"]
        self.assertEqual(planner["motion_model_for_search"], "DUBIN")
        self.assertNotIn("reverse_penalty", planner)
        self.assertAlmostEqual(planner["minimum_turning_radius"], 0.22)
        self.assertEqual(planner["tolerance"], 0.0)
        for node_name in (
            "bt_navigator",
            "bt_navigator_navigate_to_pose_rclcpp_node",
            "bt_navigator_navigate_through_poses_rclcpp_node",
        ):
            with self.subTest(node=node_name):
                parameters = self.params[node_name]["ros__parameters"]
                self.assertAlmostEqual(
                    parameters["free_heading_minimum_turning_radius"],
                    planner["minimum_turning_radius"],
                )

        forward_tags = {
            element.tag for element in ElementTree.parse(FORWARD_BT_FILE).iter()
        }
        self.assertIn("ComputeFreeHeadingPathToPose", forward_tags)

        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("free_heading_minimum_turning_radius", source)
        self.assertIn("minimum_turning_radius <= 0.0", source)

    def test_reverse_tree_replans_and_validates_before_following(self):
        root = ElementTree.parse(REVERSE_BT_FILE).getroot()
        tags = [element.tag for element in root.iter()]
        self.assertNotIn("RateController", tags)
        self.assertNotIn("PipelineSequence", tags)
        self.assertIn("NavigateReverseWithFailureReplanning", {
            element.attrib.get("name") for element in root.iter()
        })
        self.assertIn("ComputeReverseFreeHeadingPathToPose", tags)
        self.assertIn("IsPathValid", tags)
        self.assertIn("RecordFollowPath", tags)
        self.assertNotIn("FollowPath", tags)
        self.assertNotIn("ComputePathToPose", tags)
        for forbidden in ("Spin", "BackUp", "Wait", "DriveOnHeading"):
            self.assertNotIn(forbidden, tags)

        self.assert_reverse_retreat_contract(
            root,
            "ComputeReverseFreeHeadingPathToPose",
            "reverse_planner_unreachable",
        )

        reverse_compute = root.find(".//ComputeReverseFreeHeadingPathToPose")
        self.assertIsNotNone(reverse_compute)
        self.assertEqual(reverse_compute.attrib["planner_id"], "GridBased")
        planner_radius = self.params["planner_server"]["ros__parameters"][
            "GridBased"
        ]["minimum_turning_radius"]
        navigator_radius = self.params["bt_navigator"]["ros__parameters"][
            "free_heading_minimum_turning_radius"
        ]
        self.assertNotIn("minimum_turning_radius", reverse_compute.attrib)
        self.assertAlmostEqual(navigator_radius, planner_radius)
        self.assertAlmostEqual(navigator_radius, 0.22)
        self.assertAlmostEqual(
            float(reverse_compute.attrib["curvature_tolerance"]), 0.20
        )
        self.assertLessEqual(
            float(reverse_compute.attrib["goal_position_tolerance"]), 0.30
        )

        follow = root.find(".//RecordFollowPath")
        self.assertEqual(follow.attrib["controller_id"], "ReverseHandoff")
        self.assertEqual(
            follow.attrib["goal_checker_id"], "reverse_goal_checker"
        )

    def test_reverse_goal_checker_uses_the_navigation_arrival_envelope(self):
        controller = self.params["controller_server"]["ros__parameters"]
        self.assertIn("reverse_goal_checker", controller["goal_checker_plugins"])
        forward = controller["goal_checker"]
        reverse = controller["reverse_goal_checker"]
        self.assertEqual(reverse["xy_goal_tolerance"], forward["xy_goal_tolerance"])
        self.assertEqual(reverse["yaw_goal_tolerance"], forward["yaw_goal_tolerance"])
        self.assertIs(reverse["stateful"], False)

    def test_reverse_return_tree_isolated_from_the_c1_arrival_envelope(self):
        controller = self.params["controller_server"]["ros__parameters"]
        self.assertIn("return_goal_checker", controller["goal_checker_plugins"])
        returned = controller["return_goal_checker"]
        self.assertEqual(returned["plugin"], "nav2_controller::SimpleGoalChecker")
        self.assertAlmostEqual(returned["xy_goal_tolerance"], 0.15)
        self.assertAlmostEqual(returned["yaw_goal_tolerance"], 0.15)
        self.assertIs(returned["stateful"], False)

        root = ElementTree.parse(REVERSE_RETURN_THROUGH_POSES_BT_FILE).getroot()
        compute = root.find(".//ComputeReverseFreeHeadingPathThroughPoses")
        self.assertIsNotNone(compute)
        self.assertAlmostEqual(
            float(compute.attrib["goal_position_tolerance"]), 0.15)
        self.assertAlmostEqual(float(compute.attrib["goal_yaw_tolerance"]), 0.15)
        follow = root.find(".//RecordFollowPath")
        self.assertIsNotNone(follow)
        self.assertEqual(follow.attrib["controller_id"], "ReverseHandoff")
        self.assertEqual(follow.attrib["goal_checker_id"], "return_goal_checker")
        self.assert_reverse_retreat_contract(
            root,
            "ComputeReverseFreeHeadingPathThroughPoses",
            "reverse_planner_unreachable",
        )

    def test_through_poses_removes_passed_goals_on_every_bt_tick(self):
        cases = (
            (THROUGH_POSES_BT_FILE, "ComputeFreeHeadingPathThroughPoses"),
            (
                REVERSE_THROUGH_POSES_BT_FILE,
                "ComputeReverseFreeHeadingPathThroughPoses",
            ),
        )
        for behavior_tree, compute_tag in cases:
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                pipeline = root.find(".//PipelineSequence")
                self.assertIsNotNone(pipeline)
                children = list(pipeline)
                remove_passed_goals = root.find(".//RemovePassedGoals")
                expected_latch_name = (
                    "LatchReverseFreeHeadingPathThroughPoses"
                    if behavior_tree == REVERSE_THROUGH_POSES_BT_FILE
                    else "LatchFreeHeadingPathThroughPoses"
                )
                latch = next(
                    (
                        element
                        for element in root.iter("LatchSuccess")
                        if element.attrib.get("name")
                        == expected_latch_name
                    ),
                    None,
                )
                self.assertIsNotNone(remove_passed_goals)
                self.assertIsNotNone(latch)
                self.assertEqual(len(list(root.iter("RemovePassedGoals"))), 1)
                self.assertEqual(remove_passed_goals.attrib["input_goals"], "{goals}")
                self.assertEqual(remove_passed_goals.attrib["output_goals"], "{goals}")

                # PipelineSequence re-ticks completed predecessor children while
                # FollowPath runs. Keep goal removal outside the latch so it
                # uses the 20 ms BT tick, while the complete free-heading route
                # is stable until FollowPath fails and recovery resets it.
                self.assertIn(remove_passed_goals, children)
                self.assertIn(latch, children)
                self.assertLess(
                    children.index(remove_passed_goals),
                    children.index(latch),
                )
                self.assertNotIn(remove_passed_goals, list(latch.iter()))
                self.assertEqual(len(list(latch.iter(compute_tag))), 1)
                self.assertNotIn("IsPathValid", {element.tag for element in root.iter()})
                self.assertNotIn("RateController", {element.tag for element in root.iter()})
                self.assertNotIn("DistanceController", {element.tag for element in root.iter()})

                compute = root.find(f".//{compute_tag}")
                self.assertIsNotNone(compute)
                self.assertLessEqual(
                    int(compute.attrib["search_budget_ms"]), 2500
                )
                self.assertLessEqual(
                    int(compute.attrib["candidate_timeout_ms"]),
                    int(compute.attrib["search_budget_ms"]),
                )
                self.assertEqual(
                    int(compute.attrib["through_search_budget_ms"]), 12000
                )
                self.assertLessEqual(
                    int(compute.attrib["fallback_candidate_limit"]), 4
                )
                self.assertEqual(
                    int(compute.attrib["lookahead_fallback_candidate_limit"]), 0
                )
                self.assertEqual(
                    int(compute.attrib["through_solution_limit"]), 4
                )

                follow = root.find(".//RecordFollowPath")
                self.assertIsNotNone(follow)
                expected_controller = (
                    "ReverseHandoff"
                    if behavior_tree == REVERSE_THROUGH_POSES_BT_FILE
                    else (
                        "ForwardHandoff"
                        if behavior_tree == PRECISE_FORWARD_BT_FILE
                        else "ForwardAvoidance"
                    )
                )
                self.assertEqual(follow.attrib["controller_id"], expected_controller)
                self.assertEqual(
                    follow.attrib["goal_checker_id"],
                    "transit_goal_checker"
                    if behavior_tree == REVERSE_THROUGH_POSES_BT_FILE
                    else "goal_checker",
                )

                tags = {element.tag for element in root.iter()}
                self.assertTrue(
                    {"Spin", "BackUp", "Wait", "DriveOnHeading", "ReedsShepp"}
                    .isdisjoint(tags)
                )
                if behavior_tree == REVERSE_THROUGH_POSES_BT_FILE:
                    self.assert_reverse_retreat_contract(
                        root,
                        "ComputeReverseFreeHeadingPathThroughPoses",
                        "reverse_planner_unreachable",
                    )

    def test_free_heading_trees_sweep_the_configured_padded_footprint(self):
        footprint = ast.literal_eval(
            self.params["global_costmap"]["global_costmap"]["ros__parameters"]
            ["footprint"]
        )
        padding = self.params["global_costmap"]["global_costmap"]["ros__parameters"][
            "footprint_padding"
        ]
        padded_half_length = max(abs(float(point[0])) for point in footprint) + padding
        padded_half_width = max(abs(float(point[1])) for point in footprint) + padding
        trees = (
            FORWARD_BT_FILE,
            PRECISE_FORWARD_BT_FILE,
            REVERSE_BT_FILE,
            REVERSE_HANDOFF_BT_FILE,
            THROUGH_POSES_BT_FILE,
            REVERSE_THROUGH_POSES_BT_FILE,
            REVERSE_LOCKED_THROUGH_POSES_BT_FILE,
        )
        for behavior_tree in trees:
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                compute = next(
                    (
                        element for element in root.iter()
                        if element.tag in {
                            "ComputeFreeHeadingPathToPose",
                            "ComputeReverseFreeHeadingPathToPose",
                            "ComputeFreeHeadingPathThroughPoses",
                            "ComputeReverseFreeHeadingPathThroughPoses",
                        }
                    ),
                    None,
                )
                self.assertIsNotNone(compute)
                self.assertAlmostEqual(
                    float(compute.attrib["footprint_half_length_m"]),
                    padded_half_length,
                )
                self.assertAlmostEqual(
                    float(compute.attrib["footprint_half_width_m"]),
                    padded_half_width,
                )
                self.assertLessEqual(
                    float(compute.attrib["footprint_sweep_step_m"]),
                    0.5 * self.params["global_costmap"]["global_costmap"]
                    ["ros__parameters"]["resolution"],
                )
                expected_lethal_cost = (
                    "253"
                    if behavior_tree in {
                        FORWARD_BT_FILE,
                        THROUGH_POSES_BT_FILE,
                    }
                    else "254"
                )
                self.assertEqual(
                    compute.attrib["footprint_lethal_cost"],
                    expected_lethal_cost,
                )

    def test_reverse_through_poses_keeps_the_release_curvature_limit(self):
        through_root = ElementTree.parse(REVERSE_THROUGH_POSES_BT_FILE).getroot()
        single_root = ElementTree.parse(REVERSE_BT_FILE).getroot()
        through_compute = through_root.find(".//ComputeReverseFreeHeadingPathThroughPoses")
        single_compute = single_root.find(".//ComputeReverseFreeHeadingPathToPose")
        self.assertIsNotNone(through_compute)
        self.assertIsNotNone(single_compute)
        self.assertNotIn("minimum_turning_radius", through_compute.attrib)
        self.assertNotIn("minimum_turning_radius", single_compute.attrib)
        self.assertEqual(
            through_compute.attrib["curvature_tolerance"],
            single_compute.attrib["curvature_tolerance"],
        )
        self.assertAlmostEqual(
            float(through_compute.attrib["curvature_tolerance"]), 0.20
        )

        locked_root = ElementTree.parse(
            REVERSE_LOCKED_THROUGH_POSES_BT_FILE
        ).getroot()
        self.assert_reverse_retreat_contract(
            locked_root,
            "ComputeReverseFreeHeadingPathThroughPoses",
            "reverse_planner_unreachable",
        )

    def test_reverse_handoff_uses_dedicated_mppi_ackermann_controller(self):
        regular_root = ElementTree.parse(REVERSE_BT_FILE).getroot()
        handoff_root = ElementTree.parse(REVERSE_HANDOFF_BT_FILE).getroot()
        handoff_tags = [element.tag for element in handoff_root.iter()]
        self.assertIn("ComputeReverseFreeHeadingPathToPose", handoff_tags)
        self.assertIn("IsPathValid", handoff_tags)
        self.assertNotIn("ComputePathToPose", handoff_tags)
        for forbidden in (
            "RateController",
            "PipelineSequence",
            "Spin",
            "BackUp",
            "Wait",
            "DriveOnHeading",
        ):
            self.assertNotIn(forbidden, handoff_tags)

        regular_compute = regular_root.find(".//ComputeReverseFreeHeadingPathToPose")
        handoff_compute = handoff_root.find(
            ".//ComputeReverseFreeHeadingPathToPose"
        )
        self.assertIsNotNone(regular_compute)
        self.assertIsNotNone(handoff_compute)
        for attribute in (
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
                handoff_compute.attrib[attribute],
                regular_compute.attrib[attribute],
            )
        self.assertNotIn("minimum_turning_radius", handoff_compute.attrib)
        self.assertNotIn("minimum_turning_radius", regular_compute.attrib)
        # The semantic handoff pose has a nonzero quaternion, so the shared
        # node preserves exactly one locked-yaw candidate despite exposing the
        # ordinary bounded heading-search port.
        self.assertEqual(handoff_compute.attrib["heading_samples"], "12")
        self.assertNotIn("max_initial_path_length_ratio", handoff_compute.attrib)
        self.assertNotIn("max_edge_path_length_ratio", handoff_compute.attrib)

        follow = handoff_root.find(".//RecordFollowPath")
        self.assertIsNotNone(follow)
        self.assertEqual(follow.attrib["controller_id"], "ReverseHandoff")
        self.assertEqual(
            follow.attrib["goal_checker_id"], "reverse_goal_checker")

        self.assert_reverse_retreat_contract(
            handoff_root,
            "ComputeReverseFreeHeadingPathToPose",
            "reverse_handoff_planner_unreachable",
        )

        controller = self.params["controller_server"]["ros__parameters"]
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
        regular = controller["FollowPath"]
        forward_avoidance = controller["ForwardAvoidance"]
        handoff = controller["ReverseHandoff"]
        self.assertEqual(
            regular["plugin"],
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        )
        self.assertEqual(
            handoff["plugin"], "smartcar_nav2::ReverseOnlyMPPIController"
        )
        self.assertEqual(
            forward_avoidance["plugin"],
            "smartcar_nav2::ForwardOnlyRPPController",
        )
        self.assertAlmostEqual(forward_avoidance["desired_linear_vel"], 0.15)
        self.assertIs(forward_avoidance["allow_reversing"], False)
        self.assertIs(forward_avoidance["use_rotate_to_heading"], False)
        self.assertIs(forward_avoidance["use_collision_detection"], True)
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

        recovery = controller["ReverseRecovery"]
        self.assertEqual(
            recovery["plugin"], "smartcar_nav2::ReverseOnlyMPPIController"
        )
        self.assertEqual(recovery["motion_model"], "Ackermann")
        self.assertAlmostEqual(recovery["vx_min"], 0.015)
        self.assertAlmostEqual(recovery["vx_max"], 0.05)
        self.assertAlmostEqual(recovery["wz_max"], 0.0)
        self.assertLess(recovery["vx_min"], recovery["vx_max"])
        self.assertLess(recovery["vx_max"], handoff["vx_max"])
        self.assertEqual(
            recovery["AckermannConstraints"]["min_turning_r"], planner_radius
        )
        recovery_goal = controller["recovery_goal_checker"]
        self.assertEqual(
            recovery_goal["plugin"], "nav2_controller::SimpleGoalChecker"
        )
        self.assertLess(recovery_goal["xy_goal_tolerance"], 0.15)
        self.assertLess(recovery_goal["xy_goal_tolerance"], controller[
            "transit_goal_checker"]["xy_goal_tolerance"])
        self.assertAlmostEqual(recovery_goal["yaw_goal_tolerance"], 0.10)
        self.assertIs(recovery_goal["stateful"], False)

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
        self.assertIn("ComputeFreeHeadingPathToPose", tags)
        self.assertIn("RecordFollowPath", tags)
        self.assertNotIn("FollowPath", tags)
        for forbidden in ("Spin", "BackUp", "Wait"):
            self.assertNotIn(forbidden, tags)

        compute = root.find(".//ComputeFreeHeadingPathToPose")
        self.assertIsNotNone(compute)
        planner_id = compute.attrib["planner_id"]
        self.assertEqual(planner_id, "GridBased")
        planner = self.params["planner_server"]["ros__parameters"][planner_id]
        self.assertEqual(planner["tolerance"], 0.0)

        follow = root.find(".//RecordFollowPath")
        self.assertIsNotNone(follow)
        self.assertEqual(follow.attrib["controller_id"], "ForwardHandoff")
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

    def test_real_precise_tree_disables_the_simulation_departure_connector(self):
        precise = ElementTree.parse(PRECISE_FORWARD_BT_FILE).getroot()
        compute = precise.find(".//ComputeFreeHeadingPathToPose")
        self.assertIsNotNone(compute)
        self.assertEqual(compute.attrib["footprint_lethal_cost"], "254")
        self.assertEqual(compute.attrib["maximum_direction_error"], "0.15")
        # At a real-car 0.22 m lower bound, the former radius gate would no
        # longer suppress this Gazebo-only connector. The shared tree must
        # keep it explicitly disabled; sim.launch wires a separate copied BT.
        self.assertEqual(compute.attrib["departure_connector_enabled"], "false")
        self.assertEqual(
            compute.attrib["departure_connector_radius_margin_m"], "0.28"
        )
        self.assertEqual(
            compute.attrib["departure_connector_maximum_active_radius_m"],
            "0.50",
        )
        self.assertEqual(
            compute.attrib["departure_connector_terminal_radius_m"], "0.22"
        )
        self.assertNotIn(
            "departure_connector_high_right_turn_radius_m", compute.attrib
        )
        self.assertEqual(compute.attrib["departure_connector_start_x_m"], "0.0")
        self.assertEqual(compute.attrib["departure_connector_start_y_m"], "0.0")
        self.assertEqual(
            compute.attrib["departure_connector_start_yaw_rad"], "0.0"
        )
        self.assertEqual(compute.attrib["departure_connector_heading_bins"], "144")

        for behavior_tree in (
            FORWARD_BT_FILE,
            THROUGH_POSES_BT_FILE,
            REVERSE_BT_FILE,
            REVERSE_HANDOFF_BT_FILE,
            REVERSE_THROUGH_POSES_BT_FILE,
            REVERSE_LOCKED_THROUGH_POSES_BT_FILE,
        ):
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                computes = [
                    element
                    for element in root.iter()
                    if "ComputeFreeHeadingPath" in element.tag
                    or "ComputeReverseFreeHeadingPath" in element.tag
                ]
                self.assertEqual(len(computes), 1)
                self.assertNotIn(
                    "departure_connector_enabled", computes[0].attrib
                )

        real_radius = self.params["bt_navigator"]["ros__parameters"][
            "free_heading_minimum_turning_radius"
        ]
        self.assertAlmostEqual(real_radius, 0.22)
        self.assertLessEqual(real_radius + 0.28, 0.50)
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("departureConnectorRadiusWithinMaximum", source)
        self.assertIn("departureConnectorTerminalRadiusWithinEnvelope", source)
        self.assertIn("departureConnectorHighRightTurnRadiusWithinEnvelope", source)
        self.assertIn("simulation-only terminal radius", source)
        self.assertIn("departure_connector_terminal_radius_m_", source)
        self.assertIn("departure_connector_high_right_turn_radius_m_", source)
        read_departure_options = source.split(
            "bool ComputeFreeHeadingPathAction::readDepartureConnectorOptions()"
        )[1].split("bool ComputeFreeHeadingPathAction::goalsChanged()", 1)[0]
        self.assertLess(
            read_departure_options.index(
                "if (!departureConnectorRadiusWithinMaximum("
            ),
            read_departure_options.index(
                "if (!departureConnectorHighRightTurnRadiusWithinEnvelope("
            ),
        )
        self.assertLess(
            read_departure_options.index(
                "if (!departureConnectorHighRightTurnRadiusWithinEnvelope("
            ),
            read_departure_options.index(
                "return departureConnectorTerminalRadiusWithinEnvelope("
            ),
        )
        self.assertIn("p_terminal_rsl_unavailable", source)
        self.assertIn("active radius %.3f exceeds its %.3f m gate", source)
        self.assertIn("Rejected P connector lattice handoff", source)
        self.assertIn("kJoinPositionTolerance", source)
        self.assertNotIn("snapPlannerPathStartToRequestedPose(", source)

    def test_forward_trees_use_positive_ackermann_tracking(self):
        controller = self.params["controller_server"]["ros__parameters"]
        forward = controller["ForwardAvoidance"]
        handoff = controller["ForwardHandoff"]
        planner_radius = self.params["planner_server"]["ros__parameters"][
            "GridBased"
        ]["minimum_turning_radius"]
        self.assertEqual(
            forward["plugin"], "smartcar_nav2::ForwardOnlyRPPController")
        self.assertAlmostEqual(forward["desired_linear_vel"], 0.15)
        self.assertIs(forward["allow_reversing"], False)
        self.assertIs(forward["use_rotate_to_heading"], False)
        self.assertAlmostEqual(
            forward["forward_min_turning_radius"], planner_radius
        )
        self.assertGreaterEqual(
            forward["forward_max_angular_velocity"],
            forward["desired_linear_vel"] / planner_radius,
        )
        self.assertIs(forward["use_collision_detection"], True)
        self.assertEqual(
            handoff["plugin"], "smartcar_nav2::ForwardOnlyMPPIController")
        self.assertEqual(handoff["motion_model"], "Ackermann")
        self.assertGreater(handoff["vx_min"], 0.0)
        self.assertLess(handoff["vx_min"], handoff["vx_max"])
        self.assertGreaterEqual(
            handoff["wz_max"], handoff["vx_max"] / planner_radius)
        self.assertAlmostEqual(
            handoff["AckermannConstraints"]["min_turning_r"], planner_radius)
        self.assertIs(handoff["CostCritic"]["consider_footprint"], True)

        expected_controllers = {
            FORWARD_BT_FILE: "ForwardAvoidance",
            PRECISE_FORWARD_BT_FILE: "ForwardHandoff",
            THROUGH_POSES_BT_FILE: "ForwardAvoidance",
        }
        for behavior_tree, expected_controller in expected_controllers.items():
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                follow = root.find(".//RecordFollowPath")
                self.assertIsNotNone(follow)
                self.assertEqual(follow.attrib["controller_id"], expected_controller)
                self.assertEqual(len(list(root.iter("AckermannReverseRetreat"))), 0)

    def test_generic_forward_replans_validate_geometry_before_follow_path(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        validation_header = (
            PACKAGE_ROOT / "include" / "smartcar_nav2"
            / "forward_path_geometry_validation.hpp"
        ).read_text(encoding="utf-8")
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

        candidate_body = source.split(
            "BT::NodeStatus ComputeFreeHeadingPathAction::completeCandidate(",
            1,
        )[1].split(
            "BT::NodeStatus ComputeFreeHeadingPathAction::completeLookahead(",
            1,
        )[0]
        self.assertIn("validateForwardPathGeometry(", candidate_body)
        self.assertIn("!reverse_ && !has_departure_connector", candidate_body)
        self.assertIn("Rejected generic forward candidate", candidate_body)
        self.assertIn("return advanceCandidate();", candidate_body)
        self.assertIn("validateForwardPathGeometry(", source)
        self.assertIn("departure_connectors_.empty()", source)
        self.assertIn("validateReversePath(", source)

        for reason in (
            "segment_not_forward",
            "orientation_curvature_exceeded",
            "geometric_curvature_exceeded",
            "terminal_tangent_mismatch",
        ):
            with self.subTest(reason=reason):
                self.assertIn(reason, validation_header)
        self.assertIn("test_forward_path_geometry_validation", cmake)

    def test_single_goal_trees_latch_a_validated_path_while_following(self):
        cases = (
            (FORWARD_BT_FILE, "LatchFreeHeadingPathToPose"),
            (PRECISE_FORWARD_BT_FILE, "LatchPreciseFreeHeadingPathToPose"),
            (REVERSE_BT_FILE, "LatchValidatedReverseFreeHeadingPath"),
            (REVERSE_HANDOFF_BT_FILE, "LatchValidatedReverseHandoffPath"),
        )
        for behavior_tree, latch_name in cases:
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                latch = next(
                    (
                        element for element in root.iter("LatchSuccess")
                        if element.attrib.get("name") == latch_name
                    ),
                    None,
                )
                self.assertIsNotNone(latch)
                self.assertEqual(
                    len(list(latch.iter("ComputeFreeHeadingPathToPose"))) +
                    len(list(latch.iter("ComputeReverseFreeHeadingPathToPose"))),
                    1,
                )
                self.assertNotIn("RecordFollowPath", {element.tag for element in latch.iter()})

    def test_bt_plugin_list_retains_required_nav2_nodes(self):
        plugins = set(
            self.params["bt_navigator"]["ros__parameters"][
                "plugin_lib_names"
            ]
        )
        self.assertIn(
            "smartcar_compute_free_heading_path_action_bt_node", plugins
        )
        self.assertIn(
            "smartcar_ackermann_reverse_retreat_action_bt_node", plugins
        )
        self.assertIn(
            "smartcar_record_follow_path_action_bt_node", plugins
        )
        through_plugins = set(
            self.params[
                "bt_navigator_navigate_through_poses_rclcpp_node"
            ]["ros__parameters"]["plugin_lib_names"]
        )
        self.assertIn(
            "smartcar_ackermann_reverse_retreat_action_bt_node", through_plugins
        )
        self.assertIn(
            "smartcar_record_follow_path_action_bt_node", through_plugins
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

    def test_free_heading_cancellation_fails_closed_when_requested(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        cancellation = source.split(
            "BT::NodeStatus ComputeFreeHeadingPathAction::waitForCancellation()", 1
        )[1].split(
            "bool ComputeFreeHeadingPathAction::requestCancellationForActiveGoal()", 1
        )[0]
        self.assertIn("const bool fail = failure_after_cancellation_", cancellation)
        self.assertIn("clearPathOutput()", cancellation)
        self.assertIn(
            "return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;",
            cancellation,
        )

    def test_reverse_retreat_latches_before_dispatch_and_never_publishes_twist(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("setOutput(\"retreat_used\", true)", source)
        self.assertIn("retreatPathIsClear", source)
        self.assertIn("costmapFootprintPathSweep", source)
        self.assertIn("/global_costmap/costmap_raw", source)
        self.assertIn("/local_costmap/costmap_raw", source)
        self.assertIn("BT::StatefulActionNode", source)
        self.assertIn("waitForPerception", source)
        self.assertIn("perception_deadline_", source)
        self.assertIn("follow_path_result_timeout_", source)
        self.assertIn("result_deadline_", source)
        self.assertIn("dispatchedRetreatIsSafe", source)
        self.assertIn("retreat_odom_guard_", source)
        self.assertIn("/odom_combined", source)
        self.assertIn("async_send_goal", source)
        self.assertIn("async_cancel_goal", source)
        self.assertNotIn("std::this_thread::sleep_for", source)
        self.assertNotIn("on_tick", source)
        # The accepted-path evidence publisher is allowed; this recovery must
        # still never create or publish a raw velocity command.
        self.assertIn("accepted_path_publisher_", source)
        self.assertNotIn("geometry_msgs/msg/twist", source)
        self.assertNotIn("cmd_vel", source)
        header = (
            PACKAGE_ROOT / "include" / "smartcar_nav2"
            / "ackermann_reverse_retreat_action.hpp"
        ).read_text(encoding="utf-8")
        self.assertIn("BT::BidirectionalPort<bool>(", header)

    def test_reverse_retreat_rechecks_perception_and_odom_until_follow_path_is_terminal(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        goal_wait = source.split(
            "BT::NodeStatus AckermannReverseRetreatAction::waitForGoalHandle()", 1
        )[1].split(
            "BT::NodeStatus AckermannReverseRetreatAction::waitForResult()", 1
        )[0]
        result_wait = source.split(
            "BT::NodeStatus AckermannReverseRetreatAction::waitForResult()", 1
        )[1].split(
            "void AckermannReverseRetreatAction::cancelFollowPath()", 1
        )[0]
        for body in (goal_wait, result_wait):
            safety_index = body.index("dispatchedRetreatIsSafe(safety_reason)")
            cancel_index = body.index("cancelFollowPath()", safety_index)
            failure_index = body.index("return BT::NodeStatus::FAILURE;", safety_index)
            self.assertLess(cancel_index, failure_index)
        self.assertIn("retreatPathIsClear(retreat_path_, reason)", source)
        self.assertIn("retreat_odom_guard_.observe", source)
        timeout_index = result_wait.index("FollowPath result timed out")
        self.assertLess(
            result_wait.index("cancelFollowPath()", timeout_index),
            result_wait.index("return BT::NodeStatus::FAILURE;", timeout_index),
        )

    def test_reverse_retreat_applies_static_keepout_only_when_configured(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        header = (
            PACKAGE_ROOT / "include" / "smartcar_nav2"
            / "ackermann_reverse_retreat_action.hpp"
        ).read_text(encoding="utf-8")
        retreat_check = source.split(
            "bool AckermannReverseRetreatAction::retreatPathIsClear", 1
        )[1].split(
            "AckermannReverseRetreatAction::ScanSample", 1
        )[0]
        self.assertIn(
            '"static_keepout_mask_topic", ""', header
        )
        self.assertIn(
            '"allow_static_scan_only_evidence", false', header
        )
        self.assertNotIn("static_keepout_mask_topic.empty() ||", source)
        self.assertIn("staticKeepoutMaskPathIsClear", header)
        self.assertIn("staticKeepoutMaskFootprintPathSweep", source)
        self.assertIn("StaticKeepoutMaskSweepResult::kClear", source)
        self.assertIn("keepout_mask = static_keepout_mask_", retreat_check)
        self.assertIn(
            "const bool static_keepout_required = !static_keepout_mask_topic_.empty()",
            retreat_check,
        )
        self.assertIn(
            "staticKeepoutMaskPathIsClear(keepout_mask.get(), path, reason)",
            retreat_check,
        )
        self.assertIn("if (static_keepout_required &&", retreat_check)
        self.assertLess(
            retreat_check.index(
                "staticKeepoutMaskPathIsClear(keepout_mask.get(), path, reason)"
            ),
            retreat_check.index("filterStaticKeepoutPoints("),
        )
        self.assertIn(
            "required static keepout mask was not received", source,
        )
        dynamic_filter = source.split(
            "bool AckermannReverseRetreatAction::filterStaticKeepoutPoints", 1
        )[1].split(
            "bool AckermannReverseRetreatAction::retreatPathIsClear", 1
        )[0]
        self.assertIn(
            "result != StaticKeepoutMaskFilterResult::kFiltered", dynamic_filter
        )
        self.assertNotIn(
            "scan has no endpoints outside occupied or unknown keepout mask cells",
            dynamic_filter,
        )
        self.assertIn(
            "std::vector<std::pair<double, double>> global_witness_points = global_points",
            retreat_check,
        )
        self.assertIn(
            "std::vector<std::pair<double, double>> local_witness_points = local_points",
            retreat_check,
        )
        self.assertIn("if (static_keepout_required) {", retreat_check)
        self.assertIn(
            "*global_sample.costmap, global_witness_points", retreat_check
        )
        self.assertIn(
            "*local_sample.costmap, local_witness_points", retreat_check
        )
        self.assertIn("allow_static_scan_only_evidence_", retreat_check)
        self.assertIn("Unknown cells are lethal", retreat_check)
        configure = source.split(
            "bool AckermannReverseRetreatAction::configureKeepoutMaskSubscription", 1
        )[1].split("void AckermannReverseRetreatAction::armCostmapBarrier", 1)[0]
        empty_topic = configure.index("if (topic.empty())")
        self.assertIn("return true;", configure[empty_topic:])
        self.assertNotIn(
            "requires a static keepout mask topic", configure
        )

    def test_c1_simulation_retreat_allows_static_scan_evidence_only_with_keepout_sweep(self):
        sim_handoff = ElementTree.parse(
            SIM_REVERSE_HANDOFF_BT_FILE
        ).getroot()
        sim_retreat = sim_handoff.find(".//AckermannReverseRetreat")
        self.assertIsNotNone(sim_retreat)
        self.assertEqual(
            sim_retreat.attrib["allow_static_scan_only_evidence"], "true")
        self.assertEqual(
            sim_retreat.attrib["static_keepout_mask_topic"],
            "/keepout_filter_mask")
        self.assertEqual(sim_retreat.attrib["retreat_direction"], "forward")
        self.assertEqual(sim_retreat.attrib["controller_id"], "FollowPath")

        real_handoff = ElementTree.parse(
            REVERSE_HANDOFF_BT_FILE
        ).getroot()
        real_retreat = real_handoff.find(".//AckermannReverseRetreat")
        self.assertIsNotNone(real_retreat)
        self.assertNotIn("allow_static_scan_only_evidence", real_retreat.attrib)

        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "Static-only scan evidence requires a configured static keepout mask",
            source,
        )

    def test_reverse_retreat_cancels_a_late_accepted_goal_after_ack_timeout(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        body = source.split(
            "BT::NodeStatus AckermannReverseRetreatAction::waitForGoalHandle()",
            1,
        )[1].split(
            "try {\n    goal_handle_ = goal_handle_future_.get();", 1,
        )[0]
        timeout_index = body.index("goal acknowledgement timed out")
        cancel_index = body.index("cancelFollowPath()", timeout_index)
        clear_index = body.index("clearPathOutput()", timeout_index)
        self.assertLess(cancel_index, clear_index)

    def test_reverse_retreat_keeps_an_accepted_handle_for_halt_races(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        header = (
            PACKAGE_ROOT / "include" / "smartcar_nav2"
            / "ackermann_reverse_retreat_action.hpp"
        ).read_text(encoding="utf-8")
        self.assertIn("struct GoalResponseHandle", header)
        self.assertIn("goal_response_handle_mutex_", header)
        self.assertIn("goal_response_handle_", header)

        dispatch = source.split(
            "BT::NodeStatus AckermannReverseRetreatAction::dispatchRetreat()", 1,
        )[1].split(
            "BT::NodeStatus AckermannReverseRetreatAction::waitForGoalHandle()", 1,
        )[0]
        store_index = dispatch.index("goal_response_handle_.handle = goal_handle")
        cancellation_check_index = dispatch.index("cancelled_dispatch_generation_.load()")
        self.assertLess(store_index, cancellation_check_index)

        cancel = source.split(
            "void AckermannReverseRetreatAction::cancelFollowPath()", 1,
        )[1].split(
            "bool AckermannReverseRetreatAction::scanIsFresh", 1,
        )[0]
        self.assertIn("goal_response_handle_.generation == active_dispatch_generation_", cancel)
        self.assertIn("async_cancel_goal(handle_to_cancel)", cancel)

    def test_reverse_retreat_cancels_every_post_dispatch_exception_exit(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        for failure_message in (
            "FollowPath dispatch failed",
            "goal future is invalid",
            "goal transport failed",
            "goal was rejected",
            "result future is invalid",
            "FollowPath result failed",
        ):
            with self.subTest(failure_message=failure_message):
                failure_index = source.index(failure_message)
                cancel_index = source.index("cancelFollowPath()", failure_index)
                clear_index = source.index("clearPathOutput()", failure_index)
                self.assertLess(cancel_index, clear_index)

    def test_recovery_enforces_its_direction_specific_controller_contract(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'retreat_forward_ ? "FollowPath" : "ReverseRecovery"', source
        )
        self.assertIn('controller_id != expected_controller', source)
        self.assertIn('goal_checker_id != "recovery_goal_checker"', source)
        self.assertIn(
            "Forward recovery is restricted to the static-mask C1 handoff contract",
            source,
        )

    def test_reverse_retreat_cancels_non_success_terminal_results(self):
        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        body = source.split(
            "BT::NodeStatus AckermannReverseRetreatAction::waitForResult()", 1,
        )[1].split(
            "void AckermannReverseRetreatAction::cancelFollowPath()", 1,
        )[0]
        terminal_failure = body.index("FollowPath ended with result code")
        self.assertLess(
            body.index("cancelFollowPath()", terminal_failure),
            body.index("return BT::NodeStatus::FAILURE;", terminal_failure),
        )

    def test_reverse_free_heading_retreat_requires_post_clear_costmap_samples(self):
        for behavior_tree, blackboard_prefix in (
            (REVERSE_BT_FILE, "reverse_planner"),
            (REVERSE_HANDOFF_BT_FILE, "reverse_handoff_planner"),
            (REVERSE_THROUGH_POSES_BT_FILE, "reverse_planner"),
            (REVERSE_LOCKED_THROUGH_POSES_BT_FILE, "reverse_planner"),
        ):
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                compute = next(
                    element
                    for element in root.iter()
                    if element.tag.startswith("ComputeReverseFreeHeadingPath")
                )
                retreat = root.find(".//AckermannReverseRetreat")
                self.assertIsNotNone(retreat)
                self.assertEqual(
                    compute.attrib["costmap_stamp_ns"],
                    "{" + blackboard_prefix + "_costmap_stamp_ns}",
                )
                self.assertEqual(
                    compute.attrib["local_costmap_stamp_ns"],
                    "{" + blackboard_prefix + "_local_costmap_stamp_ns}",
                )
                self.assertEqual(
                    retreat.attrib["costmap_min_stamp_ns"],
                    "{" + blackboard_prefix + "_costmap_stamp_ns}",
                )
                self.assertEqual(
                    retreat.attrib["local_costmap_min_stamp_ns"],
                    "{" + blackboard_prefix + "_local_costmap_stamp_ns}",
                )

        source = (
            PACKAGE_ROOT / "src" / "ackermann_reverse_retreat_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("armCostmapBarrier", source)
        self.assertIn("costmapSampleFreshness", source)
        self.assertIn("costmap_min_stamp_ns <= 0", source)
        self.assertIn("local_costmap_min_stamp_ns <= 0", source)
        self.assertIn("local_costmap_min_stamp_ns_", source)
        self.assertIn("Ignoring non-monotonic global raw costmap", source)
        self.assertIn("Ignoring non-monotonic local raw costmap", source)
        self.assertIn("scan_barrier_sequence_ = scan_.sequence", source)
        self.assertIn("post_clear_ros_stamp_ns_ = node_->get_clock()->now().nanoseconds()", source)
        self.assertIn("minimum_scan_stamp_ns", source)
        self.assertIn("waitForPerception", source)
        self.assertIn("updateScan", source)
        self.assertIn("costmapHasLethalObservationAtPoints", source)
        self.assertIn("newestScanAtOrBefore", source)
        self.assertIn("costmapScanAssociationFreshness", source)
        self.assertIn("filterPointsOutsideStaticKeepoutMask", source)
        header = (
            PACKAGE_ROOT / "include" / "smartcar_nav2"
            / "ackermann_reverse_retreat_action.hpp"
        ).read_text(encoding="utf-8")
        self.assertIn('"static_keepout_mask_topic", ""', header)
        self.assertIn("configureKeepoutMaskSubscription", source)
        self.assertNotIn("--global_costmap_barrier_sequence_", source)
        self.assertNotIn("--local_costmap_barrier_sequence_", source)

    def test_free_heading_transport_failures_fail_closed_but_humble_aborts_exhaust_candidates(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "result.code == rclcpp_action::ResultCode::ABORTED", source
        )
        self.assertIn("Humble ComputePathToPose exposes no planner error code", source)
        self.assertIn(
            "completeLookahead(nullptr, true) : completeCandidate(nullptr, true);",
            source,
        )
        self.assertIn(
            "result.code != rclcpp_action::ResultCode::SUCCEEDED", source
        )
        self.assertIn(
            'failPlannerQuery("planner action canceled or returned an unknown result"',
            source,
        )
        self.assertIn('failPlannerQuery("planner goal transport error"', source)
        self.assertIn('failPlannerQuery("planner result transport error"', source)
        self.assertIn("timed out; cancelling and failing closed", source)
        self.assertIn("planner_query_failed_ = true", source)
        self.assertIn("search_budget_exhausted_ = true", source)
        self.assertIn(
            "eligible && !planner_query_failed_ && !search_budget_exhausted_",
            source,
        )
        self.assertNotIn(
            "result.code != rclcpp_action::ResultCode::SUCCEEDED ||",
            source,
        )
        self.assertNotIn("commit_best_after_cancellation_", source)

    def test_free_heading_trees_leave_route_length_to_nav2(self):
        for behavior_tree in (
            FORWARD_BT_FILE,
            PRECISE_FORWARD_BT_FILE,
            THROUGH_POSES_BT_FILE,
            REVERSE_BT_FILE,
            REVERSE_HANDOFF_BT_FILE,
            REVERSE_THROUGH_POSES_BT_FILE,
            REVERSE_LOCKED_THROUGH_POSES_BT_FILE,
        ):
            with self.subTest(behavior_tree=behavior_tree.name):
                root = ElementTree.parse(behavior_tree).getroot()
                computes = [
                    element
                    for element in root.iter()
                    if "ComputeFreeHeadingPath" in element.tag
                    or "ComputeReverseFreeHeadingPath" in element.tag
                ]
                self.assertEqual(len(computes), 1)
                self.assertNotIn("max_initial_path_length_ratio", computes[0].attrib)
                self.assertNotIn("max_edge_path_length_ratio", computes[0].attrib)

    def test_free_heading_has_no_hand_authored_detour_filter(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertNotIn("max_initial_path_length_ratio", source)
        self.assertNotIn("max_edge_path_length_ratio", source)
        self.assertNotIn("edgePathWithinLengthRatio", source)
        self.assertNotIn("edge detour exceeds", source)

    def test_free_heading_query_timeout_is_one_budget_for_handle_and_result(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("search_budget_ms", source)
        self.assertIn(
            "query_deadline_ = std::min(now + candidate_timeout_, search_deadline_)",
            source,
        )
        self.assertNotIn(
            "query_deadline_ = std::chrono::steady_clock::now() + candidate_timeout_",
            source,
        )

    def test_through_poses_search_is_bounded_and_backtracks_before_publication(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("completeThroughCandidate", source)
        self.assertIn("completeThroughPath", source)
        self.assertIn("backtrackThroughCandidate", source)
        self.assertIn("highestRiskThroughFrame", source)
        self.assertIn("publishBestThroughPath", source)
        self.assertIn("through_solution_limit", source)
        self.assertIn("global_costmap/costmap_raw", source)
        self.assertIn("kMaximumThroughCandidateQueries = 64U", source)
        self.assertIn("through_candidate_query_count_", source)
        self.assertIn("if (target_index_ == real_goals_.size()) {", source)

    def test_free_heading_rejects_a_lethal_footprint_sweep_before_commit(self):
        source = (
            PACKAGE_ROOT / "src" / "compute_free_heading_path_action.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("costmapFootprintPathSweep", source)
        accept_body = source.split(
            "ComputeFreeHeadingPathAction::hasAcceptableCostmapSample", 1
        )[1].split("}", 1)[0]
        self.assertIn("quality.footprint_sweep_checked", accept_body)
        self.assertIn("quality.footprint_sweep_clear", accept_body)
        self.assertIn("staticKeepoutMaskFootprintPathSweep", source)
        self.assertIn("quality.static_keepout_sweep_clear", accept_body)
        self.assertIn("staticKeepoutSweepIsInfrastructureFailure", source)
        self.assertIn("footprint_lethal_cost", source)

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

    def test_build_installs_active_bt_libraries_and_tests(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("BT_PLUGIN_EXPORT", cmake)
        self.assertIn(
            "add_library(smartcar_compute_free_heading_path_action_bt_node",
            cmake,
        )
        self.assertIn(
            "add_library(smartcar_ackermann_reverse_retreat_action_bt_node",
            cmake,
        )
        self.assertIn("test_ackermann_reverse_retreat_path", cmake)
        self.assertIn("test_costmap_sample_guard", cmake)
        self.assertIn("test_reverse_path_utils", cmake)
        self.assertIn("test_reverse_command_filter", cmake)
        self.assertIn("test_forward_command_filter", cmake)
        self.assertIn("test_forward_path_tracking_guard", cmake)
        self.assertIn("test_reverse_navigation_contracts.py", cmake)
        self.assertIn(
            "add_library(smartcar_reverse_only_mppi_controller", cmake
        )
        self.assertIn(
            "add_library(smartcar_forward_only_rpp_controller", cmake
        )
        self.assertIn("pluginlib_export_plugin_description_file(", cmake)
        self.assertIn("reverse_only_mppi_controller_plugin.xml", cmake)
        self.assertIn("forward_only_rpp_controller_plugin.xml", cmake)
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
