"""Static contracts for smartcar_task ROS wiring and reset ordering."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
AGENTS = ROOT / "AGENTS.md"
PACKAGE = ROOT / "src" / "smartcar_task"
NODE = PACKAGE / "smartcar_task" / "task_node.py"
MISSION = PACKAGE / "smartcar_task" / "mission.py"
COMPETITION = PACKAGE / "smartcar_task" / "competition.py"
NAVIGATION_GOALS = PACKAGE / "smartcar_task" / "navigation_goals.py"
LAUNCH = PACKAGE / "launch" / "smartcar_task.launch.py"
PACKAGE_XML = PACKAGE / "package.xml"
TASK_CONFIG = PACKAGE / "config" / "task.yaml"
NAV_TEST = ROOT / "scripts" / "nav_test.sh"
NAV_PREPARE = ROOT / "scripts" / "nav_prepare.sh"
NAV_STATUS = ROOT / "scripts" / "nav_status.py"
ROS_CLEANUP = ROOT / "scripts" / "ros_cleanup.sh"
SYSTEM = ROOT / "src" / "smartcar_bringup" / "launch" / "smartcar_system.launch.py"
NAV2_CONFIG = ROOT / "src" / "smartcar_nav2" / "config"
NAV2_BEHAVIOR_TREES = NAV2_CONFIG / "behavior_trees"
NAV2_CMAKE = ROOT / "src" / "smartcar_nav2" / "CMakeLists.txt"
NAV2_LAUNCH = ROOT / "src" / "smartcar_nav2" / "launch" / "navigation_launch.py"
NAV2_PACKAGE_XML = ROOT / "src" / "smartcar_nav2" / "package.xml"
DIRECTION_GUARD_CONFIG = (
    ROOT / "src" / "smartcar_safety" / "config" / "direction_guard.yaml"
)
THROUGH_POSES_TREES = (
    "navigate_through_poses_w_replanning_and_recovery.xml",
    "navigate_through_poses_precise_w_replanning_and_recovery.xml",
    "navigate_through_poses_return_w_replanning_and_recovery.xml",
    "navigate_through_poses_transit_w_replanning_and_recovery.xml",
)


class TaskLaunchContractTests(unittest.TestCase):
    def _assert_through_poses_recovery_tree(self, filename):
        source = (NAV2_BEHAVIOR_TREES / filename).read_text(encoding="utf-8")
        root = ET.fromstring(source)
        behavior_tree = root.find("./BehaviorTree")
        self.assertIsNotNone(behavior_tree)
        assert behavior_tree is not None

        top_level = list(behavior_tree)
        self.assertEqual(len(top_level), 1)
        recovery = top_level[0]
        self.assertEqual(recovery.tag, "RecoveryNode")
        self.assertEqual(recovery.attrib["number_of_retries"], "6")
        self.assertEqual(len(recovery), 2)

        primary, recovery_actions = recovery
        self.assertEqual(primary.tag, "Sequence")
        is_return_tree = (
            filename == "navigate_through_poses_return_w_replanning_and_recovery.xml"
        )
        self.assertEqual(len(primary), 5 if is_return_tree else 4)
        initial_plan, deduplicate, smooth_path = primary[:3]
        if is_return_tree:
            terminal_goal, tracking = primary[3:]
            self.assertEqual(terminal_goal.tag, "GetPoseFromPath")
            self.assertEqual(terminal_goal.attrib["path"], "{path}")
            self.assertEqual(terminal_goal.attrib["pose"], "{terminal_path_goal}")
            self.assertEqual(terminal_goal.attrib["index"], "-1")
        else:
            tracking = primary[3]
        self.assertEqual(initial_plan.tag, "ComputePathThroughPoses")
        self.assertEqual(initial_plan.attrib["goals"], "{goals}")
        self.assertEqual(initial_plan.attrib["path"], "{raw_path}")
        self.assertEqual(initial_plan.attrib["planner_id"], "GridBased")
        self.assertEqual(deduplicate.tag, "RemoveDuplicatePathPoints")
        self.assertEqual(deduplicate.attrib["input_path"], "{raw_path}")
        self.assertEqual(deduplicate.attrib["output_path"], "{deduplicated_path}")
        self.assertEqual(smooth_path.tag, "SmoothPath")
        self.assertEqual(
            smooth_path.attrib["unsmoothed_path"], "{deduplicated_path}")
        self.assertEqual(smooth_path.attrib["smoothed_path"], "{path}")
        self.assertEqual(
            smooth_path.attrib["smoother_id"], "constrained_smoother")
        self.assertEqual(
            smooth_path.attrib["max_smoothing_duration"], "0.20")
        self.assertEqual(
            smooth_path.attrib["check_for_collisions"], "true")
        self.assertEqual(tracking.tag, "PipelineSequence")
        self.assertEqual(
            [child.tag for child in tracking],
            ["RemovePassedGoals", "ReactiveFallback"]
            if is_return_tree else ["RemovePassedGoals", "FollowPath"],
        )
        continuous_prune, tracking_action = tracking
        self.assertEqual(continuous_prune.attrib["input_goals"], "{goals}")
        self.assertEqual(continuous_prune.attrib["output_goals"], "{goals}")
        self.assertEqual(continuous_prune.attrib["radius"], "0.40")
        self.assertEqual(continuous_prune.attrib["global_frame"], "odom_combined")
        self.assertEqual(
            continuous_prune.attrib["robot_base_frame"], "base_footprint")
        if is_return_tree:
            self.assertEqual(
                [child.tag for child in tracking_action],
                ["GoalReached", "FollowPath"],
            )
            goal_reached, follow_path = tracking_action
            self.assertEqual(goal_reached.attrib["goal"], "{terminal_path_goal}")
            self.assertEqual(goal_reached.attrib["global_frame"], "odom_combined")
            self.assertEqual(
                goal_reached.attrib["robot_base_frame"], "base_footprint")
        else:
            follow_path = tracking_action
        self.assertEqual(follow_path.tag, "FollowPath")
        self.assertEqual(follow_path.attrib["path"], "{path}")

        self.assertEqual(recovery_actions.tag, "Sequence")
        self.assertEqual([child.tag for child in recovery_actions], [
            "BackUp", "RemovePassedGoals"])
        backup, recovery_prune = recovery_actions
        self.assertEqual(backup.attrib["backup_dist"], "0.20")
        self.assertEqual(backup.attrib["backup_speed"], "0.25")
        self.assertEqual(recovery_prune.attrib["input_goals"], "{goals}")
        self.assertEqual(recovery_prune.attrib["output_goals"], "{goals}")
        self.assertEqual(recovery_prune.attrib["radius"], "0.40")
        self.assertEqual(recovery_prune.attrib["global_frame"], "odom_combined")
        self.assertEqual(
            recovery_prune.attrib["robot_base_frame"], "base_footprint")

    def test_package_declares_direct_runtime_dependencies(self):
        source = PACKAGE_XML.read_text(encoding="utf-8")
        for dependency in (
            "geometry_msgs",
            "nav_msgs",
            "nav2_msgs",
            "rclpy",
            "robot_localization",
            "smartcar_interfaces",
            "smartcar_nav2",
            "std_msgs",
            "std_srvs",
            "unique_identifier_msgs",
            "python3-yaml",
            "zbar_ros",
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", source)
        self.assertNotIn("<exec_depend>action_msgs</exec_depend>", source)
        self.assertNotIn("<exec_depend>smartcar_safety</exec_depend>", source)
        self.assertIn("<test_depend>action_msgs</test_depend>", source)

    def test_launch_defaults_to_no_autostart_and_existing_waypoints(self):
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"task_config_file"', source)
        self.assertNotIn('LaunchConfiguration("config_file")', source)
        self.assertIn('"autostart_mission"', source)
        self.assertIn('default_value="false"', source)
        self.assertIn('"navigation_test_end_segment_id"', source)
        self.assertIn('"supervised_p_to_a_only"', source)
        self.assertIn('"supervised_p_to_c1_only"', source)
        self.assertIn('"waypoints_calibrated"', source)
        self.assertIn('FindPackageShare("smartcar_nav2")', source)
        self.assertIn('"default_waypoints.yaml"', source)

    def test_competition_task_parameters_are_explicit_and_disabled_by_default(self):
        parameters = yaml.safe_load(
            TASK_CONFIG.read_text(encoding="utf-8")
        )["task_node"]["ros__parameters"]
        task_launch = LAUNCH.read_text(encoding="utf-8")
        node_source = NODE.read_text(encoding="utf-8")

        for name in (
            "supervised_competition_mode",
            "continue_after_qr_failure",
            "qr_reader_preloaded",
        ):
            self.assertIs(parameters[name], False)
            self.assertIn(f'"{name}"', task_launch)
            self.assertIn(
                f'"{name}": LaunchConfiguration(',
                task_launch,
            )
            self.assertIn(
                f'self.declare_parameter("{name}", False)',
                node_source,
            )

        self.assertIn(
            '"supervised_competition_mode": LaunchConfiguration(',
            task_launch,
        )
        self.assertIn(
            '"continue_after_qr_failure": LaunchConfiguration(',
            task_launch,
        )
        self.assertIn(
            '"qr_reader_preloaded": LaunchConfiguration(',
            task_launch,
        )

    def test_competition_mode_keeps_the_confirmed_wheel_imu_profile(self):
        source = SYSTEM.read_text(encoding="utf-8")
        competition_validation = source.split(
            'if _as_bool(context, "supervised_competition_mode"):',
            1,
        )[1].split('if not _as_bool(context, "autostart_mission"):', 1)[0]
        self.assertIn('"localization_profile"', competition_validation)
        self.assertIn('localization_profile=wheel_imu', competition_validation)

    def test_competition_qr_selects_and_publishes_runtime_c_zone_variant(self):
        competition_source = COMPETITION.read_text(encoding="utf-8")
        mission_source = MISSION.read_text(encoding="utf-8")
        node_source = NODE.read_text(encoding="utf-8")

        self.assertIn(
            "if classify_qr_parity(content) == QR_PARITY_ODD:",
            competition_source,
        )
        self.assertIn("return CLOCKWISE", competition_source)
        self.assertIn("return COUNTERCLOCKWISE", competition_source)
        self.assertIn("QR_PARITY_UNRECOGNIZED", competition_source)

        self.assertIn(
            "and selected_c_zone_direction != COUNTERCLOCKWISE",
            node_source,
        )
        self.assertIn(
            "supervised competition mode requires the authored ",
            node_source,
        )
        self.assertIn("clockwise_waypoints = apply_c_zone_direction(", node_source)
        self.assertIn(
            "if clockwise_planning_segments != planning_segments:",
            node_source,
        )
        self.assertIn(
            "clockwise C-zone variant changes planning segments",
            node_source,
        )
        self.assertIn("self._competition_navigation_variants = {", node_source)
        self.assertIn(
            "COUNTERCLOCKWISE: self._navigation_segments", node_source)
        self.assertIn("CLOCKWISE: clockwise_navigation_segments", node_source)
        self.assertIn(
            "c_zone_navigation_variants=self._competition_navigation_variants",
            node_source,
        )

        self.assertIn(
            "counterclockwise C-zone variant must equal the baseline route",
            mission_source,
        )
        self.assertIn(
            "direction = c_zone_direction_for_qr(task_result.text)",
            mission_source,
        )
        self.assertIn("selected_segments = variants[direction]", mission_source)
        self.assertIn(
            "c_zone_variant_rewrites_completed_segments",
            mission_source,
        )
        self.assertIn("self._publish_c_zone_direction(direction)", mission_source)
        self.assertIn(
            'String, "/smartcar/output/c_zone_direction", 10)',
            node_source,
        )
        self.assertIn("def publish_c_zone_direction(self, value):", node_source)
        self.assertIn("publish_direction(text)", mission_source)

    def test_node_uses_worker_thread_and_expected_interfaces(self):
        source = NODE.read_text(encoding="utf-8")
        for token in (
            "ActionClient(",
            "def prewarm_action_clients(self):",
            "self._navigator.prewarm_action_clients()",
            "NavigateToPose,",
            '"/navigate_to_pose"',
            '"/smartcar/direction_guard/prepare"',
            '"/smartcar/direction_guard/activate"',
            '"/smartcar/direction_guard/renew"',
            '"/smartcar/direction_guard/stop"',
            '"/smartcar/task/start"',
            '"/smartcar/task/stop"',
            '"/smartcar/task/reset"',
            '"/smartcar/task/state"',
            '"/smartcar/output/text"',
            '"/smartcar/output/speech"',
            "MultiThreadedExecutor(num_threads=4)",
            "threading.Thread",
        ):
            self.assertIn(token, source)

    def test_navigation_goals_are_uuid_and_direction_bound(self):
        source = NAVIGATION_GOALS.read_text(encoding="utf-8")
        self.assertIn("goal = NavigateToPose.Goal()", source)
        self.assertIn("goal.pose = self.pose_stamped(waypoint)", source)
        self.assertIn("pose.header.frame_id = waypoint.frame_id", source)
        self.assertIn("goal.behavior_tree = navigation_behavior_tree(", source)
        self.assertIn("navigation_goal_requires_forward_direction", source)
        self.assertIn("navigation_through_requires_forward_direction", source)
        self.assertIn("goal = NavigateThroughPoses.Goal()", source)
        self.assertIn("goal.poses = [self.pose_stamped(waypoint)", source)
        node_source = NODE.read_text(encoding="utf-8")
        self.assertIn("goal, goal_uuid=action_uuid", node_source)
        self.assertIn("direction = motion_direction()", node_source)
        self.assertIn("Nav2GoalFactory", node_source)
        self.assertNotIn("FollowWaypoints", source)

    def test_via_waypoints_are_planning_only_not_arrival_targets(self):
        rules = AGENTS.read_text(encoding="utf-8")
        planning = (PACKAGE / "smartcar_task" / "planning_segments.py").read_text(
            encoding="utf-8")
        mission = MISSION.read_text(encoding="utf-8")
        self.assertIn("明确拒绝参与到达判定", rules)
        self.assertIn("不得作为 `end_id`", rules)
        self.assertIn("ComputePathThroughPoses", rules)
        self.assertIn("ConstrainedSmoother", rules)
        self.assertIn("完整 footprint 碰撞检查", rules)
        self.assertIn("不得落盘、复用、写死", rules)
        self.assertIn("FollowPath", rules)
        self.assertIn("每段最多 6 次直线 `BackUp`", rules)
        self.assertIn("end_id must not be a via waypoint", planning)
        self.assertIn("must reference a via waypoint", planning)
        self.assertIn("navigation_segment_intermediate_not_via", mission)
        self.assertIn("navigation_segment_endpoint_is_via", mission)
        self.assertIn("self._navigator.navigate_through(segment)", mission)
        for filename in THROUGH_POSES_TREES:
            tree = (NAV2_BEHAVIOR_TREES / filename).read_text(encoding="utf-8")
            self.assertIn("ComputePathThroughPoses goals=\"{goals}\"", tree)
            self.assertEqual(tree.count("<RemoveDuplicatePathPoints "), 1)
            self.assertEqual(tree.count("<SmoothPath "), 1)
            self.assertEqual(tree.count("<FollowPath "), 1)
            self.assertEqual(tree.count("<ComputePathThroughPoses "), 1)
            self.assertIn('RecoveryNode number_of_retries="6"', tree)
            self.assertEqual(tree.count("<BackUp "), 1)
            self.assertIn('backup_dist="0.20"', tree)
            self.assertIn('backup_speed="0.25"', tree)
            self.assertEqual(tree.count("<RemovePassedGoals "), 2)
            self.assertIn("PipelineSequence", tree)
            self.assertNotIn("Spin", tree)
            self.assertNotIn("Wait", tree)
            self.assertNotIn("DriveOnHeading", tree)
            self._assert_through_poses_recovery_tree(filename)

    def test_behavior_tree_paths_are_resolved_from_the_installed_nav2_package(self):
        parameters = yaml.safe_load(
            TASK_CONFIG.read_text(encoding="utf-8")
        )["task_node"]["ros__parameters"]
        source = NODE.read_text(encoding="utf-8")
        self.assertFalse(
            any(name.endswith("_behavior_tree") for name in parameters)
        )
        self.assertIn("get_package_share_directory", source)
        self.assertIn("def _nav2_behavior_tree_path(filename):", source)
        self.assertNotIn("/root/ros2_ws", source)
        for filename in (
            "navigate_to_pose_precise_w_replanning_and_recovery.xml",
            "navigate_to_pose_transit_w_replanning_and_recovery.xml",
            "navigate_through_poses_w_replanning_and_recovery.xml",
            "navigate_through_poses_return_w_replanning_and_recovery.xml",
            "navigate_through_poses_transit_w_replanning_and_recovery.xml",
            "navigate_through_poses_precise_w_replanning_and_recovery.xml",
        ):
            self.assertIn(filename, source)
        self.assertNotIn("reverse_behavior_tree", source)
        self.assertNotIn("reverse_handoff", source)

    def test_nav2_installs_only_native_forward_trees_with_bounded_recovery(self):
        expected = {
            "navigate_to_pose_w_replanning_and_recovery.xml",
            "navigate_to_pose_precise_w_replanning_and_recovery.xml",
            "navigate_to_pose_transit_w_replanning_and_recovery.xml",
            "navigate_through_poses_w_replanning_and_recovery.xml",
            "navigate_through_poses_precise_w_replanning_and_recovery.xml",
            "navigate_through_poses_transit_w_replanning_and_recovery.xml",
            "navigate_through_poses_return_w_replanning_and_recovery.xml",
        }
        observed = {
            path.name for path in NAV2_BEHAVIOR_TREES.glob("*.xml")
        }
        self.assertEqual(observed, expected)
        forbidden = (
            "ComputeFreeHeading",
            "RecordFollowPath",
            "AckermannReverse",
            "Reverse",
            "Spin",
            "Wait",
            "DriveOnHeading",
        )
        for tree in NAV2_BEHAVIOR_TREES.glob("*.xml"):
            source = tree.read_text(encoding="utf-8")
            self.assertIn("FollowPath", source)
            self.assertTrue(
                "ComputePathToPose" in source
                or "ComputePathThroughPoses" in source
            )
            for token in forbidden:
                self.assertNotIn(token, source)
            self.assertIn('RecoveryNode number_of_retries="6"', source)
            self.assertEqual(source.count("<BackUp "), 1)
            self.assertIn('backup_dist="0.20"', source)
            self.assertIn('backup_speed="0.25"', source)
            if tree.name in THROUGH_POSES_TREES:
                self.assertIn("PipelineSequence", source)
                self.assertEqual(source.count("<RemoveDuplicatePathPoints "), 1)
                self.assertEqual(source.count("<SmoothPath "), 1)
            else:
                self.assertNotIn("PipelineSequence", source)
                self.assertNotIn("<RemoveDuplicatePathPoints ", source)
                self.assertNotIn("<SmoothPath ", source)
        for filename in THROUGH_POSES_TREES:
            tree = (NAV2_BEHAVIOR_TREES / filename).read_text(encoding="utf-8")
            self.assertEqual(tree.count("<BackUp "), 1)
            self.assertIn('backup_speed="0.25"', tree)
            self.assertEqual(tree.count("<RemovePassedGoals "), 2)
            self.assertEqual(tree.count("<ComputePathThroughPoses "), 1)
            self.assertEqual(tree.count("<RemoveDuplicatePathPoints "), 1)
            self.assertEqual(tree.count("<SmoothPath "), 1)
            self._assert_through_poses_recovery_tree(filename)
        for filename in (
            "navigate_to_pose_w_replanning_and_recovery.xml",
            "navigate_to_pose_precise_w_replanning_and_recovery.xml",
            "navigate_to_pose_transit_w_replanning_and_recovery.xml",
        ):
            tree = (NAV2_BEHAVIOR_TREES / filename).read_text(encoding="utf-8")
            self.assertNotIn("RemovePassedGoals", tree)
        cmake = NAV2_CMAKE.read_text(encoding="utf-8")
        for token in (*forbidden[:4], "ForwardOnlyRPP"):
            self.assertNotIn(token, cmake)
        params_file = NAV2_CONFIG / "nav2_params.yaml"
        params = params_file.read_text(encoding="utf-8")
        for token in ("nav2_spin", "nav2_wait", "nav2_drive_on_heading"):
            self.assertNotIn(token, params)
        self.assertIn("nav2_back_up_action_bt_node", params)
        self.assertIn("nav2_remove_passed_goals_action_bt_node", params)
        self.assertIn("nav2_smooth_path_action_bt_node", params)
        self.assertIn("smartcar_remove_duplicate_path_points_bt_node", params)
        parsed_params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
        for navigator in (
            "bt_navigator",
            "bt_navigator_navigate_through_poses_rclcpp_node",
        ):
            plugin_lib_names = parsed_params[navigator]["ros__parameters"][
                "plugin_lib_names"
            ]
            self.assertIn("nav2_pipeline_sequence_bt_node", plugin_lib_names)
            self.assertIn("nav2_smooth_path_action_bt_node", plugin_lib_names)
            self.assertIn(
                "smartcar_remove_duplicate_path_points_bt_node",
                plugin_lib_names,
            )
        smoother = parsed_params["smoother_server"]["ros__parameters"]
        self.assertEqual(smoother["smoother_plugins"], ["constrained_smoother"])
        constrained = smoother["constrained_smoother"]
        self.assertEqual(
            constrained["plugin"],
            "nav2_constrained_smoother/ConstrainedSmoother",
        )
        self.assertFalse(constrained["reversing_enabled"])
        self.assertEqual(float(constrained["minimum_turning_radius"]), 0.23)
        self.assertFalse(parsed_params["planner_server"]["ros__parameters"]
                         ["GridBased"]["smooth_path"])
        nav2_package = NAV2_PACKAGE_XML.read_text(encoding="utf-8")
        self.assertIn("<exec_depend>nav2_smoother</exec_depend>", nav2_package)
        self.assertIn(
            "<exec_depend>nav2_constrained_smoother</exec_depend>",
            nav2_package,
        )
        self.assertIn(
            "smartcar_remove_duplicate_path_points_bt_node", cmake)
        self.assertIn("remove_duplicate_path_points_action.cpp", cmake)
        self.assertIn('behavior_plugins: ["backup"]', params)
        self.assertIn("min_velocity: [-0.25", params)
        launch = NAV2_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("'smoother_server',", launch)
        self.assertIn("package='nav2_smoother'", launch)
        self.assertIn("executable='smoother_server'", launch)
        self.assertIn("'behavior_server',", launch)
        self.assertIn("package='nav2_behaviors'", launch)
        self.assertIn("executable='behavior_server'", launch)
        self.assertIn("('cmd_vel', 'cmd_vel_nav')", launch)

    def test_follow_path_keeps_an_approach_lookahead_floor(self):
        parameters = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )["controller_server"]["ros__parameters"]["FollowPath"]

        minimum = float(parameters["min_lookahead_dist"])
        approach = (
            float(parameters["min_approach_linear_velocity"])
            * float(parameters["lookahead_time"])
        )
        self.assertTrue(parameters["use_velocity_scaled_lookahead_dist"])
        self.assertEqual(float(parameters["lookahead_dist"]), 0.50)
        self.assertEqual(minimum, 0.50)
        self.assertEqual(float(parameters["max_lookahead_dist"]), 0.50)
        self.assertGreaterEqual(minimum, approach)
        self.assertLessEqual(minimum, float(parameters["max_lookahead_dist"]))
        self.assertFalse(parameters["use_rotate_to_heading"])
        self.assertFalse(parameters["allow_reversing"])

    def test_physical_navigation_speed_cap_is_consistent(self):
        parameters = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )
        controller = parameters["controller_server"]["ros__parameters"]
        smoother = parameters["velocity_smoother"]["ros__parameters"]
        safety = yaml.safe_load((
            ROOT / "src" / "smartcar_safety" / "config" / "safety.yaml"
        ).read_text(encoding="utf-8"))["safety_node"]["ros__parameters"]

        self.assertEqual(float(controller["FollowPath"]["desired_linear_vel"]), 0.50)
        self.assertEqual(float(smoother["max_velocity"][0]), 0.50)
        self.assertEqual(float(safety["max_linear_speed_mps"]), 0.50)

    def test_collision_failure_immediately_enters_native_back_up_recovery(self):
        controller = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )["controller_server"]["ros__parameters"]

        # RPP raises NoValidControl for its live-costmap collision prediction.
        # Zero tolerance lets FollowPath fail immediately, so the existing
        # RecoveryNode takes the bounded native BackUp path rather than
        # emitting a controller-side zero-velocity hold.
        self.assertEqual(float(controller["failure_tolerance"]), 0.0)
        self.assertTrue(controller["FollowPath"]["use_collision_detection"])
        for filename in THROUGH_POSES_TREES:
            self._assert_through_poses_recovery_tree(filename)

    def test_stalled_follow_path_uses_a_short_global_progress_deadline(self):
        controller = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )["controller_server"]["ros__parameters"]
        progress = controller["progress_checker"]

        self.assertEqual(
            progress["plugin"], "nav2_controller::SimpleProgressChecker")
        self.assertEqual(float(progress["required_movement_radius"]), 0.25)
        self.assertEqual(float(progress["movement_time_allowance"]), 6.0)
        self.assertEqual(float(controller["failure_tolerance"]), 0.0)

    def test_return_goal_tolerance_precedes_native_back_up(self):
        parameters = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )
        controller = parameters["controller_server"]["ros__parameters"]
        return_checker = controller["return_goal_checker"]

        self.assertEqual(
            return_checker["plugin"], "nav2_controller::PositionGoalChecker")
        self.assertEqual(float(return_checker["xy_goal_tolerance"]), 0.15)
        self.assertFalse(return_checker["stateful"])
        for navigator in (
            "bt_navigator",
            "bt_navigator_navigate_through_poses_rclcpp_node",
        ):
            navigator_parameters = parameters[navigator]["ros__parameters"]
            self.assertEqual(
                float(navigator_parameters["goal_reached_tol"]),
                float(return_checker["xy_goal_tolerance"]),
            )
            self.assertIn(
                "nav2_get_pose_from_path_action_bt_node",
                navigator_parameters["plugin_lib_names"],
            )
            self.assertIn(
                "nav2_goal_reached_condition_bt_node",
                navigator_parameters["plugin_lib_names"],
            )
        self._assert_through_poses_recovery_tree(
            "navigate_through_poses_return_w_replanning_and_recovery.xml")

    def test_costmaps_use_the_configured_inflation_radius(self):
        parameters = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )
        for name in ("local_costmap", "global_costmap"):
            inflation = parameters[name][name]["ros__parameters"][
                "inflation_layer"
            ]
            self.assertEqual(float(inflation["inflation_radius"]), 0.30)

    def test_precise_terminal_profile_keeps_tight_position_and_valid_yaw(self):
        parameters = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )["controller_server"]["ros__parameters"]
        precise = parameters["precise_goal_checker"]
        standard = parameters["goal_checker"]

        self.assertLess(
            float(precise["xy_goal_tolerance"]),
            float(standard["xy_goal_tolerance"]),
        )
        self.assertGreater(float(precise["yaw_goal_tolerance"]), 0.0)
        self.assertEqual(
            precise["plugin"], "nav2_controller::SimpleGoalChecker")
        self.assertFalse(precise["stateful"])
        for filename in (
            "navigate_to_pose_precise_w_replanning_and_recovery.xml",
            "navigate_through_poses_precise_w_replanning_and_recovery.xml",
        ):
            tree = (NAV2_BEHAVIOR_TREES / filename).read_text(encoding="utf-8")
            self.assertIn('goal_checker_id="precise_goal_checker"', tree)

    def test_direction_renewal_is_diagnostic_when_lease_expiry_is_disabled(self):
        task = yaml.safe_load(TASK_CONFIG.read_text(encoding="utf-8"))[
            "task_node"]["ros__parameters"]
        guard = yaml.safe_load(
            DIRECTION_GUARD_CONFIG.read_text(encoding="utf-8")
        )["direction_guard"]["ros__parameters"]

        self.assertEqual(float(guard["permit_timeout_sec"]), 0.0)
        self.assertEqual(float(task["direction_lease_timeout_sec"]), 0.0)
        self.assertEqual(float(task["direction_renew_period_sec"]), 0.10)
        self.assertEqual(float(task["direction_service_timeout_sec"]), 0.20)
        source = NODE.read_text(encoding="utf-8")
        self.assertIn("def _warn_renewal_failure(self, result):", source)
        self.assertIn(
            'f"direction renewal unavailable ({status}); continuing under "',
            source,
        )
        self.assertIn("continuing under", source)
        self.assertNotIn("return renewed", source)
        self.assertIn(
            'self.declare_parameter("direction_lease_timeout_sec", 0.0)',
            source,
        )

    def test_navigation_failure_is_single_attempt_by_default(self):
        parameters = yaml.safe_load(
            TASK_CONFIG.read_text(encoding="utf-8")
        )["task_node"]["ros__parameters"]

        self.assertEqual(parameters["navigation_retries"], 0)
        self.assertIn("navigation_retries: int = 0", MISSION.read_text(
            encoding="utf-8"))
        self.assertIn(
            'self.declare_parameter("navigation_retries", 0)',
            NODE.read_text(encoding="utf-8"),
        )

    def test_reset_adapter_orders_set_pose_before_odom_verification(self):
        source = NODE.read_text(encoding="utf-8")
        sequence = source[
            source.index("return run_reset_sequence("):
            source.index("def _wait_for_reset_services")
        ]
        set_pose = sequence.index("self._call_set_pose")
        verify = sequence.index("self._wait_for_verified_origin")
        self.assertLess(set_pose, verify)
        self.assertIn("self._set_pose_client.call_async", source)
        self.assertNotIn("_clear_fault_client", source)

    def test_task_node_never_publishes_chassis_velocity(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertNotIn("/cmd_vel", source)
        self.assertNotIn("geometry_msgs.msg import Twist", source)

    def test_placeholder_waypoints_are_blocked_by_default(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn(
            'self.declare_parameter("waypoints_calibrated", False)', source)
        self.assertIn(
            'waypoint_document.get("calibrated") is True', source)
        self.assertIn(
            'self._motion_gates["waypoints_calibrated"] = False', source)
        self.assertIn('"motion gates not satisfied: "', source)

    def test_navigation_test_is_limited_to_a_contiguous_route_prefix(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn('"navigation_test_end_segment_id"', source)
        self.assertIn("select_segment_prefix(", source)

    def test_supervised_navigation_modes_are_pure_and_explicit(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("supervised_p_to_a_only", False)', source)
        self.assertIn('self.declare_parameter("supervised_p_to_c1_only", False)', source)
        self.assertIn('self.declare_parameter("c_zone_direction", "counterclockwise")', source)
        self.assertIn("apply_c_zone_direction(", source)
        self.assertIn("SUPERVISED_P_TO_A_SEGMENT_ID", source)
        self.assertIn("SUPERVISED_P_TO_C1_SEGMENT_ID", source)
        self.assertIn("SUPERVISED_PREFIX_TASKS", source)
        self.assertIn(
            '"nav", "via", "via", "via", "via", "nav",',
            source.replace("\n", " "),
        )
        system_source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn("supervised navigation route requires:", system_source)
        self.assertIn('"supervised_full_route", default_value="false"', system_source)
        self.assertIn('"supervised_full_route": LaunchConfiguration(', system_source)
        self.assertIn('"c_zone_direction": LaunchConfiguration(', system_source)
        self.assertIn('"safety_emergency_stop_on_start",', system_source)
        self.assertIn('"use_camera",', system_source)
        self.assertIn('"use_vision",', system_source)
        supervised_block = system_source.split(
            "if supervised_prefixes or supervised_full_route:", 1
        )[1].split("if short_drive_test:", 1)[0]
        self.assertIn('"use_depth_camera",', supervised_block)
        self.assertNotIn('"use_lidar",', supervised_block)

    def test_qr_handoff_test_mode_is_opt_in_and_requires_camera_and_vision(self):
        parameters = yaml.safe_load(
            TASK_CONFIG.read_text(encoding="utf-8")
        )["task_node"]["ros__parameters"]
        task_launch = LAUNCH.read_text(encoding="utf-8")
        system_launch = SYSTEM.read_text(encoding="utf-8")
        self.assertIs(parameters["qr_handoff_test_mode"], False)
        self.assertIn('self.declare_parameter("qr_handoff_test_mode", False)', NODE.read_text(encoding="utf-8"))
        self.assertIn('"qr_handoff_test_mode"', task_launch)
        self.assertIn('"qr_handoff_test_mode"', system_launch)
        self.assertIn("qr_handoff_test_mode requires:", system_launch)
        self.assertIn('"use_camera"', system_launch)
        self.assertIn('"use_vision"', system_launch)

    def test_navigation_test_script_has_only_compact_start_and_go_entries(self):
        source = NAV_TEST.read_text(encoding="utf-8")
        self.assertIn(
            "bash /home/sunrise/ros2_ws/scripts/nav_test.sh "
            "[--go] [--wheel-only] [--cw]",
            source,
        )
        self.assertIn("--go) GO=true", source)
        self.assertIn("--wheel-only) WHEEL_ONLY=true", source)
        self.assertIn("--cw) C_ZONE_DIRECTION=clockwise", source)
        self.assertIn("C_ZONE_DIRECTION=counterclockwise", source)
        self.assertIn("--c-zone-direction=clockwise) C_ZONE_DIRECTION=clockwise", source)
        self.assertIn(
            "--c-zone-direction=counterclockwise) C_ZONE_DIRECTION=counterclockwise",
            source,
        )
        for retired_option in (
            "--autostart", "--supervised-p-to-a", "--supervised-p-to-c1",
            "--supervised-full-route", "--short-drive", "--verify",
            "--no-depth-camera", "--reset-origin", "--p-to-a", "--p-to-c1",
            "setsid",
        ):
            self.assertNotIn(retired_option, source)
        self.assertIn("autostart_mission:=false", source)
        self.assertIn("safety_emergency_stop_on_start:=true", source)
        self.assertNotIn("safety_emergency_stop_on_start:=false", source)
        self.assertIn('STATUS_TOOL="$WORKSPACE/scripts/nav_status.py"', source)
        self.assertNotIn(".smartcar_nav_prepare_fingerprint", source)
        self.assertNotIn("source_fingerprint", source)
        self.assertNotIn("require_prepared_workspace", source)
        self.assertIn('STATUS_ARGS=(--timeout 60 --timeline-log "$LOG")', source)
        self.assertIn('hold_started_stack', source)
        self.assertIn('nohup ros2 launch smartcar_bringup', source)
        self.assertIn('nohup rviz2 -d', source)
        self.assertIn('WHEEL_ONLY=false', source)
        self.assertIn('LOCALIZATION_PROFILE_ARG="localization_profile:=wheel_imu"', source)
        self.assertIn('LOCALIZATION_PROFILE_ARG="localization_profile:=wheel_only"', source)
        self.assertIn('$EXTRA_ARGS $LOCALIZATION_PROFILE_ARG', source)
        self.assertIn('$C_ZONE_DIRECTION_ARG', source)
        self.assertIn('set_software_emergency_stop false', source)
        self.assertIn('/smartcar/task/start', source)
        self.assertIn('re_latch_after_transition_failure', source)
        reset = source.index('ros2 service call /smartcar/task/reset')
        release = source.index('set_software_emergency_stop false')
        start = source.index('ros2 service call /smartcar/task/start')
        self.assertLess(reset, release)
        self.assertLess(release, start)
        self.assertIn('banner "等待人工发车"', source)
        self.assertNotIn("nav2_params_fixed.yaml", source)
        self.assertNotIn("ros_cleanup", source)

    def test_navigation_test_script_uses_depth_status_before_go(self):
        source = NAV_TEST.read_text(encoding="utf-8")
        status_source = NAV_STATUS.read_text(encoding="utf-8")
        prepare_source = NAV_PREPARE.read_text(encoding="utf-8")
        self.assertIn("use_depth_camera:=true", source)
        self.assertIn("aurora_ir_fps:=10 aurora_rgb_fps:=10", source)
        self.assertIn("--depth-camera", source)
        self.assertIn('expected_source = "depth_scan"', status_source)
        self.assertIn("obstacle_layer.observation_sources", status_source)
        self.assertIn("obstacle_layer.depth_scan.topic", status_source)
        self.assertIn("/local_costmap/costmap_raw", status_source)
        self.assertIn("/global_costmap/costmap_raw", status_source)
        self.assertNotIn("colcon build", source)
        self.assertIn("COLCON_PARALLEL_WORKERS=8", prepare_source)
        self.assertIn('--parallel-workers "$COLCON_PARALLEL_WORKERS"', prepare_source)
        self.assertIn("ros_cleanup", prepare_source)
        self.assertIn(
            "--packages-up-to smartcar_bringup smartcar_vision",
            prepare_source,
        )
        self.assertNotIn(".smartcar_nav_prepare_fingerprint", prepare_source)
        self.assertNotIn("source_fingerprint", prepare_source)
        self.assertIn("python3 \"$STATUS_TOOL\"", source)
        self.assertIn("startup health did not become ready", source)
        self.assertLess(
            source.index("python3 \"$STATUS_TOOL\""),
            source.rindex("start_rviz"),
        )
        self.assertNotIn('--rviz-pid "$RVIZ_PID"', source)
        self.assertTrue(NAV_STATUS.is_file())

    def test_ros_cleanup_targets_only_recorded_or_navigation_processes(self):
        source = ROS_CLEANUP.read_text(encoding="utf-8")
        self.assertIn("STATE_DIR=/tmp/smartcar_nav", source)
        self.assertIn("MEDIA_STATE_DIR=/tmp/smartcar_media", source)
        self.assertIn('stop_saved_process "$STATE_DIR/launch.pid"', source)
        self.assertIn('stop_saved_process "$STATE_DIR/rviz.pid"', source)
        self.assertIn('stop_saved_process "$MEDIA_STATE_DIR/qr_probe.pid"', source)
        self.assertIn(
            'stop_saved_process "$MEDIA_STATE_DIR/output_display.pid"',
            source,
        )
        self.assertIn(
            'stop_saved_process "$MEDIA_STATE_DIR/rgb_imshow.pid"',
            source,
        )
        self.assertIn('"ros2 launch smartcar_bringup smartcar_system.launch.py"', source)
        self.assertIn(
            '"python3 /home/sunrise/ros2_ws/scripts/competition_launch.py"',
            source,
        )
        self.assertIn(
            '"python3 /home/sunrise/ros2_ws/scripts/competition_launch.py" \\\n  USR1',
            source,
        )
        self.assertIn('"navigation.rviz"', source)
        self.assertIn("/proc/$pid/cmdline", source)
        self.assertIn('local stop_signal=${3:-TERM}', source)
        self.assertIn('kill -"$stop_signal" "$pid"', source)
        self.assertIn('for _ in 1 2 3 4 5', source)
        self.assertIn('kill -KILL "$pid"', source)
        for node in (
            "ekf_node", "controller_server", "planner_server",
            "smoother_server", "behavior_server", "bt_navigator",
            "velocity_smoother",
            "lifecycle_manager", "safety_node_cpp", "direction_guard_node",
            "origincar_base_node", "aurora930_node", "task_node",
            "depth_pointcloud_relay", "pointcloud_to_laserscan_node",
        ):
            self.assertIn(node, source)
        # Linux truncates this executable's comm name to "lifecycle_manag",
        # so pgrep -x lifecycle_manager alone cannot clear stale managers.
        self.assertIn(
            "'/nav2_lifecycle_manager/lifecycle_manager'", source)
        self.assertIn(
            "'/tf2_ros/static_transform_publisher.*__node:="
            "(base_to_link|base_to_gyro|link_to_laser|"
            "link_to_depth_camera_sensor)'", source)
        self.assertIn(
            "'/smartcar_tools/(field_reference_node|waypoint_viz)'", source)
        self.assertIn("'media_test\\.sh (qr|vlm)'", source)
        self.assertIn("'/zbar_ros/barcode_reader'", source)
        self.assertIn("image_replay_node", source)
        self.assertIn("remove_stale_fastdds_ports", source)
        self.assertIn('fuser -s "$port"', source)
        self.assertIn('rm -f -- "$port"', source)
        for unsafe_or_unrelated in (
            "setsid", "pkill -f", "ros2 daemon", "waypoint_editor",
            "\n    rviz2",
        ):
            self.assertNotIn(unsafe_or_unrelated, source)

    def test_planner_lookup_cache_is_bounded_to_the_field_scale(self):
        parameters = yaml.safe_load(
            (NAV2_CONFIG / "nav2_params.yaml").read_text(encoding="utf-8")
        )["planner_server"]["ros__parameters"]["GridBased"]

        self.assertEqual(float(parameters["lookup_table_size"]), 8.0)

    def test_vision_transport_wait_uses_the_request_deadline(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertNotIn("float(timeout_sec) + 1.0", source)
        self.assertGreaterEqual(
            source.count("future, max(0.0, float(timeout_sec))"), 2)

    def test_on_demand_zbar_uses_the_launch_resolved_image_topic(self):
        launch_source = LAUNCH.read_text(encoding="utf-8")
        node_source = NODE.read_text(encoding="utf-8")
        self.assertIn('"barcode_reader_image_topic"', launch_source)
        self.assertIn(
            'self.declare_parameter("barcode_reader_image_topic", "/image")',
            node_source,
        )
        self.assertIn('f"image:={image_topic}"', node_source)
        self.assertNotIn("barcode_reader_cmd", node_source)


if __name__ == "__main__":
    unittest.main()
