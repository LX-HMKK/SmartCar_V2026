"""Static contracts for smartcar_task ROS wiring and reset ordering."""
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "smartcar_task"
NODE = PACKAGE / "smartcar_task" / "task_node.py"
NAVIGATION_GOALS = PACKAGE / "smartcar_task" / "navigation_goals.py"
LAUNCH = PACKAGE / "launch" / "smartcar_task.launch.py"
PACKAGE_XML = PACKAGE / "package.xml"
TASK_CONFIG = PACKAGE / "config" / "task.yaml"
NAV_TEST = ROOT / "scripts" / "nav_test.sh"
SYSTEM = ROOT / "src" / "smartcar_bringup" / "launch" / "smartcar_system.launch.py"


class TaskLaunchContractTests(unittest.TestCase):
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
        self.assertIn('"autostart_mission"', source)
        self.assertIn('default_value="false"', source)
        self.assertIn('"navigation_test_end_segment_id"', source)
        self.assertIn('"supervised_p_to_a_only"', source)
        self.assertIn('"supervised_p_to_c1_only"', source)
        self.assertIn('"waypoints_calibrated"', source)
        self.assertIn('FindPackageShare("smartcar_nav2")', source)
        self.assertIn('"default_waypoints.yaml"', source)

    def test_node_uses_worker_thread_and_expected_interfaces(self):
        source = NODE.read_text(encoding="utf-8")
        for token in (
            "ActionClient(",
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
        self.assertIn("goal.behavior_tree = behavior_tree", source)
        self.assertIn("goal = NavigateThroughPoses.Goal()", source)
        self.assertIn("goal.poses = [self.pose_stamped(waypoint)", source)
        node_source = NODE.read_text(encoding="utf-8")
        self.assertIn("goal, goal_uuid=action_uuid", node_source)
        self.assertIn("motion_direction(reverse_direction)", node_source)
        self.assertIn("Nav2GoalFactory", node_source)
        self.assertNotIn("FollowWaypoints", source)

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
            "navigate_to_pose_reverse_w_replanning_and_recovery.xml",
            "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml",
            "navigate_to_pose_precise_w_replanning_and_recovery.xml",
            "navigate_to_pose_transit_w_replanning_and_recovery.xml",
            "navigate_through_poses_w_replanning_and_recovery.xml",
            "navigate_through_poses_return_w_replanning_and_recovery.xml",
            "navigate_through_poses_transit_w_replanning_and_recovery.xml",
            "navigate_through_poses_precise_w_replanning_and_recovery.xml",
            "navigate_through_poses_reverse_w_replanning_and_recovery.xml",
            "navigate_through_poses_reverse_locked_w_replanning_and_recovery.xml",
            "navigate_through_poses_reverse_return_w_replanning_and_recovery.xml",
        ):
            self.assertIn(filename, source)

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

    def test_supervised_navigation_modes_are_pure_and_prefix_limited(self):
        source = NODE.read_text(encoding="utf-8")
        script = NAV_TEST.read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("supervised_p_to_a_only", False)', source)
        self.assertIn('self.declare_parameter("supervised_p_to_c1_only", False)', source)
        self.assertIn("SUPERVISED_P_TO_A_SEGMENT_ID", source)
        self.assertIn("SUPERVISED_P_TO_C1_SEGMENT_ID", source)
        self.assertIn("SUPERVISED_PREFIX_TASKS", source)
        self.assertIn(
            '"nav", "via", "via", "via", "via", "nav",',
            source.replace("\n", " "),
        )
        self.assertIn("--supervised-p-to-a", script)
        self.assertIn("--supervised-p-to-c1", script)
        self.assertIn('"$END_SEGMENT_ID" != "p_to_qr"', script)
        self.assertIn('"$END_SEGMENT_ID" != "qr_to_vlm"', script)
        self.assertIn("if $SUPERVISED_P_TO_A || $SUPERVISED_P_TO_C1; then\n  # A watched fixed-prefix", script)
        self.assertIn("RESET_ORIGIN=true", script)
        system_source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn("supervised navigation prefix requires:", system_source)
        self.assertIn('"safety_emergency_stop_on_start",', system_source)
        self.assertIn('"use_camera",', system_source)
        self.assertIn('"use_vision",', system_source)

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

    def test_navigation_test_script_stays_latched_at_startup(self):
        source = NAV_TEST.read_text(encoding="utf-8")
        self.assertNotIn("--autostart", source)
        self.assertIn("autostart_mission:=false", source)
        self.assertIn("safety_emergency_stop_on_start:=true", source)
        self.assertNotIn("safety_emergency_stop_on_start:=false", source)
        self.assertIn("if $SUPERVISED_P_TO_A", source)
        self.assertIn("supervised_p_to_a_only:=true", source)
        self.assertIn("--short-drive 必须选择受看护 P→A 或 P→C1 前缀", source)

    def test_navigation_test_script_requires_live_obstacle_avoidance(self):
        source = NAV_TEST.read_text(encoding="utf-8")
        self.assertIn("verify_obstacle_avoidance", source)
        self.assertIn("obstacle_layer.enabled", source)
        self.assertIn("obstacle_layer.observation_sources", source)
        self.assertIn(
            'expected_sources="String value is: depth_points"', source)
        self.assertIn("if $DEPTH_CAMERA; then", source)
        self.assertIn("obstacle_layer.scan.topic", source)
        self.assertIn("obstacle_layer.scan.observation_persistence", source)
        self.assertIn("obstacle_layer.scan.min_obstacle_height", source)
        self.assertIn("obstacle_layer.scan.max_obstacle_height", source)
        self.assertIn("obstacle_layer.scan.inf_is_valid", source)
        self.assertIn("obstacle_layer.depth_points.topic", source)
        self.assertIn('LIDAR_ARGS="use_lidar:=false"', source)
        self.assertIn("aurora_ir_fps:=5 aurora_rgb_fps:=5", source)
        self.assertIn("inflation_layer.enabled", source)
        self.assertIn("ros2 param get --no-daemon", source)
        self.assertIn("ros2 topic echo --no-daemon", source)
        self.assertIn("for attempt in 1 2 3 4 5 6", source)
        self.assertIn("The status is transient-local", source)
        self.assertIn("AURORA_USBFS_BUFFER_MB=64", source)
        self.assertIn("ensure_aurora_usbfs_buffer", source)
        self.assertIn("usbfs_memory_mb", source)
        self.assertIn(
            "/local_costmap/costmap_raw nav2_msgs/msg/Costmap", source)
        self.assertIn(
            "/global_costmap/costmap_raw nav2_msgs/msg/Costmap", source)
        self.assertIn(
            "/local_costmap/costmap nav_msgs/msg/OccupancyGrid", source)
        self.assertIn(
            "/global_costmap/costmap nav_msgs/msg/OccupancyGrid", source)
        self.assertIn("避障感知未就绪；急停保持锁存", source)

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
