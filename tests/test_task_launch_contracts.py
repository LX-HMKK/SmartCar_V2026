"""Static contracts for smartcar_task ROS wiring and reset ordering."""
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "smartcar_task"
NODE = PACKAGE / "smartcar_task" / "task_node.py"
LAUNCH = PACKAGE / "launch" / "smartcar_task.launch.py"
PACKAGE_XML = PACKAGE / "package.xml"
TASK_CONFIG = PACKAGE / "config" / "task.yaml"


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
        source = NODE.read_text(encoding="utf-8")
        self.assertIn("goal = NavigateToPose.Goal()", source)
        self.assertIn("goal.pose = self._pose_stamped(waypoint)", source)
        self.assertIn("pose.header.frame_id = waypoint.frame_id", source)
        self.assertIn("goal.behavior_tree = behavior_tree", source)
        self.assertIn("goal = NavigateThroughPoses.Goal()", source)
        self.assertIn("goal.poses = [self._pose_stamped(waypoint)", source)
        self.assertIn("goal, goal_uuid=action_uuid", source)
        self.assertIn("motion_direction(reverse_direction)", source)
        self.assertNotIn("FollowWaypoints", source)

    def test_precise_forward_behavior_tree_is_wired_from_config(self):
        parameters = yaml.safe_load(
            TASK_CONFIG.read_text(encoding="utf-8")
        )["task_node"]["ros__parameters"]
        precise_tree = parameters["precise_forward_behavior_tree"]
        self.assertTrue(
            precise_tree.endswith(
                "navigate_to_pose_precise_w_replanning_and_recovery.xml"
            )
        )

        source = NODE.read_text(encoding="utf-8")
        self.assertIn('"precise_forward_behavior_tree"', source)
        self.assertIn("goal_profile=waypoint.goal_profile", source)
        self.assertIn(
            "self.get_parameter(\"precise_forward_behavior_tree\").value",
            source,
        )

    def test_reverse_handoff_behavior_tree_is_wired_from_config(self):
        parameters = yaml.safe_load(
            TASK_CONFIG.read_text(encoding="utf-8")
        )["task_node"]["ros__parameters"]
        handoff_tree = parameters["reverse_handoff_behavior_tree"]
        self.assertTrue(
            handoff_tree.endswith(
                "navigate_to_pose_reverse_handoff_"
                "w_replanning_and_recovery.xml"
            )
        )

        source = NODE.read_text(encoding="utf-8")
        self.assertIn('"reverse_handoff_behavior_tree"', source)
        self.assertIn(
            "self.get_parameter(\"reverse_handoff_behavior_tree\").value",
            source,
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
        self.assertIn('"motion gates not satisfied: "', source)

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
