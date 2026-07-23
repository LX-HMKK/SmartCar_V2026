"""Static contracts for smartcar_task ROS wiring and reset ordering."""
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "smartcar_task"
NODE = PACKAGE / "smartcar_task" / "task_node.py"
LAUNCH = PACKAGE / "launch" / "smartcar_task.launch.py"
PACKAGE_XML = PACKAGE / "package.xml"


class TaskLaunchContractTests(unittest.TestCase):
    def test_package_declares_direct_runtime_dependencies(self):
        source = PACKAGE_XML.read_text(encoding="utf-8")
        for dependency in (
            "action_msgs",
            "geometry_msgs",
            "nav_msgs",
            "nav2_msgs",
            "rclpy",
            "robot_localization",
            "smartcar_interfaces",
            "smartcar_nav2",
            "smartcar_safety",
            "std_msgs",
            "std_srvs",
            "python3-yaml",
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", source)

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
            'node, FollowWaypoints, "/follow_waypoints"',
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

    def test_follow_waypoints_goal_contains_each_segment_pose(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn("waypoints = tuple(waypoints)", source)
        self.assertIn("for waypoint in waypoints:", source)
        self.assertIn("goal.poses.append(pose)", source)
        self.assertNotIn("goal.poses = [pose]", source)

    def test_reset_adapter_orders_set_pose_odom_verification_and_clear(self):
        source = NODE.read_text(encoding="utf-8")
        sequence = source[
            source.index("return run_reset_sequence("):
            source.index("def _wait_for_reset_services")
        ]
        set_pose = sequence.index("self._call_set_pose")
        verify = sequence.index("self._wait_for_verified_origin")
        clear = sequence.index("self._clear_localization_fault")
        self.assertLess(set_pose, verify)
        self.assertLess(verify, clear)
        self.assertIn("self._set_pose_client.call_async", source)
        self.assertIn("self._clear_fault_client.call_async", source)

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


if __name__ == "__main__":
    unittest.main()
