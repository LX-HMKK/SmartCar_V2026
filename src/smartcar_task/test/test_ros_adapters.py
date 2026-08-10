"""ROS message-adapter tests for native forward Nav2 goals."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    import rclpy

    from smartcar_task.navigation_goals import Nav2GoalFactory
    from smartcar_task.waypoints import Waypoint

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def waypoint(waypoint_id, task, x, profile="standard", heading_mode=None, direction="forward"):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        task=task,
        direction=direction,
        id=waypoint_id,
        goal_profile=profile,
        heading_mode=heading_mode,
    )


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python modules are unavailable")
class Nav2GoalFactoryTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("native_nav_goal_adapter_test")
        self.factory = Nav2GoalFactory(
            self.node,
            "precise.xml",
            "transit.xml",
            "through.xml",
            "through-transit.xml",
            "through-precise.xml",
            "through-return.xml",
        )

    def tearDown(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_pose_goal_selects_standard_precise_and_transit_trees(self):
        standard = self.factory.navigate_goal(waypoint("a", "qr", 1.0))
        precise = self.factory.navigate_goal(
            waypoint("b", "qr", 2.0, "precise", "locked"))
        transit = self.factory.navigate_goal(
            waypoint("via", "via", 3.0, heading_mode="free"))

        self.assertEqual(standard.behavior_tree, "")
        self.assertEqual(precise.behavior_tree, "precise.xml")
        self.assertEqual(transit.behavior_tree, "transit.xml")
        self.assertEqual(precise.pose.header.frame_id, "odom_combined")
        self.assertEqual(precise.pose.pose.position.x, 2.0)

    def test_through_goal_selects_native_terminal_tree(self):
        precise = self.factory.navigate_through_goal((
            waypoint("via", "via", 1.0),
            waypoint("c", "vlm", 2.0, "precise", "locked"),
        ))
        returned = self.factory.navigate_through_goal((
            waypoint("via_2", "via", 3.0),
            waypoint("p_finish", "return", 0.0),
        ))
        self.assertEqual(precise.behavior_tree, "through-precise.xml")
        self.assertEqual(returned.behavior_tree, "through-return.xml")
        self.assertEqual(len(precise.poses), 2)

    def test_rejects_reverse_and_zero_quaternion_action_inputs(self):
        with self.assertRaisesRegex(ValueError, "forward_direction"):
            self.factory.navigate_goal(waypoint("bad", "nav", 1.0, direction="reverse"))
        zero = Waypoint(
            "odom_combined", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0),
            "via", id="zero")
        with self.assertRaisesRegex(ValueError, "unit quaternion"):
            self.factory.navigate_goal(zero)


if __name__ == "__main__":
    unittest.main()
