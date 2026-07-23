"""RDK-only ROS integration tests with fake action and service servers."""
from pathlib import Path
import sys
import threading
import time
import unittest
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import FollowWaypoints
    import rclpy
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from robot_localization.srv import SetPose
    from std_srvs.srv import Trigger

    from smartcar_task.task_node import RosLocalization, RosNavigator
    from smartcar_task.waypoints import Waypoint

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python modules are unavailable")
class RosAdapterTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.client_node = rclpy.create_node("task_adapter_test_client")
        self.server_node = rclpy.create_node("task_adapter_test_server")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.client_node)
        self.executor.add_node(self.server_node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()
        self.resources = []

    def tearDown(self):
        for resource in reversed(self.resources):
            resource.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.client_node.destroy_node()
        self.server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_follow_waypoints_success_and_cancel_reach_terminal_results(self):
        modes = ["success", "cancel"]
        received_poses = []

        def execute(goal_handle):
            received_poses.append(tuple(
                (pose.header.frame_id, pose.pose.position.x)
                for pose in goal_handle.request.poses
            ))
            mode = modes.pop(0)
            if mode == "success":
                goal_handle.succeed()
            else:
                deadline = time.monotonic() + 2.0
                while (
                    not goal_handle.is_cancel_requested
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.01)
                goal_handle.canceled()
            return FollowWaypoints.Result()

        action_server = ActionServer(
            self.server_node,
            FollowWaypoints,
            "/follow_waypoints",
            execute_callback=execute,
            cancel_callback=lambda _request: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.resources.append(action_server)
        navigator = RosNavigator(
            self.client_node,
            ReentrantCallbackGroup(),
            navigation_timeout_sec=3.0,
            goal_response_timeout_sec=1.0,
            cancel_timeout_sec=2.0,
        )
        item = Waypoint(
            frame_id="odom_combined",
            position=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            task="start",
        )
        endpoint = Waypoint(
            frame_id="odom_combined",
            position=(1.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            task="qr",
        )
        segment = (item, endpoint)

        self.assertTrue(navigator.wait_ready(1.0))
        success = navigator.navigate(segment)
        self.assertTrue(success.success, success.status)
        self.assertEqual(
            received_poses[0],
            (("odom_combined", 0.0), ("odom_combined", 1.0)),
        )

        class LateCancelFuture:
            @staticmethod
            def result():
                return SimpleNamespace(return_code=1, goals_canceling=[])

        # A late rejection from the old CancelGoal service must not poison a
        # generation whose GetResult already proved a terminal action state.
        navigator._on_cancel_response(
            navigator._goal_generation,
            LateCancelFuture(),
        )
        self.assertFalse(navigator.is_active())

        outcome = []
        worker = threading.Thread(
            target=lambda: outcome.append(navigator.navigate(segment)))
        worker.start()
        deadline = time.monotonic() + 1.0
        while not navigator.is_active() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(navigator.is_active())
        self.assertTrue(navigator.cancel())
        worker.join(timeout=2.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(outcome[0].status, "navigation_canceled")
        self.assertEqual(len(received_poses[1]), 2)
        self.assertFalse(navigator.is_active())
        self.assertEqual(GoalStatus.STATUS_CANCELED, 5)

    def test_localization_reset_ignores_pre_response_and_far_odometry(self):
        order = []
        odom_publisher = self.server_node.create_publisher(
            Odometry, "/odom_combined", 10)

        def publish_odom(x):
            message = Odometry()
            message.header.frame_id = "odom_combined"
            message.child_frame_id = "base_footprint"
            message.pose.pose.position.x = x
            message.pose.pose.orientation.w = 1.0
            odom_publisher.publish(message)
            order.append(f"odom:{x}")

        def set_pose(request, response):
            order.append("set_pose")
            self.assertIsInstance(request.pose, PoseWithCovarianceStamped)
            self.assertEqual(request.pose.header.frame_id, "odom_combined")
            publish_odom(0.0)
            threading.Timer(0.05, lambda: publish_odom(1.0)).start()
            threading.Timer(0.10, lambda: publish_odom(0.0)).start()
            return response

        def clear_fault(_request, response):
            order.append("clear_fault")
            response.success = True
            response.message = "cleared"
            return response

        self.server_node.create_service(SetPose, "/set_pose", set_pose)
        self.server_node.create_service(
            Trigger,
            "/smartcar/safety/clear_localization_fault",
            clear_fault,
        )

        class StoppedNavigator:
            @staticmethod
            def is_active():
                return False

        localization = RosLocalization(
            self.client_node,
            StoppedNavigator(),
            ReentrantCallbackGroup(),
            reset_timeout_sec=2.0,
            position_tolerance=0.2,
            yaw_tolerance=0.2,
        )

        result = localization.reset_origin()

        self.assertTrue(result.success, result.status)
        self.assertEqual(order[0], "set_pose")
        self.assertIn("odom:1.0", order)
        self.assertLess(order.index("odom:0.0", 2), order.index("clear_fault"))


if __name__ == "__main__":
    unittest.main()
