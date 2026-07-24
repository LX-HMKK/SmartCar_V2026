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
    from nav2_msgs.action import NavigateToPose
    import rclpy
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from robot_localization.srv import SetPose


    from smartcar_task.mission import OperationResult
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

    def test_guarded_navigate_to_pose_binds_direction_bt_and_uuid(self):
        modes = ["success", "cancel"]
        received_goals = []

        class FakeDirectionGuard:
            def __init__(self):
                self.calls = []
                self.activations = 0
                self.events = []

            @staticmethod
            def wait_ready(_timeout_sec):
                return True

            def wait_stopped(self):
                self.events.append("settle")
                return OperationResult(True, "stopped")

            def stop(self, lease):
                self.calls.append(("stop", lease))
                self.events.append("revoke")
                return OperationResult(True, "stopped")

            def prepare(self, lease):
                self.calls.append(("prepare", lease))
                return OperationResult(True, "prepared"), 7, 100 + lease.generation

            def activate(self, lease):
                self.calls.append(("activate", lease))
                self.events.append("activate")
                self.activations += 1
                return OperationResult(True, "active")

            def renew(self, lease):
                self.calls.append(("renew", lease))
                return OperationResult(True, "renewed")

        direction_guard = FakeDirectionGuard()

        def execute(goal_handle):
            mode = modes.pop(0)
            goal_index = len(received_goals) + 1
            received_goals.append((
                goal_handle.request.pose.header.frame_id,
                goal_handle.request.pose.pose.position.x,
                goal_handle.request.behavior_tree,
                bytes(goal_handle.goal_id.uuid),
            ))
            deadline = time.monotonic() + 2.0
            while (
                direction_guard.activations < goal_index
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            if mode == "success":
                goal_handle.succeed()
            else:
                deadline = time.monotonic() + 2.0
                while (
                    not goal_handle.is_cancel_requested
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.01)
                direction_guard.events.append("action_cancel")
                goal_handle.canceled()
            return NavigateToPose.Result()

        action_server = ActionServer(
            self.server_node,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=execute,
            cancel_callback=lambda _request: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.resources.append(action_server)
        navigator = RosNavigator(
            self.client_node,
            ReentrantCallbackGroup(),
            direction_guard=direction_guard,
            reverse_behavior_tree="/tmp/reverse.xml",
            navigation_timeout_sec=3.0,
            goal_response_timeout_sec=1.0,
            cancel_timeout_sec=2.0,
            direction_renew_period_sec=0.1,
            direction_prepare_timeout_sec=0.5,
            direction_prepare_retry_period_sec=0.01,
        )
        endpoint = Waypoint(
            frame_id="odom_combined",
            position=(1.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            task="qr",
        )

        self.assertTrue(navigator.wait_ready(1.0))
        success = navigator.navigate(endpoint)
        self.assertTrue(success.success, success.status)
        self.assertEqual(
            received_goals[0][:3],
            ("odom_combined", 1.0, ""),
        )
        prepare_calls = [
            lease for name, lease in direction_guard.calls
            if name == "prepare"
        ]
        self.assertEqual(prepare_calls[0].direction, 1)
        self.assertEqual(
            bytes(prepare_calls[0].action_uuid.uuid),
            received_goals[0][3],
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
            target=lambda: outcome.append(navigator.navigate(
                endpoint, reverse_direction=True)))
        worker.start()
        deadline = time.monotonic() + 1.0
        while (
            (
                not navigator.is_active()
                or direction_guard.activations < 2
            )
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        self.assertTrue(navigator.is_active())
        self.assertEqual(direction_guard.activations, 2)
        self.assertTrue(navigator.cancel())
        worker.join(timeout=2.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(outcome[0].status, "navigation_canceled")
        self.assertEqual(received_goals[1][2], "/tmp/reverse.xml")
        prepare_calls = [
            lease for name, lease in direction_guard.calls
            if name == "prepare"
        ]
        self.assertEqual(prepare_calls[1].direction, 2)
        self.assertEqual(
            bytes(prepare_calls[1].action_uuid.uuid),
            received_goals[1][3],
        )
        final_activate = len(direction_guard.events) - 1 - list(reversed(
            direction_guard.events)).index("activate")
        revoke_after_activate = direction_guard.events.index(
            "revoke", final_activate + 1)
        action_cancel = direction_guard.events.index(
            "action_cancel", revoke_after_activate + 1)
        settle_after_cancel = direction_guard.events.index(
            "settle", action_cancel + 1)
        self.assertLess(final_activate, revoke_after_activate)
        self.assertLess(revoke_after_activate, action_cancel)
        self.assertLess(action_cancel, settle_after_cancel)
        self.assertFalse(navigator.is_active())
        self.assertEqual(GoalStatus.STATUS_CANCELED, 5)

    def test_localization_reset_ignores_pre_response_and_far_odometry(self):
        order = []
        timers = []
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
            deadline = time.monotonic() + 1.0
            while (
                localization._odom_sequence == 0
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertGreater(localization._odom_sequence, 0)
            timers.extend((
                threading.Timer(0.10, lambda: publish_odom(1.0)),
                threading.Timer(0.20, lambda: publish_odom(0.0)),
            ))
            for timer in timers:
                timer.start()
            return response

        self.server_node.create_service(SetPose, "/set_pose", set_pose)

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
        for timer in timers:
            timer.join(timeout=1.0)

        self.assertTrue(result.success, result.status)
        self.assertEqual(order[0], "set_pose")
        self.assertIn("odom:1.0", order)


if __name__ == "__main__":
    unittest.main()
