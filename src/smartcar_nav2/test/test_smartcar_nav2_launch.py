import math
import os
import sys
import time
import unittest


TEST_DOMAIN_ENV = "SMARTCAR_NAV2_TEST_DOMAIN_ID"
if TEST_DOMAIN_ENV not in os.environ:
    os.environ[TEST_DOMAIN_ENV] = str(100 + os.getpid() % 100)
os.environ["ROS_DOMAIN_ID"] = os.environ[TEST_DOMAIN_ENV]


import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


FIXTURE_ARGUMENT = "--nav2-test-fixture"
MANAGED_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
)


class Nav2TestFixture:
    def __init__(self):
        self.node = rclpy.create_node("smartcar_nav2_test_fixture")
        self.odom_publisher = self.node.create_publisher(
            Odometry, "/odom_combined", 10
        )
        self.raw_odom_publisher = self.node.create_publisher(
            Odometry, "/odom", 10
        )
        self.scan_publisher = self.node.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.static_broadcaster = StaticTransformBroadcaster(self.node)
        self._publish_static_transform()
        self.node.create_timer(0.05, self._publish_odometry)
        self.node.create_timer(0.10, self._publish_scan)

    def _publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "odom_combined"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

    def _publish_odometry(self):
        odometry = Odometry()
        odometry.header.stamp = self.node.get_clock().now().to_msg()
        odometry.header.frame_id = "odom_combined"
        odometry.child_frame_id = "base_footprint"
        odometry.pose.pose.orientation.w = 1.0
        self.odom_publisher.publish(odometry)

        raw_odometry = Odometry()
        raw_odometry.header.stamp = odometry.header.stamp
        raw_odometry.header.frame_id = "odom"
        raw_odometry.child_frame_id = "base_footprint"
        raw_odometry.pose.pose.orientation.w = 1.0
        self.raw_odom_publisher.publish(raw_odometry)

    def _publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = "base_footprint"
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / 180.0
        scan.scan_time = 0.10
        scan.range_min = 0.05
        scan.range_max = 12.0
        scan.ranges = [math.inf] * 181
        self.scan_publisher.publish(scan)


def run_fixture():
    rclpy.init()
    fixture = Nav2TestFixture()
    try:
        rclpy.spin(fixture.node)
    except KeyboardInterrupt:
        pass
    finally:
        fixture.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def generate_test_description():
    package_dir = get_package_share_directory("smartcar_nav2")
    safety_package_dir = get_package_share_directory("smartcar_safety")
    launch_file = os.path.join(
        package_dir, "launch", "smartcar_nav2.launch.py"
    )

    fixture_process = ExecuteProcess(
        cmd=[sys.executable, os.path.abspath(__file__), FIXTURE_ARGUMENT],
        output="screen",
    )
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                safety_package_dir,
                "launch",
                "smartcar_safety.launch.py",
            )
        ),
        launch_arguments={
            "require_scan": "true",
            "require_odom": "true",
        }.items(),
    )
    delayed_nav2 = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={"use_sim_time": "false"}.items(),
            )
        ],
    )

    return launch.LaunchDescription(
        [
            fixture_process,
            safety_launch,
            delayed_nav2,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSmartcarNav2Lifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("smartcar_nav2_lifecycle_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_managed_nodes_active(self):
        clients = {
            name: self.node.create_client(GetState, f"/{name}/get_state")
            for name in MANAGED_NODES
        }
        pending = set(MANAGED_NODES)
        last_states = {name: "service unavailable" for name in MANAGED_NODES}
        deadline = time.monotonic() + 60.0

        while pending and time.monotonic() < deadline:
            for name in tuple(pending):
                client = clients[name]
                if not client.wait_for_service(timeout_sec=0.05):
                    continue

                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    self.node, future, timeout_sec=0.5
                )
                if not future.done() or future.result() is None:
                    future.cancel()
                    continue

                state = future.result().current_state
                last_states[name] = f"{state.label} ({state.id})"
                if state.id == State.PRIMARY_STATE_ACTIVE:
                    pending.remove(name)

            if pending:
                time.sleep(0.05)

        self.assertFalse(
            pending,
            "Nav2 lifecycle nodes did not become active: "
            + ", ".join(
                f"{name}={last_states[name]}" for name in sorted(pending)
            ),
        )

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    @staticmethod
    def _is_zero_twist(components):
        return all(value == 0.0 for value in components)

    def test_all_managed_nodes_become_active(self):
        self._wait_for_managed_nodes_active()
        print("Nav2 lifecycle ACTIVE: " + ", ".join(MANAGED_NODES))

    def test_cmd_vel_has_only_velocity_smoother_publisher(self):
        self._wait_for_managed_nodes_active()
        publishers = []

        def graph_is_ready():
            nonlocal publishers
            publishers = self.node.get_publishers_info_by_topic("/cmd_vel")
            return bool(publishers)

        self.assertTrue(graph_is_ready() or self._spin_until(graph_is_ready, 5.0))
        publisher_names = sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in publishers
        )
        self.assertEqual(publisher_names, ["/velocity_smoother"])

    def test_upstream_dropout_reaches_and_holds_exact_safe_zero(self):
        self._wait_for_managed_nodes_active()
        safe_samples = []

        def on_safe_command(message):
            safe_samples.append((
                time.monotonic(),
                (
                    message.linear.x,
                    message.linear.y,
                    message.linear.z,
                    message.angular.x,
                    message.angular.y,
                    message.angular.z,
                ),
            ))

        safe_subscription = self.node.create_subscription(
            Twist, "/cmd_vel_safe", on_safe_command, 10
        )
        command_publisher = self.node.create_publisher(
            Twist, "/cmd_vel_nav", 10
        )
        try:
            self.assertTrue(
                self._spin_until(
                    lambda: command_publisher.get_subscription_count() > 0,
                    5.0,
                )
            )
            self._spin_until(lambda: bool(safe_samples), 1.0)
            safe_samples.clear()

            command = Twist()
            command.linear.x = 0.30
            publish_until = time.monotonic() + 0.40
            last_publish_at = None
            while time.monotonic() < publish_until:
                command_publisher.publish(command)
                last_publish_at = time.monotonic()
                rclpy.spin_once(self.node, timeout_sec=0.02)
                time.sleep(0.03)

            self.assertIsNotNone(last_publish_at)
            self.assertTrue(
                self._spin_until(
                    lambda: any(
                        not self._is_zero_twist(components)
                        for _, components in safe_samples
                    ),
                    1.0,
                ),
                "Safety output never forwarded a nonzero test command",
            )

            stop_observed_at = time.monotonic()
            zero_sample = None
            deadline = last_publish_at + 0.40
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                for sample in safe_samples:
                    if (
                        sample[0] >= stop_observed_at
                        and self._is_zero_twist(sample[1])
                    ):
                        zero_sample = sample
                        break
                if zero_sample is not None:
                    break

            self.assertIsNotNone(
                zero_sample,
                "cmd_vel_safe did not reach exact zero within 0.40 seconds",
            )
            stop_delay = zero_sample[0] - last_publish_at
            self.assertLessEqual(stop_delay, 0.40)

            hold_until = zero_sample[0] + 0.20
            while time.monotonic() < hold_until:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            held_samples = [
                components
                for sample_time, components in safe_samples
                if sample_time >= zero_sample[0]
            ]
            self.assertGreaterEqual(len(held_samples), 3)
            self.assertTrue(all(map(self._is_zero_twist, held_samples)))
            print(f"cmd_vel_safe exact-zero delay: {stop_delay:.3f}s")
        finally:
            self.node.destroy_publisher(command_publisher)
            self.node.destroy_subscription(safe_subscription)


if __name__ == "__main__" and FIXTURE_ARGUMENT in sys.argv:
    run_fixture()
