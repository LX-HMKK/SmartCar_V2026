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
    "planner_server",
    "behavior_server",
    "bt_navigator",
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
        package_dir, "launch", "navigation_launch.py"
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
                launch_arguments={
                    "use_sim_time": "false",
                }.items(),
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

    def _publisher_names(self, topic):
        publishers = []

        def graph_is_ready():
            nonlocal publishers
            publishers = self.node.get_publishers_info_by_topic(topic)
            return bool(publishers)

        self.assertTrue(
            graph_is_ready() or self._spin_until(graph_is_ready, 5.0),
            f"No publisher appeared on {topic}",
        )
        return sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in publishers
        )

    @staticmethod
    def _twist_components(message):
        return (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )

    def test_all_managed_nodes_become_active(self):
        self._wait_for_managed_nodes_active()
        print("Nav2 lifecycle ACTIVE: " + ", ".join(MANAGED_NODES))

    def test_waypoint_follower_is_absent_in_lean_mode(self):
        names = {
            name
            for name, _namespace in self.node.get_node_names_and_namespaces()
        }
        self.assertNotIn("waypoint_follower", names)

    def test_velocity_topics_have_exactly_one_expected_owner(self):
        self._wait_for_managed_nodes_active()
        expected_owners = {
            "/cmd_vel_candidate": ["/velocity_smoother"],
            "/cmd_vel": ["/direction_guard"],
            "/cmd_vel_safe": ["/safety_node"],
        }
        for topic, expected in expected_owners.items():
            with self.subTest(topic=topic):
                self.assertEqual(self._publisher_names(topic), expected)

    def test_unpermitted_nonzero_candidate_is_held_at_exact_zero(self):
        self._wait_for_managed_nodes_active()
        candidate_samples = []
        guarded_samples = []
        safe_samples = []

        def record(target):
            def callback(message):
                target.append((
                    time.monotonic(),
                    self._twist_components(message),
                ))

            return callback

        candidate_subscription = self.node.create_subscription(
            Twist, "/cmd_vel_candidate", record(candidate_samples), 10
        )
        guarded_subscription = self.node.create_subscription(
            Twist, "/cmd_vel", record(guarded_samples), 10
        )

        def on_safe_command(message):
            safe_samples.append((
                time.monotonic(),
                self._twist_components(message),
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
            self._spin_until(
                lambda: bool(guarded_samples) and bool(safe_samples), 1.0
            )
            candidate_samples.clear()
            guarded_samples.clear()
            safe_samples.clear()

            command = Twist()
            command.linear.x = 0.15
            test_started_at = time.monotonic()
            publish_until = test_started_at + 0.50
            while time.monotonic() < publish_until:
                command_publisher.publish(command)
                rclpy.spin_once(self.node, timeout_sec=0.02)
                time.sleep(0.03)

            self.assertTrue(
                self._spin_until(
                    lambda: any(
                        not self._is_zero_twist(components)
                        for sample_time, components in candidate_samples
                        if sample_time >= test_started_at
                    ),
                    1.0,
                ),
                "The nonzero test command never reached /cmd_vel_candidate",
            )
            self.assertTrue(
                self._spin_until(
                    lambda: len([
                        sample
                        for sample in safe_samples
                        if sample[0] >= test_started_at
                    ]) >= 3,
                    1.0,
                ),
                "Safety output did not provide enough samples",
            )
            guarded_during_test = [
                components
                for sample_time, components in guarded_samples
                if sample_time >= test_started_at
            ]
            safe_during_test = [
                components
                for sample_time, components in safe_samples
                if sample_time >= test_started_at
            ]
            self.assertGreaterEqual(len(guarded_during_test), 3)
            self.assertGreaterEqual(len(safe_during_test), 3)
            self.assertTrue(
                all(map(self._is_zero_twist, guarded_during_test)),
                "Direction guard forwarded a command without a permit",
            )
            self.assertTrue(
                all(map(self._is_zero_twist, safe_during_test)),
                "Safety output became nonzero without a direction permit",
            )
        finally:
            self.node.destroy_publisher(command_publisher)
            self.node.destroy_subscription(candidate_subscription)
            self.node.destroy_subscription(guarded_subscription)
            self.node.destroy_subscription(safe_subscription)


if __name__ == "__main__" and FIXTURE_ARGUMENT in sys.argv:
    run_fixture()
