"""Hardware-free runtime smoke test for the complete SmartCar stack."""
import math
import os
import sys
import time
import unittest


TEST_DOMAIN_ENV = "SMARTCAR_SYSTEM_TEST_DOMAIN_ID"
if TEST_DOMAIN_ENV not in os.environ:
    os.environ[TEST_DOMAIN_ENV] = str(120 + os.getpid() % 80)
os.environ["ROS_DOMAIN_ID"] = os.environ[TEST_DOMAIN_ENV]
os.environ["ROS_LOCALHOST_ONLY"] = "1"


import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from smartcar_interfaces.srv import DescribeScene, ReadQr
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


FIXTURE_ARGUMENT = "--system-smoke-fixture"
MANAGED_NODES = (
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
)
ZERO_COMPONENTS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class SystemSmokeFixture:
    """Publish finite synthetic sensor data without touching hardware."""

    def __init__(self):
        self.node = rclpy.create_node("smartcar_system_smoke_fixture")
        self.odom_publisher = self.node.create_publisher(
            Odometry, "/odom_combined", 10
        )
        self.raw_odom_publisher = self.node.create_publisher(
            Odometry, "/odom", 10
        )
        self.depth_scan_publisher = self.node.create_publisher(
            LaserScan, "/smartcar/depth/scan", qos_profile_sensor_data
        )
        self.static_broadcaster = StaticTransformBroadcaster(self.node)
        self._publish_static_transforms()
        self.node.create_timer(0.05, self._publish_odometry)
        self.node.create_timer(0.10, self._publish_depth_scan)

    def _publish_static_transforms(self):
        stamp = self.node.get_clock().now().to_msg()
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom_combined"
        odom_to_base.child_frame_id = "base_footprint"
        odom_to_base.transform.rotation.w = 1.0

        base_to_link = TransformStamped()
        base_to_link.header.stamp = stamp
        base_to_link.header.frame_id = "base_footprint"
        base_to_link.child_frame_id = "base_link"
        base_to_link.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform([odom_to_base, base_to_link])

    def _publish_odometry(self):
        stamp = self.node.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom_combined"
        odometry.child_frame_id = "base_footprint"
        odometry.pose.pose.orientation.w = 1.0
        self.odom_publisher.publish(odometry)

        raw_odometry = Odometry()
        raw_odometry.header.stamp = stamp
        raw_odometry.header.frame_id = "odom"
        raw_odometry.child_frame_id = "base_footprint"
        raw_odometry.pose.pose.orientation.w = 1.0
        self.raw_odom_publisher.publish(raw_odometry)

    def _publish_depth_scan(self):
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
        self.depth_scan_publisher.publish(scan)


def run_fixture():
    rclpy.init()
    fixture = SystemSmokeFixture()
    try:
        rclpy.spin(fixture.node)
    except KeyboardInterrupt:
        pass
    finally:
        fixture.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def generate_test_description():
    package_dir = get_package_share_directory("smartcar_bringup")
    system_launch = os.path.join(
        package_dir, "launch", "smartcar_system.launch.py"
    )
    fixture_process = ExecuteProcess(
        cmd=[sys.executable, os.path.abspath(__file__), FIXTURE_ARGUMENT],
        output="screen",
    )
    delayed_system = TimerAction(
        period=0.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(system_launch),
                launch_arguments={
                    "use_base": "false",
                    "use_safety": "true",
                    "safety_emergency_stop_on_start": "true",
                    "use_nav": "true",
                    "allow_synthetic_obstacle_source": "true",
                    "nav_autostart": "false",
                    "use_camera": "false",
                    "use_vision": "true",
                    "use_task": "true",
                    "use_visualization": "false",
                    "autostart_mission": "false",
                    "image_topic": "/smartcar/vision/image",
                }.items(),
            )
        ],
    )
    return launch.LaunchDescription([
        fixture_process,
        delayed_system,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSystemSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("smartcar_system_smoke_test")
        cls._assert_emergency_stop_before_nav_startup()
        cls._wait_for_managed_nodes_active()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def _wait_future(cls, future, timeout_sec):
        rclpy.spin_until_future_complete(
            cls.node, future, timeout_sec=timeout_sec
        )
        if not future.done():
            future.cancel()
            raise AssertionError("ROS service call timed out")
        result = future.result()
        if result is None:
            raise AssertionError("ROS service call returned no response")
        return result

    @classmethod
    def _assert_emergency_stop_before_nav_startup(cls):
        statuses = []
        status_subscription = cls.node.create_subscription(
            String,
            "/smartcar/safety/status",
            lambda message: statuses.append(message.data),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        deadline = time.monotonic() + 20.0
        try:
            while (
                "emergency_stop" not in statuses
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(cls.node, timeout_sec=0.10)
        finally:
            cls.node.destroy_subscription(status_subscription)
        if "emergency_stop" not in statuses:
            raise AssertionError("startup emergency stop was not observed")

        state_client = cls.node.create_client(
            GetState, "/controller_server/get_state"
        )
        if not state_client.wait_for_service(timeout_sec=30.0):
            raise AssertionError("controller lifecycle service unavailable")
        state = cls._wait_future(
            state_client.call_async(GetState.Request()), 15.0
        )
        if state.current_state.id == State.PRIMARY_STATE_ACTIVE:
            raise AssertionError("Nav2 activated before emergency stop")

        lifecycle_client = cls.node.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )
        if not lifecycle_client.wait_for_service(timeout_sec=20.0):
            raise AssertionError("Nav2 lifecycle manager unavailable")
        startup = ManageLifecycleNodes.Request()
        startup.command = ManageLifecycleNodes.Request.STARTUP
        result = cls._wait_future(lifecycle_client.call_async(startup), 60.0)
        if not result.success:
            raise AssertionError("Nav2 lifecycle startup failed")

    @classmethod
    def _wait_for_managed_nodes_active(cls):
        clients = {
            name: cls.node.create_client(GetState, f"/{name}/get_state")
            for name in MANAGED_NODES
        }
        pending = set(MANAGED_NODES)
        deadline = time.monotonic() + 20.0
        while pending and time.monotonic() < deadline:
            for name in tuple(pending):
                client = clients[name]
                if not client.wait_for_service(timeout_sec=0.05):
                    continue
                result = cls._wait_future(
                    client.call_async(GetState.Request()), 1.0
                )
                if result.current_state.id == State.PRIMARY_STATE_ACTIVE:
                    pending.remove(name)
            if pending:
                time.sleep(0.05)
        if pending:
            raise AssertionError(
                "Nav2 nodes are not active: " + ",".join(sorted(pending))
            )

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def test_hardware_nodes_and_chassis_output_are_absent(self):
        names = {
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in self.node.get_node_names_and_namespaces()
        }
        forbidden = (
            "origincar_base",
            "cmd_vel_to_ackermann",
            "ekf_filter",
            "imu_filter",
            "joint_state_publisher",
            "robot_state_publisher",
            "aurora",
            "usb_cam",
            "mipi_cam",
        )
        self.assertFalse([
            name for name in names if any(value in name for value in forbidden)
        ])
        self.assertEqual(
            self.node.get_subscriptions_info_by_topic("/ackermann_cmd"), []
        )
        publishers = self.node.get_publishers_info_by_topic(
            "/ackermann_cmd"
        )
        owners = sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace(
                "//", "/"
            )
            for info in publishers
        )
        self.assertEqual(owners, ["/safety_node"])

    def test_services_exist_and_task_remains_idle(self):
        clients = (
            self.node.create_client(
                SetBool, "/smartcar/safety/emergency_stop"
            ),
            self.node.create_client(ReadQr, "/smartcar/vision/read_qr"),
            self.node.create_client(
                DescribeScene, "/smartcar/vision/describe_scene"
            ),
            self.node.create_client(Trigger, "/smartcar/task/start"),
            self.node.create_client(Trigger, "/smartcar/task/stop"),
            self.node.create_client(Trigger, "/smartcar/task/reset"),
        )
        for client in clients:
            self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        states = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = self.node.create_subscription(
            String,
            "/smartcar/task/state",
            lambda message: states.append(message.data),
            qos,
        )
        try:
            self.assertTrue(self._spin_until(lambda: bool(states), 5.0))
            self.assertEqual(states[-1], "IDLE")
        finally:
            self.node.destroy_subscription(subscription)

    def test_required_sensor_topics_are_present(self):
        topics = dict(self.node.get_topic_names_and_types())
        self.assertIn("/smartcar/depth/scan", topics)
        self.assertIn("/odom", topics)
        self.assertIn("/odom_combined", topics)

    def test_safe_output_stays_exact_zero_under_emergency_stop(self):
        samples = []

        def on_safe_command(message):
            samples.append((
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            ))

        subscription = self.node.create_subscription(
            Twist, "/cmd_vel_safe", on_safe_command, 10
        )
        try:
            self.assertTrue(self._spin_until(lambda: len(samples) >= 40, 4.0))
            self.assertTrue(all(sample == ZERO_COMPONENTS for sample in samples))
            publishers = self.node.get_publishers_info_by_topic(
                "/cmd_vel_safe"
            )
            owners = sorted(
                f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace(
                    "//", "/"
                )
                for info in publishers
            )
            self.assertEqual(owners, ["/safety_node"])
        finally:
            self.node.destroy_subscription(subscription)

    def test_safety_status_reports_emergency_stop(self):
        statuses = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = self.node.create_subscription(
            String,
            "/smartcar/safety/status",
            lambda message: statuses.append(message.data),
            qos,
        )
        try:
            self.assertTrue(
                self._spin_until(
                    lambda: "emergency_stop" in statuses,
                    3.0,
                )
            )
        finally:
            self.node.destroy_subscription(subscription)


if __name__ == "__main__" and FIXTURE_ARGUMENT in sys.argv:
    run_fixture()
