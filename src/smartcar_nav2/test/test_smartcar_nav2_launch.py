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
from geometry_msgs.msg import TransformStamped
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
    launch_file = os.path.join(
        package_dir, "launch", "smartcar_nav2.launch.py"
    )

    fixture_process = ExecuteProcess(
        cmd=[sys.executable, os.path.abspath(__file__), FIXTURE_ARGUMENT],
        output="screen",
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

    def test_all_managed_nodes_become_active(self):
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
        print("Nav2 lifecycle ACTIVE: " + ", ".join(MANAGED_NODES))


if __name__ == "__main__" and FIXTURE_ARGUMENT in sys.argv:
    run_fixture()
