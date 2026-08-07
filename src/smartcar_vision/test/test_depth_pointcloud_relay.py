"""Contracts for Aurora PointCloud2 retimestamping."""
from pathlib import Path
import struct
import sys
import time
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_vision.depth_pointcloud_relay import (  # noqa: E402
    STATUS_QOS,
    DepthPointCloudRelay,
    correct_aurora_timestamp,
    retime_point_cloud,
    validate_capture_timestamp,
)
import smartcar_vision.depth_pointcloud_relay as relay_module  # noqa: E402


def point_cloud(*, frame_id="depth_camera_link_1", data=None, fields=None):
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.height = 1
    message.width = 1
    message.fields = fields or [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 12
    message.row_step = 12
    message.data = data if data is not None else struct.pack("<fff", 1.0, 2.0, 3.0)
    message.is_dense = True
    return message


class DepthPointCloudRelayTests(unittest.TestCase):
    def test_retimes_valid_cloud_without_rewriting_its_geometry(self):
        input_cloud = point_cloud()
        input_cloud.header.stamp.sec = 1786008
        input_cloud.header.stamp.nanosec = 655408000
        receipt_stamp = Time(sec=1786008651, nanosec=716000000)

        output = retime_point_cloud(
            input_cloud, receipt_stamp, "depth_camera_link_1")

        self.assertEqual(output.header.frame_id, "depth_camera_link_1")
        self.assertEqual(output.header.stamp.sec, receipt_stamp.sec)
        self.assertEqual(output.header.stamp.nanosec, receipt_stamp.nanosec)
        self.assertEqual(output.point_step, 12)
        self.assertEqual(output.row_step, 12)
        self.assertEqual(bytes(output.data), bytes(input_cloud.data))

    def test_corrects_the_aurora_930_timestamp_scale(self):
        source = Time(sec=1786109, nanosec=159675000)

        corrected = correct_aurora_timestamp(source, 1000)

        self.assertEqual(corrected.sec, 1786109159)
        self.assertEqual(corrected.nanosec, 675000000)
        with self.assertRaisesRegex(ValueError, "positive integer"):
            correct_aurora_timestamp(source, 0)

    def test_rejects_capture_times_outside_the_transport_window(self):
        capture = Time(sec=100, nanosec=0)
        validate_capture_timestamp(
            capture, Time(sec=100, nanosec=80000000),
            max_capture_age_sec=0.10, max_future_skew_sec=0.05)
        with self.assertRaisesRegex(ValueError, "too old"):
            validate_capture_timestamp(
                capture, Time(sec=100, nanosec=101000000),
                max_capture_age_sec=0.10, max_future_skew_sec=0.05)
        with self.assertRaisesRegex(ValueError, "future"):
            validate_capture_timestamp(
                Time(sec=100, nanosec=51000000), Time(sec=100),
                max_capture_age_sec=0.10, max_future_skew_sec=0.05)

    def test_rejects_reframing_without_a_coordinate_transform(self):
        with self.assertRaisesRegex(ValueError, "does not transform coordinates"):
            retime_point_cloud(
                point_cloud(), Time(sec=10), "depth_camera_link_1", "depth_link")

    def test_rejects_an_unexpected_frame_or_truncated_transport(self):
        with self.assertRaisesRegex(ValueError, "expected"):
            retime_point_cloud(
                point_cloud(frame_id="unknown_depth_frame"),
                Time(),
                "depth_camera_link_1",
            )
        with self.assertRaisesRegex(ValueError, "truncated"):
            retime_point_cloud(
                point_cloud(data=b"short"), Time(), "depth_camera_link_1")

    def test_rejects_clouds_without_finite_xyz_coordinates(self):
        with self.assertRaisesRegex(ValueError, "missing 'z' field"):
            retime_point_cloud(
                point_cloud(fields=[
                    PointField(
                        name="x", offset=0,
                        datatype=PointField.FLOAT32, count=1),
                    PointField(
                        name="y", offset=4,
                        datatype=PointField.FLOAT32, count=1),
                ]),
                Time(), "depth_camera_link_1")

    def test_main_handles_ros_external_shutdown_without_a_second_shutdown(self):
        node = Mock()
        with patch.object(relay_module.rclpy, "init"), \
                patch.object(relay_module, "DepthPointCloudRelay", return_value=node), \
                patch.object(
                    relay_module.rclpy,
                    "spin",
                    side_effect=relay_module.ExternalShutdownException(),
                ), \
                patch.object(relay_module.rclpy, "ok", return_value=False), \
                patch.object(relay_module.rclpy, "shutdown") as shutdown:
            relay_module.main()

        node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()
        with self.assertRaisesRegex(ValueError, "no finite x/y/z sample"):
            retime_point_cloud(
                point_cloud(data=struct.pack("<fff", float("nan"), 0.0, 1.0)),
                Time(), "depth_camera_link_1")


class DepthPointCloudRelayGraphTests(unittest.TestCase):
    """Exercise the actual ROS publisher/subscriber path without a camera."""

    @classmethod
    def setUpClass(cls):
        cls._owns_context = not rclpy.ok()
        if cls._owns_context:
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if cls._owns_context:
            rclpy.shutdown()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.relay = DepthPointCloudRelay()
        self.probe = Node("depth_pointcloud_relay_test_probe")
        self.outputs = []
        self.statuses = []
        self.input_publisher = self.probe.create_publisher(
            PointCloud2, "/aurora/points2", qos_profile_sensor_data)
        self.probe.create_subscription(
            PointCloud2,
            "/smartcar/depth/points",
            self.outputs.append,
            qos_profile_sensor_data,
        )
        self.probe.create_subscription(
            String,
            "/smartcar/depth_obstacles/status",
            lambda message: self.statuses.append(message.data),
            STATUS_QOS,
        )
        self.executor.add_node(self.relay)
        self.executor.add_node(self.probe)

    def tearDown(self):
        self.executor.remove_node(self.probe)
        self.executor.remove_node(self.relay)
        self.probe.destroy_node()
        self.relay.destroy_node()
        self.executor.shutdown()

    def _spin_until(self, predicate, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def test_relay_publishes_fresh_cloud_and_active_status(self):
        source = point_cloud()
        source_timestamp_ns = self.relay.get_clock().now().nanoseconds // 1000
        source.header.stamp.sec = source_timestamp_ns // 1_000_000_000
        source.header.stamp.nanosec = source_timestamp_ns % 1_000_000_000

        self.assertTrue(self._spin_until(
            lambda: self.input_publisher.get_subscription_count() > 0
        ))
        self.input_publisher.publish(source)

        self.assertTrue(self._spin_until(
            lambda: self.outputs and "depth_points_active" in self.statuses
        ))
        relayed = self.outputs[-1]
        self.assertEqual(relayed.header.frame_id, "depth_camera_link_1")
        self.assertEqual(
            relayed.header.stamp.sec * 1_000_000_000
            + relayed.header.stamp.nanosec,
            source_timestamp_ns * 1000,
        )
        self.assertEqual(relayed.point_step, source.point_step)
        self.assertEqual(bytes(relayed.data), bytes(source.data))


if __name__ == "__main__":
    unittest.main()
