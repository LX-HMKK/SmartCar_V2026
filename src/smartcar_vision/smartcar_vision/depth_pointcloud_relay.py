"""Retimestamp Aurora point clouds before Nav2 consumes them."""
from __future__ import annotations

import math
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import String


STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_FLOAT_FIELD_FORMATS = {
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}
_XYZ_SAMPLE_LIMIT = 128


def _xyz_field_layout(message: PointCloud2) -> tuple[tuple[int, str], ...]:
    """Return validated scalar XYZ field offsets and struct formats."""
    fields = {field.name: field for field in message.fields}
    layout = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None:
            raise ValueError(f"point cloud is missing {name!r} field")
        if field.count < 1:
            raise ValueError(f"point cloud {name!r} field has zero count")
        format_info = _FLOAT_FIELD_FORMATS.get(field.datatype)
        if format_info is None:
            raise ValueError(
                f"point cloud {name!r} field must be FLOAT32 or FLOAT64")
        format_code, width = format_info
        if field.offset < 0 or field.offset + width > message.point_step:
            raise ValueError(
                f"point cloud {name!r} field offset is outside point_step")
        layout.append((field.offset, format_code))
    return tuple(layout)


def _has_finite_xyz_sample(message: PointCloud2) -> bool:
    """Bound validation cost while rejecting all-invalid depth frames."""
    point_count = message.width * message.height
    sample_count = min(point_count, _XYZ_SAMPLE_LIMIT)
    if sample_count <= 0:
        return False
    layout = _xyz_field_layout(message)
    byte_order = ">" if message.is_bigendian else "<"
    samples = range(sample_count)
    if sample_count < point_count:
        samples = (
            index * (point_count - 1) // (sample_count - 1)
            for index in range(sample_count)
        )
    payload = message.data
    for point_index in samples:
        row, column = divmod(point_index, message.width)
        base = row * message.row_step + column * message.point_step
        values = tuple(
            struct.unpack_from(byte_order + format_code, payload, base + offset)[0]
            for offset, format_code in layout
        )
        if all(math.isfinite(value) for value in values):
            return True
    return False


def retime_point_cloud(
    message: PointCloud2,
    stamp,
    expected_frame: str,
    output_frame: str = "",
) -> PointCloud2:
    """Copy a valid cloud with the local receipt timestamp and same frame.

    Aurora 930 firmware 1.7.2 publishes point-cloud seconds in the wrong
    unit. Nav2 rejects that cloud against TF's time cache, so receipt time is
    used after validating the transport and source frame.
    """
    frame_id = message.header.frame_id.strip()
    if not frame_id:
        raise ValueError("point cloud frame_id is empty")
    if expected_frame and frame_id != expected_frame:
        raise ValueError(
            f"point cloud frame {frame_id!r} != expected {expected_frame!r}")
    output_frame = output_frame.strip()
    if output_frame and output_frame != frame_id:
        raise ValueError(
            "point cloud output_frame must match input frame; "
            "the relay does not transform coordinates")
    if message.width <= 0 or message.height <= 0 or message.point_step <= 0:
        raise ValueError("point cloud has no points")
    if message.row_step < message.width * message.point_step:
        raise ValueError("point cloud row_step is shorter than its row")
    if len(message.data) < message.row_step * message.height:
        raise ValueError("point cloud data is truncated")
    if not _has_finite_xyz_sample(message):
        raise ValueError("point cloud has no finite x/y/z sample")

    relayed = PointCloud2()
    relayed.header.stamp = stamp
    relayed.header.frame_id = frame_id
    relayed.height = message.height
    relayed.width = message.width
    relayed.fields = message.fields
    relayed.is_bigendian = message.is_bigendian
    relayed.point_step = message.point_step
    relayed.row_step = message.row_step
    relayed.data = message.data
    relayed.is_dense = message.is_dense
    return relayed


class DepthPointCloudRelay(Node):
    """Republish only structurally valid, freshly received depth clouds."""

    def __init__(self) -> None:
        super().__init__("depth_pointcloud_relay")
        self.declare_parameter("input_topic", "/aurora/points2")
        self.declare_parameter("output_topic", "/smartcar/depth/points")
        self.declare_parameter("expected_frame", "depth_camera_link_1")
        self.declare_parameter("output_frame", "")
        self.declare_parameter("stale_timeout_sec", 0.50)

        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        self._expected_frame = str(
            self.get_parameter("expected_frame").value).strip()
        self._output_frame = str(
            self.get_parameter("output_frame").value).strip()
        stale_timeout = float(
            self.get_parameter("stale_timeout_sec").value)
        if not input_topic or not output_topic:
            raise ValueError("input_topic and output_topic must be non-empty")
        if (
            self._expected_frame
            and self._output_frame
            and self._expected_frame != self._output_frame
        ):
            raise ValueError(
                "expected_frame and output_frame must match; "
                "the relay does not transform coordinates")
        if not math.isfinite(stale_timeout) or stale_timeout <= 0.0:
            raise ValueError("stale_timeout_sec must be positive and finite")

        self._stale_timeout = stale_timeout
        self._last_received_at: float | None = None
        self._status = ""
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self._status_publisher = self.create_publisher(
            String, "/smartcar/depth_obstacles/status", STATUS_QOS)
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._on_point_cloud,
            qos_profile_sensor_data)
        self.create_timer(min(0.10, stale_timeout / 2.0), self._check_freshness)
        self._publish_status("waiting_for_depth_points")

    def _publish_status(self, value: str) -> None:
        if value == self._status:
            return
        self._status = value
        self._status_publisher.publish(String(data=value))

    def _on_point_cloud(self, message: PointCloud2) -> None:
        try:
            relayed = retime_point_cloud(
                message,
                self.get_clock().now().to_msg(),
                self._expected_frame,
                self._output_frame,
            )
        except ValueError as error:
            self._last_received_at = None
            self._publish_status(f"invalid_depth_points:{error}")
            self.get_logger().warning(str(error))
            return

        self._publisher.publish(relayed)
        self._last_received_at = time.monotonic()
        self._publish_status("depth_points_active")

    def _check_freshness(self) -> None:
        if self._last_received_at is None:
            return
        if time.monotonic() - self._last_received_at > self._stale_timeout:
            self._publish_status("depth_points_stale")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthPointCloudRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
