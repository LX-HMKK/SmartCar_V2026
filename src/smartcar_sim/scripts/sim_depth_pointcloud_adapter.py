#!/usr/bin/env python3
"""Publish Gazebo obstacle returns as a depth-style PointCloud2 test feed.

This node exists only to exercise Nav2's PointCloud2 obstacle path in local
Gazebo.  The physical Aurora path remains
``/aurora/points2 -> depth_pointcloud_relay -> /smartcar/depth/points``.
"""

from __future__ import annotations

import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


XYZ_POINT_STEP = 12
_XYZ_PACKER = struct.Struct("<fff")


def scan_to_depth_point_cloud(
    scan: LaserScan,
    *,
    min_range_m: float,
    max_range_m: float,
    output_frame: str = "",
    sensor_origin_x_m: float = 0.0,
    sensor_origin_y_m: float = 0.0,
    point_height_m: float = 0.0,
) -> PointCloud2:
    """Convert valid planar returns to finite XYZ obstacle points.

    The Gazebo scan fixture lives above the ground, while the physical depth
    cloud is consumed through a low obstacle-height window.  The simulator
    therefore materializes the returns in ``base_footprint`` at the measured
    obstacle height instead of reusing the scan frame's elevated Z origin.
    """
    if (
        not math.isfinite(min_range_m)
        or not math.isfinite(max_range_m)
        or min_range_m <= 0.0
        or max_range_m <= min_range_m
    ):
        raise ValueError("point-cloud range limits must be finite and ordered")
    if not all(math.isfinite(value) for value in (
        sensor_origin_x_m, sensor_origin_y_m, point_height_m,
    )):
        raise ValueError("point-cloud frame offsets must be finite")

    lower = max(min_range_m, float(scan.range_min))
    upper = min(max_range_m, float(scan.range_max))
    payload = bytearray()
    for index, measured_range in enumerate(scan.ranges):
        if (
            not math.isfinite(measured_range)
            or measured_range <= lower
            or measured_range >= upper
        ):
            continue
        angle = scan.angle_min + index * scan.angle_increment
        payload.extend(_XYZ_PACKER.pack(
            float(sensor_origin_x_m + measured_range * math.cos(angle)),
            float(sensor_origin_y_m + measured_range * math.sin(angle)),
            float(point_height_m),
        ))

    cloud = PointCloud2()
    cloud.header = scan.header
    if output_frame:
        cloud.header.frame_id = output_frame
    cloud.height = 1
    cloud.width = len(payload) // XYZ_POINT_STEP
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = XYZ_POINT_STEP
    cloud.row_step = cloud.width * XYZ_POINT_STEP
    cloud.data = bytes(payload)
    cloud.is_dense = True
    return cloud


class SimDepthPointCloudAdapter(Node):
    """Translate Gazebo's sensor fixture into the Nav2 depth test topic."""

    def __init__(self) -> None:
        super().__init__("sim_depth_pointcloud_adapter")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/smartcar/depth/points")
        self.declare_parameter("min_range_m", 0.25)
        self.declare_parameter("max_range_m", 3.5)
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("sensor_origin_x_m", 0.0341)
        self.declare_parameter("sensor_origin_y_m", 0.0)
        self.declare_parameter("point_height_m", 0.15)

        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        self._min_range_m = float(self.get_parameter("min_range_m").value)
        self._max_range_m = float(self.get_parameter("max_range_m").value)
        self._output_frame = str(
            self.get_parameter("output_frame").value).strip()
        self._sensor_origin_x_m = float(
            self.get_parameter("sensor_origin_x_m").value)
        self._sensor_origin_y_m = float(
            self.get_parameter("sensor_origin_y_m").value)
        self._point_height_m = float(
            self.get_parameter("point_height_m").value)
        if not input_topic or not output_topic:
            raise ValueError("input_topic and output_topic must be non-empty")
        if not self._output_frame:
            raise ValueError("output_frame must be non-empty")
        # Validate once at construction instead of silently publishing a
        # permanently empty obstacle stream after an invalid launch value.
        scan_to_depth_point_cloud(
            LaserScan(),
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
            output_frame=self._output_frame,
            sensor_origin_x_m=self._sensor_origin_x_m,
            sensor_origin_y_m=self._sensor_origin_y_m,
            point_height_m=self._point_height_m,
        )
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            LaserScan, input_topic, self._scan_callback, qos_profile_sensor_data)

    def _scan_callback(self, scan: LaserScan) -> None:
        self._publisher.publish(scan_to_depth_point_cloud(
            scan,
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
            output_frame=self._output_frame,
            sensor_origin_x_m=self._sensor_origin_x_m,
            sensor_origin_y_m=self._sensor_origin_y_m,
            point_height_m=self._point_height_m,
        ))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimDepthPointCloudAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
