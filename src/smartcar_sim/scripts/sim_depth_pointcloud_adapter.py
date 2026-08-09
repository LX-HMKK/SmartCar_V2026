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
from nav_msgs.msg import Odometry
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


def world_obstacles_to_depth_point_cloud(
    odom: Odometry,
    obstacles_xy_m: list[tuple[float, float]],
    *,
    min_range_m: float,
    max_range_m: float,
    output_frame: str = "base_footprint",
    min_forward_x_m: float = 0.0,
    horizontal_half_fov_rad: float = 1.20,
    point_height_m: float = 0.15,
) -> PointCloud2:
    """Project static world obstacles into a synchronized depth-camera cloud.

    Gazebo's planar lidar is intentionally not reused here: its line of sight
    can hide a cone behind another cone, while a depth-camera fixture should
    exercise the independent PointCloud2 observation path.  The cloud remains
    conservative by emitting one finite point per known obstacle and applying
    the camera's forward/FOV/range gate in the vehicle frame.
    """
    if not math.isfinite(min_range_m) or not math.isfinite(max_range_m) or \
            min_range_m <= 0.0 or max_range_m <= min_range_m:
        raise ValueError("point-cloud range limits must be finite and ordered")
    if not output_frame:
        raise ValueError("output_frame must be non-empty")
    if not math.isfinite(min_forward_x_m) or not math.isfinite(horizontal_half_fov_rad) or \
            horizontal_half_fov_rad <= 0.0 or not math.isfinite(point_height_m):
        raise ValueError("depth camera projection limits must be finite and valid")

    q = odom.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    payload = bytearray()
    visible_bearings: list[float] = []
    for world_x, world_y in obstacles_xy_m:
        if not math.isfinite(world_x) or not math.isfinite(world_y):
            continue
        delta_x = world_x - odom.pose.pose.position.x
        delta_y = world_y - odom.pose.pose.position.y
        base_x = cos_yaw * delta_x + sin_yaw * delta_y
        base_y = -sin_yaw * delta_x + cos_yaw * delta_y
        distance = math.hypot(base_x, base_y)
        bearing = math.atan2(base_y, base_x)
        if (base_x < min_forward_x_m or distance <= min_range_m or
                distance >= max_range_m or abs(bearing) > horizontal_half_fov_rad):
            continue
        payload.extend(_XYZ_PACKER.pack(float(base_x), float(base_y), float(point_height_m)))
        visible_bearings.append(bearing)

    # PointCloud2 clearing is ray-based.  Add endpoints beyond the marking
    # range so a cone that leaves the camera FOV is cleared from the rolling
    # costmap, while avoiding rays that would pass through a currently visible
    # obstacle and erase its own mark.
    clear_range = max_range_m * 0.98
    for index in range(9):
        bearing = -horizontal_half_fov_rad + (
            2.0 * horizontal_half_fov_rad * index / 8.0)
        if any(abs(math.remainder(bearing - other, 2.0 * math.pi)) < 0.08
               for other in visible_bearings):
            continue
        payload.extend(_XYZ_PACKER.pack(
            float(clear_range * math.cos(bearing)),
            float(clear_range * math.sin(bearing)),
            float(point_height_m),
        ))

    cloud = PointCloud2()
    cloud.header = odom.header
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
        self.declare_parameter("source_mode", "scan")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter(
            "world_obstacles", "1.1,0.3;1.5,0.3;2.2,0.5;1.0,0.8;1.8,0.9;2.45,0.15")
        self.declare_parameter("output_topic", "/smartcar/depth/points")
        self.declare_parameter("min_range_m", 0.25)
        self.declare_parameter("max_range_m", 3.5)
        self.declare_parameter("horizontal_half_fov_rad", 0.75)
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("sensor_origin_x_m", 0.0341)
        self.declare_parameter("sensor_origin_y_m", 0.0)
        self.declare_parameter("point_height_m", 0.15)

        input_topic = str(self.get_parameter("input_topic").value).strip()
        self._source_mode = str(self.get_parameter("source_mode").value).strip().lower()
        odom_topic = str(self.get_parameter("odom_topic").value).strip()
        self._world_obstacles = self._parse_world_obstacles(
            str(self.get_parameter("world_obstacles").value))
        output_topic = str(self.get_parameter("output_topic").value).strip()
        self._min_range_m = float(self.get_parameter("min_range_m").value)
        self._max_range_m = float(self.get_parameter("max_range_m").value)
        self._horizontal_half_fov_rad = float(
            self.get_parameter("horizontal_half_fov_rad").value)
        self._output_frame = str(
            self.get_parameter("output_frame").value).strip()
        self._sensor_origin_x_m = float(
            self.get_parameter("sensor_origin_x_m").value)
        self._sensor_origin_y_m = float(
            self.get_parameter("sensor_origin_y_m").value)
        self._point_height_m = float(
            self.get_parameter("point_height_m").value)
        if self._source_mode not in {"scan", "world_obstacles"}:
            raise ValueError("source_mode must be scan or world_obstacles")
        if not input_topic or not output_topic or not odom_topic:
            raise ValueError("input_topic, odom_topic, and output_topic must be non-empty")
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
        self._subscription = None
        self._odom_subscription = None
        if self._source_mode == "world_obstacles":
            self._odom_subscription = self.create_subscription(
                Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data)
        else:
            self._subscription = self.create_subscription(
                LaserScan, input_topic, self._scan_callback, qos_profile_sensor_data)

    @staticmethod
    def _parse_world_obstacles(raw: str) -> list[tuple[float, float]]:
        obstacles = []
        for item in raw.split(";"):
            values = [part.strip() for part in item.split(",")]
            if len(values) != 2 or not all(values):
                raise ValueError("world_obstacles must use x,y;x,y format")
            x, y = (float(value) for value in values)
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("world_obstacles coordinates must be finite")
            obstacles.append((x, y))
        if not obstacles:
            raise ValueError("world_obstacles must contain at least one point")
        return obstacles

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

    def _odom_callback(self, odom: Odometry) -> None:
        self._publisher.publish(world_obstacles_to_depth_point_cloud(
            odom,
            self._world_obstacles,
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
            output_frame=self._output_frame,
            horizontal_half_fov_rad=self._horizontal_half_fov_rad,
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
