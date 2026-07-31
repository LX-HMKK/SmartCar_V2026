#!/usr/bin/env python3
"""Publish wall obstacle points as PointCloud2 for obstacle_layer marking.

Bypasses static_layer which is broken in TROS Nav2 1.1.20.
Publishes a static PointCloud2 at 1 Hz in the odom_combined frame,
containing sample points along all B/C-zone wall edges.
The obstacle_layer marks these as obstacles in the costmap.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


# Wall sample points in odom_combined frame (metres).
# Wall definitions match track.world static models.
# NOTE: B-zone walls are deliberately EXCLUDED — the reverse ThroughPoses
# path from QR to VLM needs to pass through the 1m corridor opening at
# x∈[2.0,3.0].  Adding B-zone obstacles blocks the corridor approach and
# causes immediate planning failure (StartOccupied for reverse goal).
# C-zone walls alone are sufficient to constrain the forward stage 3 loop.
WALLS_ODOM = [
    # B-zone left:  x ∈ [0.0, 2.0], y ∈ [1.75, 2.25]
    ("b_left",   0.0, 2.0,   1.75, 2.25),
    # B-zone right: x ∈ [3.0, 5.0], y ∈ [1.75, 2.25]
    ("b_right",  3.0, 5.0,   1.75, 2.25),
    # C-zone inner: x ∈ [1.0, 4.0], y ∈ [3.00, 3.65]
    ("c_inner",  1.0, 4.0,   3.00, 3.65),
    # C-zone north: x ∈ [0.0, 5.0], y ∈ [4.15, 4.75]
    ("c_north",  0.0, 5.0,   4.15, 4.75),
    # C-zone east:  x ∈ [4.5, 5.0], y ∈ [2.50, 4.15]
    ("c_east",   4.5, 5.0,   2.50, 4.15),
    # C-zone west:  x ∈ [0.0, 0.5], y ∈ [2.50, 4.15]
    ("c_west",   0.0, 0.5,   2.50, 4.15),
]

STEP = 0.05  # 5 cm spacing


def sample_rect(x0, x1, y0, y1):
    """Generate dense sample points inside a rectangle."""
    pts = []
    nx = int((x1 - x0) / STEP) + 1
    ny = int((y1 - y0) / STEP) + 1
    for i in range(nx):
        x = x0 + i * STEP
        for j in range(ny):
            pts.append((x, y0 + j * STEP))
    return pts


class WallPointCloud(Node):
    def __init__(self):
        super().__init__("wall_publisher")
        self._pub = self.create_publisher(PointCloud2, "/walls_cloud", 10)
        self._timer = self.create_timer(1.0, self._publish)  # 1 Hz
        self._points = self._build_points()
        self._msg = self._make_cloud(self._points)
        self.get_logger().info(
            f"Wall publisher ready: {len(self._points)} points"
        )

    def _build_points(self):
        pts = []
        for _name, x0, x1, y0, y1 in WALLS_ODOM:
            pts.extend(sample_rect(x0, x1, y0, y1))
        # Add Z=0
        return [(x, y, 0.0) for x, y in pts]

    def _make_cloud(self, points):
        header = self._make_header("odom_combined")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points)

    def _make_header(self, frame_id):
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = frame_id
        header.stamp = self.get_clock().now().to_msg()
        return header

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = WallPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
