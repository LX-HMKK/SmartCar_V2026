#!/usr/bin/env python3
"""Publish a fake LaserScan with points at B/C-zone wall positions.

Bypasses the broken static_layer → costmap pipeline (cause of "Starting point
in lethal space!" with TROS Nav2 1.1.20).  The obstacle_layer already works;
this node feeds it synthetic scan data so the costmap sees the walls.

Wall geometry matches track.world and field_geometry.yaml.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# Field geometry (metres, origin at P-point = odom_combined origin)
# Walls: (name, x_min, x_max, y_min, y_max)
WALLS = [
    ("b_zone_left",   0.0, 2.0,   1.75, 2.25),     # B-zone left
    ("b_zone_right",  3.0, 5.0,   1.75, 2.25),     # B-zone right
    ("c_zone_inner",  1.0, 4.0,   3.00, 3.65),     # C inner fill
    ("c_zone_north",  0.0, 5.0,   4.15, 4.75),     # C north
    ("c_zone_west",   0.0, 0.5,   2.50, 4.15),     # C west
    ("c_zone_east",   4.5, 5.0,   2.50, 4.15),     # C east
]


def sample_wall_edges(x0: float, x1: float, y0: float, y1: float,
                      step: float = 0.05) -> list[tuple[float, float]]:
    """Generate sample points along the boundary of a rectangular wall."""
    points: list[tuple[float, float]] = []
    x_range = int((x1 - x0) / step) + 1
    y_range = int((y1 - y0) / step) + 1
    # bottom edge
    for i in range(x_range):
        points.append((x0 + i * step, y0))
    # top edge
    for i in range(x_range):
        points.append((x0 + i * step, y1))
    # left edge
    for i in range(y_range):
        points.append((x0, y0 + i * step))
    # right edge
    for i in range(y_range):
        points.append((x1, y0 + i * step))
    return points


class WallPublisher(Node):
    def __init__(self) -> None:
        super().__init__("wall_publisher")
        self._pub = self.create_publisher(LaserScan, "/scan_walls", 10)
        self._timer = self.create_timer(0.2, self._publish)  # 5 Hz
        self._points = self._build_points()
        self.get_logger().info(
            f"Wall publisher ready: {len(self._points)} synthetic scan points"
        )

    def _build_points(self) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for _name, x0, x1, y0, y1 in WALLS:
            points.extend(sample_wall_edges(x0, x1, y0, y1))
        return points

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        n = len(self._points)
        msg = LaserScan()
        msg.header.frame_id = "laser_link"
        msg.header.stamp = now
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / max(n, 1)
        msg.range_min = 0.02
        msg.range_max = 8.0
        msg.ranges = [
            math.hypot(px, py) for px, py in self._points
        ]
        msg.intensities = [0.0] * n
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
