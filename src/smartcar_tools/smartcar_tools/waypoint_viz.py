"""Publish Nav2 waypoint YAML as RViz MarkerArray for visual inspection.

Standalone tool — does not require Nav2 or a chassis driver.
Launch: ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:=<path>
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from smartcar_task.waypoints import (
    is_zero_quaternion,
    load_waypoints as load_mission_waypoints,
)
from visualization_msgs.msg import Marker, MarkerArray


MARKER_TOPIC = "/smartcar/waypoints/markers"
_TASK_COLORS = {
    "start":    (0.20, 0.85, 0.30),  # green
    "qr":       (0.25, 0.55, 1.00),  # blue
    "vlm":      (0.95, 0.55, 0.20),  # orange
    "corridor": (0.95, 0.80, 0.20),  # yellow
    "loop":     (1.00, 0.35, 0.20),  # red
    "return":   (0.70, 0.30, 0.90),  # purple
    "nav":      (0.35, 0.72, 0.90),  # light blue
    "via":      (0.67, 0.71, 0.75),  # neutral grey
}
_DEFAULT_COLOR = (0.80, 0.80, 0.80)  # grey


def _yaw_from_quaternion(orientation):
    """Compute yaw angle from a validated (x, y, z, w) tuple."""
    x, y, z, w = orientation
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def _yaw_quaternion(yaw_rad):
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def load_waypoints(path):
    """Return validated display tuples including stable waypoint IDs.

    Each tuple: (frame_id, x, y, yaw, task, waypoint_id, has_orientation).
    has_orientation is False for zero-quaternion (unconstrained) waypoints.
    """
    return [
        (
            item.frame_id,
            item.position[0],
            item.position[1],
            _yaw_from_quaternion(item.orientation),
            item.task,
            item.id,
            not is_zero_quaternion(item.orientation),
        )
        for item in load_mission_waypoints(path)
    ]


class WaypointVizNode(Node):
    """Read waypoint YAML and publish MarkerArray on a latched topic."""

    def __init__(self):
        super().__init__("waypoint_viz")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("marker_topic", MARKER_TOPIC)

        waypoints_file = str(
            self.get_parameter("waypoints_file").value
        ).strip()
        if not waypoints_file:
            raise ValueError("waypoints_file parameter is required")

        self._waypoints = load_waypoints(waypoints_file)
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} waypoints from {waypoints_file}"
        )

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), latched_qos
        )
        rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self._timer = self.create_timer(1.0 / rate, self._publish_markers)
        self._publish_markers()

    def _publish_markers(self):
        msg = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)

        if not self._waypoints:
            self._publisher.publish(msg)
            return

        stamp = self.get_clock().now().to_msg()
        frame_id = self._waypoints[0][0]  # first waypoint's frame

        # ── connecting line ──────────────────────────────────────────
        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = "waypoint_line"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.025
        line.color.a = 0.7
        line.color.r = 0.6
        line.color.g = 0.6
        line.color.b = 0.8
        for _, x, y, _, _, _, _ in self._waypoints:
            line.points.append(Point(x=x, y=y, z=0.03))
        msg.markers.append(line)

        # ── per-waypoint sphere / arrow / label ──────────────────────
        for i, (_fid, x, y, yaw, task, waypoint_id, has_orient) in enumerate(
            self._waypoints
        ):
            r, g, b = _TASK_COLORS.get(task, _DEFAULT_COLOR)

            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = "waypoint_spheres"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.06
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.10
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 1.0
            msg.markers.append(sphere)

            if has_orient:
                arrow = Marker()
                arrow.header.frame_id = frame_id
                arrow.header.stamp = stamp
                arrow.ns = "waypoint_arrows"
                arrow.id = i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = x
                arrow.pose.position.y = y
                arrow.pose.position.z = 0.09
                qx, qy, qz, qw = _yaw_quaternion(yaw)
                arrow.pose.orientation.x = qx
                arrow.pose.orientation.y = qy
                arrow.pose.orientation.z = qz
                arrow.pose.orientation.w = qw
                arrow.scale.x = 0.20
                arrow.scale.y = 0.035
                arrow.scale.z = 0.035
                arrow.color.r = r
                arrow.color.g = g
                arrow.color.b = b
                arrow.color.a = 0.95
                msg.markers.append(arrow)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = "waypoint_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"{i}: {waypoint_id} [{task}]"
            msg.markers.append(label)

        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
