"""Publish the competition field geometry as a display-only marker overlay."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from smartcar_tools.field_reference import (
    Bounds2D,
    Point2D,
    load_field_reference,
)


FIELD_MARKER_TOPIC = "/smartcar/field_reference/markers"


class FieldReferenceNode(Node):
    """Render measured rule geometry without participating in navigation."""

    def __init__(self):
        super().__init__("field_reference")
        default_geometry = str(
            Path(get_package_share_directory("smartcar_tools"))
            / "config"
            / "routes"
            / "field_geometry.yaml"
        )
        self.declare_parameter("geometry_file", default_geometry)
        self.declare_parameter("marker_topic", FIELD_MARKER_TOPIC)

        geometry_file = Path(
            str(self.get_parameter("geometry_file").value)
        ).expanduser()
        self._reference = load_field_reference(geometry_file)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            qos,
        )
        self._publish_reference()
        self.get_logger().info(
            f"published field reference from {geometry_file} in "
            f"{self._reference.frame_id}"
        )

    def _marker(self, namespace, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self._reference.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _cube(self, namespace, marker_id, bounds, z_m, color, stamp):
        marker = self._marker(namespace, marker_id, Marker.CUBE, stamp)
        marker.pose.position.x = bounds.center.x
        marker.pose.position.y = bounds.center.y
        marker.pose.position.z = z_m
        marker.scale.x = bounds.width
        marker.scale.y = bounds.height
        marker.scale.z = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _line(self, namespace, marker_id, points, width_m, color, stamp):
        marker = self._marker(namespace, marker_id, Marker.LINE_STRIP, stamp)
        marker.scale.x = width_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(x=point.x, y=point.y, z=0.02) for point in points]
        return marker

    def _landmark(self, namespace, marker_id, position, color, stamp):
        marker = self._marker(namespace, marker_id, Marker.SPHERE, stamp)
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = 0.06
        marker.scale.x = marker.scale.y = marker.scale.z = 0.14
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _text(self, namespace, marker_id, text, position, z_m, color, stamp):
        marker = self._marker(namespace, marker_id, Marker.TEXT_VIEW_FACING, stamp)
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = z_m
        marker.scale.z = 0.14
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        return marker

    @staticmethod
    def _rectangle_outline(bounds: Bounds2D):
        return (
            Point2D(bounds.x_min, bounds.y_min),
            Point2D(bounds.x_max, bounds.y_min),
            Point2D(bounds.x_max, bounds.y_max),
            Point2D(bounds.x_min, bounds.y_max),
            Point2D(bounds.x_min, bounds.y_min),
        )

    def _publish_reference(self):
        stamp = self.get_clock().now().to_msg()
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)

        zone_colors = {
            "A": (0.25, 0.50, 0.95, 0.12),
            "B": (0.65, 0.67, 0.70, 0.10),
            "C": (0.35, 0.72, 0.38, 0.12),
        }
        for marker_id, zone_name in enumerate(("A", "B", "C")):
            message.markers.append(self._cube(
                "field_zones",
                marker_id,
                self._reference.zones[zone_name],
                -0.035,
                zone_colors[zone_name],
                stamp,
            ))

        for marker_id, wall in enumerate(self._reference.b_walls):
            message.markers.append(self._cube(
                "b_walls",
                marker_id,
                wall,
                -0.020,
                (0.55, 0.57, 0.60, 0.40),
                stamp,
            ))
        message.markers.append(self._cube(
            "b_corridor",
            0,
            self._reference.corridor,
            -0.015,
            (0.98, 0.78, 0.18, 0.32),
            stamp,
        ))

        message.markers.append(self._line(
            "field_outline",
            0,
            self._rectangle_outline(self._reference.field),
            0.035,
            (0.92, 0.94, 0.98, 0.95),
            stamp,
        ))
        marker_id = 0
        for y_m in (
            self._reference.zones["A"].y_max,
            self._reference.zones["B"].y_max,
        ):
            for x_min, x_max in (
                (self._reference.field.x_min, self._reference.corridor.x_min),
                (self._reference.corridor.x_max, self._reference.field.x_max),
            ):
                message.markers.append(self._line(
                    "zone_dividers",
                    marker_id,
                    (Point2D(x_min, y_m), Point2D(x_max, y_m)),
                    0.02,
                    (0.80, 0.82, 0.86, 0.75),
                    stamp,
                ))
                marker_id += 1

        message.markers.append(self._line(
            "c_ring_outer",
            0,
            self._reference.ring_outer_outline,
            0.045,
            (1.00, 0.76, 0.10, 0.95),
            stamp,
        ))
        message.markers.append(self._line(
            "c_ring_inner",
            0,
            self._reference.ring_inner_outline,
            0.045,
            (1.00, 0.76, 0.10, 0.95),
            stamp,
        ))

        message.markers.append(self._landmark(
            "field_landmarks",
            0,
            self._reference.p_origin,
            (0.20, 0.90, 0.35, 1.0),
            stamp,
        ))
        message.markers.append(self._landmark(
            "field_landmarks",
            1,
            self._reference.task_point,
            (1.00, 0.38, 0.16, 1.0),
            stamp,
        ))
        message.markers.append(self._text(
            "landmark_labels",
            0,
            "P",
            self._reference.p_origin,
            0.20,
            (0.20, 1.00, 0.40, 1.0),
            stamp,
        ))
        message.markers.append(self._text(
            "landmark_labels",
            1,
            "Task",
            self._reference.task_point,
            0.20,
            (1.00, 0.55, 0.20, 1.0),
            stamp,
        ))
        for marker_id, label in enumerate(self._reference.labels):
            message.markers.append(self._text(
                "field_labels",
                marker_id,
                label.text,
                label.position,
                0.10,
                (0.95, 0.95, 0.95, 0.90),
                stamp,
            ))

        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FieldReferenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
