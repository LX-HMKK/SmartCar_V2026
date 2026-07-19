"""RViz 2D Goal Pose route micro-adjustment node with fail-closed startup."""
from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from smartcar_tools.route_model import (
    RoutePoint,
    RouteValidationError,
    load_route,
    normalize_yaw_deg,
    validate_route,
    write_route_atomic,
)


MARKER_TOPIC = "/smartcar/route_editor/markers"
STATUS_TOPIC = "/smartcar/route_editor/status"
SELECTION_TOPIC = "/smartcar/route_editor/selected_id"
GOAL_TOPIC = "/goal_pose"
SERVICE_PREFIX = "/smartcar/route_editor"


def quaternion_to_yaw_deg(orientation) -> float:
    values = (orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(value) for value in values):
        raise RouteValidationError("goal orientation must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise RouteValidationError("goal orientation quaternion must be nonzero")
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return normalize_yaw_deg(math.degrees(yaw))


def yaw_quaternion(yaw_deg: float):
    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


class RouteEditorNode(Node):
    """Load a route, edit points from RViz, and save only fully valid YAML."""

    def __init__(self):
        super().__init__("route_editor")
        default_route = str(
            Path(get_package_share_directory("smartcar_tools"))
            / "config" / "routes" / "full_course_route.yaml"
        )
        self.declare_parameter("route_file", default_route)
        self.declare_parameter("goal_topic", GOAL_TOPIC)
        self.declare_parameter("latch_emergency_stop", True)

        self._route_path = Path(
            str(self.get_parameter("route_file").value)).expanduser()
        self._route = load_route(self._route_path)
        self._points = list(self._route.waypoints)
        self._history: list[tuple[RoutePoint, ...]] = []
        self._selected_id = ""
        self._edit_counter = 1

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        self._marker_publisher = self.create_publisher(
            MarkerArray, MARKER_TOPIC, latched_qos)
        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self._on_goal,
            10,
        )
        self.create_subscription(
            String, SELECTION_TOPIC, self._on_selection, 10)
        self.create_service(
            Trigger, f"{SERVICE_PREFIX}/load", self._on_load)
        self.create_service(
            Trigger, f"{SERVICE_PREFIX}/undo", self._on_undo)
        self.create_service(
            Trigger, f"{SERVICE_PREFIX}/clear", self._on_clear)
        self.create_service(
            Trigger, f"{SERVICE_PREFIX}/save", self._on_save)

        self._estop_required = bool(
            self.get_parameter("latch_emergency_stop").value)
        self._estop_latched = not self._estop_required
        self._estop_pending = False
        self._last_estop_assertion_ns = None
        self._estop_client = self.create_client(
            SetBool, "/smartcar/safety/emergency_stop")
        self._estop_timer = self.create_timer(0.25, self._ensure_estop)
        self.create_timer(0.5, self._publish_markers)
        self._publish_markers()
        self._publish_status(
            f"loaded {len(self._points)} points from {self._route_path}; "
            "publish an ID on selected_id before replacing a point"
        )

    def _publish_status(self, message: str) -> None:
        self._status_publisher.publish(String(data=message))
        self.get_logger().info(message)

    def _ensure_estop(self) -> None:
        if not self._estop_required:
            return
        if self._estop_pending:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._estop_latched
            and self._last_estop_assertion_ns is not None
            and now_ns - self._last_estop_assertion_ns < 2_000_000_000
        ):
            return
        if not self._estop_client.service_is_ready():
            return
        self._estop_pending = True
        self._last_estop_assertion_ns = now_ns
        request = SetBool.Request()
        request.data = True
        future = self._estop_client.call_async(request)
        future.add_done_callback(self._on_estop_response)

    def _on_estop_response(self, future) -> None:
        self._estop_pending = False
        was_latched = self._estop_latched
        try:
            response = future.result()
        except Exception as error:  # rclpy service futures surface transport errors here.
            self._estop_latched = False
            self.get_logger().error(f"failed to latch emergency stop: {error}")
            return
        if response is not None and response.success:
            self._estop_latched = True
            if not was_latched:
                self._publish_status(
                    "emergency stop latched; route editing is motion-disabled")
        else:
            self._estop_latched = False
            message = "no response" if response is None else response.message
            self.get_logger().error(f"emergency stop rejected: {message}")

    def _push_history(self) -> None:
        self._history.append(tuple(self._points))
        if len(self._history) > 100:
            del self._history[0]

    def _on_selection(self, message: String) -> None:
        selected = message.data.strip()
        if selected and not any(point.id == selected for point in self._points):
            self._publish_status(f"selection rejected: unknown waypoint {selected}")
            return
        self._selected_id = selected
        if selected:
            self._publish_status(f"next 2D Goal Pose will replace {selected}")
        else:
            self._publish_status("selection cleared; next 2D Goal Pose will insert a point")

    def _on_goal(self, message: PoseStamped) -> None:
        if self._estop_required and not self._estop_latched:
            self._publish_status("goal ignored until emergency stop is latched")
            return
        frame_id = message.header.frame_id.strip()
        if frame_id and frame_id != self._route.frame_id:
            self._publish_status(
                f"goal rejected: frame {frame_id} is not {self._route.frame_id}")
            return
        x_m = float(message.pose.position.x)
        y_m = float(message.pose.position.y)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            self._publish_status("goal rejected: position must be finite")
            return
        try:
            yaw_deg = quaternion_to_yaw_deg(message.pose.orientation)
        except RouteValidationError as error:
            self._publish_status(f"goal rejected: {error}")
            return

        self._push_history()
        if self._selected_id:
            index = next(
                index for index, point in enumerate(self._points)
                if point.id == self._selected_id
            )
            original = self._points[index]
            self._points[index] = replace(
                original, x=x_m, y=y_m, yaw_deg=yaw_deg)
            self._publish_status(f"replaced {original.id}; save will revalidate the route")
            self._selected_id = ""
        else:
            zone = self._route.geometry.zone_for(x_m, y_m)
            if zone == "P":
                zone = "A"
            while True:
                point_id = f"edit_{zone.lower()}_{self._edit_counter:03d}"
                self._edit_counter += 1
                if not any(point.id == point_id for point in self._points):
                    break
            point = RoutePoint(point_id, zone, x_m, y_m, yaw_deg)
            insert_at = max(1, len(self._points) - 1)
            self._points.insert(insert_at, point)
            self._publish_status(
                f"inserted {point_id} before the P finish; save will revalidate the route")
        self._publish_markers()

    def _on_load(self, _request, response):
        try:
            route = load_route(self._route_path)
        except RouteValidationError as error:
            response.success = False
            response.message = str(error)
            return response
        self._route = route
        self._points = list(route.waypoints)
        self._history.clear()
        self._selected_id = ""
        self._publish_markers()
        response.success = True
        response.message = f"loaded {len(self._points)} points"
        return response

    def _on_undo(self, _request, response):
        if not self._history:
            response.success = False
            response.message = "nothing to undo"
            return response
        self._points = list(self._history.pop())
        self._selected_id = ""
        self._publish_markers()
        response.success = True
        response.message = f"restored {len(self._points)} points"
        return response

    def _on_clear(self, _request, response):
        self._push_history()
        self._points.clear()
        self._selected_id = ""
        self._publish_markers()
        response.success = True
        response.message = "editor points cleared; use undo or load to restore"
        return response

    def _on_save(self, _request, response):
        candidate = replace(
            self._route,
            calibrated=False,
            waypoints=tuple(self._points),
        )
        try:
            validate_route(candidate)
            # Resolve a symlink-install data file so atomic replacement updates source.
            destination = self._route_path.resolve()
            write_route_atomic(candidate, destination)
        except RouteValidationError as error:
            response.success = False
            response.message = f"not saved: {error}"
            return response
        self._route = candidate
        self._route_path = destination
        self._history.clear()
        response.success = True
        response.message = "route saved atomically and marked uncalibrated"
        return response

    def _publish_markers(self) -> None:
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        if not self._points:
            self._marker_publisher.publish(message)
            return

        stamp = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id = self._route.frame_id
        line.header.stamp = stamp
        line.ns = "route_line"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.025
        line.color.r = 0.10
        line.color.g = 0.85
        line.color.b = 0.95
        line.color.a = 0.9
        line.points = [Point(x=point.x, y=point.y, z=0.03) for point in self._points]
        message.markers.append(line)

        colors = {
            "P": (0.20, 0.85, 0.30),
            "A": (0.25, 0.55, 1.00),
            "B": (0.95, 0.80, 0.20),
            "C": (1.00, 0.35, 0.20),
        }
        for index, point in enumerate(self._points):
            red, green, blue = colors.get(point.zone, (1.0, 1.0, 1.0))
            sphere = Marker()
            sphere.header.frame_id = self._route.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "route_points"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = point.x
            sphere.pose.position.y = point.y
            sphere.pose.position.z = 0.06
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.10
            sphere.color.r, sphere.color.g, sphere.color.b = red, green, blue
            sphere.color.a = 1.0
            message.markers.append(sphere)

            arrow = Marker()
            arrow.header.frame_id = self._route.frame_id
            arrow.header.stamp = stamp
            arrow.ns = "route_headings"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = point.x
            arrow.pose.position.y = point.y
            arrow.pose.position.z = 0.08
            qx, qy, qz, qw = yaw_quaternion(point.yaw_deg)
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = 0.20
            arrow.scale.y = 0.035
            arrow.scale.z = 0.035
            arrow.color.r, arrow.color.g, arrow.color.b = red, green, blue
            arrow.color.a = 0.95
            message.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self._route.frame_id
            label.header.stamp = stamp
            label.ns = "route_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = point.x
            label.pose.position.y = point.y
            label.pose.position.z = 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"{index}: {point.id}"
            message.markers.append(label)
        self._marker_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RouteEditorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RouteValidationError) as error:
        if not isinstance(error, KeyboardInterrupt):
            rclpy.logging.get_logger("route_editor").fatal(str(error))
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
