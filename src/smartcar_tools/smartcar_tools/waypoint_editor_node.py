"""Motion-disabled RViz editor for the semantic mission waypoint file.

Editing via RViz toolbar tools:
  - "Publish Point"     → moves the selected waypoint to the clicked position
  - "2D Pose Estimate"  → sets both position AND orientation of the selected waypoint

Select a waypoint to edit:
  ros2 param set /waypoint_editor selected_index 3

Save / Undo / Reload via services or right-click context menu (if interactive
markers are functional in the current RViz version).
"""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from smartcar_task.waypoints import (
    is_zero_quaternion,
    load_waypoint_document,
    validate_waypoints,
    write_waypoints_atomic,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)


EDITOR_NAMESPACE = "smartcar/waypoint_editor"
MARKER_TOPIC = "/smartcar/waypoint_editor/markers"
STATUS_TOPIC = "/smartcar/waypoint_editor/status"
SERVICE_PREFIX = "/smartcar/waypoint_editor"

TASK_COLORS = {
    "start": (0.20, 0.85, 0.30),
    "qr": (0.25, 0.55, 1.00),
    "vlm": (0.95, 0.55, 0.20),
    "corridor": (0.95, 0.80, 0.20),
    "loop": (1.00, 0.35, 0.20),
    "return": (0.70, 0.30, 0.90),
    "nav": (0.35, 0.72, 0.90),
    "via": (0.67, 0.71, 0.75),
}


def _yaw_quaternion(yaw_rad):
    half = float(yaw_rad) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _quaternion_yaw(orientation):
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("interactive marker orientation must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise ValueError("interactive marker orientation must be nonzero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class WaypointEditorNode(Node):
    """Edit the one mission waypoint file through RViz."""

    def __init__(self):
        super().__init__("waypoint_editor")
        default_file = str(
            Path(get_package_share_directory("smartcar_nav2"))
            / "config" / "waypoints" / "default_waypoints.yaml"
        )
        self.declare_parameter("waypoints_file", default_file)
        self.declare_parameter("latch_emergency_stop", True)
        self.declare_parameter("selected_index", 0)

        self._path = Path(
            str(self.get_parameter("waypoints_file").value)
        ).expanduser()
        self._template, loaded = load_waypoint_document(self._path)
        self._waypoints = list(loaded)
        self._history = []
        self._dragging = set()
        self._lock = threading.Lock()

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        self._marker_publisher = self.create_publisher(
            MarkerArray, MARKER_TOPIC, latched_qos
        )
        self._status_publisher = self.create_publisher(
            String, STATUS_TOPIC, latched_qos
        )
        self.create_service(Trigger, f"{SERVICE_PREFIX}/load", self._on_load)
        self.create_service(Trigger, f"{SERVICE_PREFIX}/undo", self._on_undo)
        self.create_service(Trigger, f"{SERVICE_PREFIX}/save", self._on_save)

        # ── interactive markers (best-effort; may not render in all RViz versions) ──
        self._server = InteractiveMarkerServer(self, EDITOR_NAMESPACE)
        self._menu = MenuHandler()
        self._menu.insert("Save all waypoints", callback=self._menu_save)
        self._menu.insert("Undo last drag", callback=self._menu_undo)
        self._menu.insert("Reload from disk", callback=self._menu_load)

        # ── clicked-point / 2D-pose editing (always works via MarkerArray display) ──
        self._clicked_sub = self.create_subscription(
            PointStamped, "/clicked_point", self._on_clicked_point, 10
        )
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10
        )

        self._estop_required = bool(
            self.get_parameter("latch_emergency_stop").value
        )
        self._estop_latched = not self._estop_required
        self._estop_pending = False
        self._last_estop_assertion_ns = None
        self._estop_client = self.create_client(
            SetBool, "/smartcar/safety/emergency_stop"
        )
        self._estop_timer = self.create_timer(0.25, self._ensure_estop)
        self._param_callback_handle = self.add_on_set_parameters_callback(
            self._on_param_change
        )

        self._refresh_display()
        selected = self._selected_index
        lock_state = "locked" if selected in (0, len(self._waypoints) - 1) else "editable"
        self._publish_status(
            f"loaded {len(self._waypoints)} waypoints; "
            f"selected [{selected}] {self._waypoints[selected].id} ({lock_state}); "
            f"use Publish Point or 2D Pose Estimate tool to edit; "
            f"change selection with: ros2 param set /waypoint_editor selected_index N"
        )

    @property
    def _selected_index(self):
        return max(0, min(
            len(self._waypoints) - 1,
            self.get_parameter("selected_index").value,
        ))

    def _on_param_change(self, params):
        for param in params:
            if param.name == "selected_index":
                idx = max(0, min(len(self._waypoints) - 1, param.value))
                lock_state = "LOCKED" if idx in (0, len(self._waypoints) - 1) else "editable"
                self._publish_status(
                    f"selected [{idx}] {self._waypoints[idx].id} ({lock_state})"
                )
                self._publish_route_markers()  # re-render to show selection highlight
        return rclpy.parameter.SetParametersResult(successful=True)

    # ── status ──────────────────────────────────────────────────────────

    def _publish_status(self, message):
        self._status_publisher.publish(String(data=str(message)))
        self.get_logger().info(str(message))

    # ── emergency stop ──────────────────────────────────────────────────

    def _ensure_estop(self):
        if not self._estop_required or self._estop_pending:
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

    def _on_estop_response(self, future):
        self._estop_pending = False
        try:
            response = future.result()
        except Exception as error:
            self._estop_latched = False
            self.get_logger().error(f"failed to latch emergency stop: {error}")
            return
        self._estop_latched = bool(response is not None and response.success)
        if self._estop_latched:
            self._publish_status(
                "emergency stop latched; waypoint editing is motion-disabled"
            )
        else:
            message = "no response" if response is None else response.message
            self.get_logger().error(f"emergency stop rejected: {message}")

    # ── clicked-point / pose editing ───────────────────────────────────

    def _update_waypoint(self, index, x, y, yaw=None):
        """Move (and optionally rotate) a waypoint. Returns (success, message)."""
        if index == 0:
            return False, "cannot move start waypoint (index 0)"
        if index == len(self._waypoints) - 1:
            return False, "cannot move return waypoint (last index)"

        original = self._waypoints[index]
        if yaw is None:
            _qx, _qy, qz, qw = original.orientation
            yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return False, f"invalid pose: x={x}, y={y}, yaw={yaw}"

        self._push_history()
        with self._lock:
            self._waypoints[index] = replace(
                original,
                position=(float(x), float(y), 0.0),
                orientation=_yaw_quaternion(float(yaw)),
            )
        self._publish_route_markers()
        self._refresh_interactive_markers()
        return True, (
            f"waypoint [{index}] {original.id} → "
            f"({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)"
        )

    def _on_clicked_point(self, msg: PointStamped):
        """Publish Point tool: move the selected waypoint to the clicked location."""
        index = self._selected_index
        success, message = self._update_waypoint(index, msg.point.x, msg.point.y)
        self._publish_status(f"[clicked] {message}")

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        """2D Pose Estimate tool: set both position and orientation."""
        index = self._selected_index
        orientation = msg.pose.pose.orientation
        try:
            yaw = _quaternion_yaw(orientation)
        except ValueError:
            yaw = 0.0
        success, message = self._update_waypoint(
            index, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw
        )
        self._publish_status(f"[pose] {message}")

    # ── history ────────────────────────────────────────────────────────

    def _push_history(self):
        self._history.append(tuple(self._waypoints))
        if len(self._history) > 100:
            del self._history[0]

    # ── interactive markers (best-effort; core editing is via clicked_point) ──

    def _make_interactive_marker(self, index, waypoint):
        marker = InteractiveMarker()
        marker.header.frame_id = waypoint.frame_id
        marker.name = waypoint.id
        marker.description = f"{index}: {waypoint.id} [{waypoint.task}]"
        marker.scale = 0.32
        marker.pose.position.x = waypoint.position[0]
        marker.pose.position.y = waypoint.position[1]
        marker.pose.position.z = 0.04
        qx, qy, qz, qw = waypoint.orientation
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        red, green, blue = TASK_COLORS[waypoint.task]
        body = InteractiveMarkerControl()
        body.name = "waypoint"
        body.always_visible = True
        body.interaction_mode = InteractiveMarkerControl.MENU

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12
        sphere.color.r, sphere.color.g, sphere.color.b = red, green, blue
        sphere.color.a = 1.0
        body.markers.append(sphere)

        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = 0.24
        arrow.scale.y = arrow.scale.z = 0.045
        arrow.color.r, arrow.color.g, arrow.color.b = red, green, blue
        arrow.color.a = 0.95
        body.markers.append(arrow)
        marker.controls.append(body)

        if index not in (0, len(self._waypoints) - 1):
            move = InteractiveMarkerControl()
            move.name = "move_xy"
            move.orientation.w = math.sqrt(0.5)
            move.orientation.y = math.sqrt(0.5)
            move.orientation_mode = InteractiveMarkerControl.FIXED
            move.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
            marker.controls.append(move)

        if index != 0:
            rotate = InteractiveMarkerControl()
            rotate.name = "rotate_z"
            rotate.orientation.w = math.sqrt(0.5)
            rotate.orientation.y = math.sqrt(0.5)
            rotate.orientation_mode = InteractiveMarkerControl.FIXED
            rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(rotate)
        return marker

    def _refresh_interactive_markers(self):
        self._server.clear()
        for index, waypoint in enumerate(self._waypoints):
            marker = self._make_interactive_marker(index, waypoint)
            self._server.insert(marker, feedback_callback=self._on_feedback)
            self._menu.apply(self._server, marker.name)
        self._server.applyChanges()

    def _publish_route_markers(self):
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        if not self._waypoints:
            self._marker_publisher.publish(message)
            return

        stamp = self.get_clock().now().to_msg()
        frame_id = self._waypoints[0].frame_id
        selected = self._selected_index

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = "mission_route"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.03
        line.color.r = 0.15
        line.color.g = 0.85
        line.color.b = 0.95
        line.color.a = 0.9
        line.points = [
            Point(x=item.position[0], y=item.position[1], z=0.025)
            for item in self._waypoints
        ]
        message.markers.append(line)

        for index, waypoint in enumerate(self._waypoints):
            red, green, blue = TASK_COLORS[waypoint.task]
            is_selected = index == selected
            is_locked = index in (0, len(self._waypoints) - 1)

            # Skip arrow for zero-quaternion (orientation-unconstrained) waypoints
            if not is_zero_quaternion(waypoint.orientation):
                qx, qy, qz, qw = waypoint.orientation
                sin_yaw = 2.0 * (qw * qz + qx * qy)
                cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw = math.atan2(sin_yaw, cos_yaw)
                half = yaw / 2.0
                aqx, aqy, aqz, aqw = (0.0, 0.0, math.sin(half), math.cos(half))

                arrow = Marker()
                arrow.header.frame_id = frame_id
                arrow.header.stamp = stamp
                arrow.ns = "mission_arrows"
                arrow.id = index
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = waypoint.position[0]
                arrow.pose.position.y = waypoint.position[1]
                arrow.pose.position.z = 0.09
                arrow.pose.orientation.x = aqx
                arrow.pose.orientation.y = aqy
                arrow.pose.orientation.z = aqz
                arrow.pose.orientation.w = aqw
                arrow.scale.x = 0.22
                arrow.scale.y = 0.04
                arrow.scale.z = 0.04
                arrow.color.r = red
                arrow.color.g = green
                arrow.color.b = blue
                arrow.color.a = 0.95
                message.markers.append(arrow)

            # sphere with selection highlight
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = "mission_spheres"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = waypoint.position[0]
            sphere.pose.position.y = waypoint.position[1]
            sphere.pose.position.z = 0.06
            sphere.pose.orientation.w = 1.0
            if is_selected:
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.16
                sphere.color.r = 1.0
                sphere.color.g = 1.0
                sphere.color.b = 0.2
                sphere.color.a = 1.0
            else:
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.10
                sphere.color.r = red
                sphere.color.g = green
                sphere.color.b = blue
                sphere.color.a = 0.7 if is_locked else 1.0
            message.markers.append(sphere)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = "mission_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = waypoint.position[0]
            label.pose.position.y = waypoint.position[1]
            label.pose.position.z = 0.24
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            lock_mark = " [LOCKED]" if is_locked else ""
            sel_mark = " *" if is_selected else ""
            label.text = f"{index}: {waypoint.id}{sel_mark}{lock_mark}"
            message.markers.append(label)
        self._marker_publisher.publish(message)

    def _refresh_display(self):
        self._publish_route_markers()
        self._refresh_interactive_markers()

    def _on_feedback(self, feedback):
        index_by_id = {
            waypoint.id: index for index, waypoint in enumerate(self._waypoints)
        }
        index = index_by_id.get(feedback.marker_name)
        if index is None:
            return
        event = feedback.event_type
        if event == InteractiveMarkerFeedback.MOUSE_DOWN:
            if feedback.marker_name not in self._dragging:
                self._push_history()
                self._dragging.add(feedback.marker_name)
            return
        if event == InteractiveMarkerFeedback.MOUSE_UP:
            self._dragging.discard(feedback.marker_name)
            self._publish_status(
                f"updated {feedback.marker_name}; right-click a point to save"
            )
            return
        if event != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        if feedback.marker_name not in self._dragging:
            self._push_history()
            self._dragging.add(feedback.marker_name)

        original = self._waypoints[index]
        try:
            x_m = original.position[0] if index in (
                0, len(self._waypoints) - 1
            ) else float(feedback.pose.position.x)
            y_m = original.position[1] if index in (
                0, len(self._waypoints) - 1
            ) else float(feedback.pose.position.y)
            yaw = 0.0 if index == 0 else _quaternion_yaw(
                feedback.pose.orientation
            )
            if not all(math.isfinite(value) for value in (x_m, y_m, yaw)):
                raise ValueError("interactive marker pose must be finite")
        except (TypeError, ValueError) as error:
            self._dragging.discard(feedback.marker_name)
            self._refresh_interactive_markers()
            self._publish_status(f"rejected invalid drag for {feedback.marker_name}: {error}")
            return
        with self._lock:
            self._waypoints[index] = replace(
                original,
                position=(x_m, y_m, 0.0),
                orientation=_yaw_quaternion(yaw),
            )
        self._publish_route_markers()

    def _load(self):
        template, loaded = load_waypoint_document(self._path)
        self._template = template
        self._waypoints = list(loaded)
        self._history.clear()
        self._dragging.clear()
        self._refresh_display()
        return True, f"reloaded {len(self._waypoints)} waypoints"

    def _undo(self):
        if not self._history:
            return False, "nothing to undo"
        with self._lock:
            self._waypoints = list(self._history.pop())
        self._dragging.clear()
        self._refresh_display()
        return True, "restored the previous waypoint positions"

    def _save(self):
        with self._lock:
            validate_waypoints(self._waypoints)
            destination = self._path.resolve()
            write_waypoints_atomic(destination, self._template, self._waypoints)
            self._path = destination
            self._template, loaded = load_waypoint_document(destination)
            self._waypoints = list(loaded)
        self._history.clear()
        self._dragging.clear()
        self._refresh_display()
        return True, "waypoints saved atomically and marked uncalibrated"

    def _service_result(self, operation, response):
        try:
            success, message = operation()
        except ValueError as error:
            success, message = False, str(error)
        response.success = success
        response.message = message
        self._publish_status(message)
        return response

    def _on_load(self, _request, response):
        return self._service_result(self._load, response)

    def _on_undo(self, _request, response):
        return self._service_result(self._undo, response)

    def _on_save(self, _request, response):
        return self._service_result(self._save, response)

    def _menu_load(self, _feedback):
        self._run_menu_operation(self._load)

    def _menu_undo(self, _feedback):
        self._run_menu_operation(self._undo)

    def _menu_save(self, _feedback):
        self._run_menu_operation(self._save)

    def _run_menu_operation(self, operation):
        try:
            _success, message = operation()
        except ValueError as error:
            message = str(error)
        self._publish_status(message)

    def destroy_node(self):
        self._server.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointEditorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as error:
        if not isinstance(error, KeyboardInterrupt):
            rclpy.logging.get_logger("waypoint_editor").fatal(str(error))
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
