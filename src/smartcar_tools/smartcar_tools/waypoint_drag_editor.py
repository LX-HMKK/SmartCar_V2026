"""Drag-to-move waypoint editor using matplotlib — works around RViz InteractiveMarkers bug.

Mouse:
  - drag  a waypoint circle to move it
  - scroll wheel on a selected waypoint to rotate (±5° per tick)
  - click a waypoint to select it (highlighted ring)
  - click empty space to deselect

Keyboard:
  - Ctrl+S          → save to YAML
  - Ctrl+Z          → undo last move
  - R / Shift+R     → rotate selected waypoint ±15°
  - Delete / Escape → deselect

Usage:
  ros2 run smartcar_tools waypoint_drag_editor
"""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
from matplotlib.patches import FancyBboxPatch
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from smartcar_task.waypoints import (
    load_waypoint_document,
    validate_waypoints,
    write_waypoints_atomic,
)
from smartcar_tools.field_reference import (
    Bounds2D,
    Point2D,
    load_field_reference,
)
from visualization_msgs.msg import Marker, MarkerArray

# ── colours ────────────────────────────────────────────────────────────────
TASK_COLORS = {
    "start":    "#33D94D",
    "qr":       "#408CFF",
    "vlm":      "#F28C33",
    "corridor": "#F2CC33",
    "loop":     "#FF5933",
    "return":   "#B34DE6",
}
DEFAULT_COLOR = "#CCCCCC"
SELECTED_EDGE = "#FFFF33"
FIELD_BG = "#1A1C20"
GRID_COLOR = "#2A2C30"
LINE_COLOR = "#26D9F2"


def _yaw_from_quaternion(orientation):
    qx, qy, qz, qw = orientation
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _yaw_quaternion(yaw_rad):
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _bounds_rect(b: Bounds2D):
    """Convert Bounds2D to (x, y, width, height) anchored at bottom-left."""
    return (b.x_min, b.y_min, b.width, b.height)


class DragEditor:
    """matplotlib-based draggable waypoint editor with full field reference."""

    def __init__(self, node: Node, waypoints_path: Path, geometry_path: Path):
        self._node = node
        self._path = waypoints_path
        self._template, loaded = load_waypoint_document(self._path)
        self._waypoints = list(loaded)
        self._history: list[tuple] = []
        self._selected: int | None = None
        self._dragging: int | None = None
        self._drag_start = None
        self._panning = False
        self._pan_start = None
        self._lock = threading.Lock()

        # ── load field reference (same data as RViz overlay) ────────────
        self._field_ref = load_field_reference(geometry_path)

        # ── build matplotlib figure ────────────────────────────────────
        matplotlib.use("Qt5Agg")
        matplotlib.rcParams["keymap.save"] = []
        matplotlib.rcParams["keymap.pan"] = []
        matplotlib.rcParams["keymap.zoom"] = []
        matplotlib.rcParams["toolbar"] = "toolbar2"

        field = self._field_ref.field
        margin = 0.4
        data_w = field.width + 2 * margin
        data_h = field.height + 2 * margin

        # size figure to match data aspect, filling the screen nicely
        self._fig, self._ax = plt.subplots(figsize=(10, 10 * data_h / data_w))
        self._fig.canvas.manager.set_window_title("Waypoint Drag Editor — SmartCar")

        # minimal margins so the plot fills the window
        self._fig.subplots_adjust(left=0.06, right=0.97, bottom=0.06, top=0.97)

        self._ax.set_facecolor(FIELD_BG)
        self._fig.patch.set_facecolor(FIELD_BG)

        self._ax.set_xlim(field.x_min - margin, field.x_max + margin)
        self._ax.set_ylim(field.y_min - margin, field.y_max + margin)
        self._ax.set_aspect("equal")
        self._ax.grid(True, color=GRID_COLOR, alpha=0.5, linewidth=0.5)
        self._ax.set_xlabel("X (m)  —  odom_combined", color="#888")
        self._ax.set_ylabel("Y (m)", color="#888")
        self._ax.tick_params(colors="#888")

        self._draw_field_ref()

        # controls hint
        self._hint = self._ax.text(
            0.5, -0.06,
            "drag=move  |  scroll=rotate(sel)/zoom(bg)  |  right-drag=pan  |  "
            "+/- =zoom  |  arrows=pan  |  Ctrl+S=save  |  Ctrl+Z=undo  |  R=rotate15°",
            transform=self._ax.transAxes, fontsize=7, color="#666",
            ha="center", va="top",
        )

        # artists we'll update
        self._scatter = None
        self._arrows = []
        self._line = None
        self._labels = []
        self._sel_ring = None

        self._redraw()

        # ── connect events ─────────────────────────────────────────────
        self._fig.canvas.mpl_connect("button_press_event", self._on_press)
        self._fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self._fig.canvas.mpl_connect("button_release_event", self._on_release)
        self._fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self._fig.canvas.mpl_connect("key_press_event", self._on_key)

        self._node.get_logger().info(
            f"Drag editor ready: {len(self._waypoints)} waypoints, "
            f"field ref from {geometry_path}. "
            "Drag circles to move, scroll to rotate, Ctrl+S to save."
        )

    # ── field reference (matches RViz field_reference_node output) ─────

    def _draw_field_ref(self):
        ref = self._field_ref

        # --- zone fills ---
        zone_specs = [
            ("A", "#4050F21E"),
            ("B", "#A6ABB21A"),
            ("C", "#59B8611E"),
        ]
        for zone_name, color in zone_specs:
            bounds = ref.zones[zone_name]
            self._ax.add_patch(plt.Rectangle(
                (bounds.x_min, bounds.y_min), bounds.width, bounds.height,
                facecolor=color, edgecolor="none", zorder=0,
            ))

        # --- B-zone walls ---
        for wall in ref.b_walls:
            self._ax.add_patch(plt.Rectangle(
                (wall.x_min, wall.y_min), wall.width, wall.height,
                facecolor="#8C919966", edgecolor="none", zorder=0,
            ))

        # --- corridor ---
        c = ref.corridor
        self._ax.add_patch(plt.Rectangle(
            (c.x_min, c.y_min), c.width, c.height,
            facecolor="#FAC72E52", edgecolor="none", zorder=0,
        ))

        # --- field outline ---
        f = ref.field
        self._ax.add_patch(plt.Rectangle(
            (f.x_min, f.y_min), f.width, f.height,
            facecolor="none", edgecolor="#EBF0FAF2", linewidth=2.5, zorder=1,
        ))

        # --- zone dividers ---
        for y_m in (ref.zones["A"].y_max, ref.zones["B"].y_max):
            for x_pair in (
                (ref.field.x_min, ref.corridor.x_min),
                (ref.corridor.x_max, ref.field.x_max),
            ):
                self._ax.plot(
                    x_pair, (y_m, y_m),
                    color="#CCD1DBBF", linewidth=1.5, zorder=1,
                )

        # --- C ring (stadium outlines) ---
        for outline, label in [
            (ref.ring_outer_outline, "outer"),
            (ref.ring_inner_outline, "inner"),
        ]:
            xs = [p.x for p in outline]
            ys = [p.y for p in outline]
            self._ax.plot(xs, ys, color="#FFC219F2", linewidth=3.0, zorder=1)

        # --- P origin ---
        self._ax.scatter(
            [ref.p_origin.x], [ref.p_origin.y],
            s=100, c="#33E659", edgecolors="#33E659", linewidths=0,
            zorder=2,
        )
        self._ax.annotate(
            "P (origin)", (ref.p_origin.x, ref.p_origin.y + 0.18),
            fontsize=8, color="#33FF66", ha="center", va="bottom", zorder=2,
        )

        # --- task point ---
        self._ax.scatter(
            [ref.task_point.x], [ref.task_point.y],
            s=80, c="#FF661A", edgecolors="#FF661A", linewidths=0,
            zorder=2,
        )
        self._ax.annotate(
            "Task", (ref.task_point.x, ref.task_point.y + 0.18),
            fontsize=8, color="#FF8C42", ha="center", va="bottom", zorder=2,
        )

        # --- zone labels ---
        for label in ref.labels:
            self._ax.annotate(
                label.text, (label.position.x, label.position.y),
                fontsize=9, color="#F2F2F2E6", ha="center", va="center",
                zorder=1, alpha=0.8,
            )

    # ── rendering ──────────────────────────────────────────────────────

    def _redraw(self):
        """Full redraw of all waypoint artists."""
        xs = [w.position[0] for w in self._waypoints]
        ys = [w.position[1] for w in self._waypoints]
        yaws = [_yaw_from_quaternion(w.orientation) for w in self._waypoints]
        colors = [TASK_COLORS.get(w.task, DEFAULT_COLOR) for w in self._waypoints]

        # connecting line
        if self._line is not None:
            self._line.remove()
        self._line, = self._ax.plot(xs, ys, color=LINE_COLOR, linewidth=1.5,
                                     alpha=0.8, zorder=3)

        # scatter plot (draggable circles)
        if self._scatter is not None:
            self._scatter.remove()
        sizes = [140 if i == self._selected else 85 for i in range(len(xs))]
        edge_colors = [
            SELECTED_EDGE if i == self._selected else colors[i]
            for i in range(len(xs))
        ]
        edge_widths = [3 if i == self._selected else 1.5 for i in range(len(xs))]
        self._scatter = self._ax.scatter(
            xs, ys, s=sizes, c=colors, edgecolors=edge_colors,
            linewidths=edge_widths, zorder=5, picker=8, alpha=0.95,
        )

        # direction arrows
        for arrow in self._arrows:
            arrow.remove()
        self._arrows.clear()
        arrow_len = 0.18
        for x, y, yaw, color in zip(xs, ys, yaws, colors):
            dx = arrow_len * math.cos(yaw)
            dy = arrow_len * math.sin(yaw)
            arrow = self._ax.arrow(
                x, y, dx, dy, head_width=0.06, head_length=0.08,
                fc=color, ec=color, alpha=0.9, zorder=6,
                length_includes_head=True,
            )
            self._arrows.append(arrow)

        # labels
        for label in self._labels:
            label.remove()
        self._labels.clear()
        for i, (x, y, w) in enumerate(zip(xs, ys, self._waypoints)):
            locked = i in (0, len(self._waypoints) - 1)
            sel = " [*]" if i == self._selected else ""
            lck = " [LOCKED]" if locked else ""
            label = self._ax.annotate(
                f"{i}:{w.id}{sel}{lck}",
                (x, y + 0.20), fontsize=7, color="white",
                ha="center", va="bottom", zorder=7,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#000000CC",
                          edgecolor="none"),
            )
            self._labels.append(label)

        # selected ring
        if self._sel_ring is not None:
            self._sel_ring.remove()
            self._sel_ring = None
        if self._selected is not None:
            sx, sy = xs[self._selected], ys[self._selected]
            self._sel_ring = self._ax.add_patch(plt.Circle(
                (sx, sy), 0.12, fill=False, edgecolor=SELECTED_EDGE,
                linewidth=2.5, zorder=8, linestyle="-",
            ))

        self._fig.canvas.draw_idle()

    def _publish_markers(self):
        """Publish MarkerArray for RViz mirror."""
        from geometry_msgs.msg import Point

        msg = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)
        if not self._waypoints:
            self._ros_publisher.publish(msg)
            return

        stamp = self._node.get_clock().now().to_msg()
        frame_id = self._waypoints[0].frame_id

        # line
        line = Marker()
        line.header.frame_id = frame_id; line.header.stamp = stamp
        line.ns = "mission_route"; line.id = 0
        line.type = Marker.LINE_STRIP; line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.03
        line.color.r = 0.15; line.color.g = 0.85; line.color.b = 0.95; line.color.a = 0.9
        line.points = [Point(x=w.position[0], y=w.position[1], z=0.025) for w in self._waypoints]
        msg.markers.append(line)

        for i, w in enumerate(self._waypoints):
            r, g, b = self._hex_rgb(TASK_COLORS.get(w.task, DEFAULT_COLOR))
            yaw = _yaw_from_quaternion(w.orientation)
            half = yaw / 2.0

            s = Marker()
            s.header.frame_id = frame_id; s.header.stamp = stamp
            s.ns = "mission_spheres"; s.id = i
            s.type = Marker.SPHERE; s.action = Marker.ADD
            s.pose.position.x = w.position[0]
            s.pose.position.y = w.position[1]
            s.pose.position.z = 0.06
            s.pose.orientation.w = 1.0
            sz = 0.15 if i == self._selected else 0.10
            s.scale.x = s.scale.y = s.scale.z = sz
            if i == self._selected:
                s.color.r = 1.0; s.color.g = 1.0; s.color.b = 0.2; s.color.a = 1.0
            else:
                s.color.r = r; s.color.g = g; s.color.b = b
                s.color.a = 0.7 if i in (0, len(self._waypoints)-1) else 1.0
            msg.markers.append(s)

            a = Marker()
            a.header.frame_id = frame_id; a.header.stamp = stamp
            a.ns = "mission_arrows"; a.id = i
            a.type = Marker.ARROW; a.action = Marker.ADD
            a.pose.position.x = w.position[0]
            a.pose.position.y = w.position[1]
            a.pose.position.z = 0.09
            a.pose.orientation.z = math.sin(half)
            a.pose.orientation.w = math.cos(half)
            a.scale.x = 0.22; a.scale.y = 0.04; a.scale.z = 0.04
            a.color.r = r; a.color.g = g; a.color.b = b; a.color.a = 0.95
            msg.markers.append(a)

            l = Marker()
            l.header.frame_id = frame_id; l.header.stamp = stamp
            l.ns = "mission_labels"; l.id = i
            l.type = Marker.TEXT_VIEW_FACING; l.action = Marker.ADD
            l.pose.position.x = w.position[0]
            l.pose.position.y = w.position[1]
            l.pose.position.z = 0.24
            l.pose.orientation.w = 1.0
            l.scale.z = 0.10
            l.color.r = l.color.g = l.color.b = 1.0; l.color.a = 1.0
            lock_mark = " [LOCKED]" if i in (0, len(self._waypoints)-1) else ""
            sel_mark = " *" if i == self._selected else ""
            l.text = f"{i}: {w.id}{sel_mark}{lock_mark}"
            msg.markers.append(l)

        self._ros_publisher.publish(msg)

    @staticmethod
    def _hex_rgb(hex_color: str):
        h = hex_color.lstrip("#")
        return tuple(int(h[i:i+2], 16) / 255.0 for i in (0, 2, 4))

    # ── mouse events ───────────────────────────────────────────────────

    def _find_waypoint(self, event):
        if event.xdata is None or event.ydata is None:
            return None
        for i, w in enumerate(self._waypoints):
            dx = event.xdata - w.position[0]
            dy = event.ydata - w.position[1]
            if math.hypot(dx, dy) < 0.15:
                return i
        return None

    def _on_press(self, event):
        if event.button == MouseButton.RIGHT:
            self._panning = True
            self._pan_start = (event.x, event.y)
            return
        if event.button != MouseButton.LEFT:
            return
        idx = self._find_waypoint(event)
        if idx is not None:
            if idx in (0, len(self._waypoints) - 1):
                self._node.get_logger().info(
                    f"Waypoint [{idx}] is locked (start/return), cannot move"
                )
                self._selected = idx
                self._redraw()
                return
            self._dragging = idx
            self._selected = idx
            self._drag_start = (self._waypoints[idx].position[0],
                                self._waypoints[idx].position[1])
            self._push_history()
        else:
            self._selected = None
            self._dragging = None
        self._redraw()

    def _on_motion(self, event):
        # --- panning ---
        if self._panning and self._pan_start is not None:
            dx = event.x - self._pan_start[0]
            dy = event.y - self._pan_start[1]
            self._pan_start = (event.x, event.y)
            xlim = self._ax.get_xlim()
            ylim = self._ax.get_ylim()
            scale_x = (xlim[1] - xlim[0]) / self._fig.bbox.width
            scale_y = (ylim[1] - ylim[0]) / self._fig.bbox.height
            self._ax.set_xlim(xlim[0] - dx * scale_x, xlim[1] - dx * scale_x)
            self._ax.set_ylim(ylim[0] - dy * scale_y, ylim[1] - dy * scale_y)
            self._fig.canvas.draw_idle()
            return

        # --- waypoint drag ---
        if self._dragging is None:
            return
        if event.xdata is None or event.ydata is None:
            return
        idx = self._dragging
        with self._lock:
            w = self._waypoints[idx]
            self._waypoints[idx] = replace(
                w, position=(float(event.xdata), float(event.ydata), 0.0)
            )
        self._redraw()
        self._publish_markers()

    def _on_release(self, event):
        if self._panning:
            self._panning = False
            self._pan_start = None
            return
        if self._dragging is not None:
            w = self._waypoints[self._dragging]
            self._node.get_logger().info(
                f"Moved [{self._dragging}] → ({w.position[0]:.3f}, {w.position[1]:.3f})"
            )
        self._dragging = None
        self._drag_start = None

    def _on_scroll(self, event):
        # scroll on a waypoint → rotate it
        idx = self._find_waypoint(event)
        if idx is not None and idx not in (0, len(self._waypoints) - 1):
            self._selected = idx
            delta = 5.0 if event.button == "up" else -5.0
            self._rotate_selected(delta)
            return

        # scroll on background → zoom (centered on cursor if over axes, else center of view)
        factor = 0.85 if event.button == "up" else 1.0 / 0.85
        xlim = self._ax.get_xlim()
        ylim = self._ax.get_ylim()
        cx = event.xdata if event.xdata is not None else (xlim[0] + xlim[1]) / 2.0
        cy = event.ydata if event.ydata is not None else (ylim[0] + ylim[1]) / 2.0
        self._ax.set_xlim(cx - (cx - xlim[0]) * factor,
                          cx + (xlim[1] - cx) * factor)
        self._ax.set_ylim(cy - (cy - ylim[0]) * factor,
                          cy + (ylim[1] - cy) * factor)
        self._fig.canvas.draw_idle()

    # ── keyboard events ────────────────────────────────────────────────

    def _on_key(self, event):
        if event.key == "ctrl+s":
            self._save()
        elif event.key == "ctrl+z":
            self._undo()
        elif event.key in ("r", "R") and self._selected is not None:
            delta = 15.0 if event.key == "r" else -15.0
            self._rotate_selected(delta)
        elif event.key in ("delete", "escape"):
            self._selected = None
            self._redraw()
        # --- zoom ---
        elif event.key in ("+", "="):
            self._zoom(0.85)
        elif event.key == "-":
            self._zoom(1.0 / 0.85)
        # --- pan ---
        elif event.key == "left":
            self._pan(0.15, 0)
        elif event.key == "right":
            self._pan(-0.15, 0)
        elif event.key == "up":
            self._pan(0, -0.15)
        elif event.key == "down":
            self._pan(0, 0.15)

    def _zoom(self, factor):
        xlim = self._ax.get_xlim()
        ylim = self._ax.get_ylim()
        cx = (xlim[0] + xlim[1]) / 2
        cy = (ylim[0] + ylim[1]) / 2
        self._ax.set_xlim(cx - (cx - xlim[0]) * factor,
                          cx + (xlim[1] - cx) * factor)
        self._ax.set_ylim(cy - (cy - ylim[0]) * factor,
                          cy + (ylim[1] - cy) * factor)
        self._fig.canvas.draw_idle()

    def _pan(self, dx_frac, dy_frac):
        xlim = self._ax.get_xlim()
        ylim = self._ax.get_ylim()
        dx = (xlim[1] - xlim[0]) * dx_frac
        dy = (ylim[1] - ylim[0]) * dy_frac
        self._ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
        self._ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
        self._fig.canvas.draw_idle()

    # ── operations ─────────────────────────────────────────────────────

    def _rotate_selected(self, delta_deg):
        if self._selected is None:
            return
        idx = self._selected
        if idx in (0, len(self._waypoints) - 1):
            return
        self._push_history()
        w = self._waypoints[idx]
        yaw = _yaw_from_quaternion(w.orientation) + math.radians(delta_deg)
        with self._lock:
            self._waypoints[idx] = replace(w, orientation=_yaw_quaternion(yaw))
        self._redraw()
        self._publish_markers()
        self._node.get_logger().info(
            f"Rotated [{idx}] to {math.degrees(yaw):.1f}°"
        )

    def _push_history(self):
        self._history.append(tuple(self._waypoints))
        if len(self._history) > 100:
            del self._history[0]

    def _undo(self):
        if not self._history:
            self._node.get_logger().info("Nothing to undo")
            return
        with self._lock:
            self._waypoints = list(self._history.pop())
        self._selected = None
        self._dragging = None
        self._redraw()
        self._publish_markers()
        self._node.get_logger().info("Undo — restored previous positions")

    def _save(self):
        with self._lock:
            validate_waypoints(self._waypoints)
            destination = self._path.resolve()
            write_waypoints_atomic(destination, self._template, self._waypoints)
            self._path = destination
            self._template, loaded = load_waypoint_document(destination)
            self._waypoints = list(loaded)
        self._history.clear()
        self._selected = None
        self._dragging = None
        self._redraw()
        self._publish_markers()
        self._node.get_logger().info("✓ Waypoints saved atomically")

    # ── public API ─────────────────────────────────────────────────────

    def run(self):
        """Blocking call — show the matplotlib window and run the ROS2 spinner."""
        self._ros_publisher = self._node.create_publisher(
            MarkerArray,
            "/smartcar/waypoint_editor/markers",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self._publish_markers()

        spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        spin_thread.start()

        try:
            plt.show()
        finally:
            self._node.destroy_node()


# ── entry point ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Node("waypoint_drag_editor")

    pkg_share = get_package_share_directory
    default_wp = str(
        Path(pkg_share("smartcar_nav2")) / "config" / "waypoints" / "default_waypoints.yaml"
    )
    default_geom = str(
        Path(pkg_share("smartcar_tools")) / "config" / "routes" / "field_geometry.yaml"
    )
    node.declare_parameter("waypoints_file", default_wp)
    node.declare_parameter("geometry_file", default_geom)

    wp_path = Path(str(node.get_parameter("waypoints_file").value)).expanduser()
    geom_path = Path(str(node.get_parameter("geometry_file").value)).expanduser()

    editor = DragEditor(node, wp_path, geom_path)
    try:
        editor.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
