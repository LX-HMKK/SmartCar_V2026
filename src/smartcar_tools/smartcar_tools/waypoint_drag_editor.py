"""Interactive Chinese-language segment route editor using matplotlib.

Mouse:
  - drag a waypoint circle to move it
  - scroll a QR/VLM/P waypoint to rotate it (±5 degrees per tick)
  - click a waypoint to select it (highlighted ring)
  - click empty space to deselect

Keyboard:
  - Ctrl+S          save to YAML
  - Ctrl+Z          undo last move
  - R / Shift+R     rotate selected QR/VLM/P waypoint ±15 degrees
  - Delete / Escape deselect

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
matplotlib.use("Qt5Agg")
from matplotlib import font_manager
try:
    import matplotlib.pyplot as plt
except ImportError:
    # Source-level contracts import the editor in a display-less environment.
    # Keep Qt as the desktop default while allowing non-interactive checks to
    # inspect the route model without opening an editor window.
    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
from matplotlib.widgets import Button, RadioButtons, TextBox
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from smartcar_task.route_geometry import RouteGeometryError, materialize_free_yaws
from smartcar_task.waypoints import (
    Waypoint,
    is_heading_locked,
    is_zero_quaternion,
    load_waypoint_document,
    validate_waypoints,
    write_waypoints_atomic,
)
from smartcar_tools.field_reference import (
    Bounds2D,
    Point2D,
    load_field_reference,
)
from smartcar_task.planning_segments import (
    PlanningSegment,
    PlanningSegmentError,
    load_planning_segments,
    materialize_route,
    planning_segments_document,
    validate_planning_segments,
)
from smartcar_tools.route_preflight import RoutePreflight, preflight_route
from visualization_msgs.msg import Marker, MarkerArray

# ── colours ────────────────────────────────────────────────────────────────
TASK_COLORS = {
    "start":    "#33D94D",
    "qr":       "#408CFF",
    "vlm":      "#F28C33",
    "return":   "#B34DE6",
    "nav":      "#5AB8E6",
    "via":      "#AAB4BF",
}
DEFAULT_COLOR = "#CCCCCC"
SELECTED_EDGE = "#FFFF33"
FIELD_BG = "#1A1C20"
GRID_COLOR = "#2A2C30"
LINE_COLOR = "#26D9F2"
PANEL_BG = "#20242A"
PANEL_INPUT = "#2C323B"
PANEL_TEXT = "#E7EBF0"
PATH_OK = "#32D583"
PATH_FAILED = "#F97066"
PATH_PENDING = "#A6B0BF"
DRAG_THRESHOLD_M = 0.01

DIRECTION_LABELS = {
    "forward": "正向",
    "reverse": "倒向",
}
DIRECTION_CHOICES = {
    "正向（前进）": "forward",
    "倒向（倒车）": "reverse",
}
ENDPOINT_LABELS = {
    "start": "起点",
    "end": "终点",
}
EMPTY_THROUGH_LABEL = "（无）"


def _configure_cjk_font():
    """Use the native Ubuntu CJK font when it is available."""
    families = ["Microsoft YaHei", "Noto Sans CJK SC", "WenQuanYi Micro Hei", "DejaVu Sans"]
    candidates = (
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf"),
        Path("/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"),
    )
    for candidate in candidates:
        if not candidate.is_file():
            continue
        try:
            font_manager.fontManager.addfont(str(candidate))
            family = font_manager.FontProperties(fname=str(candidate)).get_name()
        except (OSError, RuntimeError):
            continue
        families.insert(0, family)
        break
    matplotlib.rcParams["font.family"] = "sans-serif"
    matplotlib.rcParams["font.sans-serif"] = families
    matplotlib.rcParams["axes.unicode_minus"] = False


def _direction_label(direction: str) -> str:
    return DIRECTION_LABELS.get(direction, direction)


def _endpoint_label(target: str) -> str:
    return ENDPOINT_LABELS[target]


def _preflight_message(message: str) -> str:
    """Convert offline geometric-preflight diagnostics into UI-facing Chinese."""
    continuous_suffix = (
        ": no collision-free minimum-radius route in continuous ThroughPoses action"
    )
    if message.endswith(continuous_suffix):
        leg = message.removesuffix(continuous_suffix)
        return f"{leg}：连续途经路线未找到满足场地边界与最小转弯半径的候选"
    if message.endswith(": no collision-free minimum-radius route"):
        leg = message.removesuffix(": no collision-free minimum-radius route")
        return f"{leg}：离线几何预检未找到满足场地边界与最小转弯半径的候选"
    if message.endswith(": position-only; preflight leaves heading free"):
        waypoint_id = message.removesuffix(
            ": position-only; preflight leaves heading free"
        )
        return f"{waypoint_id}：位置约束，预检不施加朝向"
    return message


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
        self._segments = list(load_planning_segments(self._template, self._waypoints))
        self._history: list[tuple[tuple, tuple[PlanningSegment, ...]]] = []
        self._selected: int | None = None
        self._dragging: int | None = None
        self._drag_start = None
        self._drag_moved = False
        self._drag_preview_position = None
        self._panning = False
        self._pan_start = None
        self._selected_segment = 0
        self._selected_through: int | None = None
        self._pick_target: str | None = None
        self._adding_through = False
        self._add_through_button: Button | None = None
        self._status_text = None
        self._preflight: RoutePreflight | None = None
        self._route_definition_valid: bool | None = None
        self._route_status = "尚未运行离线几何预检：可先编辑，完成后点击“几何预检”"
        self._lock = threading.Lock()

        # ── load field reference (same data as RViz overlay) ────────────
        self._field_ref = load_field_reference(geometry_path)

        # ── build matplotlib figure ────────────────────────────────────
        _configure_cjk_font()
        matplotlib.rcParams["keymap.save"] = []
        matplotlib.rcParams["keymap.pan"] = []
        matplotlib.rcParams["keymap.zoom"] = []
        matplotlib.rcParams["toolbar"] = "toolbar2"

        field = self._field_ref.field
        margin = 0.4
        data_w = field.width + 2 * margin
        data_h = field.height + 2 * margin

        # Keep the field unframed and reserve a fixed right-hand route panel.
        self._fig = plt.figure(figsize=(15.2, max(8.8, 11.0 * data_h / data_w)))
        self._ax = self._fig.add_axes([0.055, 0.105, 0.585, 0.84])
        self._fig.canvas.manager.set_window_title("航点分段编辑器（离线几何预检） - SmartCar")
        self._panel_axes = []
        self._panel_widgets = []

        self._ax.set_facecolor(FIELD_BG)
        self._fig.patch.set_facecolor(FIELD_BG)

        self._ax.set_xlim(field.x_min - margin, field.x_max + margin)
        self._ax.set_ylim(field.y_min - margin, field.y_max + margin)
        self._ax.set_aspect("equal")
        self._ax.grid(True, color=GRID_COLOR, alpha=0.5, linewidth=0.5)
        self._ax.set_xlabel("X（米，odom_combined）", color="#888")
        self._ax.set_ylabel("Y（米）", color="#888")
        self._ax.tick_params(colors="#888")

        self._draw_field_ref()

        # controls hint
        self._hint = self._ax.text(
            0.5, -0.06,
            "灰色虚线：航点参考约束（非 Nav2 路径）  |  彩色线：离线几何预检（非 Nav2 路径）\n"
            "左键拖动：移动点  |  滚轮：调朝向/缩放  |  右键拖动：平移  |  "
            "右侧：分段与约束  |  新增途经点后点击场地  |  点击“几何预检”：检查场地边界与最小转弯半径  |  Ctrl+S：保存",
            transform=self._ax.transAxes, fontsize=7, color="#666",
            ha="center", va="top",
        )

        # artists we'll update
        self._scatter = None
        self._arrows = {}
        self._line = None
        self._segment_artists = []
        self._labels = []
        self._sel_ring = None
        self._blit_background = None
        self._drag_preview = self._ax.scatter(
            [],
            [],
            s=150,
            facecolors="none",
            edgecolors=SELECTED_EDGE,
            linewidths=3,
            zorder=9,
            animated=True,
            visible=False,
        )

        self._mark_route_changed(redraw=False, rebuild_panel=False)
        self._redraw()
        self._build_route_panel()

        # ── connect events ─────────────────────────────────────────────
        self._fig.canvas.mpl_connect("button_press_event", self._on_press)
        self._fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self._fig.canvas.mpl_connect("button_release_event", self._on_release)
        self._fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self._fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._fig.canvas.mpl_connect("draw_event", self._on_canvas_draw)
        self._fig.canvas.mpl_connect("resize_event", self._on_canvas_resize)

        self._node.get_logger().info(
            f"路径分段编辑器已就绪：{len(self._waypoints)} 个航点，"
            f"{len(self._segments)} 个规划分段，场地参考：{geometry_path}。"
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
            "P（起点）", (ref.p_origin.x, ref.p_origin.y + 0.18),
            fontsize=8, color="#33FF66", ha="center", va="bottom", zorder=2,
        )

        # --- task point ---
        self._ax.scatter(
            [ref.task_point.x], [ref.task_point.y],
            s=80, c="#FF661A", edgecolors="#FF661A", linewidths=0,
            zorder=2,
        )
        self._ax.annotate(
            "任务点", (ref.task_point.x, ref.task_point.y + 0.18),
            fontsize=8, color="#FF8C42", ha="center", va="bottom", zorder=2,
        )

        # --- zone labels ---
        for label in ref.labels:
            self._ax.annotate(
                label.text, (label.position.x, label.position.y),
                fontsize=9, color="#F2F2F2E6", ha="center", va="center",
                zorder=1, alpha=0.8,
            )

    # ── route-segment panel and model ──────────────────────────────────

    def _current_segment(self):
        if not self._segments:
            return None
        self._selected_segment = max(
            0, min(self._selected_segment, len(self._segments) - 1)
        )
        return self._segments[self._selected_segment]

    def _new_panel_axis(self, rect):
        axis = self._fig.add_axes(rect)
        axis.set_facecolor(PANEL_BG)
        self._panel_axes.append(axis)
        return axis

    def _style_button(self, button):
        button.label.set_color(PANEL_TEXT)
        button.label.set_fontsize(8)
        self._panel_widgets.append(button)
        return button

    def _style_radio(self, radio):
        for label in radio.labels:
            label.set_color(PANEL_TEXT)
            label.set_fontsize(8)
        for circle in getattr(radio, "circles", ()):
            circle.set_edgecolor("#8D99A8")
        self._panel_widgets.append(radio)
        return radio

    def _route_ids_for_display(self):
        route = self._display_route()
        return [waypoint.id for waypoint in route]

    @staticmethod
    def _uses_authored_heading(waypoint):
        return is_heading_locked(waypoint)

    def _display_route(self):
        """Return the ordered route with position-only transit points."""
        try:
            route = materialize_route(self._waypoints, self._segments)
        except PlanningSegmentError:
            route = tuple(self._waypoints)
        try:
            return materialize_free_yaws(route)
        except RouteGeometryError as error:
            # Never fall back to a user's ignored transit yaw after a bad edit.
            self._node.get_logger().error(
                f"无法准备位置约束路线：{error}"
            )
            return tuple(
                waypoint
                if self._uses_authored_heading(waypoint)
                else replace(waypoint, orientation=(0.0, 0.0, 0.0, 0.0))
                for waypoint in route
            )

    def _display_waypoints_by_id(self):
        return {waypoint.id: waypoint for waypoint in self._display_route()}

    def _display_waypoint(self, waypoint):
        display = self._display_waypoints_by_id().get(waypoint.id)
        if display is not None:
            return display
        if self._uses_authored_heading(waypoint):
            return waypoint
        return replace(waypoint, orientation=(0.0, 0.0, 0.0, 0.0))

    @staticmethod
    def _segment_panel_label(index, segment):
        return f"第 {index + 1} 段  [{_direction_label(segment.direction)}]"

    def _set_route_status(self, message):
        self._route_status = str(message)
        self._node.get_logger().info(self._route_status)
        status_text = self._status_text
        if status_text is not None:
            status_text.set_text(self._route_status)
            status_text.set_color(self._route_status_color())
            self._fig.canvas.draw_idle()

    def _route_status_color(self):
        if self._preflight is not None and self._preflight.feasible:
            return PATH_OK
        if self._preflight is not None or self._route_definition_valid is False:
            return PATH_FAILED
        return PATH_PENDING

    def _refresh_add_through_button(self, redraw=True):
        """Update the one-click placement mode without rebuilding its widget."""
        button = self._add_through_button
        if button is None:
            return
        button.label.set_text(
            "点击场地放置" if self._adding_through else "新增途经点"
        )
        color = "#176F4D" if self._adding_through else PANEL_INPUT
        button.color = color
        button.ax.set_facecolor(color)
        if redraw:
            self._fig.canvas.draw_idle()

    def _mark_route_changed(self, redraw=True, rebuild_panel=False):
        """Refresh route invariants without running the offline geometric preflight."""
        self._preflight = None
        try:
            checked = validate_planning_segments(self._segments, self._waypoints)
            route = materialize_route(self._waypoints, checked)
            validate_waypoints(route)
        except (PlanningSegmentError, ValueError) as error:
            self._route_definition_valid = False
            self._node.get_logger().error(f"Route validation error: {error}")
            self._set_route_status(f"路线定义有误：{error}")
        else:
            self._route_definition_valid = True
            self._set_route_status(
                "路线已修改：点击“几何预检”检查场地边界与最小转弯半径；"
                "结果不等同于 Nav2 实际路径"
            )
        if redraw:
            self._redraw()
        if rebuild_panel:
            self._build_route_panel()

    def _recheck_route(self, redraw=True, rebuild_panel=False):
        """Run the offline geometric preflight; it does not execute Nav2 planning."""
        try:
            checked = validate_planning_segments(self._segments, self._waypoints)
            route = materialize_route(self._waypoints, checked)
            validate_waypoints(route)
            self._route_definition_valid = True
            self._preflight = preflight_route(
                self._field_ref, self._waypoints, checked
            )
            failed = [segment for segment in self._preflight.segments if not segment.feasible]
            if failed:
                self._set_route_status(
                    "离线几何预检未通过（非 Nav2 结果）："
                    + _preflight_message(failed[0].message)
                )
            else:
                total = sum(segment.length_m for segment in self._preflight.segments)
                warning = (
                    "\n提示：" + _preflight_message(self._preflight.warnings[0])
                    if self._preflight.warnings else ""
                )
                self._set_route_status(
                    f"离线几何预检通过：{len(checked)} 编辑段，总长 {total:.1f} 米"
                    "（同向途经点按连续动作检查）"
                    "（仅静态几何，不代表 Nav2 实际路径）"
                    + warning
                )
        except (PlanningSegmentError, ValueError) as error:
            self._preflight = None
            self._route_definition_valid = False
            self._node.get_logger().error(f"Route validation error: {error}")
            self._set_route_status(f"路线定义有误：{error}")
        if redraw:
            self._redraw()
        if rebuild_panel:
            self._build_route_panel()

    def _build_route_panel(self):
        self._invalidate_fast_canvas()
        self._add_through_button = None
        self._status_text = None
        for axis in self._panel_axes:
            axis.remove()
        self._panel_axes.clear()
        self._panel_widgets.clear()

        title = self._new_panel_axis([0.675, 0.935, 0.305, 0.04])
        title.axis("off")
        title.text(
            0.0, 0.5, "路径分段", color=PANEL_TEXT,
            fontsize=10, fontweight="bold", va="center",
        )

        labels = [
            self._segment_panel_label(index, segment)
            for index, segment in enumerate(self._segments)
        ] or ["（暂无分段）"]
        segment_axis = self._new_panel_axis([0.675, 0.755, 0.305, 0.16])
        segment_radio = self._style_radio(RadioButtons(
            segment_axis,
            labels,
            active=min(self._selected_segment, max(0, len(labels) - 1)),
            activecolor="#4F9DFF",
        ))
        segment_radio.on_clicked(self._on_segment_selected)

        split_axis = self._new_panel_axis([0.675, 0.705, 0.145, 0.035])
        self._style_button(Button(
            split_axis, "在选中途经点拆分", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(self._split_segment)
        delete_axis = self._new_panel_axis([0.835, 0.705, 0.145, 0.035])
        self._style_button(Button(
            delete_axis, "合并/删除", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(self._delete_segment)

        segment = self._current_segment()
        if segment is None:
            self._fig.canvas.draw_idle()
            return

        direction_label = self._new_panel_axis([0.675, 0.665, 0.305, 0.025])
        direction_label.axis("off")
        direction_label.text(0.0, 0.4, "行驶方向", color="#AEB8C5", fontsize=8)
        direction_axis = self._new_panel_axis([0.675, 0.585, 0.305, 0.075])
        direction_radio = self._style_radio(RadioButtons(
            direction_axis,
            tuple(DIRECTION_CHOICES),
            active=0 if segment.direction == "forward" else 1,
            activecolor="#4F9DFF",
        ))
        direction_radio.on_clicked(self._on_direction_selected)

        start_axis = self._new_panel_axis([0.675, 0.535, 0.215, 0.035])
        start_box = TextBox(
            start_axis, "起点 ", initial=segment.start_id,
            color=PANEL_INPUT, hovercolor="#3C4654", label_pad=0.02,
        )
        start_box.label.set_color(PANEL_TEXT)
        start_box.text_disp.set_color(PANEL_TEXT)
        start_box.on_submit(lambda value: self._set_segment_endpoint("start", value))
        self._panel_widgets.append(start_box)
        start_pick_axis = self._new_panel_axis([0.905, 0.535, 0.075, 0.035])
        self._style_button(Button(
            start_pick_axis, "点选", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(lambda _event: self._set_pick_target("start"))

        end_axis = self._new_panel_axis([0.675, 0.485, 0.215, 0.035])
        end_box = TextBox(
            end_axis, "终点 ", initial=segment.end_id,
            color=PANEL_INPUT, hovercolor="#3C4654", label_pad=0.02,
        )
        end_box.label.set_color(PANEL_TEXT)
        end_box.text_disp.set_color(PANEL_TEXT)
        end_box.on_submit(lambda value: self._set_segment_endpoint("end", value))
        self._panel_widgets.append(end_box)
        end_pick_axis = self._new_panel_axis([0.905, 0.485, 0.075, 0.035])
        self._style_button(Button(
            end_pick_axis, "点选", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(lambda _event: self._set_pick_target("end"))

        start_waypoint = next(
            (item for item in self._waypoints if item.id == segment.start_id), None
        )
        end_waypoint = next(
            (item for item in self._waypoints if item.id == segment.end_id), None
        )
        start_heading_label = (
            "起点朝向 "
            if start_waypoint is not None and self._uses_authored_heading(start_waypoint)
            else "起点位置约束 "
        )
        end_heading_label = (
            "终点朝向 "
            if end_waypoint is not None and self._uses_authored_heading(end_waypoint)
            else "终点位置约束 "
        )
        start_yaw_axis = self._new_panel_axis([0.675, 0.435, 0.145, 0.035])
        if start_waypoint is not None and self._uses_authored_heading(start_waypoint):
            start_yaw = TextBox(
                start_yaw_axis, start_heading_label,
                initial=f"{self._waypoint_yaw_degrees(segment.start_id):.1f}",
                color=PANEL_INPUT, hovercolor="#3C4654", label_pad=0.02,
            )
            start_yaw.label.set_color(PANEL_TEXT)
            start_yaw.text_disp.set_color(PANEL_TEXT)
            start_yaw.on_submit(lambda value: self._set_endpoint_yaw("start", value))
            self._panel_widgets.append(start_yaw)
        else:
            start_yaw_axis.axis("off")
            start_yaw_axis.text(0.0, 0.5, start_heading_label, color="#AEB8C5", fontsize=7)
        end_yaw_axis = self._new_panel_axis([0.835, 0.435, 0.145, 0.035])
        if end_waypoint is not None and self._uses_authored_heading(end_waypoint):
            end_yaw = TextBox(
                end_yaw_axis, end_heading_label,
                initial=f"{self._waypoint_yaw_degrees(segment.end_id):.1f}",
                color=PANEL_INPUT, hovercolor="#3C4654", label_pad=0.02,
            )
            end_yaw.label.set_color(PANEL_TEXT)
            end_yaw.text_disp.set_color(PANEL_TEXT)
            end_yaw.on_submit(lambda value: self._set_endpoint_yaw("end", value))
            self._panel_widgets.append(end_yaw)
        else:
            end_yaw_axis.axis("off")
            end_yaw_axis.text(0.0, 0.5, end_heading_label, color="#AEB8C5", fontsize=7)

        through_label = self._new_panel_axis([0.675, 0.415, 0.305, 0.025])
        through_label.axis("off")
        through_label.text(
            0.0, 0.4, "按顺序途经点", color="#AEB8C5", fontsize=8
        )
        through_labels = list(segment.through_ids) or [EMPTY_THROUGH_LABEL]
        through_axis = self._new_panel_axis([0.675, 0.280, 0.305, 0.125])
        through_radio = self._style_radio(RadioButtons(
            through_axis,
            through_labels,
            active=(
                min(self._selected_through, len(through_labels) - 1)
                if self._selected_through is not None else 0
            ),
            activecolor="#4F9DFF",
        ))
        through_radio.on_clicked(self._on_through_selected)

        new_through_axis = self._new_panel_axis([0.675, 0.235, 0.305, 0.030])
        new_through_button = self._style_button(Button(
            new_through_axis, "新增途经点",
            color=PANEL_INPUT, hovercolor="#3C4654"
        ))
        self._add_through_button = new_through_button
        self._refresh_add_through_button(redraw=False)
        new_through_button.on_clicked(self._begin_add_through)

        add_axis = self._new_panel_axis([0.675, 0.190, 0.145, 0.035])
        self._style_button(Button(
            add_axis, "加入已有点", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(self._add_selected_through)
        remove_axis = self._new_panel_axis([0.835, 0.190, 0.145, 0.035])
        self._style_button(Button(
            remove_axis, "删除途经点", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(self._remove_selected_through)
        up_axis = self._new_panel_axis([0.675, 0.145, 0.145, 0.035])
        self._style_button(Button(
            up_axis, "上移", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(lambda _event: self._move_selected_through(-1))
        down_axis = self._new_panel_axis([0.835, 0.145, 0.145, 0.035])
        self._style_button(Button(
            down_axis, "下移", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(lambda _event: self._move_selected_through(1))

        heading_note = self._new_panel_axis([0.675, 0.100, 0.305, 0.045])
        heading_note.axis("off")
        heading_note.text(
            0.0,
            0.75,
            "非 P/QR/VLM 点：位置约束；同向途经点连续规划，运行时由代价地图选朝向",
            color="#AEB8C5",
            fontsize=6.8,
            va="top",
            wrap=True,
        )

        status = self._new_panel_axis([0.675, 0.050, 0.305, 0.040])
        status.axis("off")
        self._status_text = status.text(
            0.0, 0.98, self._route_status, color=self._route_status_color(),
            fontsize=6.8, va="top", wrap=True,
        )
        recheck_axis = self._new_panel_axis([0.675, 0.005, 0.145, 0.035])
        self._style_button(Button(
            recheck_axis, "几何预检", color=PANEL_INPUT, hovercolor="#3C4654"
        )).on_clicked(lambda _event: self._recheck_route(rebuild_panel=True))
        save_axis = self._new_panel_axis([0.835, 0.005, 0.145, 0.035])
        self._style_button(Button(
            save_axis, "保存路线", color="#176F4D", hovercolor="#248D63"
        )).on_clicked(lambda _event: self._save())
        self._fig.canvas.draw_idle()

    def _on_segment_selected(self, label):
        for index, segment in enumerate(self._segments):
            if label == self._segment_panel_label(index, segment):
                self._selected_segment = index
                self._selected_through = None
                self._pick_target = None
                self._build_route_panel()
                self._redraw()
                return

    def _on_direction_selected(self, direction):
        segment = self._current_segment()
        direction = DIRECTION_CHOICES.get(direction)
        if direction is None:
            return
        if segment is None or direction == segment.direction:
            return
        self._push_history()
        self._segments[self._selected_segment] = replace(segment, direction=direction)
        if direction == "forward":
            # reverse_handoff is a dedicated reverse-only terminal contract.
            # A direction toggle must not leave this incompatible profile
            # behind.
            for index, waypoint in enumerate(self._waypoints):
                if (
                    waypoint.id == segment.end_id
                    and waypoint.goal_profile == "reverse_handoff"
                ):
                    self._waypoints[index] = replace(
                        waypoint, goal_profile="standard"
                    )
        self._mark_route_changed(rebuild_panel=True)

    def _set_pick_target(self, target):
        self._pick_target = target
        self._set_route_status(f"请在场地上点选本段的{_endpoint_label(target)}")
        self._build_route_panel()

    def _set_segment_endpoint(self, target, waypoint_id):
        segment = self._current_segment()
        waypoint_id = str(waypoint_id).strip()
        if segment is None or not waypoint_id:
            return
        self._push_history()
        field = "start_id" if target == "start" else "end_id"
        self._segments[self._selected_segment] = replace(segment, **{field: waypoint_id})
        self._pick_target = None
        self._selected_through = None
        self._mark_route_changed(rebuild_panel=True)

    def _waypoint_yaw_degrees(self, waypoint_id):
        waypoint = next(
            (item for item in self._waypoints if item.id == waypoint_id), None
        )
        if waypoint is None:
            return 0.0
        display = self._display_waypoint(waypoint)
        return math.degrees(_yaw_from_quaternion(display.orientation))

    def _set_endpoint_yaw(self, target, value):
        segment = self._current_segment()
        if segment is None:
            return
        try:
            yaw_deg = float(str(value).strip())
        except ValueError:
            self._set_route_status("朝向必须是有限的角度数值（单位：度）")
            self._build_route_panel()
            return
        if not math.isfinite(yaw_deg):
            self._set_route_status("朝向必须是有限的角度数值（单位：度）")
            self._build_route_panel()
            return
        waypoint_id = segment.start_id if target == "start" else segment.end_id
        index = next(
            (index for index, item in enumerate(self._waypoints) if item.id == waypoint_id),
            None,
        )
        if index is None:
            self._set_route_status(
                f"找不到{_endpoint_label(target)}航点：{waypoint_id!r}"
            )
            self._build_route_panel()
            return
        if index == 0:
            self._set_route_status("P 起点朝向固定为 +X")
            self._build_route_panel()
            return
        waypoint = self._waypoints[index]
        if not self._uses_authored_heading(waypoint):
            self._set_route_status(
                f"{waypoint.id} 为位置约束；运行时由规划器选择朝向"
            )
            self._build_route_panel()
            return
        self._push_history()
        with self._lock:
            self._waypoints[index] = replace(
                waypoint, orientation=_yaw_quaternion(math.radians(yaw_deg))
        )
        self._selected = index
        self._mark_route_changed(rebuild_panel=True)
        self._publish_markers()

    def _begin_add_through(self, _event):
        """Enter a one-click mode for adding a new route-constraint point."""
        if self._current_segment() is None:
            self._set_route_status("请先选择要添加途经点的分段")
            return
        if self._adding_through:
            self._adding_through = False
            self._set_route_status("已取消新增途经点")
        else:
            self._pick_target = None
            self._adding_through = True
            self._set_route_status("请在场地空白处点击新途经点的位置；Esc 取消")
        self._refresh_add_through_button()

    def _next_via_id(self):
        existing = {waypoint.id for waypoint in self._waypoints}
        index = 1
        while f"via_{index}" in existing:
            index += 1
        return f"via_{index}"

    def _create_through_at(self, x, y):
        segment = self._current_segment()
        field = self._field_ref.field
        if segment is None:
            return
        if not (field.x_min <= x <= field.x_max and field.y_min <= y <= field.y_max):
            self._set_route_status("新途经点必须落在比赛场地内")
            self._refresh_add_through_button()
            return
        waypoint_id = self._next_via_id()
        self._push_history()
        waypoint = Waypoint(
            frame_id=self._waypoints[0].frame_id,
            position=(float(x), float(y), 0.0),
            orientation=(0.0, 0.0, 0.0, 0.0),
            task="via",
            direction=segment.direction,
            id=waypoint_id,
            goal_profile="standard",
        )
        # The semantic document reserves its final entry for the P return
        # point.  Route order itself comes from planning_segments, so inserting
        # before that terminal entry preserves both contracts until save
        # materializes the complete ordered route.
        insertion_index = len(self._waypoints) - 1
        with self._lock:
            self._waypoints.insert(insertion_index, waypoint)
        through = list(segment.through_ids)
        insert_at = (
            self._selected_through + 1
            if self._selected_through is not None
            else len(through)
        )
        through.insert(insert_at, waypoint_id)
        self._segments[self._selected_segment] = replace(
            segment, through_ids=tuple(through)
        )
        self._selected = insertion_index
        self._selected_through = insert_at
        self._adding_through = False
        self._set_route_status(f"已新增无朝向途经点 {waypoint_id}")
        self._mark_route_changed(rebuild_panel=True)
        self._publish_markers()

    def _on_through_selected(self, label):
        segment = self._current_segment()
        if segment is None or label == EMPTY_THROUGH_LABEL:
            self._selected_through = None
            return
        self._selected_through = segment.through_ids.index(label)

    def _add_selected_through(self, _event):
        segment = self._current_segment()
        if segment is None or self._selected is None:
            self._set_route_status("请先在场地上选中要加入的途经点")
            self._build_route_panel()
            return
        waypoint_id = self._waypoints[self._selected].id
        if waypoint_id in {segment.start_id, segment.end_id}:
            self._set_route_status("本段起点或终点不能同时作为途经点")
            self._build_route_panel()
            return
        for candidate in self._segments:
            if waypoint_id in {candidate.start_id, candidate.end_id}:
                self._set_route_status(
                    "该点正在作为其他分段的端点，请先修改该分段端点"
                )
                self._build_route_panel()
                return
        self._push_history()
        for index, candidate in enumerate(self._segments):
            if waypoint_id in candidate.through_ids:
                retained = tuple(item for item in candidate.through_ids if item != waypoint_id)
                self._segments[index] = replace(candidate, through_ids=retained)
        segment = self._current_segment()
        self._segments[self._selected_segment] = replace(
            segment, through_ids=(*segment.through_ids, waypoint_id)
        )
        self._selected_through = len(segment.through_ids)
        self._mark_route_changed(rebuild_panel=True)

    def _remove_selected_through(self, _event):
        segment = self._current_segment()
        if (
            segment is None or self._selected_through is None
            or self._selected_through >= len(segment.through_ids)
        ):
            self._set_route_status("请先从途经点列表中选中要删除的点")
            self._build_route_panel()
            return
        removed = segment.through_ids[self._selected_through]
        if any(
            removed in {candidate.start_id, candidate.end_id}
            for candidate in self._segments
        ):
            self._set_route_status(
                f"{removed} 正在作为分段端点，请先合并或修改分段后再删除"
            )
            self._build_route_panel()
            return

        self._push_history()
        # A route constraint is one waypoint model, not a panel-only entry.
        # Remove every through reference as well as the waypoint so the route
        # remains complete for validation, redraw, persistence, and RViz.
        self._segments = [
            replace(
                candidate,
                through_ids=tuple(
                    waypoint_id
                    for waypoint_id in candidate.through_ids
                    if waypoint_id != removed
                ),
            )
            for candidate in self._segments
        ]
        waypoint_index = next(
            (
                index
                for index, waypoint in enumerate(self._waypoints)
                if waypoint.id == removed
            ),
            None,
        )
        if waypoint_index is not None:
            with self._lock:
                del self._waypoints[waypoint_index]
            if self._selected == waypoint_index:
                self._selected = None
            elif self._selected is not None and self._selected > waypoint_index:
                self._selected -= 1
        self._selected_through = None
        self._set_route_status(f"已删除途经点 {removed}")
        self._mark_route_changed(rebuild_panel=True)
        self._publish_markers()

    def _move_selected_through(self, offset):
        segment = self._current_segment()
        if (
            segment is None or self._selected_through is None
            or not 0 <= self._selected_through < len(segment.through_ids)
        ):
            self._set_route_status("请先选中要调整顺序的途经点")
            self._build_route_panel()
            return
        destination = self._selected_through + offset
        if not 0 <= destination < len(segment.through_ids):
            return
        self._push_history()
        through = list(segment.through_ids)
        through[self._selected_through], through[destination] = (
            through[destination], through[self._selected_through]
        )
        self._segments[self._selected_segment] = replace(
            segment, through_ids=tuple(through)
        )
        self._selected_through = destination
        self._mark_route_changed(rebuild_panel=True)

    def _next_segment_id(self):
        existing = {segment.id for segment in self._segments}
        index = 1
        while f"segment_{index}" in existing:
            index += 1
        return f"segment_{index}"

    def _split_segment(self, _event):
        segment = self._current_segment()
        if segment is None or self._selected is None:
            self._set_route_status("请先在本段的途经点中选一个点，再拆分")
            self._build_route_panel()
            return
        waypoint_id = self._waypoints[self._selected].id
        if waypoint_id not in segment.through_ids:
            self._set_route_status("拆分位置必须是当前分段的途经点")
            self._build_route_panel()
            return
        split_index = segment.through_ids.index(waypoint_id)
        self._push_history()
        first = replace(
            segment,
            end_id=waypoint_id,
            through_ids=segment.through_ids[:split_index],
        )
        second = PlanningSegment(
            id=self._next_segment_id(),
            direction=segment.direction,
            start_id=waypoint_id,
            end_id=segment.end_id,
            through_ids=segment.through_ids[split_index + 1:],
        )
        self._segments[self._selected_segment:self._selected_segment + 1] = [
            first, second,
        ]
        self._selected_segment += 1
        self._selected_through = None
        self._mark_route_changed(rebuild_panel=True)

    def _delete_segment(self, _event):
        segment = self._current_segment()
        if segment is None or len(self._segments) == 1:
            self._set_route_status("至少需要保留一个规划分段")
            self._build_route_panel()
            return
        previous_index = self._selected_segment - 1
        next_index = self._selected_segment + 1
        if previous_index >= 0 and self._segments[previous_index].direction == segment.direction:
            self._push_history()
            previous = self._segments[previous_index]
            self._segments[previous_index] = replace(
                previous,
                end_id=segment.end_id,
                through_ids=(*previous.through_ids, previous.end_id, *segment.through_ids),
            )
            del self._segments[self._selected_segment]
            self._selected_segment = previous_index
        elif next_index < len(self._segments) and self._segments[next_index].direction == segment.direction:
            self._push_history()
            following = self._segments[next_index]
            self._segments[next_index] = replace(
                following,
                start_id=segment.start_id,
                through_ids=(*segment.through_ids, segment.end_id, *following.through_ids),
            )
            del self._segments[self._selected_segment]
        else:
            self._set_route_status("只有行驶方向相同的相邻分段才能合并")
            self._build_route_panel()
            return
        self._selected_through = None
        self._mark_route_changed(rebuild_panel=True)

    # ── rendering ──────────────────────────────────────────────────────

    def _route_xy(self):
        waypoint_by_id = {waypoint.id: waypoint for waypoint in self._waypoints}
        route_points = [
            waypoint_by_id[waypoint_id]
            for waypoint_id in self._route_ids_for_display()
            if waypoint_id in waypoint_by_id
        ]
        return (
            [waypoint.position[0] for waypoint in route_points],
            [waypoint.position[1] for waypoint in route_points],
        )

    def _waypoint_label_text(self, index):
        waypoint = self._waypoints[index]
        locked = (
            " [位置锁定]"
            if index in (0, len(self._waypoints) - 1)
            else ""
        )
        heading = (
            " [位置约束]"
            if not self._uses_authored_heading(waypoint)
            else " [固定朝向]"
        )
        return f"{index}:{waypoint.id}{locked}{heading}"

    def _draw_waypoint_arrow(self, index, display_waypoint):
        waypoint = self._waypoints[index]
        if is_zero_quaternion(display_waypoint.orientation):
            return None
        x, y = waypoint.position[:2]
        yaw = _yaw_from_quaternion(display_waypoint.orientation)
        arrow_len = 0.18
        color = TASK_COLORS.get(waypoint.task, DEFAULT_COLOR)
        return self._ax.arrow(
            x,
            y,
            arrow_len * math.cos(yaw),
            arrow_len * math.sin(yaw),
            head_width=0.06,
            head_length=0.08,
            fc=color,
            ec=color,
            alpha=0.9,
            zorder=6,
            length_includes_head=True,
        )

    def _clear_preflight_artists(self):
        for artist in self._segment_artists:
            artist.remove()
        self._segment_artists.clear()

    def _refresh_selected_ring(self):
        if self._selected is None:
            if self._sel_ring is not None:
                self._sel_ring.remove()
                self._sel_ring = None
            return
        x, y = self._waypoints[self._selected].position[:2]
        if self._sel_ring is None:
            self._sel_ring = self._ax.add_patch(plt.Circle(
                (x, y),
                0.12,
                fill=False,
                edgecolor=SELECTED_EDGE,
                linewidth=2.5,
                zorder=8,
                linestyle="-",
            ))
        else:
            self._sel_ring.set_center((x, y))

    def _invalidate_fast_canvas(self):
        self._blit_background = None

    def _on_canvas_draw(self, event):
        """Refresh the field-only background after a normal figure redraw."""
        canvas = self._fig.canvas
        if event.canvas is not canvas or not canvas.supports_blit:
            self._invalidate_fast_canvas()
            return
        self._blit_background = canvas.copy_from_bbox(self._ax.bbox)
        # Qt emits this from its paint cycle. Calling ``blit`` here re-enters
        # QWidget.repaint and can recurse until the process crashes.
        # The next pointer event draws the animated artists from this cache.

    def _on_canvas_resize(self, _event):
        self._invalidate_fast_canvas()

    def _prepare_fast_canvas(self):
        """Capture the static field once so pointer updates can use blitting."""
        if self._blit_background is not None:
            return
        canvas = self._fig.canvas
        if not canvas.supports_blit:
            canvas.draw_idle()
            return
        canvas.draw()
        if self._blit_background is None:
            self._blit_background = canvas.copy_from_bbox(self._ax.bbox)
        self._blit_dynamic_artists()

    def _blit_dynamic_artists(self):
        canvas = self._fig.canvas
        if self._blit_background is None or not canvas.supports_blit:
            canvas.draw_idle()
            return
        canvas.restore_region(self._blit_background)
        if self._drag_preview.get_visible():
            self._ax.draw_artist(self._drag_preview)
        canvas.blit(self._ax.bbox)

    def _hide_drag_preview(self):
        self._drag_preview_position = None
        self._drag_preview.set_visible(False)

    def _update_selection_artists(self):
        """Redraw the persistent selection ring after a discrete click."""
        if self._scatter is None:
            self._redraw()
            return
        # Right-panel widgets issue normal canvas redraws for their hover
        # state. The ring must be part of that normal draw, otherwise it
        # appears to disappear when the pointer enters the panel.
        self._invalidate_fast_canvas()
        self._refresh_selected_ring()
        self._fig.canvas.draw_idle()

    def _update_drag_preview(self, x, y):
        """Show a lightweight pointer-following preview until mouse release."""
        self._prepare_fast_canvas()
        self._drag_preview_position = (float(x), float(y))
        self._drag_preview.set_offsets([self._drag_preview_position])
        self._drag_preview.set_visible(True)
        self._blit_dynamic_artists()

    def _redraw(self):
        """Full redraw of all waypoint artists."""
        self._invalidate_fast_canvas()
        self._hide_drag_preview()
        xs = [w.position[0] for w in self._waypoints]
        ys = [w.position[1] for w in self._waypoints]
        colors = [TASK_COLORS.get(w.task, DEFAULT_COLOR) for w in self._waypoints]

        # The muted line is the user-declared waypoint constraint order, not
        # Nav2 output.  Colored lines below are offline geometric candidates.
        if self._line is not None:
            self._line.remove()
        waypoint_by_id = {waypoint.id: waypoint for waypoint in self._waypoints}
        route_xs, route_ys = self._route_xy()
        self._line, = self._ax.plot(
            route_xs,
            route_ys,
            color=PATH_PENDING, linewidth=1.0, linestyle=":", alpha=0.75, zorder=3,
        )

        self._clear_preflight_artists()
        if self._preflight is not None:
            for index, report in enumerate(self._preflight.segments):
                points = report.points
                color = PATH_OK if report.feasible else PATH_FAILED
                linestyle = "--" if report.direction == "reverse" else "-"
                if points:
                    line, = self._ax.plot(
                        [point.x for point in points],
                        [point.y for point in points],
                        color=color, linewidth=2.4, linestyle=linestyle,
                        alpha=0.92, zorder=4,
                    )
                    self._segment_artists.append(line)
                    anchor = points[0]
                else:
                    segment = self._segments[index]
                    constrained = [
                        waypoint_by_id[waypoint_id]
                        for waypoint_id in segment.route_ids
                    ]
                    line, = self._ax.plot(
                        [waypoint.position[0] for waypoint in constrained],
                        [waypoint.position[1] for waypoint in constrained],
                        color=color, linewidth=2.0, linestyle=":", alpha=0.9, zorder=4,
                    )
                    self._segment_artists.append(line)
                    anchor = Point2D(
                        constrained[0].position[0], constrained[0].position[1]
                    )
                marker = "离线预检通过" if report.feasible else "离线预检阻塞"
                label = self._ax.annotate(
                    f"第 {index + 1} 段 {marker} {report.length_m:.1f} 米",
                    (anchor.x, anchor.y - 0.16), fontsize=6.7,
                    color=color, ha="center", va="top", zorder=7,
                )
                self._segment_artists.append(label)

        # scatter plot (draggable circles)
        if self._scatter is not None:
            self._scatter.remove()
        self._scatter = self._ax.scatter(
            xs, ys, s=85, c=colors, edgecolors=colors,
            linewidths=1.5, zorder=5, picker=8, alpha=0.95,
        )

        # Ordinary transit points intentionally have no arrow. Their zero
        # quaternion is consumed by the Nav2 free-heading planner at runtime.
        for arrow in self._arrows.values():
            arrow.remove()
        self._arrows.clear()
        display_by_id = self._display_waypoints_by_id()
        for index in range(len(self._waypoints)):
            waypoint = self._waypoints[index]
            display_waypoint = display_by_id.get(waypoint.id, waypoint)
            arrow = self._draw_waypoint_arrow(index, display_waypoint)
            if arrow is not None:
                self._arrows[index] = arrow

        # labels
        for label in self._labels:
            label.remove()
        self._labels.clear()
        for i, (x, y) in enumerate(zip(xs, ys)):
            label = self._ax.annotate(
                self._waypoint_label_text(i),
                (x, y + 0.20), fontsize=7, color="white",
                ha="center", va="bottom", zorder=7,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#000000CC",
                          edgecolor="none"),
            )
            self._labels.append(label)

        self._refresh_selected_ring()
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
        route = self._display_route()
        line.points = [
            Point(x=waypoint.position[0], y=waypoint.position[1], z=0.025)
            for waypoint in route
        ]
        msg.markers.append(line)

        display_by_id = {waypoint.id: waypoint for waypoint in route}
        for i, w in enumerate(self._waypoints):
            r, g, b = self._hex_rgb(TASK_COLORS.get(w.task, DEFAULT_COLOR))
            display_waypoint = display_by_id.get(w.id, w)
            yaw = _yaw_from_quaternion(display_waypoint.orientation)
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

            if not is_zero_quaternion(display_waypoint.orientation):
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
            lock_mark = " [位置锁定]" if i in (0, len(self._waypoints)-1) else ""
            heading_mark = (
                " [位置约束]"
                if not self._uses_authored_heading(w)
                else " [固定朝向]"
            )
            sel_mark = " *" if i == self._selected else ""
            l.text = f"{i}: {w.id}{sel_mark}{lock_mark}{heading_mark}"
            msg.markers.append(l)

        self._ros_publisher.publish(msg)

    @staticmethod
    def _hex_rgb(hex_color: str):
        h = hex_color.lstrip("#")
        return tuple(int(h[i:i+2], 16) / 255.0 for i in (0, 2, 4))

    # ── mouse events ───────────────────────────────────────────────────

    def _find_waypoint(self, event):
        if event.inaxes is not self._ax or event.xdata is None or event.ydata is None:
            return None
        for i, w in enumerate(self._waypoints):
            dx = event.xdata - w.position[0]
            dy = event.ydata - w.position[1]
            if math.hypot(dx, dy) < 0.15:
                return i
        return None

    def _on_press(self, event):
        # Widget clicks share the canvas event stream.  Do not let a right-panel
        # button clear the waypoint selected on the field.
        if event.inaxes is not self._ax:
            return
        if event.button == MouseButton.RIGHT:
            self._panning = True
            self._pan_start = (event.x, event.y)
            return
        if event.button != MouseButton.LEFT:
            return
        if self._adding_through:
            if event.xdata is None or event.ydata is None:
                return
            if self._find_waypoint(event) is not None:
                self._set_route_status("请在场地空白处新增途经点，不要覆盖已有航点")
                self._refresh_add_through_button()
                return
            self._create_through_at(event.xdata, event.ydata)
            return
        idx = self._find_waypoint(event)
        if idx is not None:
            if self._pick_target is not None:
                self._selected = idx
                self._set_segment_endpoint(
                    self._pick_target, self._waypoints[idx].id
                )
                return
            if idx in (0, len(self._waypoints) - 1):
                self._node.get_logger().info(
                    f"航点 [{idx}] 位置已锁定（起点/返回点），不能移动"
                )
                self._selected = idx
                self._update_selection_artists()
                return
            self._dragging = idx
            self._selected = idx
            self._drag_start = (self._waypoints[idx].position[0],
                                self._waypoints[idx].position[1])
            self._drag_moved = False
        else:
            self._selected = None
            self._dragging = None
            self._drag_moved = False
        self._update_selection_artists()

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
            self._invalidate_fast_canvas()
            self._ax.set_xlim(xlim[0] - dx * scale_x, xlim[1] - dx * scale_x)
            self._ax.set_ylim(ylim[0] - dy * scale_y, ylim[1] - dy * scale_y)
            self._fig.canvas.draw_idle()
            return

        # --- waypoint drag ---
        if self._dragging is None:
            return
        if event.inaxes is not self._ax or event.xdata is None or event.ydata is None:
            return
        if not self._drag_moved:
            start = self._drag_start
            if start is None or math.hypot(event.xdata - start[0], event.ydata - start[1]) < DRAG_THRESHOLD_M:
                return
            self._push_history()
            self._drag_moved = True
            if self._preflight is not None:
                # The cached field must be rebuilt once without the old
                # preflight lines before the live pointer preview begins.
                self._preflight = None
                self._route_definition_valid = None
                self._route_status = "点位已移动；松开鼠标后点击“几何预检”"
                self._redraw()
        self._update_drag_preview(event.xdata, event.ydata)

    def _on_release(self, event):
        if self._panning:
            self._panning = False
            self._pan_start = None
            return
        if self._dragging is not None:
            moved = self._drag_moved
            waypoint_index = self._dragging
            preview_position = self._drag_preview_position
            self._dragging = None
            self._drag_start = None
            self._drag_moved = False
            if moved and preview_position is not None:
                with self._lock:
                    waypoint = self._waypoints[waypoint_index]
                    self._waypoints[waypoint_index] = replace(
                        waypoint,
                        position=(preview_position[0], preview_position[1], 0.0),
                        orientation=(
                            waypoint.orientation
                            if self._uses_authored_heading(waypoint)
                            else (0.0, 0.0, 0.0, 0.0)
                        ),
                    )
                w = self._waypoints[waypoint_index]
                self._node.get_logger().info(
                    f"已移动 [{waypoint_index}] 到 "
                    f"({w.position[0]:.3f}, {w.position[1]:.3f})"
                )
                self._mark_route_changed(rebuild_panel=True)
                self._publish_markers()
            else:
                self._hide_drag_preview()
                self._blit_dynamic_artists()
            return
        self._dragging = None
        self._drag_start = None
        self._drag_moved = False

    def _on_scroll(self, event):
        if event.inaxes is not self._ax:
            return
        # scroll on a waypoint → rotate it
        idx = self._find_waypoint(event)
        if idx is not None and idx != 0:
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
        self._invalidate_fast_canvas()
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
            if self._adding_through:
                self._adding_through = False
                self._set_route_status("已取消新增途经点")
                self._refresh_add_through_button()
                return
            self._selected = None
            self._update_selection_artists()
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
        self._invalidate_fast_canvas()
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
        self._invalidate_fast_canvas()
        self._ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
        self._ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
        self._fig.canvas.draw_idle()

    # ── operations ─────────────────────────────────────────────────────

    def _rotate_selected(self, delta_deg):
        if self._selected is None:
            return
        idx = self._selected
        if idx == 0:
            return
        w = self._waypoints[idx]
        if not self._uses_authored_heading(w):
            self._set_route_status(
                f"{w.id} 为位置约束；运行时由规划器选择朝向"
            )
            self._build_route_panel()
            return
        self._push_history()
        yaw = _yaw_from_quaternion(w.orientation) + math.radians(delta_deg)
        with self._lock:
            self._waypoints[idx] = replace(w, orientation=_yaw_quaternion(yaw))
        self._mark_route_changed(rebuild_panel=True)
        self._publish_markers()
        self._node.get_logger().info(
            f"已将航点 [{idx}] 朝向调整为 {math.degrees(yaw):.1f} 度"
        )

    def _push_history(self):
        self._history.append((tuple(self._waypoints), tuple(self._segments)))
        if len(self._history) > 100:
            del self._history[0]

    def _undo(self):
        if not self._history:
            self._node.get_logger().info("没有可撤销的修改")
            return
        with self._lock:
            waypoints, segments = self._history.pop()
            self._waypoints = list(waypoints)
            self._segments = list(segments)
        self._selected = None
        self._dragging = None
        self._selected_through = None
        self._mark_route_changed(rebuild_panel=True)
        self._publish_markers()
        self._node.get_logger().info("已撤销上一步修改")

    def _save(self):
        if self._preflight is None:
            self._set_route_status("保存已阻止：请先点击“几何预检”")
            self._build_route_panel()
            return
        if not self._preflight.feasible:
            self._set_route_status("保存已阻止：请先修正全部红色分段，再点击“几何预检”")
            self._build_route_panel()
            return
        try:
            with self._lock:
                checked = validate_planning_segments(self._segments, self._waypoints)
                route = materialize_route(self._waypoints, checked)
                # Persist transit points as position-only constraints.  The
                # runtime and previews derive their valid Nav2 quaternion from
                # this ordered segment route instead of reviving an old yaw.
                route = tuple(
                    waypoint
                    if self._uses_authored_heading(waypoint)
                    else replace(waypoint, orientation=(0.0, 0.0, 0.0, 0.0))
                    for waypoint in route
                )
                validate_waypoints(route)
                destination = self._path.resolve()
                template = planning_segments_document(self._template, checked)
                write_waypoints_atomic(destination, template, route)
                self._path = destination
                self._template, loaded = load_waypoint_document(destination)
                self._waypoints = list(loaded)
                self._segments = list(
                    load_planning_segments(self._template, self._waypoints)
                )
        except (PlanningSegmentError, ValueError) as error:
            self._node.get_logger().error(f"Save route error: {error}")
            self._set_route_status(f"保存已阻止：路线定义有误：{error}")
            self._build_route_panel()
            return
        self._history.clear()
        self._selected = None
        self._dragging = None
        self._selected_through = None
        self._route_status = "航点与规划分段已保存"
        self._redraw()
        self._build_route_panel()
        self._publish_markers()
        self._node.get_logger().info("航点与规划分段已保存")

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
