"""Static contracts for the motion-disabled semantic waypoint editor."""
import ast
from pathlib import Path
import re
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
NAV2_ROOT = PACKAGE_ROOT.parent / "smartcar_nav2"
LAUNCH = PACKAGE_ROOT / "launch" / "waypoint_editor.launch.py"
SIM_EDITOR_LAUNCH = PACKAGE_ROOT / "launch" / "sim_waypoint_editor.launch.py"
SIM_LAUNCH = REPOSITORY_ROOT / "src" / "smartcar_sim" / "launch" / "sim.launch.py"
NODE = PACKAGE_ROOT / "smartcar_tools" / "waypoint_editor_node.py"
DRAG_EDITOR = PACKAGE_ROOT / "smartcar_tools" / "waypoint_drag_editor.py"
SETUP = PACKAGE_ROOT / "setup.py"
RVIZ = PACKAGE_ROOT / "rviz" / "waypoint_editor.rviz"
WAYPOINT_MODEL = (
    PACKAGE_ROOT.parent / "smartcar_task" / "smartcar_task" / "waypoints.py"
)
FIELD_MARKER_TOPIC = "/smartcar/field_reference/markers"
WAYPOINT_MARKER_TOPIC = "/smartcar/waypoint_editor/markers"
WAYPOINT_UPDATE_TOPIC = "/smartcar/waypoint_editor/update"


def display_by_name(rviz_document, name):
    displays = rviz_document["Visualization Manager"]["Displays"]
    return next(display for display in displays if display.get("Name") == name)


class TestWaypointEditorContract(unittest.TestCase):
    def test_simulation_editor_uses_the_same_nav_only_file_as_simulation(self):
        nav_only_suffix = (
            r'"config",\s*"waypoints",\s*"nav_only\.yaml",?\s*\]\)'
        )
        editor_path = re.compile(
            r'PathJoinSubstitution\(\[\s*FindPackageShare\("smartcar_nav2"\),\s*'
            + nav_only_suffix
        )
        default_path = re.compile(
            r'PathJoinSubstitution\(\[\s*FindPackageShare\("smartcar_nav2"\),\s*'
            r'"config",\s*"waypoints",\s*"default_waypoints\.yaml",?\s*\]\)'
        )
        standard = LAUNCH.read_text(encoding="utf-8")
        simulation = SIM_LAUNCH.read_text(encoding="utf-8")
        simulation_editor = SIM_EDITOR_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('pkg_nav2 = get_package_share_directory("smartcar_nav2")', simulation)
        self.assertIn('default_value=os.path.join(', simulation)
        self.assertIn(
            'pkg_nav2, "config", "waypoints", "nav_only.yaml"', simulation
        )
        self.assertRegex(simulation_editor, editor_path)
        self.assertNotRegex(standard, editor_path)
        self.assertRegex(standard, default_path)

        ast.parse(simulation_editor)
        self.assertIn("waypoint_editor.launch.py", simulation_editor)
        self.assertIn(
            'DeclareLaunchArgument("start_safety", default_value="false")',
            simulation_editor,
        )
        self.assertIn(
            'DeclareLaunchArgument("use_rviz", default_value="false")',
            simulation_editor,
        )
        self.assertIn(
            'DeclareLaunchArgument("use_sim_time", default_value="false")',
            simulation_editor,
        )
        self.assertNotIn('package="smartcar_sim"', simulation_editor)

    def test_legacy_navigation_test_chain_is_removed(self):
        legacy_paths = (
            PACKAGE_ROOT / "launch" / "navigation_test.launch.py",
            PACKAGE_ROOT / "smartcar_tools" / "navigation_runner.py",
            PACKAGE_ROOT / "smartcar_tools" / "navigation_probe.py",
            PACKAGE_ROOT / "test" / "test_navigation_contracts.py",
            PACKAGE_ROOT / "test" / "test_navigation_probe.py",
            NAV2_ROOT / "config" / "field_test_nav2_params.yaml",
            NAV2_ROOT
            / "config"
            / "behavior_trees"
            / "navigate_to_pose_field_test.xml",
            NAV2_ROOT
            / "config"
            / "behavior_trees"
            / "navigate_through_poses_field_test.xml",
            REPOSITORY_ROOT / "tests" / "test_navigation_test_contracts.py",
        )
        for path in legacy_paths:
            self.assertFalse(path.exists(), str(path))

        setup_source = SETUP.read_text(encoding="utf-8")
        for legacy_entrypoint in ("navigation_runner", "navigation_probe"):
            self.assertNotIn(legacy_entrypoint, setup_source)

    def test_launch_is_motion_disabled_and_uses_the_only_waypoint_file(self):
        source = LAUNCH.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('"emergency_stop_on_start": "true"', source)
        self.assertIn('"latch_emergency_stop": start_safety', source)
        self.assertIn('executable="waypoint_editor_node"', source)
        self.assertIn('executable="waypoint_drag_editor"', source)
        self.assertIn('DeclareLaunchArgument("use_segment_ui", default_value="true")', source)
        self.assertIn('DeclareLaunchArgument("start_safety", default_value="false")', source)
        self.assertIn('executable="field_reference_node"', source)
        self.assertIn("default_waypoints.yaml", source)
        self.assertIn("field_geometry.yaml", source)
        for legacy in ("full_course_route.yaml", "route_editor_node"):
            self.assertNotIn(legacy, source)
        for motion_runtime in (
            'package="nav2_',
            'executable="navigation_runner"',
            "origincar_base",
        ):
            self.assertNotIn(motion_runtime, source)

    def test_node_edits_semantic_waypoints_and_keeps_them_uncalibrated(self):
        source = NODE.read_text(encoding="utf-8")
        model_source = WAYPOINT_MODEL.read_text(encoding="utf-8")
        setup_source = SETUP.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn(f'MARKER_TOPIC = "{WAYPOINT_MARKER_TOPIC}"', source)
        self.assertIn('EDITOR_NAMESPACE = "smartcar/waypoint_editor"', source)
        self.assertIn('declare_parameter("waypoints_file"', source)
        self.assertIn("load_waypoint_document", source)
        self.assertIn("validate_waypoints", source)
        self.assertIn("write_waypoints_atomic", source)
        self.assertIn('root["calibrated"] = False', model_source)
        self.assertIn("os.replace", model_source)
        self.assertIn(
            "waypoint_editor_node = smartcar_tools.waypoint_editor_node:main",
            setup_source,
        )
        for legacy in ("load_route", "RoutePoint", "full_course_route"):
            self.assertNotIn(legacy, source)

    def test_editor_uses_latched_markers_and_draggable_xy_yaw_controls(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn("MarkerArray", source)
        self.assertIn("InteractiveMarkerServer", source)
        self.assertIn("InteractiveMarkerFeedback.POSE_UPDATE", source)
        self.assertIn("InteractiveMarkerControl.MOVE_PLANE", source)
        self.assertIn("InteractiveMarkerControl.ROTATE_AXIS", source)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", source)
        self.assertIn("ReliabilityPolicy.RELIABLE", source)
        self.assertIn("Marker.DELETEALL", source)
        self.assertIn("Marker.LINE_STRIP", source)
        self.assertIn("Marker.ARROW", source)
        self.assertIn("Marker.TEXT_VIEW_FACING", source)

    def test_drag_editor_exposes_segment_controls_and_local_preflight(self):
        source = DRAG_EDITOR.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertNotIn('node.declare_parameter("use_sim_time"', source)
        for token in (
            "PlanningSegment",
            "load_planning_segments",
            "planning_segments_document",
            "validate_planning_segments",
            "materialize_route",
            "preflight_route",
            "RadioButtons",
            "TextBox",
            "路径分段",
            "行驶方向",
            "正向（前进）",
            "倒向（倒车）",
            "起点朝向",
            "终点朝向",
            "在选中途经点拆分",
            "新增途经点",
            "加入已有点",
            "上移",
            "下移",
            "设为无朝向",
            "恢复路线朝向",
            "几何预检",
            "保存路线",
            "保存已阻止",
            "C 区禁区",
            "_mark_route_changed",
            "event.inaxes is not self._ax",
        ):
            self.assertIn(token, source)
        self.assertEqual(source.count("self._recheck_route("), 1)
        self.assertIn('matplotlib.use("Qt5Agg")', source)
        self.assertLess(
            source.index('matplotlib.use("Qt5Agg")'),
            source.index("import matplotlib.pyplot as plt"),
        )
        self.assertNotIn("/mnt/c/Windows/Fonts", source)
        self.assertIn("route = materialize_route(self._waypoints, self._segments)", source)

    def test_qt_draw_callback_only_captures_the_blit_background(self):
        """A Qt paint callback must not recursively request another repaint."""
        source = DRAG_EDITOR.read_text(encoding="utf-8")
        module = ast.parse(source)
        editor = next(
            node for node in module.body
            if isinstance(node, ast.ClassDef) and node.name == "DragEditor"
        )
        draw_callback = next(
            node for node in editor.body
            if isinstance(node, ast.FunctionDef) and node.name == "_on_canvas_draw"
        )
        self.assertIn("copy_from_bbox", ast.unparse(draw_callback))
        self.assertNotIn("_blit_dynamic_artists", ast.unparse(draw_callback))

    def test_add_through_mode_does_not_rebuild_the_panel_from_button_click(self):
        """Qt releases the clicked widget after this callback returns."""
        module = ast.parse(DRAG_EDITOR.read_text(encoding="utf-8"))
        editor = next(
            node for node in module.body
            if isinstance(node, ast.ClassDef) and node.name == "DragEditor"
        )
        begin_add = next(
            node for node in editor.body
            if isinstance(node, ast.FunctionDef) and node.name == "_begin_add_through"
        )
        source = ast.unparse(begin_add)
        self.assertIn("_refresh_add_through_button", source)
        self.assertNotIn("_build_route_panel", source)

    def test_selection_ring_survives_right_panel_hover_redraws(self):
        module = ast.parse(DRAG_EDITOR.read_text(encoding="utf-8"))
        editor = next(
            node for node in module.body
            if isinstance(node, ast.ClassDef) and node.name == "DragEditor"
        )
        methods = {
            node.name: ast.unparse(node)
            for node in editor.body
            if isinstance(node, ast.FunctionDef)
        }
        self.assertNotIn("animated=True", methods["_refresh_selected_ring"])
        self.assertIn("_invalidate_fast_canvas", methods["_update_selection_artists"])
        self.assertIn("draw_idle", methods["_update_selection_artists"])
        self.assertNotIn("_blit_dynamic_artists", methods["_update_selection_artists"])

    def test_editor_has_save_undo_load_menu_and_keeps_p_positions_fixed(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn("MenuHandler", source)
        for label in (
            "Save all waypoints",
            "Undo last drag",
            "Reload from disk",
        ):
            self.assertIn(label, source)
        for service in ("load", "undo", "save"):
            self.assertIn(f'f"{{SERVICE_PREFIX}}/{service}"', source)
        self.assertIn("index not in (0, len(self._waypoints) - 1)", source)
        self.assertIn("x_m = original.position[0] if index in", source)
        self.assertIn("y_m = original.position[1] if index in", source)
        self.assertIn("yaw = 0.0 if index == 0", source)

    def test_rviz_shows_field_and_semantic_waypoints_and_routes_goal_updates(self):
        document = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
        field = display_by_name(document, "Official Field Reference")
        waypoints = display_by_name(
            document, "Waypoint Reference Constraints (not a Nav2 path)"
        )
        for display, topic in (
            (field, FIELD_MARKER_TOPIC),
            (waypoints, WAYPOINT_MARKER_TOPIC),
        ):
            self.assertEqual(display["Class"], "rviz_default_plugins/MarkerArray")
            self.assertIs(display["Enabled"], True)
            self.assertEqual(display["Topic"]["Value"], topic)
            self.assertEqual(
                display["Topic"]["Durability Policy"], "Transient Local"
            )
            self.assertEqual(display["Topic"]["Reliability Policy"], "Reliable")

        interactive = display_by_name(document, "Draggable Waypoints")
        self.assertEqual(
            interactive["Class"], "rviz_default_plugins/InteractiveMarkers"
        )
        self.assertIs(interactive["Enabled"], True)
        self.assertEqual(interactive["Update Topic"], WAYPOINT_UPDATE_TOPIC)
        self.assertEqual(
            document["Visualization Manager"]["Global Options"]["Fixed Frame"],
            "odom_combined",
        )
        self.assertNotIn("Value: /map", RVIZ.read_text(encoding="utf-8"))

    def test_field_reference_node_is_a_marker_overlay_not_localization(self):
        source = (
            PACKAGE_ROOT / "smartcar_tools" / "field_reference_node.py"
        ).read_text(encoding="utf-8")
        setup_source = SETUP.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn(f'FIELD_MARKER_TOPIC = "{FIELD_MARKER_TOPIC}"', source)
        self.assertIn('declare_parameter("geometry_file"', source)
        self.assertIn("MarkerArray", source)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", source)
        self.assertIn("ReliabilityPolicy.RELIABLE", source)
        self.assertIn(
            "field_reference_node = smartcar_tools.field_reference_node:main",
            setup_source,
        )
        for forbidden in (
            "OccupancyGrid",
            '"/map"',
            "map->odom",
            "map_server",
            "slam_toolbox",
            "nav2_amcl",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
