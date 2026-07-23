"""Static contracts for the motion-disabled semantic waypoint editor."""
import ast
from pathlib import Path
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
NAV2_ROOT = PACKAGE_ROOT.parent / "smartcar_nav2"
LAUNCH = PACKAGE_ROOT / "launch" / "waypoint_editor.launch.py"
NODE = PACKAGE_ROOT / "smartcar_tools" / "waypoint_editor_node.py"
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
        self.assertIn('"latch_emergency_stop": True', source)
        self.assertIn('executable="waypoint_editor_node"', source)
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
        waypoints = display_by_name(document, "Mission Route")
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
