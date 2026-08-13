"""Static contracts that keep route-reference and Nav2 path displays distinct."""

from pathlib import Path
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
SIM_RVIZ = REPOSITORY_ROOT / "src" / "smartcar_sim" / "rviz" / "sim_nav.rviz"
NAVIGATION_RVIZ = PACKAGE_ROOT / "rviz" / "navigation.rviz"
EDITOR_RVIZ = PACKAGE_ROOT / "rviz" / "waypoint_editor.rviz"
WAYPOINT_VIZ = PACKAGE_ROOT / "smartcar_tools" / "waypoint_viz.py"
DRAG_EDITOR = PACKAGE_ROOT / "smartcar_tools" / "waypoint_drag_editor.py"
PREFLIGHT = PACKAGE_ROOT / "smartcar_tools" / "route_preflight.py"
FIELD_REFERENCE = PACKAGE_ROOT / "smartcar_tools" / "field_reference_node.py"

GLOBAL_PATH_NAME = "Planner Candidate Path (/plan)"
TRACKING_PATH_NAME = "Controller Tracking Path (/received_global_plan)"
SIM_CANDIDATE_PATH_NAME = "Planner Candidate Path (/plan)"
SIM_TRACKING_PATH_NAME = "Controller Tracking Window (/received_global_plan)"
REFERENCE_NAME = "Waypoint Reference Constraints (not a Nav2 path)"


def display_by_topic(document, topic):
    displays = document["Visualization Manager"]["Displays"]
    return next(display for display in displays if display.get("Topic", {}).get("Value") == topic)


class RouteVisualizationContracts(unittest.TestCase):
    def test_navigation_rviz_distinguishes_nav2_paths_from_waypoint_constraints(self):
        navigation = yaml.safe_load(NAVIGATION_RVIZ.read_text(encoding="utf-8"))
        global_path = display_by_topic(navigation, "/plan")
        tracking_path = display_by_topic(
            navigation, "/received_global_plan")
        reference = display_by_topic(navigation, "/smartcar/waypoints/markers")

        self.assertEqual(global_path["Class"], "rviz_default_plugins/Path")
        self.assertEqual(global_path["Name"], GLOBAL_PATH_NAME)
        self.assertEqual(tracking_path["Class"], "rviz_default_plugins/Path")
        self.assertEqual(tracking_path["Name"], TRACKING_PATH_NAME)
        self.assertFalse(any(
            display.get("Topic", {}).get("Value") in {
                "/local_plan", "/transformed_plan",
                "/transformed_global_plan",
            }
            for display in navigation["Visualization Manager"]["Displays"]
        ))
        self.assertEqual(reference["Class"], "rviz_default_plugins/MarkerArray")
        self.assertEqual(reference["Name"], REFERENCE_NAME)

        simulation = yaml.safe_load(SIM_RVIZ.read_text(encoding="utf-8"))
        candidate = display_by_topic(simulation, "/plan")
        tracking = display_by_topic(simulation, "/received_global_plan")
        reference = display_by_topic(
            simulation, "/smartcar/waypoints/markers")

        self.assertEqual(candidate["Class"], "rviz_default_plugins/Path")
        self.assertEqual(candidate["Name"], SIM_CANDIDATE_PATH_NAME)
        self.assertTrue(candidate["Enabled"])
        self.assertFalse(any(
            display.get("Topic", {}).get("Value") ==
            "/smartcar/accepted_global_plan"
            for display in simulation["Visualization Manager"]["Displays"]
        ))
        self.assertEqual(tracking["Class"], "rviz_default_plugins/Path")
        self.assertEqual(tracking["Name"], SIM_TRACKING_PATH_NAME)
        self.assertEqual(reference["Class"], "rviz_default_plugins/MarkerArray")
        self.assertEqual(reference["Name"], REFERENCE_NAME)

    def test_navigation_depth_displays_show_only_the_newest_frame(self):
        navigation = yaml.safe_load(NAVIGATION_RVIZ.read_text(encoding="utf-8"))
        scan_topics = (
            "/scan",
            "/smartcar/depth/scan",
            "/smartcar/depth/points",
        )

        for topic in scan_topics:
            with self.subTest(topic=topic):
                display = display_by_topic(navigation, topic)
                self.assertEqual(display["Topic"]["Depth"], 1)
                self.assertEqual(
                    display["Topic"]["Durability Policy"], "Volatile")
                self.assertEqual(
                    display["Topic"]["Reliability Policy"], "Best Effort")
                self.assertEqual(display["Queue Size"], 1)
                self.assertEqual(display["Decay Time"], 0.0)

        self.assertEqual(
            navigation["Visualization Manager"]["Global Options"]["Frame Rate"],
            10,
        )

    def test_editor_reference_display_is_not_named_as_a_nav2_path(self):
        document = yaml.safe_load(EDITOR_RVIZ.read_text(encoding="utf-8"))
        reference = display_by_topic(document, "/smartcar/waypoint_editor/markers")

        self.assertEqual(reference["Class"], "rviz_default_plugins/MarkerArray")
        self.assertEqual(reference["Name"], REFERENCE_NAME)

    def test_editor_shows_only_waypoint_constraints_not_offline_paths(self):
        waypoint_viz = WAYPOINT_VIZ.read_text(encoding="utf-8")
        drag_editor = DRAG_EDITOR.read_text(encoding="utf-8")
        preflight = PREFLIGHT.read_text(encoding="utf-8")

        self.assertIn("Nav2-generated ``/plan``", waypoint_viz)
        self.assertIn("``/local_plan``", waypoint_viz)
        self.assertIn("非 Nav2 路径", drag_editor)
        self.assertIn("运行时路径由 Nav2 实时规划", drag_editor)
        self.assertIn("CONSTRAINT_COLOR", drag_editor)
        for offline_path_token in (
            "RoutePreflight",
            "preflight_route",
            "_recheck_route",
            "离线几何预检",
            "self._segment_artists",
        ):
            self.assertNotIn(offline_path_token, drag_editor)
        self.assertIn("Offline geometric preflight", preflight)
        self.assertIn("not a Nav2 planning or reachability result", preflight)

    def test_waypoint_viz_is_latched_and_does_not_republish_when_static(self):
        waypoint_viz = WAYPOINT_VIZ.read_text(encoding="utf-8")

        self.assertIn('declare_parameter("reload_interval_sec", 0.0)', waypoint_viz)
        self.assertNotIn('declare_parameter("publish_rate"', waypoint_viz)
        self.assertIn("self._reload_if_changed(force=True)", waypoint_viz)
        self.assertIn("mtime_ns == self._last_mtime_ns", waypoint_viz)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", waypoint_viz)

    def test_waypoint_viz_uses_the_task_c_zone_transform(self):
        waypoint_viz = WAYPOINT_VIZ.read_text(encoding="utf-8")
        self.assertIn(
            "from smartcar_task.c_zone_direction import apply_c_zone_direction",
            waypoint_viz,
        )
        self.assertIn(
            "authored = apply_c_zone_direction(authored, c_zone_direction)",
            waypoint_viz,
        )
        self.assertIn('declare_parameter("c_zone_direction", "counterclockwise")', waypoint_viz)

    def test_previews_keep_transit_headings_free(self):
        waypoint_viz = WAYPOINT_VIZ.read_text(encoding="utf-8")
        drag_editor = DRAG_EDITOR.read_text(encoding="utf-8")
        preflight = PREFLIGHT.read_text(encoding="utf-8")

        for source in (waypoint_viz, drag_editor, preflight):
            self.assertIn("materialize_free_yaws", source)
            self.assertIn("materialize_route", source)
        self.assertIn("[位置约束]", waypoint_viz)
        self.assertIn("[位置约束]", drag_editor)
        self.assertIn("position-only; preflight leaves heading free", preflight)
        self.assertIn("not is_zero_quaternion(item.orientation)", waypoint_viz)
        for source in (waypoint_viz, drag_editor, preflight):
            self.assertNotIn("automatic route tangent", source)

    def test_c_ring_reference_style_does_not_match_the_local_plan(self):
        source = FIELD_REFERENCE.read_text(encoding="utf-8")

        self.assertIn("(0.70, 0.75, 0.82, 0.70)", source)
        self.assertNotIn("(1.00, 0.76, 0.10, 0.95)", source)


if __name__ == "__main__":
    unittest.main()
