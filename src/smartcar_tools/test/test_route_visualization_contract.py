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

GLOBAL_PATH_NAME = "Actual Nav2 Global Path (/plan)"
LOCAL_PATH_NAME = "Actual Nav2 Local Path (/local_plan)"
REFERENCE_NAME = "Waypoint Reference Constraints (not a Nav2 path)"


def display_by_topic(document, topic):
    displays = document["Visualization Manager"]["Displays"]
    return next(display for display in displays if display.get("Topic", {}).get("Value") == topic)


class RouteVisualizationContracts(unittest.TestCase):
    def test_navigation_rviz_distinguishes_nav2_paths_from_waypoint_constraints(self):
        for path in (SIM_RVIZ, NAVIGATION_RVIZ):
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            global_path = display_by_topic(document, "/plan")
            local_path = display_by_topic(document, "/local_plan")
            reference = display_by_topic(document, "/smartcar/waypoints/markers")

            self.assertEqual(global_path["Class"], "rviz_default_plugins/Path")
            self.assertEqual(global_path["Name"], GLOBAL_PATH_NAME)
            self.assertEqual(local_path["Class"], "rviz_default_plugins/Path")
            self.assertEqual(local_path["Name"], LOCAL_PATH_NAME)
            self.assertEqual(reference["Class"], "rviz_default_plugins/MarkerArray")
            self.assertEqual(reference["Name"], REFERENCE_NAME)

    def test_editor_reference_display_is_not_named_as_a_nav2_path(self):
        document = yaml.safe_load(EDITOR_RVIZ.read_text(encoding="utf-8"))
        reference = display_by_topic(document, "/smartcar/waypoint_editor/markers")

        self.assertEqual(reference["Class"], "rviz_default_plugins/MarkerArray")
        self.assertEqual(reference["Name"], REFERENCE_NAME)

    def test_offline_preflight_copy_does_not_claim_nav2_reachability(self):
        waypoint_viz = WAYPOINT_VIZ.read_text(encoding="utf-8")
        drag_editor = DRAG_EDITOR.read_text(encoding="utf-8")
        preflight = PREFLIGHT.read_text(encoding="utf-8")

        self.assertIn("Nav2-generated ``/plan``", waypoint_viz)
        self.assertIn("``/local_plan``", waypoint_viz)
        self.assertIn("离线几何预检", drag_editor)
        self.assertIn("非 Nav2 路径", drag_editor)
        self.assertNotIn('"路径规划"', drag_editor)
        self.assertIn("Offline geometric preflight", preflight)
        self.assertIn("not a Nav2 planning or reachability result", preflight)

    def test_waypoint_viz_is_latched_and_does_not_republish_when_static(self):
        waypoint_viz = WAYPOINT_VIZ.read_text(encoding="utf-8")

        self.assertIn('declare_parameter("reload_interval_sec", 0.0)', waypoint_viz)
        self.assertNotIn('declare_parameter("publish_rate"', waypoint_viz)
        self.assertIn("self._reload_if_changed(force=True)", waypoint_viz)
        self.assertIn("mtime_ns == self._last_mtime_ns", waypoint_viz)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", waypoint_viz)

    def test_c_ring_reference_style_does_not_match_the_local_plan(self):
        source = FIELD_REFERENCE.read_text(encoding="utf-8")

        self.assertIn("(0.70, 0.75, 0.82, 0.70)", source)
        self.assertNotIn("(1.00, 0.76, 0.10, 0.95)", source)


if __name__ == "__main__":
    unittest.main()
