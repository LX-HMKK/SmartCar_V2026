"""Contracts for the display-only competition field reference overlay."""
from pathlib import Path
import sys
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAV2_ROOT = PACKAGE_ROOT.parent / "smartcar_nav2"
GEOMETRY_FILE = PACKAGE_ROOT / "config" / "routes" / "field_geometry.yaml"
sys.path.insert(0, str(PACKAGE_ROOT))


class TestFieldReferenceModel(unittest.TestCase):
    """Keep the rule-diagram geometry independent from ROS and RViz."""

    @classmethod
    def setUpClass(cls):
        from smartcar_tools.field_reference import load_field_reference

        cls.reference = load_field_reference(GEOMETRY_FILE)

    def assert_bounds(self, bounds, expected):
        self.assertEqual(
            (bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max),
            expected,
        )

    def test_model_module_has_no_ros_dependency(self):
        source = (
            PACKAGE_ROOT / "smartcar_tools" / "field_reference.py"
        ).read_text(encoding="utf-8")
        for forbidden in ("rclpy", "visualization_msgs", "nav_msgs"):
            self.assertNotIn(forbidden, source)

    def test_field_and_zone_bounds_use_the_p_origin_frame(self):
        self.assertEqual(self.reference.frame_id, "odom_combined")
        self.assert_bounds(
            self.reference.field,
            (-0.5, 4.5, -0.25, 4.75),
        )
        self.assert_bounds(
            self.reference.zones["A"],
            (-0.5, 4.5, -0.25, 1.75),
        )
        self.assert_bounds(
            self.reference.zones["B"],
            (-0.5, 4.5, 1.75, 2.25),
        )
        self.assert_bounds(
            self.reference.zones["C"],
            (-0.5, 4.5, 2.25, 4.75),
        )

    def test_corridor_and_ring_match_the_official_dimensions(self):
        self.assert_bounds(
            self.reference.corridor,
            (1.5, 2.5, 1.75, 2.5),
        )
        self.assert_bounds(
            self.reference.ring_outer,
            (0.0, 4.0, 2.5, 4.15),
        )
        self.assert_bounds(
            self.reference.ring_inner,
            (0.5, 3.5, 3.0, 3.65),
        )
        self.assertEqual(
            self.reference.ring_outer_outline[0],
            type(self.reference.p_origin)(2.5, 2.5),
        )
        self.assertEqual(
            self.reference.ring_outer_outline[-1],
            type(self.reference.p_origin)(1.5, 2.5),
        )
        self.assertNotEqual(
            self.reference.ring_outer_outline[0],
            self.reference.ring_outer_outline[-1],
        )
        self.assertTrue(
            self.reference.corridor.x_min <= 2.0 <= self.reference.corridor.x_max
            and self.reference.corridor.y_min <= 2.4 <= self.reference.corridor.y_max
        )

    def test_reference_landmarks_distinguish_p_and_the_rule_task_point(self):
        self.assertEqual(
            (self.reference.p_origin.x, self.reference.p_origin.y),
            (0.0, 0.0),
        )
        self.assertEqual(
            (self.reference.task_point.x, self.reference.task_point.y),
            (4.15, 1.35),
        )


class TestFieldReferenceIsVisualizationOnly(unittest.TestCase):
    def test_nav2_costmaps_remain_rolling_and_have_no_static_layer(self):
        params = yaml.safe_load(
            (NAV2_ROOT / "config" / "nav2_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        for name in ("local_costmap", "global_costmap"):
            costmap = params[name][name]["ros__parameters"]
            self.assertIs(costmap["rolling_window"], True)
            self.assertNotIn("static_layer", costmap["plugins"])
            self.assertEqual(costmap["global_frame"], "odom_combined")


if __name__ == "__main__":
    unittest.main()
