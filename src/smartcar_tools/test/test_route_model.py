"""ROS-independent tests for rule geometry and full-course route contracts."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.route_model import (  # noqa: E402
    MAX_LOOP_POINT_SPACING_M,
    RoutePoint,
    RouteValidationError,
    angular_distance_deg,
    generate_baseline_route,
    load_field_geometry,
    load_route,
    replace_waypoints,
    validate_geometry,
    validate_route,
    write_route_atomic,
)


GEOMETRY_FILE = PACKAGE_ROOT / "config" / "routes" / "field_geometry.yaml"
ROUTE_FILE = PACKAGE_ROOT / "config" / "routes" / "full_course_route.yaml"


class TestFieldGeometry(unittest.TestCase):
    def setUp(self):
        self.geometry = load_field_geometry(GEOMETRY_FILE)

    def test_rule_dimensions_are_encoded_exactly(self):
        geometry = self.geometry
        self.assertEqual((geometry.field_width_m, geometry.field_height_m), (5.0, 5.0))
        self.assertEqual(
            (geometry.zone_a_height_m, geometry.zone_b_height_m, geometry.zone_c_height_m),
            (2.0, 0.5, 2.5),
        )
        self.assertEqual(geometry.corridor_width_m, 1.0)
        self.assertEqual(
            (geometry.ring_outer_width_m, geometry.ring_outer_height_m),
            (4.0, 1.65),
        )
        self.assertEqual(
            (geometry.ring_inner_width_m, geometry.ring_inner_height_m),
            (3.0, 0.65),
        )

    def test_centerline_is_concentric_stadium(self):
        self.assertAlmostEqual(self.geometry.ring_centerline_radius_m, 0.575)
        self.assertAlmostEqual(self.geometry.ring_cap_center_separation_m, 2.35)

    def test_source_rule_diagrams_are_preserved(self):
        document = yaml.safe_load(GEOMETRY_FILE.read_text(encoding="utf-8"))
        for relative_path in document["source"]["diagrams"]:
            self.assertTrue((GEOMETRY_FILE.parent / relative_path).resolve().is_file())

    def test_zone_heights_must_cover_field(self):
        invalid = replace(self.geometry, zone_c_height_m=2.4)
        with self.assertRaisesRegex(RouteValidationError, "zone heights"):
            validate_geometry(invalid)


class TestBaselineRoute(unittest.TestCase):
    def setUp(self):
        self.geometry = load_field_geometry(GEOMETRY_FILE)
        self.route = generate_baseline_route(self.geometry)
        self.loop = tuple(
            point for point in self.route.waypoints
            if point.id.startswith("c_loop_")
        )

    def test_repository_route_matches_generator(self):
        stored = load_route(ROUTE_FILE)
        self.assertEqual(stored, self.route)

    def test_baseline_is_explicitly_uncalibrated(self):
        self.assertFalse(self.route.calibrated)
        self.assertIn("UNCALIBRATED", self.route.source["warning"])

    def test_route_starts_and_finishes_at_p_origin(self):
        first, last = self.route.waypoints[0], self.route.waypoints[-1]
        self.assertEqual((first.id, first.zone, first.x, first.y, first.yaw_deg),
                         ("p_start", "P", 0.0, 0.0, 0.0))
        self.assertEqual((last.id, last.zone, last.x, last.y),
                         ("p_finish", "P", 0.0, 0.0))

    def test_route_visits_task_corridor_loop_and_return_in_order(self):
        ids = [point.id for point in self.route.waypoints]
        self.assertLess(ids.index("a_task"), ids.index("b_corridor_01"))
        self.assertLess(ids.index("b_corridor_01"), ids.index("c_loop_000"))
        self.assertLess(ids.index("c_loop_000"), ids.index("b_corridor_return"))
        self.assertLess(ids.index("b_corridor_return"), ids.index("p_finish"))

    def test_loop_is_closed_clockwise_and_well_sampled(self):
        self.assertAlmostEqual(self.loop[0].x, self.loop[-1].x)
        self.assertAlmostEqual(self.loop[0].y, self.loop[-1].y)
        signed_area = 0.5 * sum(
            first.x * second.y - second.x * first.y
            for first, second in zip(self.loop, self.loop[1:])
        )
        self.assertLess(signed_area, 0.0)
        spacings = [
            math.hypot(second.x - first.x, second.y - first.y)
            for first, second in zip(self.loop, self.loop[1:])
        ]
        self.assertGreater(min(spacings), 0.0)
        self.assertLessEqual(max(spacings), MAX_LOOP_POINT_SPACING_M)

    def test_loop_extents_are_rule_centerline(self):
        self.assertAlmostEqual(min(point.x for point in self.loop), 0.25)
        self.assertAlmostEqual(max(point.x for point in self.loop), 3.75)
        self.assertAlmostEqual(min(point.y for point in self.loop), 2.75)
        self.assertAlmostEqual(max(point.y for point in self.loop), 3.90)

    def test_loop_heading_is_continuous(self):
        self.assertLessEqual(max(
            angular_distance_deg(first.yaw_deg, second.yaw_deg)
            for first, second in zip(self.loop, self.loop[1:])
        ), 45.0)

    def test_every_coordinate_is_finite_and_inside_field(self):
        for point in self.route.waypoints:
            self.assertTrue(math.isfinite(point.x))
            self.assertTrue(math.isfinite(point.y))
            self.assertGreaterEqual(point.x, self.geometry.x_min_m)
            self.assertLessEqual(point.x, self.geometry.x_max_m)
            self.assertGreaterEqual(point.y, self.geometry.y_min_m)
            self.assertLessEqual(point.y, self.geometry.y_max_m)


class TestRouteValidation(unittest.TestCase):
    def setUp(self):
        self.route = generate_baseline_route(load_field_geometry(GEOMETRY_FILE))

    def test_duplicate_ids_are_rejected(self):
        points = list(self.route.waypoints)
        points[1] = replace(points[1], id=points[2].id)
        with self.assertRaisesRegex(RouteValidationError, "unique"):
            validate_route(replace(self.route, waypoints=tuple(points)))

    def test_nonfinite_coordinate_is_rejected(self):
        points = list(self.route.waypoints)
        points[1] = replace(points[1], x=float("nan"))
        with self.assertRaisesRegex(RouteValidationError, "finite"):
            validate_route(replace(self.route, waypoints=tuple(points)))

    def test_wrong_zone_is_rejected(self):
        points = list(self.route.waypoints)
        points[1] = replace(points[1], zone="C")
        with self.assertRaisesRegex(RouteValidationError, "lies in"):
            validate_route(replace(self.route, waypoints=tuple(points)))

    def test_missing_or_open_c_loop_is_rejected(self):
        points = [
            point for point in self.route.waypoints
            if point.id != "c_loop_000"
        ]
        with self.assertRaises(RouteValidationError):
            validate_route(replace(self.route, waypoints=tuple(points)))

    def test_load_rejects_string_calibrated_flag(self):
        raw = yaml.safe_load(ROUTE_FILE.read_text(encoding="utf-8"))
        raw["calibrated"] = "false"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.yaml"
            path.write_text(yaml.safe_dump(raw), encoding="utf-8")
            with self.assertRaisesRegex(RouteValidationError, "calibrated"):
                load_route(path)

    def test_atomic_write_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "route.yaml"
            write_route_atomic(self.route, path)
            self.assertEqual(load_route(path), self.route)
            self.assertEqual(list(path.parent.glob("*.tmp")), [])

    def test_replace_waypoints_validates_before_return(self):
        points = list(self.route.waypoints)
        points[-1] = RoutePoint("not_p", "A", 0.1, 0.1, 0.0)
        with self.assertRaises(RouteValidationError):
            replace_waypoints(self.route, points)


if __name__ == "__main__":
    unittest.main()
