"""Contracts for the start-time C-zone direction selection."""

import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.c_zone_direction import (  # noqa: E402
    CLOCKWISE,
    COUNTERCLOCKWISE,
    C_ZONE_MIRROR_WAYPOINT_IDS,
    apply_c_zone_direction,
    normalize_c_zone_direction,
)
from smartcar_task.waypoints import load_waypoint_document  # noqa: E402


NAV_ONLY_FILE = (
    PACKAGE_ROOT.parent / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml"
)


def yaw(orientation):
    _x, _y, z, w = orientation
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class CZoneDirectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _document, cls.route = load_waypoint_document(NAV_ONLY_FILE)

    def test_counterclockwise_is_the_authored_identity(self):
        self.assertIs(apply_c_zone_direction(self.route), self.route)
        self.assertIs(
            apply_c_zone_direction(self.route, COUNTERCLOCKWISE),
            self.route,
        )

    def test_clockwise_reflects_only_reviewed_c_zone_constraints(self):
        clockwise = apply_c_zone_direction(
            self.route, CLOCKWISE)
        authored = {waypoint.id: waypoint for waypoint in self.route}
        reflected = {waypoint.id: waypoint for waypoint in clockwise}

        self.assertEqual(
            [waypoint.id for waypoint in clockwise],
            [waypoint.id for waypoint in self.route],
        )
        for waypoint_id, original in authored.items():
            with self.subTest(waypoint_id=waypoint_id):
                transformed = reflected[waypoint_id]
                self.assertEqual(transformed.task, original.task)
                self.assertEqual(transformed.direction, original.direction)
                self.assertEqual(transformed.goal_profile, original.goal_profile)
                self.assertEqual(transformed.heading_mode, original.heading_mode)
                if waypoint_id in C_ZONE_MIRROR_WAYPOINT_IDS:
                    self.assertAlmostEqual(
                        transformed.position[0], 4.0 - original.position[0]
                    )
                    self.assertEqual(transformed.position[1:], original.position[1:])
                else:
                    self.assertEqual(transformed, original)

        self.assertEqual(
            reflected["c_corner_1"].orientation,
            (0.0, 0.0, 0.0, 1.0),
        )
        self.assertAlmostEqual(yaw(reflected["c_corner_1"].orientation), 0.0)
        self.assertAlmostEqual(
            reflected["via_2"].position[0], 1.9585801761583017,
        )
        self.assertAlmostEqual(
            reflected["via_3"].position[0], 0.48370787081724487,
        )

    def test_clockwise_transform_is_an_involution(self):
        restored = apply_c_zone_direction(
            apply_c_zone_direction(self.route, CLOCKWISE),
            CLOCKWISE,
        )
        for original, recovered in zip(self.route, restored):
            with self.subTest(waypoint_id=original.id):
                self.assertEqual(original.id, recovered.id)
                for original_value, recovered_value in zip(
                        original.position, recovered.position):
                    self.assertAlmostEqual(original_value, recovered_value)
                self.assertAlmostEqual(
                    math.cos(yaw(original.orientation)),
                    math.cos(yaw(recovered.orientation)),
                )
                self.assertAlmostEqual(
                    math.sin(yaw(original.orientation)),
                    math.sin(yaw(recovered.orientation)),
                )

    def test_normalization_accepts_only_declared_directions(self):
        self.assertEqual(
            normalize_c_zone_direction(" CLOCKWISE "),
            CLOCKWISE,
        )
        with self.assertRaisesRegex(ValueError, "c_zone_direction"):
            normalize_c_zone_direction("left")

    def test_missing_fixed_c_zone_id_fails_closed(self):
        incomplete = tuple(
            waypoint for waypoint in self.route if waypoint.id != "via_6"
        )

        for direction in (COUNTERCLOCKWISE, CLOCKWISE):
            with self.subTest(direction=direction):
                with self.assertRaisesRegex(ValueError, "missing via_6"):
                    apply_c_zone_direction(incomplete, direction)


if __name__ == "__main__":
    unittest.main()
