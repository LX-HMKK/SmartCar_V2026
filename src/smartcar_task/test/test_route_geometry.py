"""Contracts for Nav2 position-only transit waypoints."""
import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.route_geometry import (  # noqa: E402
    RouteGeometryError,
    ZERO_QUATERNION,
    materialize_free_yaws,
)
from smartcar_task.waypoints import (  # noqa: E402
    HEADING_LOCKED_TASKS,
    Waypoint,
    is_heading_locked,
    is_zero_quaternion,
)


def quaternion(yaw):
    half_yaw = yaw / 2.0
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def waypoint(
    waypoint_id,
    task,
    x,
    y,
    heading=0.0,
    direction="forward",
    heading_mode=None,
):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, y, 0.0),
        orientation=quaternion(heading),
        task=task,
        direction=direction,
        id=waypoint_id,
        heading_mode=heading_mode,
    )


class RouteGeometryTests(unittest.TestCase):
    def test_only_start_qr_and_vlm_positions_keep_authored_headings(self):
        route = (
            waypoint("p_start", "start", 0.0, 0.0, 0.0),
            waypoint("qr", "qr", 1.0, 0.0, 0.31),
            waypoint("nav", "nav", 2.0, 0.0, -2.2),
            waypoint("via", "via", 3.0, 1.0, -1.8),
            waypoint("loop", "loop", 3.0, 2.0, -1.4),
            waypoint("corridor", "corridor", 2.0, 2.0, -0.9),
            waypoint("vlm", "vlm", 1.0, 2.0, 1.17, "reverse"),
            waypoint("p_finish", "return", 0.0, 2.0, -0.72, "reverse"),
        )

        normalized = materialize_free_yaws(route)

        self.assertEqual(
            HEADING_LOCKED_TASKS,
            frozenset({"start", "qr", "vlm"}),
        )
        for index in (0, 1, 6):
            self.assertEqual(normalized[index].orientation, route[index].orientation)
        for index in (2, 3, 4, 5, 7):
            self.assertEqual(normalized[index].orientation, ZERO_QUATERNION)
            self.assertTrue(is_zero_quaternion(normalized[index].orientation))

    def test_stale_transit_arrow_cannot_change_an_action_goal(self):
        route = (
            waypoint("p_start", "start", 0.0, 0.0),
            waypoint("first", "nav", 1.0, 0.0, -2.6),
            waypoint("second", "corridor", 1.0, 1.0, 2.2, "reverse"),
            waypoint("p_finish", "return", 0.0, 1.0, 0.5, "reverse"),
        )

        normalized = materialize_free_yaws(route)

        self.assertEqual(normalized[1].orientation, ZERO_QUATERNION)
        self.assertEqual(normalized[2].orientation, ZERO_QUATERNION)
        self.assertEqual(normalized[0].orientation, route[0].orientation)
        self.assertEqual(normalized[3].orientation, ZERO_QUATERNION)

    def test_nav_can_explicitly_keep_an_authored_heading(self):
        nav = waypoint(
            "nav_qr_substitute", "nav", 1.0, 0.0, 0.63,
            heading_mode="locked",
        )

        normalized = materialize_free_yaws((nav,))

        self.assertTrue(is_heading_locked(nav))
        self.assertEqual(normalized[0].orientation, nav.orientation)

    def test_coincident_transit_positions_remain_position_constraints(self):
        route = (
            waypoint("p_start", "start", 0.0, 0.0),
            waypoint("nav", "nav", 0.0, 0.0),
        )

        normalized = materialize_free_yaws(route)

        self.assertTrue(is_zero_quaternion(normalized[-1].orientation))

    def test_heading_locked_positions_require_finite_unit_quaternions(self):
        invalid_zero = Waypoint(
            "odom_combined", (0.0, 0.0, 0.0), ZERO_QUATERNION, "qr", id="qr"
        )
        invalid_nan = Waypoint(
            "odom_combined", (0.0, 0.0, 0.0), (0.0, 0.0, math.nan, 1.0), "vlm", id="vlm"
        )
        invalid_locked_nav = Waypoint(
            "odom_combined", (0.0, 0.0, 0.0), ZERO_QUATERNION, "nav",
            id="nav", heading_mode="locked",
        )

        with self.assertRaisesRegex(RouteGeometryError, "unit orientation"):
            materialize_free_yaws((invalid_zero,))
        with self.assertRaisesRegex(RouteGeometryError, "finite unit orientation"):
            materialize_free_yaws((invalid_nan,))
        with self.assertRaisesRegex(RouteGeometryError, "unit orientation"):
            materialize_free_yaws((invalid_locked_nav,))

    def test_rejects_invalid_direction_and_nonfinite_position(self):
        invalid_direction = waypoint("nav", "nav", 1.0, 0.0, direction="sideways")
        invalid_position = waypoint("nav", "nav", math.nan, 0.0)

        with self.assertRaisesRegex(RouteGeometryError, "invalid travel direction"):
            materialize_free_yaws((invalid_direction,))
        with self.assertRaisesRegex(RouteGeometryError, "position must be finite"):
            materialize_free_yaws((invalid_position,))


if __name__ == "__main__":
    unittest.main()
