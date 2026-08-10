"""Contracts for native Nav2 transit-pose materialization."""

import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.route_geometry import (  # noqa: E402
    RouteGeometryError,
    materialize_free_yaws,
)
from smartcar_task.waypoints import Waypoint, is_heading_locked  # noqa: E402


def quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def waypoint(waypoint_id, task, x, y, orientation=(0.0, 0.0, 0.0, 0.0), **kwargs):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, y, 0.0),
        orientation=orientation,
        task=task,
        direction="forward",
        id=waypoint_id,
        **kwargs,
    )


class RouteGeometryTests(unittest.TestCase):
    def test_transit_waypoints_become_valid_outgoing_tangent_poses(self):
        route = (
            waypoint("p_start", "start", 0.0, 0.0, quaternion(0.0)),
            waypoint("a", "qr", 1.0, 0.0, quaternion(0.2), goal_profile="precise"),
            waypoint("via", "via", 1.0, 1.0),
            waypoint("c", "vlm", 2.0, 1.0, quaternion(0.0), goal_profile="precise"),
            waypoint("p_finish", "return", 0.0, 0.0),
        )
        materialized = materialize_free_yaws(route)

        self.assertEqual(materialized[0].orientation, route[0].orientation)
        self.assertEqual(materialized[1].orientation, route[1].orientation)
        self.assertEqual(materialized[3].orientation, route[3].orientation)
        self.assertAlmostEqual(materialized[2].orientation[2], 0.0, places=6)
        self.assertAlmostEqual(materialized[2].orientation[3], 1.0, places=6)
        self.assertAlmostEqual(
            math.sqrt(sum(value * value for value in materialized[-1].orientation)),
            1.0,
        )

    def test_final_free_goal_uses_its_incoming_tangent(self):
        route = (
            waypoint("p_start", "start", 0.0, 0.0, quaternion(0.0)),
            waypoint("p_finish", "return", 0.0, 1.0),
        )
        materialized = materialize_free_yaws(route)
        self.assertAlmostEqual(materialized[-1].orientation[2], math.sin(math.pi / 4.0))
        self.assertAlmostEqual(materialized[-1].orientation[3], math.cos(math.pi / 4.0))

    def test_explicit_locked_nav_heading_is_preserved(self):
        locked = waypoint(
            "nav_locked", "nav", 1.0, 0.0, quaternion(0.7), heading_mode="locked")
        materialized = materialize_free_yaws((locked,))
        self.assertTrue(is_heading_locked(locked))
        self.assertEqual(materialized[0].orientation, locked.orientation)

    def test_free_waypoint_without_a_route_tangent_is_rejected(self):
        with self.assertRaisesRegex(RouteGeometryError, "no route tangent"):
            materialize_free_yaws((waypoint("via", "via", 1.0, 1.0),))

    def test_rejects_reverse_direction_and_invalid_locked_orientation(self):
        reverse = waypoint("via", "via", 1.0, 0.0)
        reverse = reverse.__class__(**{**reverse.__dict__, "direction": "reverse"})
        with self.assertRaisesRegex(RouteGeometryError, "invalid travel direction"):
            materialize_free_yaws((reverse,))

        invalid = waypoint("qr", "qr", 1.0, 0.0, (0.0, 0.0, 0.0, 0.0))
        with self.assertRaisesRegex(RouteGeometryError, "unit orientation"):
            materialize_free_yaws((invalid,))


if __name__ == "__main__":
    unittest.main()
