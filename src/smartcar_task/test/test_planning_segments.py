"""Contracts for the current forward-only planning-segment route."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.planning_segments import (  # noqa: E402
    PlanningSegment,
    PlanningSegmentError,
    load_planning_segments,
    materialize_mission_route,
    materialize_navigation_segments,
    select_segment_prefix,
)
from smartcar_task.waypoints import (  # noqa: E402
    Waypoint,
    is_heading_locked,
    load_waypoint_document,
)


NAV_ONLY_FILE = (
    PACKAGE_ROOT.parent / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml"
)
DEFAULT_FILE = (
    PACKAGE_ROOT.parent / "smartcar_nav2" / "config" / "waypoints" / "default_waypoints.yaml"
)


def waypoint(waypoint_id, task, x, goal_profile="standard", heading_mode=None):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        task=task,
        direction="forward",
        id=waypoint_id,
        goal_profile=goal_profile,
        heading_mode=heading_mode,
    )


class PlanningSegmentTests(unittest.TestCase):
    def test_current_route_has_three_forward_costmap_replanning_boundaries(self):
        document, waypoints = load_waypoint_document(NAV_ONLY_FILE)
        segments = load_planning_segments(document, waypoints)
        actions = materialize_navigation_segments(waypoints, segments)

        self.assertEqual([segment.id for segment in segments], [
            "p_to_qr", "qr_to_vlm", "return_to_p",
        ])
        self.assertTrue(all(segment.direction == "forward" for segment in segments))
        self.assertEqual([[waypoint.id for waypoint in action] for action in actions], [
            ["a_task_observe"],
            ["via_1", "via_2", "via_3", "via_6", "c_corner_1"],
            ["via_4", "via_5", "via_7", "p_finish"],
        ])
        self.assertTrue(all(
            waypoint.direction == "forward"
            for action in actions for waypoint in action
        ))

    def test_default_and_navigation_only_keep_identical_route_geometry(self):
        default_document, default_waypoints = load_waypoint_document(DEFAULT_FILE)
        nav_document, nav_waypoints = load_waypoint_document(NAV_ONLY_FILE)
        default_segments = load_planning_segments(default_document, default_waypoints)
        nav_segments = load_planning_segments(nav_document, nav_waypoints)

        self.assertEqual(
            [(waypoint.id, waypoint.position, waypoint.orientation, waypoint.direction,
              waypoint.goal_profile, is_heading_locked(waypoint))
             for waypoint in default_waypoints],
            [(waypoint.id, waypoint.position, waypoint.orientation, waypoint.direction,
              waypoint.goal_profile, is_heading_locked(waypoint))
             for waypoint in nav_waypoints],
        )
        self.assertEqual(default_segments, nav_segments)
        self.assertEqual(
            [waypoint.task for waypoint in default_waypoints],
            ["start", "qr", "via", "via", "via", "via", "vlm", "via", "via", "via", "return"],
        )
        self.assertEqual(
            [waypoint.task for waypoint in nav_waypoints],
            ["start", "nav", "via", "via", "via", "via", "nav", "via", "via", "via", "return"],
        )

    def test_forward_precise_endpoint_is_the_only_nonstandard_through_shape(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("via", "via", 1.0),
            waypoint("qr", "qr", 2.0, "precise", "locked"),
            waypoint("p_finish", "return", 0.0),
        )
        segments = (
            PlanningSegment("to_qr", "forward", "p_start", "qr", ("via",)),
            PlanningSegment("return", "forward", "qr", "p_finish"),
        )
        actions = materialize_navigation_segments(waypoints, segments)
        self.assertEqual([[item.id for item in action] for action in actions], [
            ["via", "qr"], ["p_finish"],
        ])

    def test_via_waypoints_can_only_be_planning_constraints(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("via", "via", 1.0),
            waypoint("nav_through", "nav", 2.0),
            waypoint("nav_end", "nav", 3.0),
            waypoint("p_finish", "return", 0.0),
        )
        via_as_endpoint = (
            PlanningSegment("first", "forward", "p_start", "via"),
            PlanningSegment(
                "return", "forward", "via", "p_finish", ("nav_through",)
            ),
        )
        with self.assertRaisesRegex(PlanningSegmentError, "end_id must not be a via"):
            materialize_navigation_segments(waypoints, via_as_endpoint)

        non_via_as_through = (
            PlanningSegment(
                "first", "forward", "p_start", "nav_end",
                ("via", "nav_through"),
            ),
            PlanningSegment("return", "forward", "nav_end", "p_finish"),
        )
        with self.assertRaisesRegex(PlanningSegmentError, "must reference a via"):
            materialize_navigation_segments(waypoints, non_via_as_through)

    def test_reverse_segment_is_rejected_before_navigation(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("qr", "qr", 1.0),
            waypoint("p_finish", "return", 0.0),
        )
        segments = (
            PlanningSegment("to_qr", "reverse", "p_start", "qr"),
            PlanningSegment("return", "forward", "qr", "p_finish"),
        )
        with self.assertRaisesRegex(PlanningSegmentError, "must be one of forward"):
            materialize_navigation_segments(waypoints, segments)

    def test_route_prefix_can_only_end_at_a_declared_segment(self):
        document, waypoints = load_waypoint_document(NAV_ONLY_FILE)
        segments = load_planning_segments(document, waypoints)
        self.assertEqual(
            [segment.id for segment in select_segment_prefix(segments, "qr_to_vlm")],
            ["p_to_qr", "qr_to_vlm"],
        )
        with self.assertRaisesRegex(PlanningSegmentError, "not in the route"):
            select_segment_prefix(segments, "c_corner_1")

    def test_materialized_mission_route_is_semantically_valid_and_forward(self):
        document, waypoints = load_waypoint_document(DEFAULT_FILE)
        route = materialize_mission_route(
            waypoints, load_planning_segments(document, waypoints))
        self.assertEqual(route[0].id, "p_start")
        self.assertEqual(route[-1].id, "p_finish")
        self.assertTrue(all(waypoint.direction == "forward" for waypoint in route))


if __name__ == "__main__":
    unittest.main()
