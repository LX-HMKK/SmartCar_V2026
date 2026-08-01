"""Contracts for task-owned planning-segment action materialization."""

from dataclasses import replace
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.planning_segments import (  # noqa: E402
    PlanningSegment,
    PlanningSegmentError,
    materialize_mission_route,
    materialize_navigation_segments,
)
from smartcar_task.waypoints import Waypoint, load_waypoint_document  # noqa: E402
from smartcar_task.planning_segments import load_planning_segments  # noqa: E402


NAV_ONLY_FILE = (
    PACKAGE_ROOT.parent
    / "smartcar_nav2"
    / "config"
    / "waypoints"
    / "nav_only.yaml"
)
DEFAULT_WAYPOINTS_FILE = (
    PACKAGE_ROOT.parent
    / "smartcar_nav2"
    / "config"
    / "waypoints"
    / "default_waypoints.yaml"
)


def waypoint(
    waypoint_id,
    task,
    x,
    direction="forward",
    goal_profile="standard",
):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        task=task,
        direction=direction,
        id=waypoint_id,
        goal_profile=goal_profile,
    )


class PlanningSegmentActionTests(unittest.TestCase):
    def test_strict_route_rejects_segment_direction_override(self):
        document, waypoints = load_waypoint_document(NAV_ONLY_FILE)
        segments = list(load_planning_segments(document, waypoints))
        segments[2] = replace(segments[2], direction="forward")

        with self.assertRaisesRegex(ValueError, "direction must be reverse"):
            materialize_mission_route(waypoints, segments)

    def test_strict_route_rejects_reordered_qr_and_vlm_waypoints(self):
        _document, waypoints = load_waypoint_document(DEFAULT_WAYPOINTS_FILE)
        segments = (
            PlanningSegment("to_vlm", "reverse", "p_start", "c_corner_1"),
            PlanningSegment("to_qr", "forward", "c_corner_1", "a_task_observe"),
            PlanningSegment(
                "return", "forward", "a_task_observe", "p_finish",
                ("c_corner_3", "b_corridor_return"),
            ),
        )

        with self.assertRaisesRegex(ValueError, "expected qr or nav, got vlm"):
            materialize_mission_route(waypoints, segments)

    def test_preserves_adjacent_same_direction_replanning_boundaries(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("a", "via", 1.0),
            waypoint("b", "corridor", 2.0),
            waypoint("p_finish", "return", 3.0),
        )
        segments = (
            PlanningSegment("one", "forward", "p_start", "a"),
            PlanningSegment("two", "forward", "a", "b"),
            PlanningSegment("three", "forward", "b", "p_finish"),
        )

        actions = materialize_navigation_segments(waypoints, segments)

        self.assertEqual(
            [[waypoint.id for waypoint in action] for action in actions],
            [["a"], ["b"], ["p_finish"]],
        )

    def test_keeps_qr_and_vlm_as_hard_action_boundaries(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("qr", "qr", 1.0),
            waypoint("via", "via", 2.0, "reverse"),
            waypoint("vlm", "vlm", 3.0, "reverse"),
            waypoint("p_finish", "return", 4.0, "reverse"),
        )
        segments = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment("to_vlm", "reverse", "qr", "vlm", ("via",)),
            PlanningSegment("return", "reverse", "vlm", "p_finish"),
        )

        actions = materialize_navigation_segments(waypoints, segments)

        self.assertEqual(
            [[waypoint.id for waypoint in action] for action in actions],
            [["qr"], ["via", "vlm"], ["p_finish"]],
        )

    def test_nav_only_route_uses_bounded_reverse_actions(self):
        document, waypoints = load_waypoint_document(NAV_ONLY_FILE)
        segments = load_planning_segments(document, waypoints)

        actions = materialize_navigation_segments(waypoints, segments)

        self.assertEqual(
            [
                (action[0].direction, [waypoint.id for waypoint in action])
                for action in actions
            ],
            [
                ("forward", ["a_task_observe"]),
                ("reverse", ["b_corridor_gate", "b_corridor_enter"]),
                (
                    "reverse",
                    ["c_entry_west", "c_corner_1", "c_corner_2", "c_corner_3"],
                ),
                ("reverse", ["c_corner_4", "b_corridor_return_enter"]),
                (
                    "reverse",
                    ["b_corridor_return_drop", "b_corridor_return", "p_finish"],
                ),
            ],
        )

    def test_rejects_nonstandard_goal_in_multi_goal_action(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("via", "via", 1.0),
            waypoint("qr", "qr", 2.0, goal_profile="precise"),
            waypoint("p_finish", "return", 3.0),
        )
        segments = (
            PlanningSegment("to_qr", "forward", "p_start", "qr", ("via",)),
            PlanningSegment("return", "forward", "qr", "p_finish"),
        )

        with self.assertRaisesRegex(
            PlanningSegmentError,
            "NavigateThroughPoses cannot combine nonstandard goal profiles",
        ):
            materialize_navigation_segments(waypoints, segments)

    def test_action_goals_take_direction_from_their_segment(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("handoff", "via", 1.0, "forward"),
            waypoint("p_finish", "return", 2.0, "reverse"),
        )
        segments = (
            PlanningSegment("outbound", "reverse", "p_start", "handoff"),
            PlanningSegment("return", "reverse", "handoff", "p_finish"),
        )

        actions = materialize_navigation_segments(waypoints, segments)

        self.assertEqual(
            [[waypoint.id for waypoint in action] for action in actions],
            [["handoff"], ["p_finish"]],
        )
        self.assertEqual(
            [waypoint.direction for action in actions for waypoint in action],
            ["reverse", "reverse"],
        )


if __name__ == "__main__":
    unittest.main()
