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
    select_segment_prefix,
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
    heading_mode=None,
):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        task=task,
        direction=direction,
        id=waypoint_id,
        goal_profile=goal_profile,
        heading_mode=heading_mode,
    )


class PlanningSegmentActionTests(unittest.TestCase):
    def test_strict_route_rejects_forward_reverse_handoff_and_return(self):
        document, waypoints = load_waypoint_document(NAV_ONLY_FILE)
        segments = list(load_planning_segments(document, waypoints))
        reverse_handoff_segments = list(segments)
        reverse_handoff_segments[1] = replace(
            reverse_handoff_segments[1], direction="forward")
        semantic_waypoints = list(waypoints)
        semantic_waypoints[2] = replace(
            semantic_waypoints[2], goal_profile="standard")

        with self.assertRaisesRegex(
            ValueError, "reverse_handoff goals must be reverse"
        ):
            materialize_mission_route(
                semantic_waypoints, reverse_handoff_segments)

        direct_return_segments = list(segments)
        direct_return_segments[2] = replace(
            direct_return_segments[2], direction="reverse")
        reverse_route = materialize_mission_route(
            waypoints, direct_return_segments)
        self.assertEqual(reverse_route[-1].direction, "reverse")

        direct_return_segments[2] = replace(
            direct_return_segments[2], direction="forward")
        with self.assertRaisesRegex(ValueError, "direction must be reverse"):
            materialize_mission_route(waypoints, direct_return_segments)

    def test_strict_route_rejects_reordered_qr_and_vlm_waypoints(self):
        _document, waypoints = load_waypoint_document(DEFAULT_WAYPOINTS_FILE)
        waypoints = tuple(
            replace(waypoint, goal_profile="standard")
            if waypoint.id == "c_corner_1" else waypoint
            for waypoint in waypoints
        )
        segments = (
            PlanningSegment(
                "to_vlm", "forward", "p_start", "c_corner_1", ("via_2",)
            ),
            PlanningSegment(
                "to_qr", "forward", "c_corner_1", "a_task_observe",
                ("via_1", "via_3"),
            ),
            PlanningSegment(
                "return", "forward", "a_task_observe", "p_finish",
            ),
        )

        with self.assertRaisesRegex(ValueError, "expected qr or nav, got vlm"):
            materialize_mission_route(waypoints, segments)

    def test_preserves_adjacent_same_direction_replanning_boundaries(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("a", "via", 1.0),
            waypoint("b", "via", 2.0),
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

    def test_nav_only_route_uses_three_semantic_navigation_actions(self):
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
                ("reverse", ["via_2", "c_corner_1"]),
                ("reverse", ["via_1", "via_3", "p_finish"]),
            ],
        )

    def test_navigation_test_prefix_cannot_skip_earlier_segments(self):
        document, waypoints = load_waypoint_document(NAV_ONLY_FILE)
        segments = load_planning_segments(document, waypoints)

        selected = select_segment_prefix(segments, "p_to_qr")

        self.assertEqual([segment.id for segment in selected], ["p_to_qr"])
        selected = select_segment_prefix(segments, "qr_to_vlm")
        self.assertEqual(
            [segment.id for segment in selected],
            ["p_to_qr", "qr_to_vlm"],
        )
        with self.assertRaisesRegex(PlanningSegmentError, "not in the route"):
            select_segment_prefix(segments, "return_to_p_typo")

    def test_deployment_route_uses_the_verified_segments_with_media_tasks(self):
        document, waypoints = load_waypoint_document(DEFAULT_WAYPOINTS_FILE)
        segments = load_planning_segments(document, waypoints)

        actions = materialize_navigation_segments(waypoints, segments)

        self.assertFalse(document["calibrated"])
        self.assertEqual(
            [
                (action[0].direction, [waypoint.id for waypoint in action])
                for action in actions
            ],
            [
                ("forward", ["a_task_observe"]),
                ("reverse", ["via_2", "c_corner_1"]),
                ("reverse", ["via_1", "via_3", "p_finish"]),
            ],
        )
        self.assertEqual(
            [waypoint.task for waypoint in waypoints],
            ["start", "qr", "via", "vlm", "via", "via", "return"],
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

    def test_allows_terminal_locked_reverse_handoff_in_reverse_through_poses(self):
        waypoints = (
            waypoint("p_start", "start", 0.0),
            waypoint("via", "via", 1.0, "reverse"),
            waypoint(
                "c_corner_1",
                "nav",
                2.0,
                "reverse",
                goal_profile="reverse_handoff",
                heading_mode="locked",
            ),
            waypoint("p_finish", "return", 3.0, "reverse"),
        )
        segments = (
            PlanningSegment(
                "to_c1", "reverse", "p_start", "c_corner_1", ("via",)),
            PlanningSegment("return", "reverse", "c_corner_1", "p_finish"),
        )

        actions = materialize_navigation_segments(waypoints, segments)

        self.assertEqual(
            [[waypoint.id for waypoint in action] for action in actions],
            [["via", "c_corner_1"], ["p_finish"]],
        )
        self.assertEqual(
            [waypoint.direction for waypoint in actions[0]], ["reverse", "reverse"]
        )

    def test_rejects_invalid_reverse_handoff_through_poses_shapes(self):
        locked_handoff = waypoint(
            "c_corner_1",
            "nav",
            2.0,
            "reverse",
            goal_profile="reverse_handoff",
            heading_mode="locked",
        )
        cases = (
            (
                "handoff_is_not_terminal",
                (
                    waypoint("p_start", "start", 0.0),
                    locked_handoff,
                    waypoint("via", "via", 3.0, "reverse"),
                    waypoint("p_finish", "return", 4.0, "reverse"),
                ),
                (
                    PlanningSegment(
                        "to_finish",
                        "reverse",
                        "p_start",
                        "p_finish",
                        ("c_corner_1", "via"),
                    ),
                ),
            ),
            (
                "handoff_is_unlocked",
                (
                    waypoint("p_start", "start", 0.0),
                    waypoint("via", "via", 1.0, "reverse"),
                    waypoint(
                        "c_corner_1",
                        "nav",
                        2.0,
                        "reverse",
                        goal_profile="reverse_handoff",
                    ),
                    waypoint("p_finish", "return", 3.0, "reverse"),
                ),
                (
                    PlanningSegment(
                        "to_c1",
                        "reverse",
                        "p_start",
                        "c_corner_1",
                        ("via",),
                    ),
                    PlanningSegment(
                        "return", "reverse", "c_corner_1", "p_finish"),
                ),
            ),
            (
                "handoff_in_forward_action",
                (
                    waypoint("p_start", "start", 0.0),
                    waypoint("via", "via", 1.0),
                    locked_handoff,
                    waypoint("p_finish", "return", 3.0),
                ),
                (
                    PlanningSegment(
                        "to_c1",
                        "forward",
                        "p_start",
                        "c_corner_1",
                        ("via",),
                    ),
                    PlanningSegment(
                        "return", "forward", "c_corner_1", "p_finish"),
                ),
            ),
            (
                "nonstandard_precedes_handoff",
                (
                    waypoint("p_start", "start", 0.0),
                    waypoint(
                        "precise_via",
                        "via",
                        1.0,
                        "reverse",
                        goal_profile="precise",
                    ),
                    locked_handoff,
                    waypoint("p_finish", "return", 3.0, "reverse"),
                ),
                (
                    PlanningSegment(
                        "to_c1",
                        "reverse",
                        "p_start",
                        "c_corner_1",
                        ("precise_via",),
                    ),
                    PlanningSegment(
                        "return", "reverse", "c_corner_1", "p_finish"),
                ),
            ),
        )

        for label, waypoints, segments in cases:
            with self.subTest(label=label):
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
