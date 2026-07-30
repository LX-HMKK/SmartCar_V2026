"""Contracts for the editable planning-segment route model."""
import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT.parent / "smartcar_task"))

from smartcar_task.waypoints import (  # noqa: E402
    Waypoint,
    load_waypoint_document,
    validate_waypoints,
)
from smartcar_tools.field_reference import load_field_reference  # noqa: E402
from smartcar_tools.planning_segments import (  # noqa: E402
    PlanningSegment,
    PlanningSegmentError,
    derive_planning_segments,
    load_planning_segments,
    materialize_route,
    planning_segments_document,
    validate_planning_segments,
)
from smartcar_tools.route_preflight import (  # noqa: E402
    LatticePreflightPlanner,
    Pose2D,
    preflight_route,
)


GEOMETRY_FILE = PACKAGE_ROOT / "config" / "routes" / "field_geometry.yaml"
NAV_ONLY_FILE = (
    PACKAGE_ROOT.parent / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml"
)


def waypoint(waypoint_id, task, x, y, direction="forward"):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, y, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        task=task,
        direction=direction,
        id=waypoint_id,
    )


def mission_waypoints():
    return (
        waypoint("p_start", "start", 0.0, 0.0),
        waypoint("qr", "qr", 1.0, 0.0),
        waypoint("corridor", "corridor", 1.5, 1.0, "reverse"),
        waypoint("vlm", "vlm", 2.0, 1.0, "reverse"),
        waypoint("loop", "loop", 2.5, 3.9),
        waypoint("return_corridor", "corridor", 1.5, 1.5),
        waypoint("p_finish", "return", 0.0, 0.0),
    )


class PlanningSegmentTests(unittest.TestCase):
    def test_segments_materialize_an_ordered_semantic_route(self):
        source = mission_waypoints()
        segments = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment(
                "reverse_to_vlm", "reverse", "qr", "vlm", ("corridor",)
            ),
            PlanningSegment(
                "return", "forward", "vlm", "p_finish", ("loop", "return_corridor")
            ),
        )

        checked = validate_planning_segments(segments, source)
        route = materialize_route(source, checked)

        self.assertEqual(
            [item.id for item in route],
            [
                "p_start", "qr", "corridor", "vlm", "loop",
                "return_corridor", "p_finish",
            ],
        )
        self.assertEqual(
            [item.direction for item in route],
            [
                "forward", "forward", "reverse", "reverse", "forward",
                "forward", "forward",
            ],
        )
        self.assertEqual(validate_waypoints(route), route)

    def test_segments_require_one_contiguous_coverage_of_all_waypoints(self):
        source = mission_waypoints()
        incomplete = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment("to_vlm", "reverse", "qr", "vlm"),
            PlanningSegment("return", "forward", "vlm", "p_finish"),
        )
        with self.assertRaisesRegex(PlanningSegmentError, "cover every waypoint"):
            validate_planning_segments(incomplete, source)

        discontinuous = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment("bad", "reverse", "corridor", "vlm"),
            PlanningSegment(
                "return", "forward", "vlm", "p_finish", ("loop", "return_corridor")
            ),
        )
        with self.assertRaisesRegex(PlanningSegmentError, "previous segment end"):
            validate_planning_segments(discontinuous, source)

    def test_legacy_waypoints_gain_editable_boundaries_without_losing_yaml_schema(self):
        source = mission_waypoints()
        derived = derive_planning_segments(source)
        self.assertEqual(
            [(item.start_id, item.end_id, item.direction) for item in derived],
            [
                ("p_start", "qr", "forward"),
                ("qr", "vlm", "reverse"),
                ("vlm", "p_finish", "forward"),
            ],
        )
        root = planning_segments_document({"calibrated": False}, derived)
        reloaded = load_planning_segments(root, source)
        self.assertEqual(reloaded, derived)
        self.assertIn("planning_segments", root)

    def test_local_preflight_finds_open_space_and_rejects_keepout_center(self):
        planner = LatticePreflightPlanner(load_field_reference(GEOMETRY_FILE))
        open_path = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
        self.assertIsNotNone(open_path)
        points, expanded = open_path
        self.assertGreater(len(points), 2)
        self.assertGreater(expanded, 0)

        blocked = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(2.0, 3.3, 0.0))
        self.assertIsNone(blocked)

    def test_orientation_free_through_pose_keeps_a_continuous_joint_heading(self):
        source = (
            Waypoint(
                frame_id="odom_combined",
                position=(-0.1, 0.0, 0.0),
                orientation=(0.0, 0.0, 1.0, 0.0),
                task="start",
                direction="forward",
                id="start",
            ),
            Waypoint(
                frame_id="odom_combined",
                position=(0.7, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 0.0),
                task="via",
                direction="reverse",
                id="middle",
            ),
            Waypoint(
                frame_id="odom_combined",
                position=(1.5, 0.0, 0.0),
                orientation=(0.0, 0.0, 1.0, 0.0),
                task="return",
                direction="reverse",
                id="end",
            ),
        )
        segment = PlanningSegment(
            "joint_reverse", "reverse", "start", "end", ("middle",)
        )

        report = preflight_route(
            load_field_reference(GEOMETRY_FILE), source, (segment,)
        )

        self.assertTrue(report.feasible)
        self.assertEqual(
            report.warnings,
            (
                "middle: orientation unconstrained; preflight selects "
                "a continuous heading",
            ),
        )
        first_leg, second_leg = report.segments[0].legs
        self.assertEqual(first_leg.points[-1], second_leg.points[0])
        self.assertLessEqual(
            math.hypot(first_leg.points[-1].x - 0.7, first_leg.points[-1].y),
            0.15,
        )

    def test_simulation_segment_baseline_reports_current_c_entry_blocked(self):
        document, source = load_waypoint_document(NAV_ONLY_FILE)
        segments = load_planning_segments(document, source)
        report = preflight_route(load_field_reference(GEOMETRY_FILE), source, segments)

        self.assertFalse(report.feasible)
        self.assertEqual(
            [item.segment_id for item in report.segments],
            [
                "p_to_qr",
                "reverse_corridor",
                "reverse_c_entry",
                "c_exit",
                "return_to_p",
            ],
        )
        self.assertEqual(
            report.warnings,
            (
                "c_corner_1: orientation unconstrained; preflight selects "
                "a continuous heading",
            ),
        )
        c_entry = next(
            item for item in report.segments if item.segment_id == "reverse_c_entry"
        )
        self.assertFalse(c_entry.feasible)
        self.assertIn("b_corridor_enter -> c_corner_1", c_entry.message)


if __name__ == "__main__":
    unittest.main()
