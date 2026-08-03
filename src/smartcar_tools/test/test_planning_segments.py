"""Contracts for the editable planning-segment route model."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest import mock


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT.parent / "smartcar_task"))

from smartcar_task.route_geometry import materialize_free_yaws  # noqa: E402
from smartcar_task.waypoints import (  # noqa: E402
    HEADING_LOCKED_TASKS,
    Waypoint,
    is_heading_locked,
    is_zero_quaternion,
    load_waypoint_document,
    validate_waypoints,
)
from smartcar_tools.field_reference import load_field_reference  # noqa: E402
from smartcar_task.planning_segments import (  # noqa: E402
    PlanningSegment,
    PlanningSegmentError,
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
import smartcar_tools.route_preflight as route_preflight_module  # noqa: E402
from smartcar_tools.waypoint_drag_editor import DragEditor  # noqa: E402


GEOMETRY_FILE = PACKAGE_ROOT / "config" / "routes" / "field_geometry.yaml"
NAV_ONLY_FILE = (
    PACKAGE_ROOT.parent / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml"
)
DEFAULT_WAYPOINTS_FILE = (
    PACKAGE_ROOT.parent
    / "smartcar_nav2"
    / "config"
    / "waypoints"
    / "default_waypoints.yaml"
)


def waypoint(waypoint_id, task, x, y, direction="forward", heading_mode=None):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, y, 0.0),
        orientation=(
            (0.0, 0.0, 0.0, 1.0)
            if heading_mode == "locked" or (
                heading_mode is None and task in HEADING_LOCKED_TASKS
            )
            else (0.0, 0.0, 0.0, 0.0)
        ),
        task=task,
        direction=direction,
        id=waypoint_id,
        heading_mode=heading_mode,
    )


def mission_waypoints():
    return (
        waypoint("p_start", "start", 0.0, 0.0),
        waypoint("qr", "qr", 1.0, 0.0),
        waypoint("via_to_vlm", "via", 1.5, 1.0, "reverse"),
        waypoint("vlm", "vlm", 2.0, 1.0, "reverse"),
        waypoint("via_return_1", "via", 2.5, 3.9, "reverse"),
        waypoint("via_return_2", "via", 1.5, 1.5, "reverse"),
        waypoint("p_finish", "return", 0.0, 0.0, "reverse"),
    )


class DragEditorDeletionTests(unittest.TestCase):
    def test_deleting_a_through_point_removes_its_model_and_keeps_route_valid(self):
        document, loaded = load_waypoint_document(NAV_ONLY_FILE)
        editor = DragEditor.__new__(DragEditor)
        editor._waypoints = list(loaded)
        editor._segments = list(load_planning_segments(document, loaded))
        editor._history = []
        editor._selected_segment = 2
        editor._selected_through = 1
        editor._selected = next(
            index
            for index, waypoint in enumerate(editor._waypoints)
            if waypoint.id == "via_3"
        )
        editor._lock = threading.Lock()
        editor._set_route_status = mock.Mock()
        editor._build_route_panel = mock.Mock()
        editor._mark_route_changed = mock.Mock()
        editor._publish_markers = mock.Mock()

        editor._remove_selected_through(None)

        self.assertNotIn("via_3", [item.id for item in editor._waypoints])
        self.assertEqual(editor._segments[2].through_ids, ("via_1",))
        self.assertIsNone(editor._selected)
        self.assertIsNone(editor._selected_through)
        self.assertEqual(len(editor._history), 1)
        self.assertEqual(
            validate_planning_segments(editor._segments, editor._waypoints),
            tuple(editor._segments),
        )
        route = materialize_route(editor._waypoints, editor._segments)
        self.assertEqual(validate_waypoints(route), route)
        editor._mark_route_changed.assert_called_once_with(rebuild_panel=True)
        editor._publish_markers.assert_called_once_with()


class PlanningSegmentTests(unittest.TestCase):
    def assert_position_only_orientation_contract(self, route):
        for item in route:
            with self.subTest(waypoint=item.id):
                norm = math.sqrt(sum(value * value for value in item.orientation))
                if is_heading_locked(item):
                    self.assertFalse(is_zero_quaternion(item.orientation))
                    self.assertAlmostEqual(norm, 1.0)
                else:
                    self.assertEqual(item.orientation, (0.0, 0.0, 0.0, 0.0))

    def test_segments_materialize_an_ordered_semantic_route(self):
        source = mission_waypoints()
        segments = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment(
                "reverse_to_vlm", "reverse", "qr", "vlm", ("via_to_vlm",)
            ),
            PlanningSegment(
                "return", "reverse", "vlm", "p_finish",
                ("via_return_1", "via_return_2"),
            ),
        )

        checked = validate_planning_segments(segments, source)
        route = materialize_route(source, checked)

        self.assertEqual(
            [item.id for item in route],
            [
                "p_start", "qr", "via_to_vlm", "vlm", "via_return_1",
                "via_return_2", "p_finish",
            ],
        )
        self.assertEqual(
            [item.direction for item in route],
            [
                "forward", "forward", "reverse", "reverse", "reverse",
                "reverse", "reverse",
            ],
        )
        self.assertEqual(validate_waypoints(route), route)
        self.assert_position_only_orientation_contract(
            materialize_free_yaws(route)
        )

    def test_segments_require_one_contiguous_coverage_of_all_waypoints(self):
        source = mission_waypoints()
        incomplete = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment("to_vlm", "reverse", "qr", "vlm"),
            PlanningSegment("return", "reverse", "vlm", "p_finish"),
        )
        with self.assertRaisesRegex(PlanningSegmentError, "cover every waypoint"):
            validate_planning_segments(incomplete, source)

        discontinuous = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment("bad", "reverse", "via_to_vlm", "vlm"),
            PlanningSegment(
                "return", "reverse", "vlm", "p_finish",
                ("via_return_1", "via_return_2"),
            ),
        )
        with self.assertRaisesRegex(PlanningSegmentError, "previous segment end"):
            validate_planning_segments(discontinuous, source)

    def test_route_requires_explicit_segments_and_round_trips_yaml_schema(self):
        source = mission_waypoints()
        with self.assertRaisesRegex(
            PlanningSegmentError, "must define planning_segments"
        ):
            load_planning_segments({"calibrated": False}, source)

        explicit = (
            PlanningSegment("to_qr", "forward", "p_start", "qr"),
            PlanningSegment("to_vlm", "reverse", "qr", "vlm", ("via_to_vlm",)),
            PlanningSegment(
                "return",
                "reverse",
                "vlm",
                "p_finish",
                ("via_return_1", "via_return_2"),
            ),
        )
        self.assertEqual(
            [(item.start_id, item.end_id, item.direction) for item in explicit],
            [
                ("p_start", "qr", "forward"),
                ("qr", "vlm", "reverse"),
                ("vlm", "p_finish", "reverse"),
            ],
        )
        root = planning_segments_document({"calibrated": False}, explicit)
        reloaded = load_planning_segments(root, source)
        self.assertEqual(reloaded, explicit)
        self.assertIn("planning_segments", root)

    def test_yaml_and_action_routes_keep_transit_headings_free(self):
        for waypoint_file in (NAV_ONLY_FILE, DEFAULT_WAYPOINTS_FILE):
            with self.subTest(waypoint_file=waypoint_file.name):
                document, authored = load_waypoint_document(waypoint_file)
                segments = load_planning_segments(document, authored)
                executable = materialize_free_yaws(
                    materialize_route(authored, segments)
                )

                self.assert_position_only_orientation_contract(authored)
                self.assert_position_only_orientation_contract(executable)

    def test_local_preflight_finds_open_space_and_rejects_keepout_center(self):
        planner = LatticePreflightPlanner(load_field_reference(GEOMETRY_FILE))
        open_path = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
        self.assertIsNotNone(open_path)
        points, expanded = open_path
        self.assertGreater(len(points), 2)
        self.assertGreater(expanded, 0)

        blocked = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(2.0, 3.3, 0.0))
        self.assertIsNone(blocked)

    def test_transit_heading_remains_free_in_preflight(self):
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
                "middle: position-only; preflight leaves heading free",
            ),
        )
        first_leg, second_leg = report.segments[0].legs
        self.assertEqual(first_leg.points[-1], second_leg.points[0])
        self.assertLessEqual(
            math.hypot(first_leg.points[-1].x - 0.7, first_leg.points[-1].y),
            0.15,
        )

    def test_preflight_normalizes_legacy_transit_yaw_to_free_heading(self):
        source = (
            waypoint("start", "start", 0.0, 0.0),
            waypoint("middle", "via", 0.7, 0.0),
            waypoint("end", "return", 1.5, 0.0),
        )
        segment = PlanningSegment("pass", "forward", "start", "end", ("middle",))
        reference = load_field_reference(GEOMETRY_FILE)
        baseline = preflight_route(reference, source, (segment,))
        authored_yaw = replace(
            source[1], orientation=(0.0, 0.0, 1.0, 0.0)
        )
        changed = preflight_route(
            reference, (source[0], authored_yaw, source[2]), (segment,)
        )

        self.assertEqual(baseline, changed)
        self.assertEqual(
            baseline.warnings,
            ("middle: position-only; preflight leaves heading free",),
        )

    def test_preflight_preserves_adjacent_transit_action_boundaries(self):
        source = (
            waypoint("p_start", "start", 0.0, 0.0),
            waypoint("middle", "via", 0.8, 0.0),
            waypoint("p_finish", "return", 0.0, 0.0),
        )
        segments = (
            PlanningSegment("first", "forward", "p_start", "middle"),
            PlanningSegment("second", "forward", "middle", "p_finish"),
        )
        first_action = route_preflight_module._ThroughPlan(
            legs=((
                route_preflight_module.Point2D(0.0, 0.0),
                route_preflight_module.Point2D(0.8, 0.0),
            ),),
            expanded_states=(2,),
            completed_goals=1,
        )
        second_action = route_preflight_module._ThroughPlan(
            legs=((
                route_preflight_module.Point2D(0.8, 0.0),
                route_preflight_module.Point2D(0.0, 0.0),
            ),),
            expanded_states=(3,),
            completed_goals=1,
        )
        with mock.patch.object(
            route_preflight_module,
            "_plan_segment_through_constraints",
            autospec=True,
            side_effect=(first_action, second_action),
        ) as plan_segment:
            report = preflight_route(
                load_field_reference(GEOMETRY_FILE), source, segments
            )

        self.assertEqual(plan_segment.call_count, 2)
        self.assertEqual(
            [
                [constraint.yaw for constraint in call.args[1]]
                for call in plan_segment.call_args_list
            ],
            [[0.0, None], [None, 0.0]],
        )
        self.assertTrue(report.feasible)
        self.assertEqual(
            [item.segment_id for item in report.segments], ["first", "second"]
        )
        first_leg, second_leg = report.segments[0].legs[0], report.segments[1].legs[0]
        self.assertEqual(first_leg.points[-1], second_leg.points[0])

    def test_preflight_failure_is_bounded_to_its_explicit_action(self):
        source = (
            waypoint("p_start", "start", 0.0, 0.0),
            waypoint("middle", "via", 0.8, 0.0),
            waypoint("p_finish", "return", 0.0, 0.0),
        )
        segments = (
            PlanningSegment("first", "forward", "p_start", "middle"),
            PlanningSegment("second", "forward", "middle", "p_finish"),
        )
        failed = route_preflight_module._ThroughPlan(
            legs=(), expanded_states=(4,), completed_goals=0
        )
        succeeding = route_preflight_module._ThroughPlan(
            legs=((
                route_preflight_module.Point2D(0.8, 0.0),
                route_preflight_module.Point2D(0.0, 0.0),
            ),),
            expanded_states=(5,),
            completed_goals=1,
        )
        with mock.patch.object(
            route_preflight_module,
            "_plan_segment_through_constraints",
            autospec=True,
            side_effect=(failed, succeeding),
        ):
            report = preflight_route(
                load_field_reference(GEOMETRY_FILE), source, segments
            )

        self.assertFalse(report.feasible)
        self.assertEqual(
            [item.feasible for item in report.segments], [False, True]
        )
        self.assertEqual(
            (report.segments[0].legs[0].start_id, report.segments[0].legs[0].end_id),
            ("p_start", "middle"),
        )
        self.assertEqual(report.segments[1].legs[0].start_id, "middle")
        self.assertIn("navigation action", report.segments[0].message)

    def test_simulation_segment_baseline_uses_only_semantic_replanning_boundaries(self):
        document, source = load_waypoint_document(NAV_ONLY_FILE)
        segments = load_planning_segments(document, source)
        runtime_actions = route_preflight_module._runtime_navigation_actions(
            source, segments
        )
        self.assertEqual(len(runtime_actions), 3)
        self.assertEqual(
            [
                (
                    action.start_id,
                    tuple(segment.id for segment in action.segments),
                    action.goal_ids,
                )
                for action in runtime_actions
            ],
            [
                ("p_start", ("p_to_qr",), ("a_task_observe",)),
                (
                    "a_task_observe",
                    ("qr_to_vlm",),
                    ("via_2", "c_corner_1"),
                ),
                (
                    "c_corner_1",
                    ("return_to_p",),
                    ("via_1", "via_3", "p_finish"),
                ),
            ],
        )

        def planned_action(_planner, constraints, _direction):
            points = tuple(
                (
                    route_preflight_module.Point2D(start.x, start.y),
                    route_preflight_module.Point2D(end.x, end.y),
                )
                for start, end in zip(constraints, constraints[1:])
            )
            return route_preflight_module._ThroughPlan(
                legs=points,
                expanded_states=(1,) * len(points),
                completed_goals=len(points),
            )

        with mock.patch.object(
            route_preflight_module,
            "_plan_segment_through_constraints",
            autospec=True,
            side_effect=planned_action,
        ) as plan_segment:
            report = preflight_route(
                load_field_reference(GEOMETRY_FILE), source, segments
            )

        self.assertTrue(report.feasible)
        self.assertEqual(plan_segment.call_count, 3)
        self.assertEqual(
            [item.segment_id for item in report.segments],
            [
                "p_to_qr",
                "qr_to_vlm",
                "return_to_p",
            ],
        )
        expected_warnings = tuple(
            f"{waypoint.id}: position-only; preflight leaves heading free"
            for waypoint in source
            if not is_heading_locked(waypoint)
        )
        self.assertEqual(report.warnings, expected_warnings)
        self.assert_position_only_orientation_contract(source)
        self.assert_position_only_orientation_contract(
            materialize_free_yaws(materialize_route(source, segments))
        )
        qr_to_vlm = next(
            item for item in report.segments if item.segment_id == "qr_to_vlm"
        )
        self.assertTrue(qr_to_vlm.feasible)
        self.assertEqual(
            [(leg.start_id, leg.end_id) for leg in qr_to_vlm.legs],
            [
                ("a_task_observe", "via_2"),
                ("via_2", "c_corner_1"),
            ],
        )
        return_to_p = next(
            item for item in report.segments if item.segment_id == "return_to_p"
        )
        self.assertTrue(return_to_p.feasible)
        self.assertEqual(
            [(leg.start_id, leg.end_id) for leg in return_to_p.legs],
            [
                ("c_corner_1", "via_1"),
                ("via_1", "via_3"),
                ("via_3", "p_finish"),
            ],
        )

    def test_vlm_return_passes_preflight_through_saved_transit_points(self):
        document, source = load_waypoint_document(NAV_ONLY_FILE)
        segments = load_planning_segments(document, source)

        report = preflight_route(
            load_field_reference(GEOMETRY_FILE), source, segments
        )

        self.assertTrue(report.feasible)
        direct_return = next(
            item for item in report.segments if item.segment_id == "return_to_p"
        )
        self.assertTrue(direct_return.feasible)
        self.assertEqual(
            [(leg.start_id, leg.end_id) for leg in direct_return.legs],
            [
                ("c_corner_1", "via_1"),
                ("via_1", "via_3"),
                ("via_3", "p_finish"),
            ],
        )
        direct_leg = direct_return.legs[0]
        start = next(item for item in source if item.id == direct_leg.start_id)
        end = next(item for item in source if item.id == direct_leg.end_id)
        chord = math.hypot(
            end.position[0] - start.position[0],
            end.position[1] - start.position[1],
        )
        self.assertGreater(chord, 0.0)
        self.assertLessEqual(direct_leg.length_m, chord * 1.75)


if __name__ == "__main__":
    unittest.main()
