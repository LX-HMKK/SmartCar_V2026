"""Tests for semantic waypoint loading and validation."""
from dataclasses import replace
from pathlib import Path
import math
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.waypoints import (  # noqa: E402
    is_heading_locked,
    is_zero_quaternion,
    load_waypoint_document,
    load_waypoints,
    write_waypoints_atomic,
)


def valid_document():
    return {
        "calibrated": False,
        "waypoints": [
            {
                "id": "p_start",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "start",
            },
            {
                "id": "a_task_observe",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 3.45, "y": 0.80, "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.2588190451,
                        "w": 0.9659258263,
                    },
                },
                "task": "qr",
            },
            {
                "id": "c_corner_1",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 0.825, "y": 2.75, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0},
                },
                "task": "vlm",
                "direction": "reverse",
            },
            {
                "id": "transit_1",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 3.175, "y": 3.90, "z": 0.0},
                },
                "task": "via",
                "direction": "reverse",
            },
            {
                "id": "transit_2",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 2.0, "y": 2.40, "z": 0.0},
                },
                "task": "via",
                "direction": "reverse",
            },
            {
                "id": "p_finish",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": -1.0, "w": 0.0},
                },
                "task": "return",
                "direction": "reverse",
            },
        ]
    }


class WaypointTests(unittest.TestCase):
    def write_document(self, directory, document):
        path = Path(directory) / "waypoints.yaml"
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        return path

    def test_loads_existing_schema_without_ros_dependencies(self):
        with tempfile.TemporaryDirectory() as directory:
            waypoints = load_waypoints(
                self.write_document(directory, valid_document()))

        self.assertEqual(len(waypoints), 6)
        self.assertEqual(waypoints[1].frame_id, "odom_combined")
        self.assertEqual(waypoints[1].id, "a_task_observe")
        self.assertEqual(waypoints[1].position, (3.45, 0.8, 0.0))
        self.assertAlmostEqual(waypoints[1].orientation[2], 0.2588190451)
        self.assertAlmostEqual(waypoints[1].orientation[3], 0.9659258263)
        self.assertEqual(waypoints[1].task, "qr")
        self.assertEqual(
            [item.direction for item in waypoints],
            [
                "forward", "forward", "reverse", "reverse", "reverse", "reverse",
            ],
        )
        self.assertEqual(
            [item.goal_profile for item in waypoints],
            ["standard"] * len(waypoints),
        )

    def test_goal_profiles_parse_and_reject_invalid_direction_pairings(self):
        document = valid_document()
        document["waypoints"][1]["goal_profile"] = "precise"
        with tempfile.TemporaryDirectory() as directory:
            waypoints = load_waypoints(
                self.write_document(directory, document))

        self.assertEqual(waypoints[1].goal_profile, "precise")
        self.assertEqual(
            [item.goal_profile for item in waypoints[2:]],
            ["standard"] * (len(waypoints) - 2),
        )

        reverse_handoff = valid_document()
        reverse_handoff["waypoints"][2][
            "goal_profile"] = "reverse_handoff"
        with tempfile.TemporaryDirectory() as directory:
            handoff_waypoints = load_waypoints(
                self.write_document(directory, reverse_handoff))
        self.assertEqual(
            handoff_waypoints[2].goal_profile, "reverse_handoff")

        for invalid in ("", "unknown", None, True):
            document = valid_document()
            document["waypoints"][1]["goal_profile"] = invalid
            with tempfile.TemporaryDirectory() as directory:
                with self.subTest(invalid=invalid):
                    with self.assertRaisesRegex(
                        ValueError, "unknown goal_profile"
                    ):
                        load_waypoints(
                            self.write_document(directory, document))

        reverse_precise = valid_document()
        reverse_precise["waypoints"][2]["goal_profile"] = "precise"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                ValueError,
                "reverse goals must use the standard or reverse_handoff profile",
            ):
                load_waypoints(
                    self.write_document(directory, reverse_precise))

        forward_handoff = valid_document()
        forward_handoff["waypoints"][1][
            "goal_profile"] = "reverse_handoff"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                ValueError, "reverse_handoff goals must be reverse"
            ):
                load_waypoints(
                    self.write_document(directory, forward_handoff))

    def test_rejects_malformed_documents_and_missing_fields(self):
        malformed_documents = (
            None,
            {},
            {"waypoints": []},
            {"waypoints": "not-a-list"},
            {"waypoints": [None]},
            {"waypoints": [{"frame_id": "odom_combined"}]},
        )
        with tempfile.TemporaryDirectory() as directory:
            for index, document in enumerate(malformed_documents):
                with self.subTest(index=index):
                    path = self.write_document(directory, document)
                    with self.assertRaises(ValueError):
                        load_waypoints(path)

    def test_rejects_unknown_task(self):
        document = valid_document()
        document["waypoints"][1]["task"] = "unknown"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "unknown task"):
                load_waypoints(self.write_document(directory, document))

        document = valid_document()
        document["waypoints"][1]["task"] = []
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "unknown task"):
                load_waypoints(self.write_document(directory, document))

    def test_rejects_removed_legacy_tasks(self):
        for task in ("corridor", "loop"):
            document = valid_document()
            document["waypoints"][1]["task"] = task
            with tempfile.TemporaryDirectory() as directory:
                with self.subTest(task=task):
                    with self.assertRaisesRegex(
                        ValueError, f"unknown task {task!r}"
                    ):
                        load_waypoints(self.write_document(directory, document))

    def test_rejects_duplicate_ids(self):
        document = valid_document()
        document["waypoints"][2]["id"] = document["waypoints"][1]["id"]
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "ids must be unique"):
                load_waypoints(self.write_document(directory, document))

    def test_rejects_non_normalized_quaternion(self):
        document = valid_document()
        document["waypoints"][1]["pose"]["orientation"]["w"] = 0.5
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "unit length"):
                load_waypoints(self.write_document(directory, document))

    def test_rejects_nonfinite_and_boolean_coordinates(self):
        for value in (float("nan"), float("inf"), True):
            document = valid_document()
            document["waypoints"][1]["pose"]["position"]["x"] = value
            with tempfile.TemporaryDirectory() as directory:
                with self.subTest(value=value):
                    with self.assertRaises(ValueError):
                        load_waypoints(self.write_document(directory, document))

    def test_requires_start_heading_and_start_return_positions_at_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            document = valid_document()
            document["waypoints"][0]["task"] = "qr"
            with self.assertRaisesRegex(ValueError, "first waypoint"):
                load_waypoints(self.write_document(directory, document))

            document = valid_document()
            document["waypoints"][-1]["pose"]["position"]["x"] = 0.1
            with self.assertRaisesRegex(ValueError, "P-zone origin"):
                load_waypoints(self.write_document(directory, document))

            document = valid_document()
            document["waypoints"][0]["pose"]["orientation"] = {
                "x": 0.0, "y": 0.0, "z": -1.0, "w": 0.0,
            }
            with self.assertRaisesRegex(ValueError, r"start.*\+X"):
                load_waypoints(self.write_document(directory, document))

    def test_requires_authored_headings_at_start_qr_and_vlm_positions(self):
        for index, task in ((0, "start"), (1, "qr"), (2, "vlm")):
            document = valid_document()
            document["waypoints"][index]["pose"].pop("orientation")
            with tempfile.TemporaryDirectory() as directory:
                with self.subTest(task=task):
                    with self.assertRaisesRegex(
                        ValueError, f"{task} waypoint requires an authored orientation"
                    ):
                        load_waypoints(self.write_document(directory, document))

    def test_nav_heading_mode_can_explicitly_lock_a_substitute_goal(self):
        document = valid_document()
        document["waypoints"][1]["task"] = "nav"
        document["waypoints"][1]["heading_mode"] = "locked"
        with tempfile.TemporaryDirectory() as directory:
            waypoints = load_waypoints(self.write_document(directory, document))

        self.assertEqual(waypoints[1].task, "nav")
        self.assertEqual(waypoints[1].heading_mode, "locked")
        self.assertTrue(is_heading_locked(waypoints[1]))
        self.assertFalse(is_zero_quaternion(waypoints[1].orientation))

        document = valid_document()
        document["waypoints"][1]["task"] = "nav"
        document["waypoints"][1]["heading_mode"] = "locked"
        document["waypoints"][1]["pose"].pop("orientation")
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                ValueError, "nav waypoint requires an authored orientation"
            ):
                load_waypoints(self.write_document(directory, document))

    def test_heading_mode_rejects_invalid_and_free_protected_values(self):
        document = valid_document()
        document["waypoints"][1]["heading_mode"] = "sideways"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "heading_mode must be free or locked"):
                load_waypoints(self.write_document(directory, document))

        document = valid_document()
        document["waypoints"][1]["heading_mode"] = "free"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "cannot use a free heading"):
                load_waypoints(self.write_document(directory, document))

    def test_transit_heading_may_be_omitted_for_runtime_materialization(self):
        document = valid_document()
        for index in (3, 4):
            document["waypoints"][index]["pose"].pop("orientation", None)
        with tempfile.TemporaryDirectory() as directory:
            waypoints = load_waypoints(self.write_document(directory, document))

        self.assertTrue(is_zero_quaternion(waypoints[3].orientation))
        self.assertTrue(is_zero_quaternion(waypoints[4].orientation))

    def test_semantic_sequence_allows_transit_points_but_not_reordered_tasks(self):
        with tempfile.TemporaryDirectory() as directory:
            waypoints = load_waypoints(
                self.write_document(directory, valid_document()))
            self.assertEqual(
                [item.task for item in waypoints],
                [
                    "start", "qr", "vlm", "via", "via", "return",
                ],
            )

            # VLM cannot replace the first QR/nav semantic endpoint.
            document = valid_document()
            document["waypoints"][1]["task"] = "vlm"
            with self.assertRaisesRegex(ValueError, "expected qr or nav"):
                load_waypoints(self.write_document(directory, document))

    def test_via_points_are_unoriented_navigation_constraints(self):
        document = valid_document()
        original = document["waypoints"]

        def via(waypoint_id, direction, x, y):
            return {
                "id": waypoint_id,
                "frame_id": "odom_combined",
                "pose": {"position": {"x": x, "y": y, "z": 0.0}},
                "task": "via",
                "direction": direction,
            }

        document["waypoints"] = [
            original[0],
            via("via_outbound", "forward", 0.8, 0.2),
            original[1],
            via("via_reverse", "reverse", 2.3, 1.0),
            original[2],
            via("via_return_1", "reverse", 1.2, 3.0),
            original[3],
            via("via_return_2", "reverse", 2.5, 2.8),
            original[4],
            original[5],
        ]

        with tempfile.TemporaryDirectory() as directory:
            path = self.write_document(directory, document)
            template, waypoints = load_waypoint_document(path)
            via_waypoints = [item for item in waypoints if item.task == "via"]
            self.assertEqual(len(via_waypoints), 6)
            self.assertTrue(all(
                is_zero_quaternion(item.orientation) for item in via_waypoints
            ))
            write_waypoints_atomic(path, template, waypoints)
            written = yaml.safe_load(path.read_text(encoding="utf-8"))

        written_via = [item for item in written["waypoints"] if item["task"] == "via"]
        self.assertTrue(all("orientation" not in item["pose"] for item in written_via))

    def test_current_route_is_all_forward_and_nav_only_keeps_the_same_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            all_forward = valid_document()
            for waypoint in all_forward["waypoints"]:
                waypoint["direction"] = "forward"
            loaded = load_waypoints(self.write_document(directory, all_forward))
            self.assertTrue(all(
                waypoint.direction == "forward" for waypoint in loaded
            ))

            invalid_direction = valid_document()
            invalid_direction["waypoints"][2]["direction"] = "sideways"
            with self.assertRaisesRegex(ValueError, "unknown direction"):
                load_waypoints(self.write_document(directory, invalid_direction))

        nav_only_file = (
            PACKAGE_ROOT.parent
            / "smartcar_nav2"
            / "config"
            / "waypoints"
            / "nav_only.yaml"
        )
        nav_only = load_waypoints(nav_only_file)
        self.assertEqual(
            [item.id for item in nav_only],
            [
                "p_start", "a_task_observe", "via_2_entry", "a_departure_exit",
                "via_2_corridor", "via_2", "c_corner_1", "c_north_1",
                "c_north_2", "via_1", "via_3", "return_corridor_exit",
                "p_return_approach", "p_finish",
            ],
        )
        self.assertEqual(
            [item.task for item in nav_only],
            [
                "start", "nav", "via", "via", "via", "via", "nav", "via",
                "via", "via", "via", "via", "via", "return",
            ],
        )
        self.assertEqual(
            [item.direction for item in nav_only],
            ["forward"] * 14,
        )
        self.assertEqual(
            [item.goal_profile for item in nav_only],
            ["standard", "precise"] + ["standard"] * 4
            + ["precise"] + ["standard"] * 7,
        )
        self.assertTrue(all(
            is_heading_locked(item)
            for item in nav_only
            if item.task not in {"via", "return"}
        ))
        self.assertTrue(all(
            not is_zero_quaternion(item.orientation) for item in nav_only
            if item.task != "via"
        ))
        self.assertEqual(nav_only[1].goal_profile, "precise")
        c_corner_1 = next(item for item in nav_only if item.id == "c_corner_1")
        self.assertEqual(c_corner_1.task, "nav")
        self.assertEqual(
            c_corner_1.position,
            (0.9330276705276708, 3.8337068653474913, 0.0),
        )
        self.assertEqual(c_corner_1.goal_profile, "precise")
        self.assertTrue(all(
            item.direction == "forward" for item in nav_only if item.task == "via"
        ))
        self.assertEqual(nav_only[-1].task, "return")
        self.assertEqual(nav_only[-1].direction, "forward")

        nav_only_document = yaml.safe_load(nav_only_file.read_text(encoding="utf-8"))
        next(
            waypoint for waypoint in nav_only_document["waypoints"]
            if waypoint["id"] == "c_corner_1"
        )["goal_profile"] = "reverse_handoff"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                ValueError, "reverse_handoff goals must be reverse"
            ):
                load_waypoints(self.write_document(directory, nav_only_document))

    def test_deployment_route_uses_forward_c1_heading(self):
        waypoints = load_waypoints(
            PACKAGE_ROOT.parent
            / "smartcar_nav2"
            / "config"
            / "waypoints"
            / "default_waypoints.yaml"
        )
        vlm = next(item for item in waypoints if item.id == "c_corner_1")
        self.assertEqual(vlm.task, "vlm")
        self.assertEqual(vlm.position,
                         (0.9330276705276708, 3.8337068653474913, 0.0))
        _, _, qz, qw = vlm.orientation
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.assertAlmostEqual(yaw, 1.17, delta=1.0e-6)

    def test_deployment_route_keeps_qr_standoff_and_forward_return(self):
        default_file = (
            PACKAGE_ROOT.parent
            / "smartcar_nav2"
            / "config"
            / "waypoints"
            / "default_waypoints.yaml"
        )
        document = yaml.safe_load(default_file.read_text(encoding="utf-8"))
        self.assertIs(document["calibrated"], False)
        waypoints = load_waypoints(default_file)
        qr = waypoints[1]
        vlm = next(item for item in waypoints if item.id == "c_corner_1")
        return_waypoint = waypoints[-1]
        self.assertEqual(qr.position, (3.025965510598539, 1.2045443727459233, 0.0))
        standoff = math.hypot(4.15 - qr.position[0], 1.35 - qr.position[1])
        self.assertAlmostEqual(standoff, 1.13340676, delta=1.0e-6)
        self.assertGreater(standoff, 0.5)
        self.assertEqual(
            [item.id for item in waypoints],
            [
                "p_start", "a_task_observe", "via_2_entry", "a_departure_exit",
                "via_2_corridor", "via_2", "c_corner_1", "c_north_1",
                "c_north_2", "via_1", "via_3", "return_corridor_exit",
                "p_return_approach", "p_finish",
            ],
        )
        self.assertEqual(vlm.task, "vlm")
        self.assertEqual(vlm.direction, "forward")
        self.assertEqual(vlm.goal_profile, "precise")
        self.assertEqual(return_waypoint.task, "return")
        self.assertEqual(return_waypoint.direction, "forward")
        return_yaw = math.atan2(
            2.0 * return_waypoint.orientation[3] * return_waypoint.orientation[2],
            1.0 - 2.0 * return_waypoint.orientation[2] ** 2,
        )
        self.assertAlmostEqual(return_yaw, -2.498091544796509, delta=1.0e-6)
        self.assertEqual(
            [item.direction for item in waypoints],
            ["forward"] * 14,
        )
        self.assertEqual(
            [item.id for item in waypoints if item.task == "via"],
            [
                "via_2_entry", "a_departure_exit", "via_2_corridor", "via_2",
                "c_north_1", "c_north_2", "via_1", "via_3",
                "return_corridor_exit", "p_return_approach",
            ],
        )

    def test_atomic_editor_write_preserves_ids_and_clears_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            document = valid_document()
            document["waypoints"][1]["goal_profile"] = "precise"
            path = self.write_document(directory, document)
            template, waypoints = load_waypoint_document(path)
            template["calibrated"] = True
            write_waypoints_atomic(path, template, waypoints)
            written = yaml.safe_load(path.read_text(encoding="utf-8"))

        self.assertIs(written["calibrated"], False)
        self.assertEqual(
            [item["id"] for item in written["waypoints"]],
            [item.id for item in waypoints],
        )
        self.assertEqual(
            [item["goal_profile"] for item in written["waypoints"]],
            [item.goal_profile for item in waypoints],
        )
        self.assertEqual(written["waypoints"][1]["goal_profile"], "precise")

    def test_atomic_editor_write_rejects_nonfinite_in_memory_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_document(directory, valid_document())
            original = path.read_bytes()
            template, waypoints = load_waypoint_document(path)
            edited = list(waypoints)
            edited[1] = replace(
                edited[1],
                position=(float("nan"), edited[1].position[1], 0.0),
            )

            with self.assertRaisesRegex(ValueError, "finite number"):
                write_waypoints_atomic(path, template, edited)

            self.assertEqual(path.read_bytes(), original)

    def test_repository_default_sequence_is_valid(self):
        default_file = (
            PACKAGE_ROOT.parent
            / "smartcar_nav2"
            / "config"
            / "waypoints"
            / "default_waypoints.yaml"
        )
        waypoints = load_waypoints(default_file)
        self.assertEqual(
            [item.task for item in waypoints],
            [
                "start", "qr", "via", "via", "via", "via", "vlm", "via",
                "via", "via", "via", "via", "via", "return",
            ],
        )
        self.assertEqual(
            [item.id for item in waypoints],
            [
                "p_start",
                "a_task_observe",
                "via_2_entry",
                "a_departure_exit",
                "via_2_corridor",
                "via_2",
                "c_corner_1",
                "c_north_1",
                "c_north_2",
                "via_1",
                "via_3",
                "return_corridor_exit",
                "p_return_approach",
                "p_finish",
            ],
        )
        self.assertEqual(waypoints[1].goal_profile, "precise")
        self.assertEqual(waypoints[6].goal_profile, "precise")
        self.assertTrue(all(
            item.goal_profile == "standard"
            for index, item in enumerate(waypoints)
            if index not in {1, 6}
        ))


if __name__ == "__main__":
    unittest.main()
