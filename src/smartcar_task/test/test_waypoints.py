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
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "loop",
            },
            {
                "id": "transit_2",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 2.0, "y": 2.40, "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": -0.7071067812,
                        "w": 0.7071067812,
                    },
                },
                "task": "corridor",
            },
            {
                "id": "p_finish",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": -1.0, "w": 0.0},
                },
                "task": "return",
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
                "forward", "forward", "reverse", "forward", "forward", "forward",
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

    def test_requires_authored_headings_at_p_qr_and_vlm_positions(self):
        for index, task in ((0, "start"), (1, "qr"), (2, "vlm"), (-1, "return")):
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
            document["waypoints"][index]["pose"].pop("orientation")
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
                    "start", "qr", "vlm", "loop", "corridor", "return",
                ],
            )

            # Swap vlm (index 2) and loop (index 3) — produces invalid sequence
            document = valid_document()
            document["waypoints"][2], document["waypoints"][3] = (
                document["waypoints"][3], document["waypoints"][2]
            )
            document["waypoints"][2]["direction"] = "forward"
            with self.assertRaisesRegex(ValueError, "direction|sequence|order|out-of|expected"):
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
            via("via_loop", "forward", 1.2, 3.0),
            original[3],
            via("via_return", "forward", 2.5, 2.8),
            original[4],
            original[5],
        ]

        with tempfile.TemporaryDirectory() as directory:
            path = self.write_document(directory, document)
            template, waypoints = load_waypoint_document(path)
            via_waypoints = [item for item in waypoints if item.task == "via"]
            self.assertEqual(len(via_waypoints), 4)
            self.assertTrue(all(
                is_zero_quaternion(item.orientation) for item in via_waypoints
            ))
            write_waypoints_atomic(path, template, waypoints)
            written = yaml.safe_load(path.read_text(encoding="utf-8"))

        written_via = [item for item in written["waypoints"] if item["task"] == "via"]
        self.assertTrue(all("orientation" not in item["pose"] for item in written_via))

    def test_direction_window_is_mandatory_and_nav_only_uses_same_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            all_forward = valid_document()
            for waypoint in all_forward["waypoints"]:
                waypoint["direction"] = "forward"
            with self.assertRaisesRegex(ValueError, "direction must be reverse"):
                load_waypoints(self.write_document(directory, all_forward))

            wrong_qr = valid_document()
            wrong_qr["waypoints"][1]["direction"] = "reverse"
            with self.assertRaisesRegex(ValueError, "direction must be forward"):
                load_waypoints(self.write_document(directory, wrong_qr))

            wrong_after_vlm = valid_document()
            wrong_after_vlm["waypoints"][4]["direction"] = "reverse"
            with self.assertRaisesRegex(ValueError, "direction must be forward"):
                load_waypoints(self.write_document(directory, wrong_after_vlm))

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
                "p_start", "a_task_observe", "via_2", "c_corner_1",
                "via_1", "via_3", "p_finish",
            ],
        )
        self.assertEqual(
            [item.task for item in nav_only],
            ["start", "nav", "via", "nav", "via", "via", "return"],
        )
        self.assertEqual(
            [item.direction for item in nav_only],
            [
                "forward", "forward", "reverse", "reverse", "reverse",
                "reverse", "reverse",
            ],
        )
        self.assertEqual(
            [item.goal_profile for item in nav_only],
            [
                "standard", "precise", "standard", "reverse_handoff",
                "standard", "standard", "standard",
            ],
        )
        self.assertTrue(all(
            is_heading_locked(item)
            for item in nav_only
            if item.id not in {"via_1", "via_2", "via_3"}
        ))
        self.assertTrue(all(
            not is_zero_quaternion(item.orientation) for item in nav_only
            if item.id not in {"via_1", "via_2", "via_3"}
        ))
        self.assertEqual(nav_only[1].goal_profile, "precise")
        self.assertEqual(nav_only[3].task, "nav")
        self.assertEqual(
            nav_only[3].position,
            (0.9330276705276708, 3.8337068653474913, 0.0),
        )
        self.assertEqual(nav_only[3].goal_profile, "reverse_handoff")
        self.assertEqual(nav_only[2].task, "via")
        self.assertEqual(nav_only[2].direction, "reverse")
        self.assertEqual(nav_only[4].task, "via")
        self.assertEqual(nav_only[4].direction, "reverse")
        self.assertEqual(nav_only[5].task, "via")
        self.assertEqual(nav_only[5].direction, "reverse")
        self.assertEqual(nav_only[6].task, "return")
        self.assertEqual(nav_only[6].direction, "reverse")

        nav_only_document = yaml.safe_load(nav_only_file.read_text(encoding="utf-8"))
        next(
            waypoint for waypoint in nav_only_document["waypoints"]
            if waypoint["id"] == "c_corner_1"
        )["direction"] = "forward"
        next(
            waypoint for waypoint in nav_only_document["waypoints"]
            if waypoint["id"] == "c_corner_1"
        )["goal_profile"] = "standard"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "direction must be reverse"):
                load_waypoints(self.write_document(directory, nav_only_document))

        direct_return_document = yaml.safe_load(
            nav_only_file.read_text(encoding="utf-8"))
        next(
            waypoint for waypoint in direct_return_document["waypoints"]
            if waypoint["id"] == "p_finish"
        )["direction"] = "reverse"
        with tempfile.TemporaryDirectory() as directory:
            reverse_return = load_waypoints(
                self.write_document(directory, direct_return_document))
        self.assertEqual(reverse_return[-1].direction, "reverse")

        next(
            waypoint for waypoint in direct_return_document["waypoints"]
            if waypoint["id"] == "p_finish"
        )["direction"] = "forward"
        with tempfile.TemporaryDirectory() as directory:
            forward_return = load_waypoints(
                self.write_document(directory, direct_return_document))
        self.assertEqual(forward_return[-1].direction, "forward")

    def test_rule_baseline_uses_direct_vlm_handoff_heading(self):
        waypoints = load_waypoints(
            PACKAGE_ROOT.parent
            / "smartcar_nav2"
            / "config"
            / "waypoints"
            / "default_waypoints.yaml"
        )
        vlm = waypoints[2]  # c_corner_1
        self.assertEqual(vlm.task, "vlm")
        self.assertEqual(vlm.position,
                         (0.3867094808286349, 2.65, 0.0))
        _, _, qz, qw = vlm.orientation
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.assertAlmostEqual(yaw, 1.0471975511965976, delta=1.0e-6)

    def test_rule_baseline_keeps_qr_standoff_and_direct_semantic_return(self):
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
        vlm = waypoints[2]
        direct_return = waypoints[-1]
        self.assertEqual(qr.position, (3.127294927294929, 0.9765623265623269, 0.0))
        standoff = math.hypot(4.15 - qr.position[0], 1.35 - qr.position[1])
        self.assertAlmostEqual(standoff, 1.08875220, delta=1.0e-6)
        self.assertGreater(standoff, 0.5)
        self.assertEqual(
            [item.id for item in waypoints],
            ["p_start", "a_task_observe", "c_corner_1", "p_finish"],
        )
        self.assertEqual(vlm.task, "vlm")
        self.assertEqual(vlm.direction, "reverse")
        self.assertEqual(direct_return.task, "return")
        self.assertEqual(direct_return.direction, "forward")
        self.assertFalse({"via", "corridor", "loop"} & {
            item.task for item in waypoints
        })

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
                "start", "qr", "vlm", "return",
            ],
        )
        self.assertEqual(
            [item.id for item in waypoints],
            [
                "p_start",
                "a_task_observe",
                "c_corner_1",
                "p_finish",
            ],
        )
        self.assertEqual(waypoints[1].goal_profile, "precise")
        self.assertEqual(waypoints[2].goal_profile, "reverse_handoff")
        self.assertTrue(all(
            item.goal_profile == "standard"
            for index, item in enumerate(waypoints)
            if index not in {1, 2}
        ))


if __name__ == "__main__":
    unittest.main()
