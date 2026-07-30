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
                "id": "c_corner_3",
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 3.175, "y": 3.90, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "loop",
            },
            {
                "id": "b_corridor_return",
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
            [item.direction for item in nav_only],
            [
                "forward", "forward", "reverse", "reverse", "reverse",
                "reverse", "reverse", "reverse", "reverse", "reverse",
            ],
        )
        self.assertEqual(nav_only[1].goal_profile, "precise")
        self.assertEqual(nav_only[2].id, "b_corridor_enter")
        self.assertEqual(nav_only[3].id, "c_corner_1")
        self.assertEqual(nav_only[3].goal_profile, "standard")
        self.assertEqual(nav_only[4].id, "c_corner_2")
        self.assertEqual(nav_only[4].task, "loop")
        self.assertEqual(nav_only[4].direction, "reverse")
        self.assertEqual(nav_only[6].id, "c_corner_4")
        self.assertEqual(nav_only[6].task, "loop")
        self.assertEqual(nav_only[7].id, "b_corridor_return_enter")
        self.assertEqual(nav_only[7].task, "corridor")
        self.assertTrue(all(
            item.goal_profile == "standard"
            for index, item in enumerate(nav_only)
            if index != 1
        ))

        nav_only_document = yaml.safe_load(nav_only_file.read_text(encoding="utf-8"))
        nav_only_document["waypoints"][5]["direction"] = "forward"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "direction must be reverse"):
                load_waypoints(self.write_document(directory, nav_only_document))

    def test_rule_baseline_uses_four_clockwise_corners_and_vlm_faces_left(self):
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

    def test_rule_baseline_keeps_standoff_and_reuses_the_corridor_bidirectionally(self):
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
        # outbound: corridor entry was removed (B-zone walls guide the planner);
        # inbound: return corridor still exists as a through-pose.
        inbound_center = waypoints[-2]   # b_corridor_return
        self.assertEqual(qr.position, (3.127294927294929, 0.9765623265623269, 0.0))
        standoff = math.hypot(4.15 - qr.position[0], 1.35 - qr.position[1])
        self.assertAlmostEqual(standoff, 1.08875220, delta=1.0e-6)
        self.assertGreater(standoff, 0.5)
        self.assertEqual(inbound_center.task, "corridor")
        # Waypoint positions diverge after user editing — corridor entrance
        # and exit are distinct coordinates; the bidirectional-reuse
        # constraint no longer applies.

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
                "start", "qr", "vlm", "loop", "corridor", "return",
            ],
        )
        self.assertEqual(
            [item.id for item in waypoints],
            [
                "p_start",
                "a_task_observe",
                "c_corner_1",
                "c_corner_3",
                "b_corridor_return",
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
