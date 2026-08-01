"""Contracts for the legacy geometry-candidate write helper."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "src" / "smartcar_sim" / "scripts" / "apply_candidate.py"


def load_script_module():
    spec = importlib.util.spec_from_file_location("apply_candidate_for_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(spec.name, None)
    return module


def waypoint(waypoint_id, x, y, orientation=None, task="via"):
    pose = {"position": {"x": x, "y": y, "z": 0.0}}
    if orientation is not None:
        pose["orientation"] = orientation
    return {"id": waypoint_id, "task": task, "pose": pose}


class ApplyCandidateTests(unittest.TestCase):
    def test_only_updates_allowed_b_guide_and_never_persists_transit_yaw(self):
        module = load_script_module()
        document = {
            "waypoints": [
                waypoint(
                    "p_start",
                    0.0,
                    0.0,
                    {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    task="start",
                ),
                waypoint(
                    "b_corridor_gate",
                    1.8,
                    2.5,
                    {"x": 0.0, "y": 0.0, "z": 0.4, "w": 0.9},
                ),
                waypoint(
                    "b_corridor_enter",
                    1.6,
                    2.65,
                    {"x": 0.0, "y": 0.0, "z": 0.4, "w": 0.9},
                ),
                waypoint(
                    "c_corner_2",
                    2.0,
                    3.8,
                    {"x": 0.0, "y": 0.0, "z": 0.4, "w": 0.9},
                ),
                waypoint(
                    "c_corner_3",
                    3.0,
                    3.8,
                    {"x": 0.0, "y": 0.0, "z": 0.4, "w": 0.9},
                ),
                waypoint(
                    "c_corner_4",
                    3.2,
                    2.8,
                    {"x": 0.0, "y": 0.0, "z": 0.4, "w": 0.9},
                ),
                waypoint(
                    "b_corridor_return_enter",
                    2.2,
                    2.4,
                    {"x": 0.0, "y": 0.0, "z": 0.4, "w": 0.9},
                ),
            ]
        }
        candidate = {
            "b_corridor_gate": {"x": 2.0, "y": 2.65, "yaw_deg": 124.0},
            "b_corridor_enter": {"x": 1.0, "y": 2.95, "yaw_deg": 163.3},
            "c_corner_2": {"x": 9.0, "y": 9.0, "yaw_deg": 30.0},
            "c_corner_3": {"x": 9.1, "y": 9.1, "yaw_deg": -30.0},
            "c_corner_4": {"x": 9.2, "y": 9.2, "yaw_deg": -150.0},
            "b_corridor_return_enter": {
                "x": 2.0,
                "y": 2.48,
                "yaw_deg": -135.0,
            },
        }

        with tempfile.TemporaryDirectory() as temporary:
            waypoint_file = Path(temporary) / "waypoints.yaml"
            waypoint_file.write_text(
                yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
            )
            module.apply_candidate(str(waypoint_file), candidate)
            written = yaml.safe_load(waypoint_file.read_text(encoding="utf-8"))

        by_id = {item["id"]: item for item in written["waypoints"]}
        for waypoint_id, expected in (
            ("c_corner_2", (2.0, 3.8)),
            ("c_corner_3", (3.0, 3.8)),
            ("c_corner_4", (3.2, 2.8)),
        ):
            with self.subTest(waypoint_id=waypoint_id):
                position = by_id[waypoint_id]["pose"]["position"]
                self.assertEqual((position["x"], position["y"]), expected)
                self.assertNotIn("orientation", by_id[waypoint_id]["pose"])

        for waypoint_id, expected in (
            ("b_corridor_gate", (2.0, 2.65)),
            ("b_corridor_enter", (1.0, 2.95)),
            ("b_corridor_return_enter", (2.0, 2.48)),
        ):
            with self.subTest(waypoint_id=waypoint_id):
                updated = by_id[waypoint_id]["pose"]
                self.assertEqual(
                    (updated["position"]["x"], updated["position"]["y"]),
                    expected,
                )
                self.assertNotIn("orientation", updated)
        self.assertIn("orientation", by_id["p_start"]["pose"])

    def test_main_accepts_legacy_candidate_without_a_total_path_field(self):
        module = load_script_module()
        candidate = {"score": 8.33, "c3yaw": 5.0, "c4yaw": -50.0}

        with tempfile.TemporaryDirectory() as temporary:
            candidate_file = Path(temporary) / "legacy.json"
            candidate_file.write_text(
                json.dumps([candidate]), encoding="utf-8"
            )
            arguments = [
                "apply_candidate.py",
                "unused.yaml",
                "--candidate",
                str(candidate_file),
                "--dry-run",
            ]
            with mock.patch.object(sys, "argv", arguments), mock.patch.object(
                module, "apply_candidate"
            ) as apply_candidate:
                self.assertEqual(module.main(), 0)

        apply_candidate.assert_called_once_with(
            "unused.yaml", candidate, dry_run=True
        )


if __name__ == "__main__":
    unittest.main()
