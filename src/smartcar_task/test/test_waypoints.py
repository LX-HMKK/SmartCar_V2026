"""Tests for semantic waypoint loading and validation."""
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.waypoints import load_waypoints  # noqa: E402


def valid_document():
    return {
        "waypoints": [
            {
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "start",
            },
            {
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "qr",
            },
            {
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 2.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "vlm",
            },
            {
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 3.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "corridor",
            },
            {
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 4.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "task": "loop",
            },
            {
                "frame_id": "odom_combined",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
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
        self.assertEqual(waypoints[1].position, (1.0, 2.0, 0.0))
        self.assertEqual(waypoints[1].orientation, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(waypoints[1].task, "qr")

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

    def test_requires_start_and_return_to_match_the_reset_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            document = valid_document()
            document["waypoints"][0]["task"] = "qr"
            with self.assertRaisesRegex(ValueError, "first waypoint"):
                load_waypoints(self.write_document(directory, document))

            document = valid_document()
            document["waypoints"][-1]["pose"]["position"]["x"] = 0.1
            with self.assertRaisesRegex(ValueError, "P-zone origin"):
                load_waypoints(self.write_document(directory, document))

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
            ["start", "qr", "vlm", "corridor", "loop", "return"],
        )


if __name__ == "__main__":
    unittest.main()
