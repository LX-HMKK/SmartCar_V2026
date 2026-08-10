"""Tests for semantic waypoints on the current forward-only route."""

from pathlib import Path
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.waypoints import (  # noqa: E402
    Waypoint,
    is_heading_locked,
    is_zero_quaternion,
    load_waypoint_document,
    load_waypoints,
    validate_waypoints,
    write_waypoints_atomic,
)


def waypoint_document():
    return {
        "calibrated": False,
        "waypoints": [
            {
                "id": "p_start", "frame_id": "odom_combined", "task": "start",
                "direction": "forward", "goal_profile": "standard",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            {
                "id": "a_task_observe", "frame_id": "odom_combined", "task": "qr",
                "direction": "forward", "goal_profile": "precise",
                "pose": {
                    "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            {
                "id": "c_corner_1", "frame_id": "odom_combined", "task": "vlm",
                "direction": "forward", "goal_profile": "precise",
                "pose": {
                    "position": {"x": 2.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            {
                "id": "p_finish", "frame_id": "odom_combined", "task": "return",
                "direction": "forward", "goal_profile": "standard",
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}},
            },
        ],
    }


class WaypointTests(unittest.TestCase):
    def write_document(self, directory, document):
        path = Path(directory) / "waypoints.yaml"
        path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        return path

    def test_loads_forward_semantic_route_and_free_transit_sentinel(self):
        document = waypoint_document()
        document["waypoints"].insert(2, {
            "id": "via_1", "frame_id": "odom_combined", "task": "via",
            "direction": "forward", "goal_profile": "standard",
            "pose": {"position": {"x": 1.5, "y": 0.1, "z": 0.0}},
        })
        with tempfile.TemporaryDirectory() as directory:
            _root, waypoints = load_waypoint_document(self.write_document(directory, document))
        self.assertTrue(all(item.direction == "forward" for item in waypoints))
        self.assertTrue(is_heading_locked(waypoints[1]))
        self.assertFalse(is_heading_locked(waypoints[2]))
        self.assertTrue(is_zero_quaternion(waypoints[2].orientation))

    def test_rejects_reverse_directions_and_removed_profiles(self):
        for direction, profile, message in (
            ("reverse", "standard", "unknown direction"),
            ("forward", "reverse_handoff", "unknown goal_profile"),
        ):
            document = waypoint_document()
            document["waypoints"][1]["direction"] = direction
            document["waypoints"][1]["goal_profile"] = profile
            with tempfile.TemporaryDirectory() as directory:
                with self.subTest(direction=direction, profile=profile):
                    with self.assertRaisesRegex(ValueError, message):
                        load_waypoints(self.write_document(directory, document))

    def test_rejects_removed_tasks_and_invalid_headings(self):
        document = waypoint_document()
        document["waypoints"][1]["task"] = "corridor"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "unknown task"):
                load_waypoints(self.write_document(directory, document))

        document = waypoint_document()
        document["waypoints"][1]["pose"].pop("orientation")
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "requires an authored orientation"):
                load_waypoints(self.write_document(directory, document))

    def test_requires_p_origin_and_positive_x_start_heading(self):
        document = waypoint_document()
        document["waypoints"][0]["pose"]["orientation"]["w"] = 0.0
        document["waypoints"][0]["pose"]["orientation"]["z"] = 1.0
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, r"start.*\+X"):
                load_waypoints(self.write_document(directory, document))

        document = waypoint_document()
        document["waypoints"][-1]["pose"]["position"]["x"] = 0.1
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "P-zone origin"):
                load_waypoints(self.write_document(directory, document))

    def test_atomic_write_preserves_unconstrained_transit_as_yaml_omission(self):
        points = validate_waypoints((
            Waypoint("odom_combined", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "start", id="p_start"),
            Waypoint("odom_combined", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "qr", id="a"),
            Waypoint("odom_combined", (1.5, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), "via", id="via"),
            Waypoint("odom_combined", (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "vlm", id="c"),
            Waypoint("odom_combined", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), "return", id="p_finish"),
        ))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "round_trip.yaml"
            write_waypoints_atomic(path, {"calibrated": True}, points)
            serialized = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertFalse(serialized["calibrated"])
        self.assertNotIn("orientation", serialized["waypoints"][2]["pose"])
        self.assertNotIn("orientation", serialized["waypoints"][-1]["pose"])


if __name__ == "__main__":
    unittest.main()
