"""Focused tests for route_tool's validated atomic edit commands."""
import contextlib
import io
from pathlib import Path
import shutil
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.route_model import load_route  # noqa: E402
from smartcar_tools.route_tool import main  # noqa: E402


GEOMETRY_FILE = PACKAGE_ROOT / "config" / "routes" / "field_geometry.yaml"
ROUTE_FILE = PACKAGE_ROOT / "config" / "routes" / "full_course_route.yaml"


class TestRouteTool(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.route_file = Path(self.directory.name) / "route.yaml"
        shutil.copyfile(ROUTE_FILE, self.route_file)

    def tearDown(self):
        self.directory.cleanup()

    def call(self, arguments):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = main(arguments)
        return code, stdout.getvalue(), stderr.getvalue()

    def test_validate_and_list(self):
        code, output, _ = self.call(["validate", str(self.route_file)])
        self.assertEqual(code, 0)
        self.assertIn("UNCALIBRATED", output)
        code, output, _ = self.call(["list", str(self.route_file)])
        self.assertEqual(code, 0)
        self.assertIn("a_task", output)
        self.assertIn("c_loop_000", output)

    def test_generate_requires_force_for_existing_file(self):
        code, _, error = self.call([
            "generate", str(self.route_file),
            "--geometry", str(GEOMETRY_FILE),
        ])
        self.assertEqual(code, 2)
        self.assertIn("--force", error)
        code, _, _ = self.call([
            "generate", str(self.route_file),
            "--geometry", str(GEOMETRY_FILE),
            "--force",
        ])
        self.assertEqual(code, 0)
        self.assertFalse(load_route(self.route_file).calibrated)

    def test_nudge_is_atomic_and_clears_calibration(self):
        self.assertEqual(
            self.call(["mark-calibrated", str(self.route_file)])[0], 0)
        self.assertTrue(load_route(self.route_file).calibrated)
        code, _, _ = self.call([
            "nudge", str(self.route_file), "a_task", "--dx", "-0.01",
        ])
        self.assertEqual(code, 0)
        route = load_route(self.route_file)
        self.assertFalse(route.calibrated)
        point = next(point for point in route.waypoints if point.id == "a_task")
        self.assertAlmostEqual(point.x, 4.14)

    def test_set_changes_selected_fields(self):
        code, _, _ = self.call([
            "set", str(self.route_file), "a_task",
            "--x", "4.10", "--y", "1.30", "--yaw-deg", "12",
        ])
        self.assertEqual(code, 0)
        point = next(
            point for point in load_route(self.route_file).waypoints
            if point.id == "a_task"
        )
        self.assertEqual((point.x, point.y, point.yaw_deg), (4.1, 1.3, 12.0))

    def test_noop_set_and_nudge_are_rejected_without_rewriting(self):
        original = self.route_file.read_bytes()
        self.assertEqual(self.call([
            "set", str(self.route_file), "a_task",
        ])[0], 2)
        self.assertEqual(self.call([
            "nudge", str(self.route_file), "a_task",
        ])[0], 2)
        self.assertEqual(self.route_file.read_bytes(), original)

    def test_insert_then_delete_round_trip(self):
        before = load_route(self.route_file)
        code, _, error = self.call([
            "insert", str(self.route_file),
            "--after", "a_depart", "--id", "a_field_adjustment",
            "--zone", "A", "--x", "1.0", "--y", "0.1", "--yaw-deg", "7",
        ])
        self.assertEqual(code, 0, error)
        self.assertTrue(any(
            point.id == "a_field_adjustment"
            for point in load_route(self.route_file).waypoints
        ))
        self.assertEqual(
            self.call(["delete", str(self.route_file), "a_field_adjustment"])[0], 0)
        after = load_route(self.route_file)
        self.assertEqual(after.waypoints, before.waypoints)

    def test_invalid_delete_does_not_replace_file(self):
        original = self.route_file.read_bytes()
        code, _, error = self.call([
            "delete", str(self.route_file), "p_start",
        ])
        self.assertEqual(code, 2)
        self.assertIn("first", error)
        self.assertEqual(self.route_file.read_bytes(), original)

    def test_mark_calibrated_can_be_cleared(self):
        self.assertEqual(
            self.call(["mark-calibrated", str(self.route_file)])[0], 0)
        self.assertTrue(load_route(self.route_file).calibrated)
        self.assertEqual(self.call([
            "mark-calibrated", str(self.route_file), "--uncalibrated",
        ])[0], 0)
        self.assertFalse(load_route(self.route_file).calibrated)


if __name__ == "__main__":
    unittest.main()
