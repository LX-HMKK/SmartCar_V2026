"""Static contracts keep the editor isolated and fail-closed without ROS imports."""
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class TestRouteEditorContract(unittest.TestCase):
    def test_launch_starts_safety_latched_and_never_starts_navigation(self):
        source = (PACKAGE_ROOT / "launch" / "route_editor.launch.py").read_text(
            encoding="utf-8")
        self.assertIn('"emergency_stop_on_start": "true"', source)
        self.assertIn('"require_scan": "true"', source)
        self.assertIn('"require_odom": "true"', source)
        self.assertIn('"require_raw_odom": "true"', source)
        self.assertIn('"latch_emergency_stop": True', source)
        self.assertNotIn('package="nav2_', source)
        self.assertNotIn('executable="navigation_runner"', source)
        self.assertNotIn("origincar_base", source)

    def test_editor_exposes_required_services_and_rviz_goal_topic(self):
        source = (PACKAGE_ROOT / "smartcar_tools" / "route_editor_node.py").read_text(
            encoding="utf-8")
        for service in ("load", "undo", "clear", "save"):
            self.assertIn(f'f"{{SERVICE_PREFIX}}/{service}"', source)
        rviz = (PACKAGE_ROOT / "rviz" / "route_editor.rviz").read_text(
            encoding="utf-8")
        self.assertIn("/goal_pose", rviz)
        self.assertIn("/smartcar/route_editor/markers", rviz)


if __name__ == "__main__":
    unittest.main()
