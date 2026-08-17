"""Contracts for the low-overhead competition launch runner."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock, patch
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "competition_launch.py"
spec = importlib.util.spec_from_file_location("competition_launch", SCRIPT)
competition_launch = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = competition_launch
spec.loader.exec_module(competition_launch)


class CompetitionLaunchContracts(unittest.TestCase):
    def test_parses_known_launch_arguments_without_ros_imports(self):
        self.assertEqual(
            competition_launch.parse_launch_arguments([
                "use_nav:=true",
                "waypoints_file:=/tmp/a:=b.yaml",
            ]),
            [
                ("use_nav", "true"),
                ("waypoints_file", "/tmp/a:=b.yaml"),
            ],
        )

    def test_rejects_non_launch_arguments(self):
        with self.assertRaisesRegex(ValueError, "name:=value"):
            competition_launch.parse_launch_arguments(["--show-args"])
        with self.assertRaisesRegex(ValueError, "name:=value"):
            competition_launch.parse_launch_arguments([":=true"])
        with self.assertRaisesRegex(ValueError, "name:=value"):
            competition_launch.parse_launch_arguments(["use_nav:="])

    def test_duplicate_argument_uses_the_final_value(self):
        self.assertEqual(
            competition_launch.parse_launch_arguments([
                "use_nav:=false",
                "use_nav:=true",
            ]),
            [("use_nav", "true")],
        )

    def test_runner_contains_no_ros_cli_dependency(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("LaunchService", source)
        self.assertIn("install_empty_node_extension_cache", source)
        self.assertNotIn("ros2launch", source)

    def test_usr1_requests_graceful_launch_shutdown(self):
        launch_service = Mock()
        with patch.object(competition_launch.signal, "signal") as register:
            competition_launch.install_graceful_shutdown_signal(launch_service)

        signum, handler = register.call_args.args
        self.assertEqual(signum, competition_launch.signal.SIGUSR1)
        handler(signum, None)
        launch_service.shutdown.assert_called_once_with(force_sync=True)


if __name__ == "__main__":
    unittest.main()
