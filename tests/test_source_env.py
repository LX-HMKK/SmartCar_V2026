"""Contracts for the RDK shell environment bootstrap."""
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ENV = ROOT / "scripts" / "source_env.sh"
NAV_PREPARE = ROOT / "scripts" / "nav_prepare.sh"
SIM_ENV = ROOT / "src" / "smartcar_sim" / "scripts" / "sim_env.sh"
SIM_LAUNCH = ROOT / "src" / "smartcar_sim" / "launch" / "sim.launch.py"
SIM_PACKAGE = ROOT / "src" / "smartcar_sim" / "package.xml"


class SourceEnvironmentContracts(unittest.TestCase):
    def test_competition_fast_path_is_optional_and_uses_a_snapshot(self):
        source = SOURCE_ENV.read_text(encoding="utf-8")

        self.assertIn(
            "COMPETITION_ENV_SNAPSHOT=/home/sunrise/ros2_ws/"
            ".smartcar_competition_env.sh",
            source,
        )
        self.assertIn('SMARTCAR_COMPETITION_FAST_ENV:-', source)
        self.assertIn('source "$COMPETITION_ENV_SNAPSHOT"', source)

    def test_workspace_setup_is_the_primary_entrypoint(self):
        source = SOURCE_ENV.read_text(encoding="utf-8")

        workspace_setup = source.index(
            "source /home/sunrise/ros2_ws/install/setup.bash")
        fallback = source.index("source /opt/tros/humble/setup.bash")
        self.assertLess(workspace_setup, fallback)
        self.assertIn("else", source[workspace_setup:fallback])

    def test_prepare_generates_a_post_build_snapshot_without_dds_state(self):
        source = NAV_PREPARE.read_text(encoding="utf-8")

        self.assertIn("write_competition_env_snapshot()", source)
        self.assertIn("SMARTCAR_COMPETITION_FAST_ENV=0", source)
        self.assertIn("Prepared competition environment snapshot", source)
        self.assertNotIn("ROS_DOMAIN_ID", source)
        self.assertNotIn("RMW_IMPLEMENTATION", source)

    def test_simulation_does_not_select_a_specific_rmw(self):
        sim_environment = SIM_ENV.read_text(encoding="utf-8")
        self.assertNotIn("RMW_IMPLEMENTATION", sim_environment)
        self.assertNotIn("CYCLONEDDS_URI", sim_environment)
        self.assertNotIn(
            'SetEnvironmentVariable("RMW_IMPLEMENTATION"',
            SIM_LAUNCH.read_text(encoding="utf-8"),
        )
        self.assertNotIn(
            "rmw_fastrtps_cpp", SIM_PACKAGE.read_text(encoding="utf-8")
        )


if __name__ == "__main__":
    unittest.main()
