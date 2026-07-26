"""Static contracts for the WSL2 Gazebo simulation launch path."""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src" / "smartcar_sim"
LAUNCH = SIM / "launch" / "sim.launch.py"
SIM_ENV = SIM / "scripts" / "sim_env.sh"
SIM_START = SIM / "scripts" / "sim_start.sh"
WSLG_WAIT = SIM / "scripts" / "wait_for_wslg.sh"
PACKAGE_XML = SIM / "package.xml"


class SimulationContractTests(unittest.TestCase):
    def test_fastdds_uses_default_transport_across_entrypoints(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        sim_env = SIM_ENV.read_text(encoding="utf-8")

        self.assertIn('"RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"', launch)
        self.assertNotIn('SetEnvironmentVariable(\n        "FASTRTPS_DEFAULT_PROFILES_FILE"', launch)
        self.assertIn("RMW_IMPLEMENTATION=rmw_fastrtps_cpp", sim_env)
        self.assertIn("unset FASTRTPS_DEFAULT_PROFILES_FILE", sim_env)

    def test_cleanup_finishes_before_simulation_nodes_start(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn("RegisterEventHandler", launch)
        self.assertIn("OnProcessExit", launch)
        self.assertIn("target_action=sim_cleanup", launch)
        self.assertIn("on_exit=[start_after_cleanup]", launch)

    def test_lidar_sensor_is_a_child_of_laser_link(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")

        expected = (
            '"laser_link", "origincar/laser_link/lidar_sensor"'
        )
        reversed_frames = (
            '"origincar/laser_link/lidar_sensor", "laser_link"'
        )
        self.assertIn(expected, launch)
        self.assertNotIn(reversed_frames, launch)

    def test_rviz_waits_for_the_wslg_socket(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        wait_script = WSLG_WAIT.read_text(encoding="utf-8")

        self.assertIn("wait_for_wslg.sh", launch)
        self.assertIn('[ -S "$wayland_socket" ]', wait_script)
        self.assertIn('exec rviz2 -d "$rviz_config"', wait_script)

    def test_start_script_rejects_mirrored_wsl_networking(self) -> None:
        source = SIM_START.read_text(encoding="utf-8")

        self.assertIn("ip link show loopback0", source)
        self.assertIn("networkingMode=nat", source)
        self.assertIn("wsl.exe --shutdown", source)

    def test_runtime_dependencies_cover_simulation_entrypoints(self) -> None:
        package = PACKAGE_XML.read_text(encoding="utf-8")

        for dependency in ("rmw_fastrtps_cpp", "rviz2", "smartcar_tools"):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", package)


if __name__ == "__main__":
    unittest.main()
