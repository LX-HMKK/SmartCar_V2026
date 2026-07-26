"""Static contracts for the WSL2 Gazebo simulation launch path."""

import copy
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src" / "smartcar_sim"
LAUNCH = SIM / "launch" / "sim.launch.py"
SIM_ENV = SIM / "scripts" / "sim_env.sh"
SIM_START = SIM / "scripts" / "sim_start.sh"
WSLG_WAIT = SIM / "scripts" / "wait_for_wslg.sh"
AUTO_TRAIN = SIM / "scripts" / "auto_train.py"
ODOM_RELAY = SIM / "scripts" / "odom_relay.py"
ODOM_COMBINED_RELAY = SIM / "scripts" / "odom_combined_relay.py"
SIM_TUNE = SIM / "scripts" / "sim_tune.sh"
TUNE_PARAMS = SIM / "scripts" / "tune_params.py"
RVIZ = SIM / "rviz" / "sim_nav.rviz"
WORLD = SIM / "worlds" / "track.world"
FIELD_MODEL_CONFIG = SIM / "config" / "competition_field_model.yaml"
FIELD_MODEL_GENERATOR = SIM / "scripts" / "generate_competition_field.py"
FIELD_MODEL = SIM / "models" / "competition_field" / "model.sdf"
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

    def test_simulation_shows_the_complete_field_at_a_readable_scale(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        rviz = RVIZ.read_text(encoding="utf-8")

        self.assertIn('executable="field_reference_node"', launch)
        self.assertIn("field_geometry.yaml", launch)
        self.assertIn('"IGN_GAZEBO_RESOURCE_PATH"', launch)
        self.assertIn("Name: Official Field Reference", rviz)
        self.assertIn("Value: /smartcar/field_reference/markers", rviz)
        self.assertIn("Scale: 80.0", rviz)
        self.assertIn("X: 2.0", rviz)
        self.assertIn("Y: 2.25", rviz)

    def test_world_includes_only_the_generated_field_model(self) -> None:
        world = WORLD.read_text(encoding="utf-8")

        self.assertEqual(world.count("model://competition_field"), 1)
        for stale_model in (
            'name="p_zone"',
            'name="qr_zone"',
            'name="b_wall_west"',
            'name="b_wall_east"',
            'name="corridor_wall_',
            'name="czone_wall"',
            'name="wp_corner_',
        ):
            self.assertNotIn(stale_model, world)

    def test_generated_field_model_is_current_and_collision_scoped(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(FIELD_MODEL_GENERATOR), "--check"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(
            completed.returncode,
            0,
            msg=completed.stdout + completed.stderr,
        )

        config = yaml.safe_load(FIELD_MODEL_CONFIG.read_text(encoding="utf-8"))
        collision_policy = config["collision"]
        self.assertIs(collision_policy["b_zone_walls"], True)
        self.assertIs(collision_policy["diagnosis_room_inner_ring"], False)
        self.assertIs(collision_policy["c_ring_outer_boundary"], False)
        self.assertIs(collision_policy["field_outer_boundary"], False)

        root = ET.parse(FIELD_MODEL).getroot()
        model = root.find("model")
        self.assertIsNotNone(model)
        self.assertEqual(model.attrib["name"], "competition_field")
        link = model.find("link")
        visuals = {item.attrib["name"]: item for item in link.findall("visual")}
        collisions = {
            item.attrib["name"]: item for item in link.findall("collision")
        }

        for visual in (
            "zone_a_surface",
            "zone_b_surface",
            "zone_c_surface",
            "b_corridor_surface",
            "c_ring_outer_center",
            "c_ring_outer_west_cap",
            "c_ring_outer_east_cap",
            "diagnosis_room_inner_center",
            "diagnosis_room_inner_west_cap",
            "diagnosis_room_inner_east_cap",
            "b_wall_west",
            "b_wall_east",
        ):
            self.assertIn(visual, visuals)
        self.assertEqual(
            set(collisions),
            {"b_wall_west_collision", "b_wall_east_collision"},
        )

        self.assertEqual(
            [
                float(value)
                for value in collisions["b_wall_west_collision"].findtext(
                    "pose"
                ).split()
            ],
            [0.5, 2.0, 0.15, 0.0, 0.0, 0.0],
        )
        self.assertEqual(
            [
                float(value)
                for value in collisions["b_wall_west_collision"].findtext(
                    "geometry/box/size"
                ).split()
            ],
            [2.0, 0.5, 0.3],
        )
        self.assertEqual(
            [
                float(value)
                for value in collisions["b_wall_east_collision"].findtext(
                    "pose"
                ).split()
            ],
            [3.5, 2.0, 0.15, 0.0, 0.0, 0.0],
        )

        outer_center = visuals["c_ring_outer_center"]
        self.assertEqual(
            [float(value) for value in outer_center.findtext("pose").split()[:2]],
            [2.0, 3.325],
        )
        self.assertEqual(
            [
                float(value)
                for value in outer_center.findtext("geometry/box/size").split()[:2]
            ],
            [2.35, 1.65],
        )
        for side, expected_x in (("west", 0.825), ("east", 3.175)):
            cap = visuals[f"c_ring_outer_{side}_cap"]
            self.assertEqual(float(cap.findtext("pose").split()[0]), expected_x)
            self.assertEqual(
                float(cap.findtext("geometry/cylinder/radius")), 0.825
            )

    def test_field_generator_resolves_isolated_install_layout(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "generate_competition_field", FIELD_MODEL_GENERATOR
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as temporary:
            install = Path(temporary) / "install"
            script = (
                install
                / "smartcar_sim"
                / "share"
                / "smartcar_sim"
                / "scripts"
                / "generate_competition_field.py"
            )
            tools_share = (
                install / "smartcar_tools" / "share" / "smartcar_tools"
            )
            geometry = tools_share / "config" / "routes" / "field_geometry.yaml"
            geometry.parent.mkdir(parents=True)
            geometry.touch()

            resolved = module.resolve_tools_share(
                script,
                lambda package: str(tools_share) if package == "smartcar_tools" else "",
            )
            self.assertEqual(resolved, tools_share)

    def test_rviz_waits_for_the_wslg_socket(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        wait_script = WSLG_WAIT.read_text(encoding="utf-8")

        self.assertIn("wait_for_wslg.sh", launch)
        self.assertIn('[ -S "$wayland_socket" ]', wait_script)
        self.assertIn('exec rviz2 -d "$rviz_config"', wait_script)

    def test_odom_relays_handle_launch_shutdown_idempotently(self) -> None:
        for relay in (ODOM_RELAY, ODOM_COMBINED_RELAY):
            with self.subTest(relay=relay.name):
                source = relay.read_text(encoding="utf-8")
                self.assertIn(
                    "from rclpy.executors import ExternalShutdownException",
                    source,
                )
                self.assertIn(
                    "except (KeyboardInterrupt, ExternalShutdownException):",
                    source,
                )
                self.assertIn("if rclpy.ok():", source)

    def test_start_script_rejects_mirrored_wsl_networking(self) -> None:
        source = SIM_START.read_text(encoding="utf-8")

        self.assertIn("ip link show loopback0", source)
        self.assertIn("networkingMode=nat", source)
        self.assertIn("wsl.exe --shutdown", source)

    def test_runtime_dependencies_cover_simulation_entrypoints(self) -> None:
        package = PACKAGE_XML.read_text(encoding="utf-8")
        cmake = (SIM / "CMakeLists.txt").read_text(encoding="utf-8")

        for dependency in (
            "ament_index_python",
            "python3-yaml",
            "rmw_fastrtps_cpp",
            "rviz2",
            "smartcar_tools",
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", package)
        self.assertIn("scripts/generate_competition_field.py", cmake)

    def test_run_route_uses_an_isolated_switch_and_explicit_trees(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        runner = AUTO_TRAIN.read_text(encoding="utf-8")

        self.assertIn('LaunchConfiguration("run_route"', launch)
        self.assertIn("condition=IfCondition(run_route)", launch)
        self.assertNotIn('LaunchConfiguration("autostart"', launch)
        self.assertIn("target_action=auto_train", launch)
        self.assertIn("complete route runner exited", launch)
        for parameter in (
            '"waypoints_file"',
            '"forward_behavior_tree"',
            '"precise_behavior_tree"',
            '"reverse_behavior_tree"',
            '"reverse_handoff_behavior_tree"',
            '"nav2_params_file"',
        ):
            self.assertIn(parameter, launch)
            self.assertIn(parameter, runner)
        self.assertNotIn("TRAIN_WAYPOINTS", runner)
        self.assertIn('goal.behavior_tree = str(behavior_tree)', runner)
        self.assertIn('item.get("task") != "start"', runner)
        self.assertIn("os.replace(temporary, path)", runner)
        self.assertIn("raise SystemExit(exit_code)", runner)
        self.assertIn("self._input_manifest_cache = self._input_manifest()", runner)
        self.assertIn("path_start = len(self._path_messages)", runner)
        self.assertIn("rclpy.spin_once(self, timeout_sec=0.1)", runner)
        self.assertNotIn("time.sleep(delay)", runner)
        self.assertIn("EXPECTED_ROUTE", runner)
        self.assertIn('("c_corner_1", "reverse", "reverse_handoff")', runner)

    def test_route_results_preserve_planning_and_execution_yaw_evidence(self) -> None:
        runner = AUTO_TRAIN.read_text(encoding="utf-8")

        for field in (
            '"target_yaw_rad"',
            '"final_yaw_rad"',
            '"signed_goal_yaw_error_rad"',
            '"plan_final_yaw_rad"',
            '"signed_plan_goal_yaw_error_rad"',
            '"cmd_positive_sample_count"',
            '"cmd_negative_sample_count"',
            '"goal_profile"',
        ):
            self.assertIn(field, runner)
        self.assertIn("orientation = endpoint_pose.orientation", runner)
        self.assertIn("matching_paths[-1][2]", runner)
        self.assertIn('contract_errors.append("reverse_velocity_sign")', runner)
        self.assertIn("if positive_command_samples > 0:", runner)

    def test_rviz_path_qos_matches_nav2_volatile_publishers(self) -> None:
        rviz = RVIZ.read_text(encoding="utf-8")

        for topic in ("/plan", "/local_plan"):
            topic_offset = rviz.index(f"Value: {topic}")
            qos_block = rviz[max(0, topic_offset - 180):topic_offset]
            self.assertIn("Durability Policy: Volatile", qos_block)

    def test_tuning_rebuilds_fixed_params_and_targets_qr_checker(self) -> None:
        runner = SIM_TUNE.read_text(encoding="utf-8")
        tuner = TUNE_PARAMS.read_text(encoding="utf-8")

        self.assertIn("--packages-up-to smartcar_sim", runner)
        self.assertIn("run_route:=true", runner)
        self.assertIn('for run_index in $(seq 1 "$loop_count")', runner)
        self.assertIn("precise_goal_checker.yaw_goal_tolerance", tuner)
        self.assertIn("precise_goal_checker.xy_goal_tolerance", tuner)
        self.assertNotIn("GridBased.curvature_tolerance", tuner)
        self.assertIn("render_params", tuner)
        self.assertIn("flock -n 9", runner)
        self.assertIn("Invalid SMARTCAR_SRC", runner)
        self.assertNotIn("SMARTCAR_NAV2_PARAMS", tuner)
        self.assertIn("fcntl.flock", tuner)

    def test_tunable_defaults_match_yaml_and_render_preserves_comments(self) -> None:
        spec = importlib.util.spec_from_file_location("tune_params", TUNE_PARAMS)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        params_file = ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
        data = module.load_params(params_file)

        for info in module.TUNABLE_PARAMS.values():
            self.assertEqual(
                module.get_nested(data, info["path"]), info["default"])

        changed = copy.deepcopy(data)
        precise = module.TUNABLE_PARAMS["2"]
        module.set_nested(changed, precise["path"], 0.20)
        source = params_file.read_text(encoding="utf-8")
        rendered = module.render_params(changed, source)
        self.assertIn("# The QR pose seeds a direction change.", rendered)
        self.assertEqual(len(rendered.splitlines()), len(source.splitlines()))
        self.assertIn("yaw_goal_tolerance: 0.2", rendered)

    def test_restore_only_changes_tunable_parameters(self) -> None:
        spec = importlib.util.spec_from_file_location("tune_params_restore", TUNE_PARAMS)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            current_file = root / "nav2_params.yaml"
            backup_dir = root / "backups"
            backup_dir.mkdir()

            current = module.load_params(source)
            backup = copy.deepcopy(current)
            tunable_path = module.TUNABLE_PARAMS["2"]["path"]
            non_tunable_path = "controller_server.ros__parameters.controller_frequency"
            module.set_nested(current, tunable_path, 0.15)
            module.set_nested(current, non_tunable_path, 17.0)
            module.set_nested(backup, tunable_path, 0.20)
            module.set_nested(backup, non_tunable_path, 99.0)
            current_file.write_text(
                yaml.safe_dump(current, sort_keys=False),
                encoding="utf-8",
            )
            (backup_dir / "sample.yaml").write_text(
                yaml.safe_dump(backup, sort_keys=False),
                encoding="utf-8",
            )

            module.PARAMS_FILE = current_file
            module.BACKUP_DIR = backup_dir
            self.assertTrue(module.restore_params("sample"))
            restored = module.load_params(current_file)
            self.assertEqual(module.get_nested(restored, tunable_path), 0.20)
            self.assertEqual(module.get_nested(restored, non_tunable_path), 17.0)
            self.assertFalse(module.restore_params("../sample"))


if __name__ == "__main__":
    unittest.main()
