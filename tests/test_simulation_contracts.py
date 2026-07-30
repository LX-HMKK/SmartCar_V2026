"""Static contracts for the WSL2 Gazebo simulation launch path."""

import copy
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src" / "smartcar_sim"
LAUNCH = SIM / "launch" / "sim.launch.py"
SIM_ENV = SIM / "scripts" / "sim_env.sh"
SIM_START = SIM / "scripts" / "sim_start.sh"
SIM_CLEANUP = SIM / "scripts" / "sim_cleanup.sh"
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
FIELD_MAP_GENERATOR = SIM / "scripts" / "generate_field_map.py"
FIELD_MAP = SIM / "maps" / "field_map.pgm"
FIELD_MAP_YAML = SIM / "maps" / "field_map.yaml"
KEEPOUT_OVERLAY = SIM / "config" / "nav2_keepout_filter.yaml"
PACKAGE_XML = SIM / "package.xml"


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    raw = path.read_bytes()
    magic, dimensions, max_value, pixels = raw.split(b"\n", 3)
    if magic != b"P5" or max_value != b"255":
        raise ValueError(f"unsupported PGM header in {path}")
    width, height = (int(value) for value in dimensions.split())
    if len(pixels) != width * height:
        raise ValueError(f"unexpected PGM payload size in {path}")
    return width, height, pixels


def pgm_value_at(
    pixels: bytes,
    width: int,
    height: int,
    origin: tuple[float, float],
    resolution: float,
    x: float,
    y: float,
) -> int:
    col = int((x - origin[0]) // resolution)
    row_from_bottom = int((y - origin[1]) // resolution)
    if not 0 <= col < width or not 0 <= row_from_bottom < height:
        raise ValueError(f"point ({x}, {y}) lies outside the PGM")
    row = height - 1 - row_from_bottom
    return pixels[row * width + col]


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

    def test_cleanup_reclaims_stale_keepout_stack(self) -> None:
        cleanup = SIM_CLEANUP.read_text(encoding="utf-8")

        self.assertIn(
            '"/nav2_map_server/map_server.*__node:=keepout_mask_server"',
            cleanup,
        )
        self.assertIn(
            '"/nav2_map_server/costmap_filter_info_server.*'
            '__node:=keepout_filter_info_server"',
            cleanup,
        )

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
            'name="b_zone_wall_',
            'name="c_zone_',
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
            "nav2_lifecycle_manager",
            "nav2_map_server",
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", package)
        self.assertIn("scripts/generate_competition_field.py", cmake)

    def test_keepout_mask_is_current_and_uses_authoritative_geometry(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(FIELD_MAP_GENERATOR), "--check"],
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

        descriptor = yaml.safe_load(FIELD_MAP_YAML.read_text(encoding="utf-8"))
        self.assertEqual(descriptor["mode"], "trinary")
        self.assertEqual(descriptor["resolution"], 0.1)
        self.assertEqual(descriptor["origin"], [-0.5, -0.25, 0.0])
        self.assertEqual(descriptor["negate"], 0)

        width, height, pixels = read_pgm(FIELD_MAP)
        self.assertEqual((width, height), (50, 50))
        origin = tuple(descriptor["origin"][:2])
        resolution = descriptor["resolution"]
        samples = {
            "P origin": ((0.0, 0.0), 254),
            "B corridor": ((2.0, 2.0), 254),
            "B west wall": ((0.5, 2.0), 0),
            "C inner": ((2.0, 3.3), 0),
            "C inner west lane": ((1.1, 3.3), 254),
            "C inner north lane": ((2.0, 3.6), 254),
            "C north": ((2.0, 4.4), 0),
            "C west": ((-0.25, 3.0), 0),
            "C east": ((4.25, 3.0), 0),
            "C ring": ((0.7, 4.0), 254),
        }
        for label, ((x, y), expected) in samples.items():
            with self.subTest(label=label):
                self.assertEqual(
                    pgm_value_at(
                        pixels,
                        width,
                        height,
                        origin,
                        resolution,
                        x,
                        y,
                    ),
                    expected,
                )

        source = FIELD_MAP_GENERATOR.read_text(encoding="utf-8")
        self.assertIn("load_field_reference", source)
        self.assertIn("keepout_bounds", source)
        self.assertNotIn("WALLS = [", source)

    def test_keepout_filter_is_scoped_to_simulation_and_ready_before_nav2(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        overlay = yaml.safe_load(KEEPOUT_OVERLAY.read_text(encoding="utf-8"))

        for costmap_name in ("local_costmap", "global_costmap"):
            parameters = overlay[costmap_name][costmap_name]["ros__parameters"]
            self.assertEqual(parameters["filters"], ["keepout_filter"])
            keepout = parameters["keepout_filter"]
            self.assertEqual(keepout["plugin"], "nav2_costmap_2d::KeepoutFilter")
            self.assertIs(keepout["enabled"], True)
            self.assertEqual(keepout["filter_info_topic"], "/keepout_filter_info")
            self.assertEqual(parameters["inflation_layer"]["inflation_radius"], 0.20)

        self.assertIn('executable="costmap_filter_info_server"', launch)
        self.assertIn('"topic_name": "/keepout_filter_mask"', launch)
        self.assertIn('"mask_topic": "/keepout_filter_mask"', launch)
        self.assertIn("OnStateTransition", launch)
        self.assertIn('goal_state="active"', launch)
        self.assertIn('"bond_timeout": 20.0', launch)
        self.assertIn("launch_nav2_once", launch)
        self.assertIn("OpaqueFunction(function=launch_nav2_once)", launch)
        self.assertIn('"params_overlay_file": nav2_keepout_overlay', launch)
        self.assertNotIn("ros2 lifecycle set /map_server", launch)

    def test_run_route_uses_an_isolated_switch_and_explicit_trees(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        runner = AUTO_TRAIN.read_text(encoding="utf-8")

        self.assertIn('LaunchConfiguration("run_route"', launch)
        self.assertIn('LaunchConfiguration("waypoints_file")', launch)
        self.assertIn('DeclareLaunchArgument(\n            "waypoints_file"', launch)
        self.assertIn('pkg_nav2, "config", "waypoints", "nav_only.yaml"', launch)
        self.assertIn("condition=IfCondition(run_route)", launch)
        self.assertNotIn('LaunchConfiguration("autostart"', launch)
        self.assertIn("target_action=auto_train", launch)
        self.assertIn("complete route runner exited", launch)
        self.assertIn('LaunchConfiguration(\n        "shutdown_on_route_exit"', launch)
        self.assertIn('DeclareLaunchArgument(\n            "shutdown_on_route_exit"', launch)
        self.assertIn("Gazebo/RViz", launch)
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
        self.assertIn("segment.end_id", runner)
        self.assertIn("os.replace(temporary, path)", runner)
        self.assertIn("raise SystemExit(exit_code)", runner)
        self.assertIn("self._input_manifest_cache = self._input_manifest()", runner)
        self.assertIn("path_start = len(self._path_messages)", runner)
        self.assertIn("rclpy.spin_once(self, timeout_sec=0.1)", runner)
        self.assertNotIn("time.sleep(delay)", runner)
        self.assertIn('"action_endpoint_settle_sec"', runner)
        self.assertIn("wait_for_server() only proves the request path exists", runner)
        self.assertIn("Settling Nav2 action endpoints", runner)
        self.assertIn("self._settle_action_endpoints()", runner)
        self.assertIn("load_planning_segments", runner)
        self.assertIn('stage["direction"]', runner)
        self.assertIn("_run_stage", runner)
        self.assertNotIn("EXPECTED_ROUTE", runner)
        self.assertNotIn("reverse_tp_start_id", runner)

        nav_only = yaml.safe_load(
            (ROOT / "src" / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml")
            .read_text(encoding="utf-8")
        )
        self.assertEqual(
            [
                (segment["id"], segment["direction"], segment["start_id"], segment["end_id"])
                for segment in nav_only["planning_segments"]
            ],
            [
                ("p_to_qr", "forward", "p_start", "a_task_observe"),
                ("reverse_corridor", "reverse", "a_task_observe", "b_corridor_enter"),
                ("reverse_c_entry", "reverse", "b_corridor_enter", "c_corner_3"),
                ("c_exit", "reverse", "c_corner_3", "b_corridor_return_enter"),
                ("return_to_p", "reverse", "b_corridor_return_enter", "p_finish"),
            ],
        )
        reverse_corridor, reverse_c_entry = nav_only["planning_segments"][1:3]
        self.assertEqual(reverse_corridor["through_ids"], [])
        self.assertEqual(
            reverse_c_entry["through_ids"],
            ["c_corner_1", "c_corner_2"],
        )

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
        self.assertIn('source_root="${workspace}/src"', runner)
        self.assertIn("--sync-from-windows", runner)
        self.assertIn("--sync-only requires --sync-from-windows", runner)
        self.assertIn("SMARTCAR_WINDOWS_SRC", runner)
        self.assertIn("backup_wsl_file_before_sync", runner)
        self.assertIn('"nav_only.yaml"', runner)
        self.assertIn('"nav2_params.yaml"', runner)
        self.assertIn("${file_label%.yaml}.before-windows-sync-", runner)
        self.assertIn("nav2_params.yaml did not match Windows source", runner)
        self.assertIn('sha256sum "$1"', runner)
        self.assertNotIn("    --delete \\", runner)
        self.assertIn('waypoints_file:="$snapshot_dir/nav_only.yaml"', runner)
        self.assertIn('--waypoints-file "$snapshot_dir/nav_only.yaml"', runner)
        self.assertIn('SOURCE_ROOT = WS / "src"', tuner)
        self.assertIn("require_workspace_source", tuner)
        self.assertIn("SMARTCAR_SRC is no longer accepted", tuner)
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

    def test_tuning_uses_workspace_source_and_rejects_legacy_override(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            legacy_source = Path(temporary) / "windows_source"
            with mock.patch.dict(
                os.environ,
                {
                    "SMARTCAR_WS": str(workspace),
                    "SMARTCAR_SRC": str(legacy_source),
                },
                clear=False,
            ):
                spec = importlib.util.spec_from_file_location(
                    "tune_params_workspace_source", TUNE_PARAMS)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)

                self.assertEqual(module.SOURCE_ROOT, workspace / "src")
                with self.assertRaisesRegex(
                    RuntimeError, "SMARTCAR_SRC is no longer accepted"
                ):
                    module.require_workspace_source()

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
