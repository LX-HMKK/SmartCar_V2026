"""Static contracts for the native Ubuntu Gazebo simulation launch path."""

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
LOCAL_SIM = ROOT / "scripts" / "local_sim.sh"
SIM = ROOT / "src" / "smartcar_sim"
LAUNCH = SIM / "launch" / "sim.launch.py"
SIM_ENV = SIM / "scripts" / "sim_env.sh"
SIM_START = SIM / "scripts" / "sim_start.sh"
SIM_CLEANUP = SIM / "scripts" / "sim_cleanup.sh"
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
SIM_SPEED_PROFILES = SIM / "launch" / "sim_speed_profiles.py"
NAV2_PARAMS = ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
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


def load_sim_speed_profiles():
    spec = importlib.util.spec_from_file_location(
        "sim_speed_profiles_for_test", SIM_SPEED_PROFILES)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(spec.name, None)
    return module


class SimulationContractTests(unittest.TestCase):
    def test_local_sim_clears_snap_gui_runtime_without_losing_desktop_session(self) -> None:
        runner = LOCAL_SIM.read_text(encoding="utf-8")

        self.assertIn("is_snap_terminal=false", runner)
        self.assertIn("SNAP_REAL_HOME", runner)
        self.assertIn("XDG_DATA_DIRS_VSCODE_SNAP_ORIG", runner)
        self.assertIn("XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG", runner)
        for variable in (
            "LD_PRELOAD",
            "GTK_DATA_PREFIX",
            "GTK_EXE_PREFIX",
            "GTK_PATH",
            "GTK_IM_MODULE_FILE",
            "GDK_PIXBUF_MODULE_FILE",
            "GDK_PIXBUF_MODULEDIR",
            "GIO_EXTRA_MODULES",
            "GIO_MODULE_DIR",
            "GSETTINGS_SCHEMA_DIR",
            "GI_TYPELIB_PATH",
        ):
            with self.subTest(variable=variable):
                self.assertIn(variable, runner)
        for session_variable in (
            "DISPLAY",
            "WAYLAND_DISPLAY",
            "XAUTHORITY",
            "XDG_RUNTIME_DIR",
            "DBUS_SESSION_BUS_ADDRESS",
        ):
            with self.subTest(session_variable=session_variable):
                self.assertNotIn(f"unset {session_variable}", runner)

    def test_sim_speed_profiles_are_constrained_and_preserve_keepout(self) -> None:
        profiles = load_sim_speed_profiles()
        config_dir = SIM / "config"

        self.assertEqual(
            set(profiles.SIM_SPEED_PROFILES), {"baseline", "0.20", "0.25"})
        baseline = profiles.resolve_sim_speed_profile("baseline", config_dir)
        self.assertEqual(baseline.linear_speed_mps, 0.15)
        self.assertIsNone(profiles.speed_overlay_path(baseline, config_dir))

        keepout = yaml.safe_load(KEEPOUT_OVERLAY.read_text(encoding="utf-8"))
        for name, expected_speed in (("0.20", 0.20), ("0.25", 0.25)):
            with self.subTest(profile=name):
                profile = profiles.resolve_sim_speed_profile(name, config_dir)
                overlay_path = profiles.speed_overlay_path(profile, config_dir)
                self.assertIsNotNone(overlay_path)
                profiles.validate_speed_overlay(profile, overlay_path)
                overlay = yaml.safe_load(overlay_path.read_text(encoding="utf-8"))
                controller = overlay["controller_server"]["ros__parameters"]
                smoother = overlay["velocity_smoother"]["ros__parameters"]
                self.assertEqual(
                    controller["FollowPath"]["desired_linear_vel"],
                    expected_speed,
                )
                self.assertEqual(
                    smoother["max_velocity"], [expected_speed, 0.0, 0.75])
                self.assertEqual(
                    smoother["min_velocity"], [-expected_speed, 0.0, -0.75])
                self.assertNotIn("ReverseHandoff", controller)

                with tempfile.TemporaryDirectory() as temporary:
                    merged_path = Path(temporary) / "merged.yaml"
                    profiles.write_merged_nav2_overlay(
                        KEEPOUT_OVERLAY, overlay_path, merged_path)
                    merged = yaml.safe_load(merged_path.read_text(encoding="utf-8"))
                self.assertEqual(
                    merged["controller_server"]["ros__parameters"]
                    ["FollowPath"]["desired_linear_vel"],
                    expected_speed,
                )
                self.assertEqual(
                    merged["local_costmap"]["local_costmap"]["ros__parameters"]
                    ["filters"],
                    keepout["local_costmap"]["local_costmap"]["ros__parameters"]
                    ["filters"],
                )

        with self.assertRaises(ValueError):
            profiles.resolve_sim_speed_profile("0.30", config_dir)

    def test_sim_speed_launch_is_opt_in_and_keeps_real_defaults(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        runner = AUTO_TRAIN.read_text(encoding="utf-8")
        params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))

        self.assertIn('LaunchConfiguration("sim_speed_profile")', launch)
        self.assertIn('"sim_speed_profile",\n            default_value="baseline"', launch)
        self.assertIn(
            "from launch_ros.parameter_descriptions import ParameterValue", launch
        )
        self.assertIn(
            "ParameterValue(\n                sim_speed_profile, value_type=str)", launch
        )
        self.assertIn("prepare_sim_speed_overlay", launch)
        self.assertIn("validate_speed_overlay", launch)
        self.assertIn("write_merged_nav2_overlay", launch)
        self.assertIn('"nav2_params_overlay_file": active_nav2_overlay_file', launch)
        self.assertIn("OnShutdown", launch)
        self.assertIn('self.declare_parameter("nav2_params_overlay_file", "")', runner)
        self.assertIn('self.declare_parameter("sim_speed_profile", "baseline")', runner)
        self.assertIn('"sim_speed_profile": str(', runner)
        self.assertEqual(
            params["controller_server"]["ros__parameters"]["FollowPath"]
            ["desired_linear_vel"],
            0.15,
        )
        self.assertEqual(
            params["velocity_smoother"]["ros__parameters"]["max_velocity"][0],
            0.15,
        )
        self.assertEqual(
            params["velocity_smoother"]["ros__parameters"]["min_velocity"][0],
            -0.15,
        )

    def test_fastdds_uses_default_transport_across_entrypoints(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        sim_env = SIM_ENV.read_text(encoding="utf-8")

        self.assertIn('"RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"', launch)
        self.assertNotIn('SetEnvironmentVariable(\n        "FASTRTPS_DEFAULT_PROFILES_FILE"', launch)
        self.assertIn("RMW_IMPLEMENTATION=rmw_fastrtps_cpp", sim_env)
        self.assertIn("unset FASTRTPS_DEFAULT_PROFILES_FILE", sim_env)
        self.assertIn("sim_env_script_dir=", sim_env)
        self.assertIn("sim_env_workspace=", sim_env)
        self.assertNotIn("unset script_dir", sim_env)
        self.assertNotIn("unset workspace", sim_env)

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

    def test_rviz_launches_directly_on_native_ubuntu(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn('package="rviz2"', launch)
        self.assertIn('executable="rviz2"', launch)
        self.assertIn('name="sim_rviz"', launch)
        self.assertNotIn("wait_for_wslg.sh", launch)

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

    def test_odom_combined_relay_uses_wall_clock_and_preserves_input_stamp(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        relay = ODOM_COMBINED_RELAY.read_text(encoding="utf-8")

        self.assertIn(
            'name="odom_combined_relay",\n        parameters=[{"use_sim_time": False}]',
            launch,
        )
        self.assertIn("stamp = msg.header.stamp", relay)
        self.assertIn("msg.header.stamp = stamp", relay)
        self.assertIn("t.header.stamp = stamp", relay)
        self.assertNotIn("create_timer(", relay)
        self.assertNotIn("get_clock()", relay)
        self.assertNotIn("_fallback", relay)

    def test_start_script_has_no_wsl_network_dependency(self) -> None:
        source = SIM_START.read_text(encoding="utf-8")

        self.assertIn("native Ubuntu", source)
        self.assertIn("sim_cleanup.sh", source)
        self.assertNotIn("loopback0", source)
        self.assertNotIn("networkingMode", source)
        self.assertNotIn("wsl.exe", source)

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
        rpp_cost_distance = overlay["controller_server"]["ros__parameters"][
            "FollowPath"
        ]["cost_scaling_dist"]

        for costmap_name in ("local_costmap", "global_costmap"):
            parameters = overlay[costmap_name][costmap_name]["ros__parameters"]
            self.assertEqual(parameters["filters"], ["keepout_filter"])
            keepout = parameters["keepout_filter"]
            self.assertEqual(keepout["plugin"], "nav2_costmap_2d::KeepoutFilter")
            self.assertIs(keepout["enabled"], True)
            self.assertEqual(keepout["filter_info_topic"], "/keepout_filter_info")
            self.assertEqual(parameters["inflation_layer"]["inflation_radius"], 0.20)
            self.assertLessEqual(
                rpp_cost_distance,
                parameters["inflation_layer"]["inflation_radius"],
            )

        self.assertIn('executable="costmap_filter_info_server"', launch)
        self.assertIn('"topic_name": "/keepout_filter_mask"', launch)
        self.assertIn('"mask_topic": "/keepout_filter_mask"', launch)
        self.assertIn("OnStateTransition", launch)
        self.assertIn('goal_state="active"', launch)
        self.assertIn('"bond_timeout": 20.0', launch)
        self.assertIn("launch_nav2_once", launch)
        self.assertIn("OpaqueFunction(function=launch_nav2_once)", launch)
        # Every trial starts from the same simulation-only keepout overlay;
        # optional speed caps are merged into a temporary file before Nav2.
        self.assertIn(
            "keepout_overlay_path = Path(nav2_keepout_overlay.perform(context))",
            launch,
        )
        self.assertIn("write_merged_nav2_overlay(", launch)
        self.assertIn('"params_overlay_file": str(overlay_path)', launch)
        self.assertIn('"lifecycle_manager_delay_sec": "2.0"', launch)
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
        self.assertIn("materialize_navigation_segments", runner)
        self.assertIn("materialize_free_yaws", runner)
        self.assertIn("NavigateThroughPoses", runner)
        self.assertIn("NavigateThroughPoses.Goal()", runner)
        self.assertIn("goal.poses.append(ps)", runner)
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
        self.assertEqual(
            reverse_corridor["through_ids"],
            ["b_corridor_gate"],
        )
        self.assertEqual(
            reverse_c_entry["through_ids"],
            ["c_entry_west", "c_corner_1", "c_corner_2"],
        )
        return_to_p = nav_only["planning_segments"][-1]
        self.assertEqual(
            return_to_p["through_ids"],
            ["b_corridor_return_drop", "b_corridor_return"],
        )
        c_entry_west = next(
            waypoint for waypoint in nav_only["waypoints"]
            if waypoint["id"] == "c_entry_west"
        )
        self.assertEqual(c_entry_west["task"], "via")
        self.assertEqual(c_entry_west["direction"], "reverse")
        self.assertNotIn("orientation", c_entry_west["pose"])
        return_drop = next(
            waypoint for waypoint in nav_only["waypoints"]
            if waypoint["id"] == "b_corridor_return_drop"
        )
        self.assertEqual(return_drop["task"], "via")
        self.assertEqual(return_drop["direction"], "reverse")
        self.assertNotIn("orientation", return_drop["pose"])

    def test_route_results_preserve_planning_and_execution_yaw_evidence(self) -> None:
        runner = AUTO_TRAIN.read_text(encoding="utf-8")

        for field in (
            '"heading_mode"',
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
        self.assertIn('if heading_mode == "locked":', runner)
        self.assertIn("free-heading route goals must use the zero quaternion", runner)

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
        self.assertIn("workspace_lock_id=$(printf", runner)
        self.assertIn('/tmp/smartcar_sim_tune_${workspace_lock_id}.lock', runner)
        self.assertIn("WORKSPACE_LOCK_ID = hashlib.sha256", tuner)
        self.assertIn("/tmp/smartcar_sim_tune_{WORKSPACE_LOCK_ID}.lock", tuner)
        self.assertNotIn('WS / ".sim_tune.lock"', tuner)
        self.assertIn('source_root="${workspace}/src"', runner)
        self.assertIn('workspace=${SMARTCAR_WS:-$(cd "${script_dir}/../../.." && pwd)}', runner)
        self.assertNotIn("--sync-from-windows", runner)
        self.assertNotIn("SMARTCAR_WINDOWS_SRC", runner)
        self.assertNotIn("backup_wsl_file_before_sync", runner)
        self.assertIn(
            'waypoints_file="${source_root}/smartcar_nav2/config/waypoints/nav_only.yaml"',
            runner,
        )
        self.assertIn(
            '"${source_root}/smartcar_nav2/config/nav2_params.yaml"',
            runner,
        )
        self.assertIn('sha256sum "$1"', runner)
        self.assertIn('waypoints_file:="$snapshot_dir/nav_only.yaml"', runner)
        self.assertIn('--waypoints-file "$snapshot_dir/nav_only.yaml"', runner)
        self.assertIn('SOURCE_ROOT = WS / "src"', tuner)
        self.assertIn('SCRIPT = Path(__file__).resolve()', tuner)
        self.assertNotIn("require_workspace_source", tuner)
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

    def test_tuning_uses_explicit_workspace_when_requested(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            with mock.patch.dict(
                os.environ,
                {
                    "SMARTCAR_WS": str(workspace),
                },
                clear=False,
            ):
                spec = importlib.util.spec_from_file_location(
                    "tune_params_workspace_source", TUNE_PARAMS)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)

                self.assertEqual(module.SOURCE_ROOT, workspace / "src")

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
