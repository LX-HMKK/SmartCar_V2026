"""Static contracts for full-system composition and physical-test gates."""
import ast
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[3]
SYSTEM = ROOT / "src" / "smartcar_bringup" / "launch" / "smartcar_system.launch.py"
BRINGUP = ROOT / "src" / "smartcar_bringup" / "launch" / "smartcar_bringup.launch.py"
PACKAGE_XML = ROOT / "src" / "smartcar_bringup" / "package.xml"
COORD = ROOT / "src" / "smartcar_bringup" / "config" / "bringup_coord.yaml"
VENDOR = ROOT / "src" / "origincar" / "origincar_base" / "launch" / "origincar_bringup.launch.py"
NAV = ROOT / "src" / "smartcar_nav2" / "launch" / "navigation_launch.py"
VISION = ROOT / "src" / "smartcar_vision" / "launch" / "smartcar_vision.launch.py"
TASK_NODE = ROOT / "src" / "smartcar_task" / "smartcar_task" / "task_node.py"
DEPTH_OVERLAY = (
    ROOT / "src" / "smartcar_nav2" / "config"
    / "depth_camera_obstacle_overlay.yaml"
)
UDEV_RULES = ROOT / "config" / "udev" / "99-smartcar-aurora-usb-power.rules"


def launch_default(path, name):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == name
        ):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(
                keyword.value, ast.Constant
            ):
                return keyword.value.value
    raise AssertionError(f"launch argument {name!r} not found")


def assigned_literal(path, name):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and target.id == name:
            return ast.literal_eval(node.value)
    raise AssertionError(f"assignment {name!r} not found")


class SystemContractTests(unittest.TestCase):
    def test_system_exposes_all_switches_and_never_autostarts_motion(self):
        expected_defaults = {
            "use_base": "true",
            "use_lidar": "true",
            "use_laser_odometry": "false",
            "use_imu_filter": "false",
            "use_robot_description": "false",
            "use_safety": "true",
            "use_nav": "true",
            "use_camera": "true",
            "use_vision": "true",
            "use_depth_camera": "false",
            "use_task": "true",
            "use_visualization": "false",
            "use_speech": "false",
            "autostart_mission": "false",
            "supervised_p_to_a_only": "false",
            "supervised_p_to_c1_only": "false",
            "c_zone_direction": "counterclockwise",
            "qr_handoff_test_mode": "false",
            "use_sim_time": "false",
            "nav_autostart": "true",
            "nav2_lifecycle_manager_delay_sec": "20.0",
            "safety_emergency_stop_on_start": "false",
        }
        for name, expected in expected_defaults.items():
            with self.subTest(name=name):
                self.assertEqual(launch_default(SYSTEM, name), expected)

    def test_system_composes_all_layers_and_common_waypoints(self):
        source = SYSTEM.read_text(encoding="utf-8")
        for package, launch_file in (
            ("smartcar_bringup", "smartcar_bringup.launch.py"),
            ("smartcar_nav2", "navigation_launch.py"),
            ("smartcar_vision", "smartcar_vision.launch.py"),
            ("smartcar_task", "smartcar_task.launch.py"),
            ("smartcar_speech", "smartcar_speech.launch.py"),
        ):
            self.assertIn(f'FindPackageShare("{package}")', source)
            self.assertIn(f'"{launch_file}"', source)
        self.assertIn('"waypoints_file": waypoints_file', source)
        self.assertIn(
            '"waypoints_file": LaunchConfiguration("waypoints_file")',
            source,
        )
        self.assertIn(
            '"c_zone_direction": LaunchConfiguration("c_zone_direction")',
            source,
        )
        self.assertIn('"c_zone_direction": LaunchConfiguration("c_zone_direction"),', source)
        self.assertIn(
            '"autostart": LaunchConfiguration("nav_autostart")', source)
        self.assertIn(
            '"lifecycle_manager_delay_sec": LaunchConfiguration(', source)
        self.assertIn('"nav2_lifecycle_manager_delay_sec")', source)
        self.assertNotIn('"use_waypoint_follower"', source)
        self.assertIn(
            '"autostart_mission": LaunchConfiguration("autostart_mission")',
            source,
        )
        self.assertIn(
            '"supervised_p_to_a_only": LaunchConfiguration(', source)
        self.assertIn(
            '"supervised_p_to_c1_only": LaunchConfiguration(', source)
        self.assertIn(
            '"qr_handoff_test_mode": LaunchConfiguration(', source)
        self.assertIn('"qr_handoff_test_mode")', source)
        self.assertIn('"use_base": use_base', source)
        self.assertIn('"use_safety_ackermann": "true"', source)

    def test_camera_topic_is_resolved_once_for_vision_and_task(self):
        source = SYSTEM.read_text(encoding="utf-8")
        topics = assigned_literal(SYSTEM, "CAMERA_TOPICS")
        self.assertEqual(launch_default(SYSTEM, "camera_driver"), "usb")
        self.assertEqual(topics, assigned_literal(VISION, "DRIVER_TOPICS"))
        self.assertIn("def _resolve_camera_source(context):", source)
        self.assertIn('"barcode_reader_image_topic": image_topic', source)
        self.assertIn("OpaqueFunction(function=_task_actions)", source)

    def test_depth_camera_can_be_the_only_obstacle_source(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertEqual(launch_default(SYSTEM, "use_depth_camera"), "false")
        self.assertIn(
            'use_nav requires use_lidar=true or use_depth_camera=true ', source)
        self.assertEqual(
            launch_default(SYSTEM, "allow_synthetic_obstacle_source"),
            "false",
        )
        self.assertIn("synthetic_fixture_is_safe", source)
        self.assertIn('and _as_bool(context, "use_safety")', source)
        self.assertIn('"safety_emergency_stop_on_start"', source)
        self.assertIn('not _as_bool(context, "autostart_mission")', source)
        self.assertIn('executable="depth_pointcloud_relay"', source)
        self.assertIn('package="pointcloud_to_laserscan"', source)
        self.assertIn('executable="pointcloud_to_laserscan_node"', source)
        self.assertIn('("scan", DEPTH_LASER_SCAN_TOPIC)', source)
        self.assertIn('"use_inf": True', source)
        self.assertIn('"target_frame": DEPTH_SCAN_TARGET_FRAME', source)
        self.assertIn('"queue_size": 1', source)
        self.assertIn('"max_capture_age_sec": 0.10', source)
        self.assertEqual(
            assigned_literal(SYSTEM, "DEPTH_SCAN_CLEARING_ENVELOPE_M"), 3.01
        )
        self.assertIn('"range_max": DEPTH_SCAN_CLEARING_ENVELOPE_M', source)
        self.assertIn('<exec_depend>pointcloud_to_laserscan</exec_depend>',
                      PACKAGE_XML.read_text(encoding="utf-8"))
        self.assertNotIn("depth_camera_calibrated", source)
        self.assertIn('"safety_require_depth_points": LaunchConfiguration(', source)
        self.assertIn('"use_depth_camera")', source)
        self.assertIn('"safety_require_scan": use_lidar', source)
        # Run the depth-only Aurora source at 10 Hz and leave relay headroom
        # so legitimate capture frames are never rate-limited downstream.
        self.assertIn('"max_publish_rate_hz": 12.0', source)
        self.assertEqual(launch_default(SYSTEM, "aurora_ir_fps"), "10")
        self.assertEqual(
            launch_default(SYSTEM, "aurora_resolution_mode_index"), "0")
        self.assertEqual(launch_default(SYSTEM, "aurora_heart_enable"), "false")
        self.assertEqual(launch_default(VISION, "aurora_ir_fps"), "10")
        self.assertEqual(
            launch_default(VISION, "aurora_resolution_mode_index"), "0")
        self.assertEqual(launch_default(VISION, "aurora_heart_enable"), "false")
        vision_source = VISION.read_text(encoding="utf-8")
        self.assertIn('"heart_enable": aurora_heart_enable', vision_source)
        self.assertIn("effective_aurora_rgb_fps", vision_source)

        config = yaml.safe_load(COORD.read_text(encoding="utf-8"))
        self.assertIs(config["toggles"]["use_lidar"], False)
        self.assertIs(config["toggles"]["use_depth_camera"], True)
        depth = config["extrinsics"]["link_to_depth_camera"]
        self.assertIs(depth["measured"], True)
        self.assertEqual(depth["child"], "depth_camera_link_1")
        self.assertEqual(depth["xyz"], [0.0599, 0.0, 0.12])
        self.assertEqual(depth["rpy"], [-1.5708, 0.0, -1.5708])
        self.assertNotIn("depth_camera_calibrated", config["motion_gates"])

        overlay = yaml.safe_load(DEPTH_OVERLAY.read_text(encoding="utf-8"))
        for costmap in ("local_costmap", "global_costmap"):
            layer = overlay[costmap][costmap]["ros__parameters"][
                "obstacle_layer"]
            self.assertEqual(layer["observation_sources"], "depth_scan")
            self.assertNotIn("depth_points", layer)
            self.assertEqual(layer["depth_scan"]["topic"], "/smartcar/depth/scan")
            self.assertEqual(layer["depth_scan"]["data_type"], "LaserScan")
            self.assertEqual(layer["depth_scan"]["observation_persistence"], 0.0)
            self.assertEqual(layer["depth_scan"]["expected_update_rate"], 1.0)
            self.assertTrue(layer["depth_scan"]["clearing"])
            self.assertTrue(layer["depth_scan"]["marking"])
            self.assertTrue(layer["depth_scan"]["inf_is_valid"])
            self.assertEqual(layer["depth_scan"]["obstacle_max_range"], 3.0)
            self.assertEqual(layer["depth_scan"]["raytrace_max_range"], 3.0)

        rules = UDEV_RULES.read_text(encoding="utf-8")
        self.assertIn('ATTR{idVendor}=="05e3", ATTR{idProduct}=="0610"', rules)
        self.assertIn('ATTR{idVendor}=="3251", ATTR{idProduct}=="1930"', rules)
        self.assertEqual(rules.count('ATTR{power/control}="on"'), 2)

    def test_system_rejects_external_nav2_overlays(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn("requested_overlay", source)
        self.assertIn(
            "smartcar_system does not accept external Nav2 parameter overlays",
            source,
        )
        self.assertIn('"allow_params_overlay": allow_params_overlay', source)

    def test_base_switch_only_gates_vendor_chassis_include(self):
        source = BRINGUP.read_text(encoding="utf-8")
        self.assertIn(
            "DeclareLaunchArgument('use_base', default_value='true')",
            source,
        )
        self.assertIn("use_base = LaunchConfiguration('use_base')", source)
        self.assertIn("condition=IfCondition(use_base)", source)
        self.assertIn("safety = IncludeLaunchDescription", source)

    def test_unused_sensor_and_visualization_nodes_are_opt_in(self):
        system_source = SYSTEM.read_text(encoding="utf-8")
        vendor_source = VENDOR.read_text(encoding="utf-8")
        self.assertEqual(launch_default(SYSTEM, "use_visualization"), "false")
        self.assertEqual(launch_default(SYSTEM, "use_imu_filter"), "false")
        self.assertEqual(
            launch_default(SYSTEM, "use_robot_description"), "false"
        )
        self.assertIn('"use_imu_filter": use_imu_filter', system_source)
        self.assertIn(
            '"use_robot_description": use_robot_description', system_source
        )
        self.assertEqual(
            system_source.count("condition=IfCondition(use_visualization)"),
            2,
        )
        self.assertIn('executable="field_reference_node"', system_source)
        self.assertIn('executable="waypoint_viz"', system_source)
        self.assertIn("condition=IfCondition(use_imu_filter)", vendor_source)
        self.assertGreaterEqual(
            vendor_source.count(
                "condition=IfCondition(use_robot_description)"
            ),
            2,
        )

    def test_autostart_requires_a_real_base_chain(self):
        source = SYSTEM.read_text(encoding="utf-8")
        required_block = source.split("required_components = (", 1)[1].split(
            ")", 1
        )[0]
        self.assertIn('"use_base"', required_block)

    def test_physical_base_cannot_run_without_the_safety_gate(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn(
            'raise RuntimeError("use_base requires use_safety")', source)
        self.assertIn('if _as_bool(context, "use_base")', source)

    def test_lower_bringup_rejects_physical_safety_bypasses(self):
        source = BRINGUP.read_text(encoding="utf-8")
        self.assertIn("OpaqueFunction(function=_validate_configuration)", source)
        self.assertIn("raise RuntimeError('use_base requires use_safety')", source)
        self.assertIn(
            "raise RuntimeError('use_base requires use_safety_ackermann')",
            source,
        )

    def test_startup_emergency_stop_is_explicit_and_defaults_false(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn('"safety_emergency_stop_on_start"', source)
        self.assertIn(
            '"safety_emergency_stop_on_start": LaunchConfiguration(', source)

    def test_laser_odometry_is_opt_in_and_requires_calibration(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertEqual(
            launch_default(SYSTEM, "use_laser_odometry"), "false")
        self.assertEqual(
            launch_default(SYSTEM, "laser_odometry_calibrated"), "false")
        self.assertIn('"use_laser_odometry": use_laser_odometry', source)
        self.assertIn('missing.append("laser_odometry_calibrated")', source)

    def test_base_and_laser_tf_are_parameterized_at_the_unique_vendor_owner(self):
        source = VENDOR.read_text(encoding="utf-8")
        for name in (
            "base_x", "base_y", "base_z", "base_roll", "base_pitch", "base_yaw",
            "laser_x", "laser_y", "laser_z", "laser_roll", "laser_pitch", "laser_yaw",
        ):
            self.assertIn(f"DeclareLaunchArgument('{name}', default_value='0.0')", source)
            self.assertIn(f"LaunchConfiguration('{name}')", source)
        self.assertNotIn("'0.41'", source)
        self.assertNotIn("'0.12'", source)

    def test_camera_tf_targets_real_driver_frames_and_is_overrideable(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn('"aurora": "rgb_camera_link"', source)
        self.assertIn('"usb": "default_usb_cam"', source)
        self.assertIn('"mipi": "default_cam"', source)
        self.assertIn('"camera_frame").perform(context)', source)
        self.assertIn('"--frame-id", "base_link"', source)

    def test_nav_and_component_launches_propagate_runtime_clock_controls(self):
        nav_source = NAV.read_text(encoding="utf-8")
        self.assertEqual(launch_default(NAV, "autostart"), "true")
        self.assertIn("'autostart': autostart", nav_source)
        self.assertNotIn("LaunchConfiguration('namespace')", nav_source)
        self.assertNotIn("DeclareLaunchArgument(\n        'namespace'", nav_source)
        for path in (BRINGUP, VISION):
            source = path.read_text(encoding="utf-8")
            self.assertIn("use_sim_time", source)

    def test_vision_can_run_services_without_starting_a_camera(self):
        source = VISION.read_text(encoding="utf-8")
        for argument in ("use_camera", "use_services", "use_zbar"):
            self.assertIn(f'"{argument}"', source)
        self.assertIn("if use_camera:", source)
        self.assertIn("if use_services:", source)
        self.assertIn("if use_zbar:", source)

    def test_system_forwards_an_explicit_vision_config_file(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn('"vision_config_file"', source)
        self.assertIn('"config_file": LaunchConfiguration(', source)
        self.assertIn('"vision_config_file").perform(context)', source)
        self.assertIn('FindPackageShare("smartcar_vision")', source)

    def test_speech_is_optional_and_receives_an_explicit_config_file(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertEqual(launch_default(SYSTEM, "use_speech"), "false")
        self.assertIn('condition=IfCondition(use_speech)', source)
        self.assertIn(
            '"config_file": LaunchConfiguration("speech_config_file")',
            source,
        )
        required_block = source.split("required_components = (", 1)[1].split(
            ")", 1
        )[0]
        self.assertNotIn('"use_speech"', required_block)

    def test_all_motion_gates_default_false_in_task_and_system(self):
        for name in (
            "waypoints_calibrated",
            "extrinsics_calibrated",
            "steering_calibrated",
            "emergency_stop_ready",
            "operator_approved",
        ):
            self.assertEqual(launch_default(SYSTEM, name), "false")
            self.assertIn(
                f'self.declare_parameter("{name}", False)',
                TASK_NODE.read_text(encoding="utf-8"),
            )

    def test_coord_config_preserves_measured_extrinsics_and_closes_pending_gates(self):
        config = yaml.safe_load(COORD.read_text(encoding="utf-8"))
        self.assertIs(config["toggles"]["use_base"], True)
        self.assertIs(config["extrinsics"]["base_to_link"]["measured"], True)
        self.assertIs(config["extrinsics"]["link_to_laser"]["measured"], True)
        self.assertIs(config["extrinsics"]["link_to_camera"]["measured"], True)
        gates = config["motion_gates"]
        for name in (
            "waypoints_calibrated",
            "extrinsics_calibrated",
            "steering_calibrated",
            "emergency_stop_ready",
            "operator_approved",
            "laser_odometry_calibrated",
        ):
            self.assertIs(gates[name], False)

    def test_competition_system_defaults_match_measured_calibration(self):
        config = yaml.safe_load(COORD.read_text(encoding="utf-8"))
        base = config["extrinsics"]["base_to_link"]
        laser = config["extrinsics"]["link_to_laser"]
        camera = config["extrinsics"]["link_to_camera"]
        depth_camera = config["extrinsics"]["link_to_depth_camera"]

        extrinsic_defaults = assigned_literal(SYSTEM, "extrinsic_defaults")
        camera_defaults = assigned_literal(
            SYSTEM, "camera_extrinsic_defaults"
        )
        depth_camera_defaults = assigned_literal(
            SYSTEM, "depth_camera_extrinsic_defaults"
        )
        expected_base = dict(zip(
            ("base_x", "base_y", "base_z"),
            (str(value) for value in base["xyz"]),
        ))
        expected_laser = dict(zip(
            ("laser_x", "laser_y", "laser_z"),
            (str(value) for value in laser["xyz"]),
        ))
        expected_laser.update(dict(zip(
            ("laser_roll", "laser_pitch", "laser_yaw"),
            (str(value) for value in laser["rpy"]),
        )))
        expected_camera = dict(zip(
            ("camera_x", "camera_y", "camera_z"),
            (str(value) for value in camera["xyz"]),
        ))
        expected_depth_camera = dict(zip(
            ("depth_camera_x", "depth_camera_y", "depth_camera_z"),
            (str(value) for value in depth_camera["xyz"]),
        ))
        expected_depth_camera.update(dict(zip(
            ("depth_camera_roll", "depth_camera_pitch", "depth_camera_yaw"),
            (str(value) for value in depth_camera["rpy"]),
        )))

        for name, expected in {**expected_base, **expected_laser}.items():
            with self.subTest(name=name):
                self.assertEqual(extrinsic_defaults.get(name, "0.0"), expected)
        for name, expected in expected_camera.items():
            with self.subTest(name=name):
                self.assertEqual(camera_defaults.get(name, "0.0"), expected)
        for name, expected in expected_depth_camera.items():
            with self.subTest(name=name):
                self.assertAlmostEqual(
                    float(depth_camera_defaults.get(name, "0.0")),
                    float(expected),
                )
        self.assertAlmostEqual(
            base["xyz"][0] + depth_camera["xyz"][0],
            config["calibration"]["wheelbase"],
        )
        self.assertEqual(
            launch_default(SYSTEM, "gyro_z_bias"),
            str(config["calibration"]["gyro_z_bias"]),
        )
        for name in (
            "longitudinal_velocity_scale",
            "gyro_z_scale",
            "steering_command_scale",
            "steering_command_offset_rad",
            "max_calibrated_steering_command_rad",
        ):
            with self.subTest(name=name):
                self.assertAlmostEqual(
                    float(launch_default(SYSTEM, name)),
                    float(config["calibration"][name]),
                )
        self.assertEqual(config["calibration"]["steering_command_scale"], 1.0)
        self.assertEqual(
            config["calibration"]["max_calibrated_steering_command_rad"],
            0.70,
        )
        self.assertEqual(config["calibration"]["max_steering_angle"], 0.70)
        for launch_file in (SYSTEM, BRINGUP, VENDOR):
            with self.subTest(launch_file=launch_file.name):
                self.assertEqual(
                    launch_default(launch_file, "steering_command_scale"),
                    "1.0",
                )
                self.assertEqual(
                    launch_default(
                        launch_file,
                        "max_calibrated_steering_command_rad",
                    ),
                    "0.70",
                )

    def test_bringup_declares_direct_runtime_and_test_dependencies(self):
        source = PACKAGE_XML.read_text(encoding="utf-8")
        for dependency in (
            "ament_index_python",
            "launch",
            "launch_ros",
            "smartcar_nav2",
            "smartcar_safety",
            "smartcar_task",
            "smartcar_vision",
            "smartcar_speech",
            "smartcar_tools",
            "rf2o_laser_odometry",
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", source)
        self.assertIn("<test_depend>ament_cmake_pytest</test_depend>", source)


if __name__ == "__main__":
    unittest.main()
