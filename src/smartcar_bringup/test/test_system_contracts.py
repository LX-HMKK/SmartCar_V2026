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
NAV = ROOT / "src" / "smartcar_nav2" / "launch" / "smartcar_nav2.launch.py"
VISION = ROOT / "src" / "smartcar_vision" / "launch" / "smartcar_vision.launch.py"
TASK_NODE = ROOT / "src" / "smartcar_task" / "smartcar_task" / "task_node.py"


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


class SystemContractTests(unittest.TestCase):
    def test_system_exposes_all_switches_and_never_autostarts_motion(self):
        expected_defaults = {
            "use_base": "true",
            "use_lidar": "true",
            "use_obstacle": "true",
            "use_safety": "true",
            "use_nav": "true",
            "use_camera": "true",
            "use_vision": "true",
            "use_task": "true",
            "autostart_mission": "false",
            "use_sim_time": "false",
            "nav_autostart": "true",
            "safety_emergency_stop_on_start": "false",
        }
        for name, expected in expected_defaults.items():
            with self.subTest(name=name):
                self.assertEqual(launch_default(SYSTEM, name), expected)

    def test_system_composes_all_four_layers_and_common_waypoints(self):
        source = SYSTEM.read_text(encoding="utf-8")
        for package, launch_file in (
            ("smartcar_bringup", "smartcar_bringup.launch.py"),
            ("smartcar_nav2", "smartcar_nav2.launch.py"),
            ("smartcar_vision", "smartcar_vision.launch.py"),
            ("smartcar_task", "smartcar_task.launch.py"),
        ):
            self.assertIn(f'FindPackageShare("{package}")', source)
            self.assertIn(f'"{launch_file}"', source)
        self.assertGreaterEqual(source.count('"waypoints_file": waypoints_file'), 2)
        self.assertIn('"autostart": nav_autostart', source)
        self.assertIn('"autostart_mission": autostart_mission', source)
        self.assertIn('"use_base": use_base', source)

    def test_base_switch_only_gates_vendor_chassis_include(self):
        source = BRINGUP.read_text(encoding="utf-8")
        self.assertIn(
            "DeclareLaunchArgument('use_base', default_value='true')",
            source,
        )
        self.assertIn("use_base = LaunchConfiguration('use_base')", source)
        self.assertIn("condition=IfCondition(use_base)", source)
        self.assertIn("safety = IncludeLaunchDescription", source)

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

    def test_startup_emergency_stop_is_explicit_and_defaults_false(self):
        source = SYSTEM.read_text(encoding="utf-8")
        self.assertIn('"safety_emergency_stop_on_start"', source)
        self.assertIn(
            '"safety_emergency_stop_on_start": LaunchConfiguration(', source)

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

    def test_coord_config_marks_unmeasured_extrinsics_and_placeholder_waypoints(self):
        config = yaml.safe_load(COORD.read_text(encoding="utf-8"))
        self.assertIs(config["toggles"]["use_base"], True)
        self.assertIs(config["extrinsics"]["base_to_link"]["measured"], False)
        self.assertIs(config["extrinsics"]["link_to_laser"]["measured"], False)
        self.assertIs(config["extrinsics"]["link_to_camera"]["measured"], False)
        gates = config["motion_gates"]
        self.assertTrue(all(value is False for value in gates.values()))

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
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", source)
        self.assertIn("<test_depend>ament_cmake_pytest</test_depend>", source)


if __name__ == "__main__":
    unittest.main()
