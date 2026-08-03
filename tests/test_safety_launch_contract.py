"""Static launch contracts for SmartCar safety-topic routing.

The test is deliberately ROS-runtime independent so it can run on the host.
It verifies the exact launch-argument forwarding required to keep a disabled
safety node from orphaning the chassis command path.
"""
import ast
from pathlib import Path
import unittest

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ORIGINCAR_BRINGUP = (
    REPOSITORY_ROOT
    / "src"
    / "origincar"
    / "origincar_base"
    / "launch"
    / "origincar_bringup.launch.py"
)
SMARTCAR_BRINGUP = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_bringup"
    / "launch"
    / "smartcar_bringup.launch.py"
)
SMARTCAR_SYSTEM = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_bringup"
    / "launch"
    / "smartcar_system.launch.py"
)
SMARTCAR_SAFETY_LAUNCH = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_safety"
    / "launch"
    / "smartcar_safety.launch.py"
)
BASE_SERIAL = (
    REPOSITORY_ROOT
    / "src"
    / "origincar"
    / "origincar_base"
    / "launch"
    / "base_serial.launch.py"
)
SAFETY_CONFIG = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_safety"
    / "config"
    / "safety.yaml"
)
CPP_SAFETY_NODE = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_safety"
    / "src"
    / "safety_node.cpp"
)
PYTHON_SAFETY_NODE = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_safety"
    / "smartcar_safety"
    / "safety_node.py"
)


def source_tree(path):
    return ast.parse(path.read_text(encoding="utf-8"))


def string_values(node):
    return {
        item.value
        for item in ast.walk(node)
        if isinstance(item, ast.Constant) and isinstance(item.value, str)
    }


def launch_argument_default(tree, argument_name):
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == argument_name
        ):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(
                keyword.value, ast.Constant
            ):
                return keyword.value.value
    raise AssertionError(f"launch argument {argument_name!r} not found")


def chassis_input_expression_parts(tree):
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "chassis_input_topic"
                for target in node.targets
            )
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name)
            and node.value.func.id == "PythonExpression"
        ):
            continue
        expression_items = node.value.args[0].elts
        return (
            expression_items[0].value,
            expression_items[1].id,
            expression_items[2].value,
        )
    raise AssertionError("chassis_input_topic PythonExpression not found")


class SafetyLaunchContractTests(unittest.TestCase):
    def test_ackermann_steering_cap_matches_the_verified_command_limit(self):
        params = yaml.safe_load(SAFETY_CONFIG.read_text(encoding="utf-8"))["safety_node"][
            "ros__parameters"
        ]
        self.assertAlmostEqual(params["wheelbase"], 0.189)
        self.assertAlmostEqual(params["max_steering_angle"], 0.70)

        for node in (CPP_SAFETY_NODE, PYTHON_SAFETY_NODE):
            with self.subTest(node=node.name):
                source = node.read_text(encoding="utf-8")
                self.assertIn('"max_steering_angle", 0.70', source)

    def test_safety_launch_exposes_required_raw_odom_switch(self):
        tree = source_tree(SMARTCAR_SAFETY_LAUNCH)
        values = string_values(tree)
        self.assertIn("require_raw_odom", values)
        self.assertEqual(
            launch_argument_default(tree, "require_raw_odom"),
            "true",
        )

        source = SMARTCAR_SAFETY_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"require_raw_odom": require_raw_odom', source)

    def test_safety_launch_exposes_startup_emergency_stop(self):
        tree = source_tree(SMARTCAR_SAFETY_LAUNCH)
        self.assertEqual(
            launch_argument_default(tree, "emergency_stop_on_start"),
            "false",
        )

    def test_safety_launch_always_starts_one_direction_guard(self):
        tree = source_tree(SMARTCAR_SAFETY_LAUNCH)
        self.assertIn("direction_guard_config_file", string_values(tree))
        source = SMARTCAR_SAFETY_LAUNCH.read_text(encoding="utf-8")
        self.assertEqual(source.count('executable="direction_guard_node"'), 1)
        self.assertIn('name="direction_guard"', source)
        self.assertIn('"direction_guard.yaml"', source)
        direction_block = source.split(
            'executable="direction_guard_node"', 1)[0].rsplit("Node(", 1)[1]
        self.assertNotIn("condition=", direction_block)

    def test_python_fallback_still_consumes_guarded_cmd_vel(self):
        safety_node = (
            REPOSITORY_ROOT
            / "src"
            / "smartcar_safety"
            / "smartcar_safety"
            / "safety_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'Twist, "/cmd_vel", self._on_command, LATEST_RELIABLE_QOS',
            safety_node,
        )
        source = SMARTCAR_SAFETY_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"emergency_stop_on_start": emergency_stop_on_start',
            source,
        )

    def test_top_level_conditionally_starts_safety(self):
        source = SMARTCAR_BRINGUP.read_text(encoding="utf-8")
        self.assertIn("smartcar_safety.launch.py", source)
        self.assertIn("condition=IfCondition(use_safety)", source)

    def test_physical_base_cannot_bypass_safety_ackermann(self):
        source = SMARTCAR_BRINGUP.read_text(encoding="utf-8")
        self.assertIn("OpaqueFunction(function=_validate_configuration)", source)
        self.assertIn("raise RuntimeError('use_base requires use_safety')", source)
        self.assertIn(
            "raise RuntimeError('use_base requires use_safety_ackermann')",
            source,
        )

    def test_top_level_routes_selected_topic_into_vendor_bringup(self):
        tree = source_tree(SMARTCAR_BRINGUP)
        values = string_values(tree)
        self.assertIn("use_safety", values)

        before, switch, after = chassis_input_expression_parts(tree)
        self.assertEqual(switch, "use_safety")
        self.assertEqual(eval(before + "true" + after), "/cmd_vel_safe")
        self.assertEqual(eval(before + "false" + after), "/cmd_vel")

        source = SMARTCAR_BRINGUP.read_text(encoding="utf-8")
        self.assertIn("'input_topic': chassis_input_topic", source)

    def test_top_level_forwards_required_raw_odom_switch(self):
        tree = source_tree(SMARTCAR_BRINGUP)
        values = string_values(tree)
        self.assertIn("safety_require_raw_odom", values)
        self.assertEqual(
            launch_argument_default(tree, "safety_require_raw_odom"),
            "true",
        )

        source = SMARTCAR_BRINGUP.read_text(encoding="utf-8")
        self.assertIn(
            "'require_raw_odom': safety_require_raw_odom",
            source,
        )

    def test_safety_and_laser_odometry_receive_distinct_config_files(self):
        source = SMARTCAR_BRINGUP.read_text(encoding="utf-8")
        self.assertIn("'laser_odometry_config_file'", source)
        self.assertIn("'safety_config_file'", source)
        self.assertIn("'config_file': LaunchConfiguration(\n                'laser_odometry_config_file')", source)
        self.assertIn(
            "'config_file': LaunchConfiguration('safety_config_file')",
            source,
        )

        system_source = SMARTCAR_SYSTEM.read_text(encoding="utf-8")
        self.assertIn('"safety_config_file": LaunchConfiguration(', system_source)
        self.assertIn('DeclareLaunchArgument(\n            "safety_config_file"', system_source)

    def test_vendor_bringup_forwards_topic_into_base_serial(self):
        tree = source_tree(ORIGINCAR_BRINGUP)
        values = string_values(tree)
        self.assertIn("input_topic", values)

        source = ORIGINCAR_BRINGUP.read_text(encoding="utf-8")
        self.assertIn("base_serial.launch.py", source)
        self.assertIn("'input_topic': input_topic", source)

    def test_base_serial_maps_adapter_and_driver_topics(self):
        source = BASE_SERIAL.read_text(encoding="utf-8")
        self.assertIn("'input_topic': input_topic", source)
        self.assertIn("'output_topic': output_topic", source)
        self.assertIn("'akm_cmd_vel': output_topic", source)
        self.assertIn("'cmd_vel': input_topic", source)
        self.assertIn("condition=IfCondition(akmcar)", source)
        self.assertIn("condition=UnlessCondition(akmcar)", source)


if __name__ == "__main__":
    unittest.main()
