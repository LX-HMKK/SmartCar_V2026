"""Static launch contracts for SmartCar safety-topic routing.

The test is deliberately ROS-runtime independent so it can run on the host.
It verifies the exact launch-argument forwarding required to keep a disabled
safety node from orphaning the chassis command path.
"""
import ast
from pathlib import Path
import unittest


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
BASE_SERIAL = (
    REPOSITORY_ROOT
    / "src"
    / "origincar"
    / "origincar_base"
    / "launch"
    / "base_serial.launch.py"
)


def source_tree(path):
    return ast.parse(path.read_text(encoding="utf-8"))


def string_values(node):
    return {
        item.value
        for item in ast.walk(node)
        if isinstance(item, ast.Constant) and isinstance(item.value, str)
    }


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
    def test_top_level_conditionally_starts_safety(self):
        source = SMARTCAR_BRINGUP.read_text(encoding="utf-8")
        self.assertIn("smartcar_safety.launch.py", source)
        self.assertIn("condition=IfCondition(use_safety)", source)

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
