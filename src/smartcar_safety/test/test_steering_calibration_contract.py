"""Source contracts for the default-disabled safety-owned steering hold."""

from pathlib import Path
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CPP_NODE = PACKAGE_ROOT / "src" / "safety_node.cpp"
PYTHON_NODE = PACKAGE_ROOT / "smartcar_safety" / "safety_node.py"
CONFIG = PACKAGE_ROOT / "config" / "safety.yaml"
INTERFACE = (
    PACKAGE_ROOT.parent / "smartcar_interfaces" / "srv"
    / "HoldSteeringCalibration.srv"
)
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"


class SteeringCalibrationSafetyContractTests(unittest.TestCase):
    def test_interface_has_only_angle_and_bounded_duration_inputs(self):
        source = INTERFACE.read_text(encoding="utf-8")
        request_fields = source.split("---", maxsplit=1)[0]
        self.assertIn("float64 steering_angle", source)
        self.assertIn("float64 duration_sec", source)
        self.assertNotIn("float64 speed", request_fields)
        self.assertIn("bool success", source)
        self.assertIn("string status", source)

    def test_default_configuration_leaves_static_steering_disabled(self):
        params = yaml.safe_load(
            CONFIG.read_text(encoding="utf-8"))[
                "safety_node"]["ros__parameters"]
        self.assertFalse(params["allow_steering_calibration"])
        self.assertEqual(params["steering_calibration_max_hold_sec"], 15.0)
        self.assertEqual(params["max_steering_angle"], 0.70)

    def test_both_safety_implementations_fail_closed_before_overriding(
            self):
        for node in (CPP_NODE, PYTHON_NODE):
            with self.subTest(node=node.name):
                source = node.read_text(encoding="utf-8")
                self.assertIn("allow_steering_calibration", source)
                self.assertIn("steering_calibration_max_hold_sec", source)
                self.assertIn("steering_calibration_disabled", source)
                self.assertIn("steering_calibration_request_invalid", source)
                self.assertIn(
                    "steering_calibration_command_not_quiescent", source)
                self.assertIn("steering_calibration_safety_blocked", source)
                self.assertIn("steering_calibration_cancelled", source)
                self.assertIn("expires", source)
                self.assertIn("quiescent", source)
                self.assertIn("steering_override", source)

    def test_cpp_safety_build_links_the_shared_service_interface(self):
        source = CMAKE.read_text(encoding="utf-8")
        self.assertIn("smartcar_interfaces", source)
        self.assertIn("test_steering_calibration_contract_py", source)


if __name__ == "__main__":
    unittest.main()
