"""Source-level contracts for the no-motion odometry diagnostic."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_FILE = PACKAGE_ROOT / "smartcar_tools" / "odom_diag.py"


class OdomDiagContractTests(unittest.TestCase):
    def test_monitors_all_localization_inputs_and_output(self):
        source = SOURCE_FILE.read_text(encoding="utf-8")
        for topic in (
            "/odom",
            "/imu/data_raw",
            "/odom_combined",
            "/diagnostics",
        ):
            with self.subTest(topic=topic):
                self.assertIn(topic, source)

    def test_shutdown_occurs_outside_ros_callbacks(self):
        source = SOURCE_FILE.read_text(encoding="utf-8")
        self.assertIn("self.finished = True", source)
        self.assertIn("while rclpy.ok() and not node.finished", source)
        self.assertEqual(source.count("rclpy.shutdown()"), 1)

    def test_reports_relative_displacements_and_stationary_drift(self):
        source = SOURCE_FILE.read_text(encoding="utf-8")
        self.assertIn("Relative odometry displacement", source)
        self.assertIn("Relative displacement versus odom_combined", source)
        self.assertIn("expect_stationary", source)
        self.assertIn("stationary drift exceeds tolerance", source)

    def test_manifest_declares_diagnostic_messages(self):
        root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {item.text for item in root.findall("exec_depend")}
        self.assertIn("diagnostic_msgs", dependencies)


if __name__ == "__main__":
    unittest.main()
