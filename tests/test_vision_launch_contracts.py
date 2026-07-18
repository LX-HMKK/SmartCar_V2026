"""Static contracts for the SmartCar camera, zbar, and vision launch wiring."""
import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "smartcar_vision"
LAUNCH_FILE = PACKAGE / "launch" / "smartcar_vision.launch.py"
NODE_FILE = PACKAGE / "smartcar_vision" / "vision_node.py"
PACKAGE_XML = PACKAGE / "package.xml"


def launch_default(source, name):
    tree = ast.parse(source)
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


class VisionLaunchContractTests(unittest.TestCase):
    def test_aurora_is_default_rgb_only_camera(self):
        source = LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertEqual(launch_default(source, "camera_driver"), "aurora")
        self.assertIn("deptrum-ros-driver-aurora930", source)
        self.assertIn("aurora930_launch.py", source)
        for setting in (
            '"rgb_enable": "true"',
            '"rgb_fps": "15"',
            '"depth_enable": "false"',
            '"ir_enable": "false"',
            '"point_cloud_enable": "false"',
            '"rgbd_enable": "false"',
            '"align_mode": "false"',
        ):
            self.assertIn(setting, source)

    def test_camera_selector_and_topic_defaults_are_explicit(self):
        source = LAUNCH_FILE.read_text(encoding="utf-8")
        for driver in ("aurora", "usb", "mipi", "none"):
            self.assertIn(f'"{driver}"', source)
        self.assertIn('"aurora": "/aurora/rgb/image_raw"', source)
        self.assertIn('"usb": "/image"', source)
        self.assertIn('"mipi": "/image_raw"', source)
        self.assertIn("hobot_usb_cam.launch.py", source)
        self.assertIn("mipi_cam_640x480_bgr8.launch.py", source)

    def test_zbar_remaps_directly_to_selected_image_source(self):
        source = LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertIn('package="zbar_ros"', source)
        self.assertIn('executable="barcode_reader"', source)
        self.assertIn('("image", source_topic)', source)
        self.assertIn('("barcode", "/barcode")', source)
        self.assertNotIn("relay_enabled", source)
        self.assertIn('"usb_pixel_format": "mjpeg2rgb"', source)
        self.assertIn('"mipi_io_method": "ros"', source)

    def test_node_uses_required_concurrency_and_receipt_time(self):
        source = NODE_FILE.read_text(encoding="utf-8")
        self.assertIn("ReentrantCallbackGroup", source)
        self.assertIn("MutuallyExclusiveCallbackGroup", source)
        self.assertIn("MultiThreadedExecutor(num_threads=3)", source)
        self.assertIn("qos_profile_sensor_data", source)
        self.assertIn("self.get_clock().now().nanoseconds", source)

    def test_package_declares_direct_runtime_dependencies(self):
        source = PACKAGE_XML.read_text(encoding="utf-8")
        for dependency in (
            "rclpy",
            "cv_bridge",
            "sensor_msgs",
            "std_msgs",
            "smartcar_interfaces",
            "zbar_ros",
            "deptrum-ros-driver-aurora930",
            "hobot_usb_cam",
            "mipi_cam",
            "launch",
            "launch_ros",
            "python3-opencv",
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", source)


if __name__ == "__main__":
    unittest.main()
