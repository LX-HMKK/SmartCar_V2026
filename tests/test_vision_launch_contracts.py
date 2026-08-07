"""Static contracts for the SmartCar camera, zbar, and vision launch wiring."""
import ast
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "smartcar_vision"
LAUNCH_FILE = PACKAGE / "launch" / "smartcar_vision.launch.py"
NODE_FILE = PACKAGE / "smartcar_vision" / "vision_node.py"
VOLCENGINE_CLI = (
    PACKAGE / "smartcar_vision" / "volcengine_vlm_cli.py"
)
PACKAGE_XML = PACKAGE / "package.xml"
SETUP_FILE = PACKAGE / "setup.py"
DEFAULT_CONFIG = PACKAGE / "config" / "vision.yaml"
VOLCENGINE_CONFIG = PACKAGE / "config" / "vision_volcengine.yaml"


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
    def test_usb_is_default_and_aurora_depth_is_explicit_opt_in(self):
        source = LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertEqual(launch_default(source, "camera_driver"), "usb")
        self.assertEqual(launch_default(source, "aurora_depth_enable"), "false")
        self.assertEqual(
            launch_default(source, "aurora_point_cloud_enable"), "false")
        self.assertIn("deptrum-ros-driver-aurora930", source)
        self.assertIn("aurora930_launch.py", source)
        for setting in (
            '"rgb_enable": aurora_rgb_enable',
            '"rgb_fps": "15"',
            '"depth_enable": aurora_depth_enable',
            '"ir_enable": "false"',
            '"point_cloud_enable": aurora_point_cloud_enable',
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
        self.assertIn("aurora_point_cloud_enable requires aurora_depth_enable", source)

        for config_file in (DEFAULT_CONFIG, VOLCENGINE_CONFIG):
            parameters = yaml.safe_load(
                config_file.read_text(encoding="utf-8")
            )["vision_node"]["ros__parameters"]
            self.assertEqual(parameters["image_topic"], "/image")

        node_source = NODE_FILE.read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("image_topic", "/image")', node_source)

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
        self.assertIn("depth_pointcloud_relay", SETUP_FILE.read_text(encoding="utf-8"))

    def test_volcengine_vlm_is_explicit_opt_in_without_stored_credentials(self):
        default = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
        default_params = default["vision_node"]["ros__parameters"]
        self.assertEqual(default_params["vlm_backend_mode"], "disabled")

        volcengine = yaml.safe_load(
            VOLCENGINE_CONFIG.read_text(encoding="utf-8"))
        params = volcengine["vision_node"]["ros__parameters"]
        self.assertEqual(params["vlm_backend_mode"], "command")
        argv = params["vlm_command_argv"]
        self.assertEqual(
            argv[:5],
            [
                "python3",
                "-X",
                "utf8",
                "-m",
                "smartcar_vision.volcengine_vlm_cli",
            ],
        )
        self.assertIn("{image}", argv)
        self.assertIn("{prompt}", argv)
        config_source = VOLCENGINE_CONFIG.read_text(encoding="utf-8")
        self.assertNotIn("Authorization", config_source)
        self.assertNotIn("api_key", config_source.lower())

        cli_source = VOLCENGINE_CLI.read_text(encoding="utf-8")
        self.assertIn("ARK_API_KEY", cli_source)
        self.assertIn("DOUBAO_KEY", cli_source)
        self.assertIn("https://ark.cn-beijing.volces.com/api/v3", cli_source)
        setup_source = SETUP_FILE.read_text(encoding="utf-8")
        self.assertIn("volcengine_vlm_cli", setup_source)


if __name__ == "__main__":
    unittest.main()
