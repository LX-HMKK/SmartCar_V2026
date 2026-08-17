"""Static isolation contracts for standalone media test launch files."""
import ast
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"
SPEECH = LAUNCH_DIR / "speech_test.launch.py"
QR = LAUNCH_DIR / "qr_test.launch.py"
VLM = LAUNCH_DIR / "vlm_test.launch.py"


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
            if (
                keyword.arg == "default_value"
                and isinstance(keyword.value, ast.Constant)
            ):
                return keyword.value.value
    raise AssertionError(f"launch argument {name!r} not found")


class MediaLaunchContractTests(unittest.TestCase):
    def test_launch_files_are_valid_python(self):
        for path in (SPEECH, QR, VLM):
            with self.subTest(path=path.name):
                ast.parse(path.read_text(encoding="utf-8"))

    def test_speech_entry_only_composes_speech(self):
        source = SPEECH.read_text(encoding="utf-8")
        self.assertEqual(launch_default(SPEECH, "enabled"), "true")
        self.assertIn('FindPackageShare("smartcar_speech")', source)
        for forbidden in (
            "smartcar_bringup",
            "smartcar_nav2",
            "smartcar_task",
            "smartcar_vision",
        ):
            self.assertNotIn(forbidden, source)

    def test_qr_entry_supports_camera_and_file_without_motion_stack(self):
        source = QR.read_text(encoding="utf-8")
        self.assertEqual(launch_default(QR, "input_source"), "camera")
        self.assertEqual(launch_default(QR, "camera_driver"), "aurora")
        self.assertEqual(
            launch_default(QR, "aurora_resolution_mode_index"), "0")
        self.assertIn('input_source not in ("camera", "file")', source)
        self.assertIn('executable="image_replay_node"', source)
        self.assertIn('FindPackageShare("smartcar_vision")', source)
        self.assertIn('"use_zbar": "true"', source)
        self.assertIn('"aurora_resolution_mode_index": LaunchConfiguration(', source)
        for forbidden in (
            "smartcar_bringup",
            "smartcar_nav2",
            "smartcar_task",
            "smartcar_speech",
        ):
            self.assertNotIn(forbidden, source)

    def test_vlm_entry_uses_hdmi_text_and_never_starts_speech(self):
        source = VLM.read_text(encoding="utf-8")
        self.assertEqual(launch_default(VLM, "display"), ":0")
        self.assertEqual(launch_default(VLM, "camera_driver"), "aurora")
        self.assertEqual(
            launch_default(VLM, "xauthority"),
            "/var/run/lightdm/root/:0",
        )
        self.assertEqual(launch_default(VLM, "fullscreen"), "true")
        self.assertEqual(launch_default(VLM, "auto_request"), "false")
        self.assertIn("SetEnvironmentVariable", source)
        self.assertIn('name="XAUTHORITY"', source)
        self.assertIn('executable="vlm_display"', source)
        self.assertIn('"use_zbar": "false"', source)
        self.assertIn('"output_topic": "/smartcar/output/text"', source)
        self.assertIn('"auto_request": auto_request', source)
        self.assertIn("通用的猜测描述", source)
        self.assertIn("vision_volcengine.yaml", source)
        for forbidden in (
            "smartcar_bringup",
            "smartcar_nav2",
            "smartcar_task",
            "smartcar_speech",
        ):
            self.assertNotIn(forbidden, source)

    def test_vlm_entry_supports_camera_and_fresh_file_replay(self):
        source = VLM.read_text(encoding="utf-8")
        self.assertIn('input_source not in ("camera", "file")', source)
        self.assertIn('executable="image_replay_node"', source)
        self.assertIn('"publish_rate_hz"', source)
        self.assertIn('"image_topic": source_topic', source)

    def test_file_replay_offers_reliable_images_for_zbar(self):
        source = (PACKAGE_ROOT / "smartcar_tools" / "image_replay_node.py").read_text(
            encoding="utf-8")
        self.assertIn("ReliabilityPolicy.RELIABLE", source)
        self.assertIn("DurabilityPolicy.VOLATILE", source)
        self.assertIn("image_qos", source)

    def test_vlm_auto_request_is_opt_in_and_runs_once_after_a_ready_frame(self):
        source = (PACKAGE_ROOT / "smartcar_tools" / "vlm_display.py").read_text(
            encoding="utf-8")
        self.assertIn("auto_request = pyqtSignal()", source)
        self.assertIn('self.declare_parameter("auto_request", False)', source)
        self.assertIn("and not self._auto_request_sent", source)
        self.assertIn("and self._client.service_is_ready()", source)
        self.assertIn("qt_bridge.auto_request.emit()", source)
        self.assertIn("qt_bridge.auto_request.connect(window._request)", source)
        self.assertIn("Ignoring invalid camera frame before VLM request", source)
        self.assertNotIn("frame_received = pyqtSignal", source)
        self.assertNotIn("cv2.imshow", source)
        self.assertNotIn("cv_bridge", source)
        self.assertNotIn("CvBridge", source)

    def test_media_display_tools_are_rgb_only_and_competition_ui_is_text_only(self):
        rgb_source = (PACKAGE_ROOT / "smartcar_tools" / "rgb_imshow.py").read_text(
            encoding="utf-8")
        output_source = (
            PACKAGE_ROOT / "smartcar_tools" / "competition_output_display.py"
        ).read_text(encoding="utf-8")
        setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
        self.assertIn("cv2.imshow", rgb_source)
        self.assertIn('"/aurora/rgb/image_raw"', rgb_source)
        self.assertIn("qos_profile_sensor_data", rgb_source)
        self.assertIn('"/smartcar/output/text"', output_source)
        self.assertIn('"/smartcar/output/qr"', output_source)
        self.assertIn('"/smartcar/output/c_zone_direction"', output_source)
        self.assertIn('"/smartcar/output/vlm"', output_source)
        self.assertIn('"/smartcar/task/state"', output_source)
        self.assertIn("text_received", output_source)
        self.assertIn("qr_received", output_source)
        self.assertIn("c_zone_direction_received", output_source)
        self.assertIn("vlm_received", output_source)
        self.assertIn("state_received", output_source)
        self.assertIn("rgb_imshow", setup_source)
        self.assertIn("competition_output_display", setup_source)
        for forbidden in (
            "sensor_msgs",
            "cv2.",
            "cv_bridge",
            "qos_profile_sensor_data",
        ):
            self.assertNotIn(forbidden, output_source)
        for forbidden in ("smartcar_bringup", "smartcar_nav2", "smartcar_task"):
            self.assertNotIn(forbidden, rgb_source)
            self.assertNotIn(forbidden, output_source)

    def test_competition_ui_remote_start_is_opt_in_and_single_click(self):
        source = (
            PACKAGE_ROOT / "smartcar_tools" / "competition_output_display.py"
        ).read_text(encoding="utf-8")
        self.assertIn("from PyQt5.QtCore import QProcess", source)
        self.assertIn(
            'self.declare_parameter("remote_start_enabled", False)', source)
        self.assertIn(
            'self.declare_parameter("remote_start_command", "")', source)
        self.assertIn("competition_start_argv(remote_start_command)", source)
        self.assertIn("if remote_start_enabled else ()", source)
        self.assertIn("self._start_process = QProcess(self)", source)
        self.assertIn("QProcess.MergedChannels", source)
        self.assertNotIn('confirmation.setWindowTitle("确认发车")', source)
        self.assertNotIn("confirmation.exec_() != QMessageBox.Yes", source)
        self.assertIn("self._start_process.start(", source)
        self.assertIn(
            "self._start_command[0], list(self._start_command[1:]))", source)
        self.assertNotIn("shell=True", source)


if __name__ == "__main__":
    unittest.main()
