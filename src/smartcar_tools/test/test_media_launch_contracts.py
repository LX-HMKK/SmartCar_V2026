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
        self.assertIn('input_source not in ("camera", "file")', source)
        self.assertIn('executable="image_replay_node"', source)
        self.assertIn('FindPackageShare("smartcar_vision")', source)
        self.assertIn('"use_zbar": "true"', source)
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
        self.assertEqual(
            launch_default(VLM, "xauthority"),
            "/var/run/lightdm/root/:0",
        )
        self.assertEqual(launch_default(VLM, "fullscreen"), "true")
        self.assertIn("SetEnvironmentVariable", source)
        self.assertIn('name="XAUTHORITY"', source)
        self.assertIn('executable="vlm_display"', source)
        self.assertIn('"use_zbar": "false"', source)
        self.assertIn('"output_topic": "/smartcar/output/text"', source)
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


if __name__ == "__main__":
    unittest.main()
