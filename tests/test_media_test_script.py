"""Static contract for the short on-vehicle Aurora RGB media launcher."""
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "media_test.sh"


class MediaTestScriptTests(unittest.TestCase):
    def test_only_accepts_short_qr_or_vlm_suffix(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("media_test.sh qr|vlm", source)
        self.assertIn('case "$1" in', source)
        self.assertIn("  qr)", source)
        self.assertIn("  vlm)", source)

    def test_always_selects_aurora_rgb_without_motion_stack(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertEqual(source.count("camera_driver:=aurora"), 2)
        self.assertIn("input_source:=camera", source)
        self.assertNotIn("camera_driver:=usb", source)
        self.assertNotIn("camera_driver:=mipi", source)
        self.assertNotIn("aurora_depth_enable:=", source)
        self.assertNotIn("aurora_point_cloud_enable:=", source)
        self.assertEqual(source.count("aurora_resolution_mode_index:=2"), 1)
        for forbidden in ("smartcar_bringup", "use_base:=true", "use_nav:=true"):
            self.assertNotIn(forbidden, source)

    def test_qr_output_uses_the_qr_display_and_vlm_auto_requests(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("--continuous", source)
        self.assertIn("--output-topic /smartcar/output/qr", source)
        self.assertIn("QR_PROBE_LOG", source)
        self.assertIn("MEDIA_STATE_DIR=/tmp/smartcar_media", source)
        self.assertIn("record_media_pid qr_probe", source)
        self.assertIn("record_media_pid vlm_launch", source)
        self.assertNotIn("ros2 topic pub --times 3 --rate 2", source)
        self.assertIn(
            "ros2 run smartcar_tools rgb_imshow",
            source,
        )
        self.assertIn("start_rgb_imshow QR", source)
        self.assertIn("start_rgb_imshow VLM", source)
        self.assertIn("competition_output_display", source)
        self.assertIn("auto_request:=true", source)
        self.assertIn("通用的猜测描述", (
            ROOT / "src" / "smartcar_tools" / "launch" / "vlm_test.launch.py"
        ).read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
