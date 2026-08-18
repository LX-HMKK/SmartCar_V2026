"""ROS-independent tests for media probes, replay, and display state."""
import os
from pathlib import Path
import sys
import tempfile
import unittest


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.image_replay_node import resolve_image_file  # noqa: E402
from smartcar_tools.competition_output_display import (  # noqa: E402
    CompetitionOutputWindow,
    OutputBridge as CompetitionOutputBridge,
    competition_start_argv,
    normalize_output_text,
)
from smartcar_tools.qr_probe import (  # noqa: E402
    INVALID_ARGUMENT,
    QR_NOT_FOUND,
    SUCCESS,
    competition_output_text,
    response_exit_code,
)
from smartcar_tools.rgb_imshow import (  # noqa: E402
    normalize_window_title,
    should_close,
)
from smartcar_tools.vlm_display import (  # noqa: E402
    QApplication,
    DisplayResult,
    UiBridge,
    VlmWindow,
    result_kind,
    result_status_text,
)


class QrProbeTests(unittest.TestCase):
    def test_exit_code_is_explicit(self):
        self.assertEqual(response_exit_code(True, "ok"), SUCCESS)
        self.assertEqual(
            response_exit_code(False, "qr_timeout"), QR_NOT_FOUND)
        self.assertEqual(response_exit_code(True, "unexpected"), QR_NOT_FOUND)
        self.assertNotEqual(INVALID_ARGUMENT, QR_NOT_FOUND)

    def test_competition_output_keeps_the_decoded_qr_payload(self):
        self.assertEqual(
            competition_output_text(True, "9697", "ok"),
            "9697",
        )
        self.assertEqual(
            competition_output_text(True, " 0007 ", "ok"),
            "0007",
        )
        self.assertEqual(
            competition_output_text(True, "640", "ok"),
            "640",
        )
        self.assertEqual(
            competition_output_text(True, "  ", "ok"),
            "未识别",
        )
        self.assertEqual(
            competition_output_text(False, "奇数", "qr_timeout"),
            "未识别",
        )


class ImageReplayTests(unittest.TestCase):
    def test_resolve_requires_an_existing_file(self):
        with self.assertRaisesRegex(ValueError, "must be provided"):
            resolve_image_file("")
        with self.assertRaisesRegex(ValueError, "does not exist"):
            resolve_image_file("definitely-missing-image.jpg")
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "frame.jpg"
            image.write_bytes(b"placeholder")
            self.assertEqual(resolve_image_file(image), image.resolve())


class MediaDisplayTests(unittest.TestCase):
    def test_imshow_title_and_close_keys_are_explicit(self):
        self.assertEqual(normalize_window_title(""), "Aurora RGB")
        self.assertEqual(normalize_window_title("  RGB  "), "RGB")
        self.assertTrue(should_close(27))
        self.assertTrue(should_close(ord("q")))
        self.assertFalse(should_close(ord("a")))

    def test_competition_output_has_a_nonempty_waiting_state(self):
        self.assertEqual(normalize_output_text(""), "等待比赛输出")
        self.assertEqual(normalize_output_text("  偶数  "), "偶数")

    def test_competition_remote_start_uses_fixed_argv(self):
        self.assertEqual(competition_start_argv(None), ())
        self.assertEqual(competition_start_argv("  "), ())
        self.assertEqual(
            competition_start_argv(" /opt/smartcar/competition_mode.sh "),
            ("bash", "/opt/smartcar/competition_mode.sh", "start", "--confirm"),
        )

    @unittest.skipUnless(
        CompetitionOutputWindow is not None,
        "PyQt5 is not installed on this test host",
    )
    def test_competition_start_button_requires_explicit_authorization(self):
        app = QApplication.instance() or QApplication(["competition-ui-test"])
        bridge = CompetitionOutputBridge()
        disabled_window = CompetitionOutputWindow(
            bridge,
            "比赛输出",
            "等待比赛输出",
            "等待二维码",
            "等待二维码后选择",
            "等待诊疗区描述",
            "急停锁存",
            False,
            "/opt/smartcar/competition_mode.sh",
        )
        self.assertFalse(disabled_window.start_button.isEnabled())
        self.assertEqual(disabled_window._start_command, ())
        self.assertEqual(
            disabled_window.c_zone_direction_label.text(),
            "等待二维码后选择",
        )

        enabled_window = CompetitionOutputWindow(
            bridge,
            "比赛输出",
            "等待比赛输出",
            "等待二维码",
            "等待二维码后选择",
            "等待诊疗区描述",
            "急停锁存",
            True,
            "/opt/smartcar/competition_mode.sh",
        )
        self.assertTrue(enabled_window.start_button.isEnabled())
        self.assertEqual(
            enabled_window._start_command,
            ("bash", "/opt/smartcar/competition_mode.sh", "start", "--confirm"),
        )
        enabled_window.set_qr("0024")
        self.assertEqual(enabled_window.qr_label.text(), "0024")
        enabled_window.set_c_zone_direction("顺时针")
        self.assertEqual(enabled_window.c_zone_direction_label.text(), "顺时针")
        enabled_window.close()
        disabled_window.close()
        app.processEvents()

    @unittest.skipUnless(
        CompetitionOutputWindow is not None,
        "PyQt5 is not installed on this test host",
    )
    def test_competition_layout_fits_the_1280x720_hdmi_panel(self):
        from PyQt5.QtCore import QPoint

        app = QApplication.instance() or QApplication(["competition-ui-hdmi-test"])
        window = CompetitionOutputWindow(
            CompetitionOutputBridge(),
            "比赛输出",
            "等待比赛输出",
            "等待二维码",
            "等待二维码后选择",
            "等待诊疗区描述",
            "急停锁存",
            True,
            "/opt/smartcar/competition_mode.sh",
        )
        try:
            window.resize(1280, 720)
            window.show()
            app.processEvents()
            root = window.centralWidget()
            self.assertEqual((root.width(), root.height()), (1280, 720))
            for widget in (
                window.start_button,
                window.start_status_label,
                window.qr_label,
                window.c_zone_direction_label,
                window.vlm_output,
                window.output_label,
            ):
                position = widget.mapTo(root, QPoint(0, 0))
                self.assertGreaterEqual(position.x(), 0)
                self.assertGreaterEqual(position.y(), 0)
                self.assertLessEqual(position.x() + widget.width(), root.width())
                self.assertLessEqual(position.y() + widget.height(), root.height())
        finally:
            window.close()
            app.processEvents()


class DisplayStateTests(unittest.TestCase):
    def test_success_and_failure_are_distinct(self):
        success = DisplayResult(True, "描述", "ok", 1.0)
        failure = DisplayResult(False, "", "image_timeout", 8.0)
        self.assertEqual(result_kind(success), "success")
        self.assertEqual(result_kind(failure), "failed")
        self.assertEqual(result_status_text(success), "生成完成")
        self.assertEqual(result_status_text(failure), "生成失败")

    @unittest.skipUnless(
        QApplication is not None,
        "PyQt5 is not installed on this test host",
    )
    def test_offscreen_window_accepts_long_text(self):
        app = QApplication.instance() or QApplication(["vlm-display-test"])
        bridge = UiBridge()
        window = VlmWindow(bridge, lambda: None)
        long_text = "人物穿着白色上衣，正在挥手。" * 80
        window.set_external_text(long_text)
        app.processEvents()
        self.assertEqual(window.result_text.toPlainText(), long_text)
        window.finish_request(DisplayResult(
            True, "人物正在挥手。", "ok", 8.0))
        self.assertEqual(window.status_label.text(), "生成完成 · 8.0 s")
        self.assertTrue(window.trigger_button.isEnabled())
        window.close()


if __name__ == "__main__":
    unittest.main()
