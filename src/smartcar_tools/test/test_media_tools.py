"""ROS-independent tests for media probes, replay, and display state."""
import json
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
    normalize_output_text,
)
from smartcar_tools.qr_probe import (  # noqa: E402
    INVALID_ARGUMENT,
    QR_NOT_FOUND,
    SUCCESS,
    competition_output_text,
    response_exit_code,
)
from smartcar_tools.speech_probe import (  # noqa: E402
    SpeechStatusTracker,
    parse_status,
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


class SpeechProbeTests(unittest.TestCase):
    def test_parser_rejects_non_request_status(self):
        self.assertIsNone(parse_status("not json"))
        self.assertIsNone(parse_status(json.dumps({
            "state": "ready",
            "request_id": "",
            "detail": "",
        })))

    def test_tracker_stays_on_one_request_until_terminal(self):
        tracker = SpeechStatusTracker()
        queued = json.dumps({
            "state": "queued",
            "request_id": "request-a",
            "detail": "",
        })
        self.assertIsNone(tracker.consume(queued))
        tracker.arm()
        self.assertIsNone(tracker.consume(json.dumps({
            "state": "completed",
            "request_id": "stale-request",
            "detail": "",
        })))
        self.assertEqual(tracker.consume(queued)["state"], "queued")
        self.assertIsNone(tracker.consume(json.dumps({
            "state": "synthesizing",
            "request_id": "request-b",
            "detail": "",
        })))
        for state in ("synthesizing", "playing", "completed"):
            tracker.consume(json.dumps({
                "state": state,
                "request_id": "request-a",
                "detail": "",
            }))
        self.assertEqual(
            tracker.history,
            ["queued", "synthesizing", "playing", "completed"],
        )
        self.assertEqual(tracker.terminal["state"], "completed")


class QrProbeTests(unittest.TestCase):
    def test_exit_code_is_explicit(self):
        self.assertEqual(response_exit_code(True, "ok"), SUCCESS)
        self.assertEqual(
            response_exit_code(False, "qr_timeout"), QR_NOT_FOUND)
        self.assertEqual(response_exit_code(True, "unexpected"), QR_NOT_FOUND)
        self.assertNotEqual(INVALID_ARGUMENT, QR_NOT_FOUND)

    def test_competition_output_is_limited_to_odd_or_even(self):
        self.assertEqual(
            competition_output_text(True, "第3题：奇数", "ok"),
            "奇数",
        )
        self.assertEqual(
            competition_output_text(True, "偶数", "ok"),
            "偶数",
        )
        self.assertEqual(
            competition_output_text(True, "9697", "ok"),
            "奇数",
        )
        self.assertEqual(
            competition_output_text(True, "640", "ok"),
            "偶数",
        )
        self.assertEqual(
            competition_output_text(True, "第3题", "ok"),
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


class DisplayStateTests(unittest.TestCase):
    def test_success_fallback_and_failure_are_distinct(self):
        success = DisplayResult(True, False, "描述", "ok", 1.0)
        fallback = DisplayResult(True, True, "兜底", "vlm_timeout", 8.0)
        failure = DisplayResult(False, False, "", "image_timeout", 8.0)
        self.assertEqual(result_kind(success), "success")
        self.assertEqual(result_kind(fallback), "fallback")
        self.assertEqual(result_kind(failure), "failed")
        self.assertEqual(result_status_text(success), "生成完成")
        self.assertEqual(result_status_text(fallback), "生成完成")
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
            True, True, "画面中可能是一名穿着浅色上衣的人物，正面站立并挥手。", "vlm_timeout", 8.0))
        self.assertEqual(window.status_label.text(), "生成完成 · 8.0 s")
        self.assertTrue(window.trigger_button.isEnabled())
        window.close()


if __name__ == "__main__":
    unittest.main()
