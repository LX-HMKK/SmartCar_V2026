"""Pure competition-result formatting tests."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.competition import (  # noqa: E402
    C_ZONE_DIRECTION_CLOCKWISE_TEXT,
    C_ZONE_DIRECTION_COUNTERCLOCKWISE_TEXT,
    QR_PARITY_EVEN,
    QR_PARITY_ODD,
    QR_PARITY_UNRECOGNIZED,
    c_zone_direction_for_qr,
    c_zone_direction_text,
    classify_qr_parity,
)
from smartcar_task.c_zone_direction import CLOCKWISE, COUNTERCLOCKWISE  # noqa: E402


class CompetitionQrTests(unittest.TestCase):
    def test_classifies_numeric_qr_payloads(self):
        self.assertEqual(classify_qr_parity("13"), QR_PARITY_ODD)
        self.assertEqual(classify_qr_parity("24"), QR_PARITY_EVEN)
        self.assertEqual(classify_qr_parity(" 7 "), QR_PARITY_ODD)

    def test_classifies_explicit_chinese_parity(self):
        self.assertEqual(classify_qr_parity("奇数任务"), QR_PARITY_ODD)
        self.assertEqual(classify_qr_parity("偶数任务"), QR_PARITY_EVEN)

    def test_ambiguous_or_non_numeric_qr_payloads_are_unrecognized(self):
        for payload in ("", "WARD-A", "奇数或偶数", None):
            with self.subTest(payload=payload):
                self.assertEqual(
                    classify_qr_parity(payload),
                    QR_PARITY_UNRECOGNIZED,
                )

    def test_qr_parity_selects_the_authorized_c_zone_variant(self):
        self.assertEqual(c_zone_direction_for_qr("13"), CLOCKWISE)
        self.assertEqual(c_zone_direction_for_qr("24"), COUNTERCLOCKWISE)
        self.assertEqual(c_zone_direction_for_qr("奇数任务"), CLOCKWISE)
        self.assertEqual(c_zone_direction_for_qr("偶数任务"), COUNTERCLOCKWISE)
        self.assertEqual(c_zone_direction_for_qr("奇数或偶数"), COUNTERCLOCKWISE)
        self.assertEqual(c_zone_direction_for_qr("WARD-A"), COUNTERCLOCKWISE)

    def test_c_zone_direction_display_text_is_explicit(self):
        self.assertEqual(
            c_zone_direction_text(COUNTERCLOCKWISE),
            C_ZONE_DIRECTION_COUNTERCLOCKWISE_TEXT,
        )
        self.assertEqual(
            c_zone_direction_text(CLOCKWISE),
            C_ZONE_DIRECTION_CLOCKWISE_TEXT,
        )
        with self.assertRaisesRegex(ValueError, "unknown C-zone direction"):
            c_zone_direction_text("left")


if __name__ == "__main__":
    unittest.main()
