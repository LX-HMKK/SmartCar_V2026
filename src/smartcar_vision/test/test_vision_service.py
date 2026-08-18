"""Tests for the bounded QR and in-memory image-to-text service core."""
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_vision.service_core import VisionServiceCore  # noqa: E402


class FakeClock:
    def __init__(self):
        self.now_ns = 0

    def __call__(self):
        return self.now_ns

    def advance(self, seconds):
        self.now_ns += int(seconds * 1_000_000_000)


class FakeBuffer:
    def __init__(self, value, clock=None, advance_sec=0.0):
        self.value = value
        self.clock = clock
        self.advance_sec = advance_sec
        self.calls = []

    def wait_for(self, not_before_ns, timeout_sec):
        self.calls.append((not_before_ns, timeout_sec))
        if not_before_ns < 0:
            raise ValueError("not_before_ns must be nonnegative")
        if self.clock is not None:
            self.clock.advance(self.advance_sec)
        return self.value


class VisionServiceCoreTests(unittest.TestCase):
    def make_core(
        self,
        image=b"image",
        description="description",
        clock=None,
        buffer_advance=0.0,
        encoder_advance=0.0,
        backend_advance=0.0,
        max_timeout=30.0,
    ):
        clock = clock or FakeClock()
        barcode_buffer = FakeBuffer(None)
        image_buffer = FakeBuffer(image, clock, buffer_advance)
        calls = []

        def encode(value):
            self.assertEqual(value, image)
            clock.advance(encoder_advance)
            return b"jpeg-bytes"

        def describe(jpeg, timeout):
            calls.append((jpeg, timeout))
            clock.advance(backend_advance)
            if isinstance(description, Exception):
                raise description
            return description

        return (
            VisionServiceCore(
                barcode_buffer=barcode_buffer,
                image_buffer=image_buffer,
                describe_jpeg=describe,
                jpeg_encoder=encode,
                max_vlm_timeout_sec=max_timeout,
                monotonic_ns=clock,
            ),
            barcode_buffer,
            image_buffer,
            calls,
        )

    def test_qr_success_timeout_and_empty_content(self):
        for value, expected in (
            ("WARD-A", (True, "WARD-A", "ok")),
            (None, (False, "", "qr_timeout")),
            ("", (False, "", "qr_empty")),
        ):
            with self.subTest(value=value):
                core, barcode, _, _ = self.make_core()
                barcode.value = value
                self.assertEqual(core.read_qr(123, 1.0), expected)

    def test_scene_success_uses_in_memory_jpeg(self):
        core, _, image, calls = self.make_core()
        outcome = core.describe_scene(100, 30.0)
        self.assertEqual(outcome, (True, "description", "ok"))
        self.assertEqual(image.calls[0], (100, 30.0))
        self.assertEqual(calls, [(b"jpeg-bytes", 30.0)])

    def test_no_fresh_image_is_a_failure(self):
        core, _, _, calls = self.make_core(image=None)
        self.assertEqual(
            core.describe_scene(100, 30.0),
            (False, "", "image_timeout"),
        )
        self.assertEqual(calls, [])

    def test_encoder_and_backend_fail_without_inventing_text(self):
        core, _, _, _ = self.make_core(description=RuntimeError("http_timeout"))
        self.assertEqual(
            core.describe_scene(100, 30.0),
            (False, "", "http_timeout"),
        )

        failing = VisionServiceCore(
            barcode_buffer=FakeBuffer(None),
            image_buffer=FakeBuffer(b"image"),
            describe_jpeg=lambda *_args: self.fail("must not describe"),
            jpeg_encoder=lambda _image: (_ for _ in ()).throw(RuntimeError()),
        )
        self.assertEqual(
            failing.describe_scene(0, 1.0),
            (False, "", "jpeg_error:RuntimeError"),
        )

    def test_single_deadline_covers_wait_encode_and_request(self):
        clock = FakeClock()
        core, _, image, calls = self.make_core(
            clock=clock,
            buffer_advance=2.0,
            encoder_advance=3.0,
        )
        self.assertTrue(core.describe_scene(500, 30.0).success)
        self.assertAlmostEqual(image.calls[0][1], 30.0, places=6)
        self.assertAlmostEqual(calls[0][1], 25.0, places=6)

    def test_hard_thirty_second_limit_and_expiry(self):
        clock = FakeClock()
        capped, _, image, calls = self.make_core(
            clock=clock, max_timeout=40.0)
        self.assertTrue(capped.describe_scene(0, 40.0).success)
        self.assertAlmostEqual(image.calls[0][1], 30.0, places=6)
        self.assertAlmostEqual(calls[0][1], 30.0, places=6)

        expired, _, _, calls = self.make_core(
            clock=FakeClock(), encoder_advance=30.1)
        self.assertEqual(
            expired.describe_scene(0, 30.0),
            (False, "", "vlm_timeout"),
        )
        self.assertEqual(calls, [])

    def test_invalid_requests_fail_without_work(self):
        core, _, image, calls = self.make_core()
        self.assertEqual(
            core.describe_scene(0, 0.0),
            (False, "", "invalid_timeout"),
        )
        self.assertEqual(image.calls, [])
        self.assertEqual(calls, [])
        self.assertEqual(core.read_qr(-1, 1.0), (False, "", "invalid_request"))


if __name__ == "__main__":
    unittest.main()
