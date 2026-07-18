"""Tests for bounded QR and scene-description service orchestration."""
import os
from pathlib import Path
import stat
import sys
import tempfile
import threading
import time
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_vision.service_core import VisionServiceCore  # noqa: E402
from smartcar_vision.vlm_backend import VlmResult  # noqa: E402


FALLBACK_TEXT = "检测到人物立牌"


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


class RecordingBackend:
    def __init__(self, result, clock=None, advance_sec=0.0):
        self.result = result
        self.clock = clock
        self.advance_sec = advance_sec
        self.calls = []
        self.image_path = None
        self.image_mode = None

    def describe(self, image_path, prompt, timeout_sec):
        self.calls.append((image_path, prompt, timeout_sec))
        self.image_path = Path(image_path)
        self.image_mode = stat.S_IMODE(self.image_path.stat().st_mode)
        self.asserted_bytes = self.image_path.read_bytes()
        if self.clock is not None:
            self.clock.advance(self.advance_sec)
        return self.result


class VisionServiceCoreTests(unittest.TestCase):
    def make_core(
        self,
        directory,
        barcode=None,
        image=b"image",
        backend_result=None,
        clock=None,
        buffer_advance=0.0,
        writer_advance=0.0,
        max_vlm_timeout_sec=8.0,
    ):
        clock = clock or FakeClock()
        backend_result = backend_result or VlmResult(True, "description", "ok")
        barcode_buffer = FakeBuffer(barcode)
        image_buffer = FakeBuffer(image, clock, buffer_advance)
        backend = RecordingBackend(backend_result, clock)

        def jpeg_writer(value, file_object):
            self.assertEqual(value, image)
            file_object.write(b"jpeg-bytes")
            clock.advance(writer_advance)

        core = VisionServiceCore(
            barcode_buffer=barcode_buffer,
            image_buffer=image_buffer,
            backend=backend,
            jpeg_writer=jpeg_writer,
            runtime_dir=directory,
            max_vlm_timeout_sec=max_vlm_timeout_sec,
            monotonic_ns=clock,
        )
        return core, barcode_buffer, image_buffer, backend, clock

    def test_qr_success_timeout_and_empty_content(self):
        with tempfile.TemporaryDirectory() as directory:
            success, _, _, _, _ = self.make_core(directory, barcode="WARD-A")
            timeout, _, _, _, _ = self.make_core(directory, barcode=None)
            empty, _, _, _, _ = self.make_core(directory, barcode="")

            self.assertEqual(
                success.read_qr(123, 1.0),
                (True, "WARD-A", "ok"),
            )
            self.assertEqual(
                timeout.read_qr(123, 1.0),
                (False, "", "qr_timeout"),
            )
            self.assertEqual(
                empty.read_qr(123, 1.0),
                (False, "", "qr_empty"),
            )

    def test_no_fresh_image_is_a_service_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            core, _, _, backend, _ = self.make_core(directory, image=None)
            outcome = core.describe_scene(100, 8.0, "describe")

            self.assertEqual(outcome, (False, False, "", "image_timeout"))
            self.assertEqual(backend.calls, [])

    def test_backend_success_uses_closed_jpeg_and_always_removes_it(self):
        with tempfile.TemporaryDirectory() as directory:
            core, _, _, backend, _ = self.make_core(directory)
            outcome = core.describe_scene(100, 8.0, "describe")

            self.assertEqual(outcome, (True, False, "description", "ok"))
            self.assertEqual(backend.asserted_bytes, b"jpeg-bytes")
            self.assertFalse(backend.image_path.exists())
            if os.name == "posix":
                self.assertEqual(backend.image_mode, 0o600)

    def test_backend_error_returns_required_fallback_and_removes_file(self):
        with tempfile.TemporaryDirectory() as directory:
            core, _, _, backend, _ = self.make_core(
                directory,
                backend_result=VlmResult(False, "", "backend_disabled"),
            )
            outcome = core.describe_scene(100, 8.0, "describe")

            self.assertEqual(
                outcome,
                (True, True, FALLBACK_TEXT, "backend_disabled"),
            )
            self.assertFalse(backend.image_path.exists())

    def test_one_deadline_is_shared_across_wait_encode_and_backend(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeClock()
            core, _, image_buffer, backend, _ = self.make_core(
                directory,
                clock=clock,
                buffer_advance=2.0,
                writer_advance=3.0,
            )
            outcome = core.describe_scene(500, 8.0, "describe")

            self.assertTrue(outcome.success)
            self.assertAlmostEqual(image_buffer.calls[0][1], 8.0, places=6)
            self.assertAlmostEqual(backend.calls[0][2], 3.0, places=6)

    def test_deadline_expiry_after_encoding_uses_fallback_without_backend(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeClock()
            core, _, _, backend, _ = self.make_core(
                directory,
                clock=clock,
                buffer_advance=2.0,
                writer_advance=6.1,
            )
            outcome = core.describe_scene(500, 20.0, "describe")

            self.assertEqual(
                outcome,
                (True, True, FALLBACK_TEXT, "vlm_timeout"),
            )
            self.assertEqual(backend.calls, [])

    def test_configuration_cannot_raise_hard_eight_second_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            core, _, image_buffer, backend, _ = self.make_core(
                directory,
                max_vlm_timeout_sec=20.0,
            )
            outcome = core.describe_scene(500, 20.0, "describe")

            self.assertTrue(outcome.success)
            self.assertAlmostEqual(image_buffer.calls[0][1], 8.0, places=6)
            self.assertAlmostEqual(backend.calls[0][2], 8.0, places=6)

    def test_jpeg_error_returns_fallback_and_removes_partial_file(self):
        with tempfile.TemporaryDirectory() as directory:
            image_path = []

            def failing_writer(_image, file_object):
                image_path.append(Path(file_object.name))
                file_object.write(b"partial")
                raise RuntimeError("encode failed")

            core = VisionServiceCore(
                barcode_buffer=FakeBuffer(None),
                image_buffer=FakeBuffer(b"image"),
                backend=RecordingBackend(VlmResult(True, "unused", "ok")),
                jpeg_writer=failing_writer,
                runtime_dir=directory,
            )
            outcome = core.describe_scene(0, 1.0, "describe")

            self.assertEqual(
                outcome,
                (True, True, FALLBACK_TEXT, "jpeg_error:RuntimeError"),
            )
            self.assertEqual(len(image_path), 1)
            self.assertFalse(image_path[0].exists())

    def test_blocking_jpeg_encoder_is_bounded_by_the_request_deadline(self):
        started = threading.Event()
        release = threading.Event()

        def blocking_writer(_image, _file_object):
            started.set()
            release.wait(0.20)

        with tempfile.TemporaryDirectory() as directory:
            core = VisionServiceCore(
                barcode_buffer=FakeBuffer(None),
                image_buffer=FakeBuffer(b"image"),
                backend=RecordingBackend(VlmResult(True, "unused", "ok")),
                jpeg_writer=blocking_writer,
                runtime_dir=directory,
            )
            started_at = time.monotonic()
            outcome = core.describe_scene(0, 0.03, "describe")
            elapsed = time.monotonic() - started_at
            release.set()

            self.assertEqual(
                outcome,
                (True, True, FALLBACK_TEXT, "vlm_timeout"),
            )
            self.assertTrue(elapsed < 0.18)

    def test_invalid_timeout_fails_without_waiting(self):
        with tempfile.TemporaryDirectory() as directory:
            core, _, image_buffer, backend, _ = self.make_core(directory)
            outcome = core.describe_scene(0, 0.0, "describe")

            self.assertFalse(outcome.success)
            self.assertEqual(outcome.status, "invalid_timeout")
            self.assertEqual(image_buffer.calls, [])
            self.assertEqual(backend.calls, [])

    def test_invalid_not_before_is_classified_as_invalid_request(self):
        with tempfile.TemporaryDirectory() as directory:
            core, _, _, _, _ = self.make_core(directory)
            self.assertEqual(
                core.read_qr(-1, 1.0),
                (False, "", "invalid_request"),
            )


if __name__ == "__main__":
    unittest.main()
