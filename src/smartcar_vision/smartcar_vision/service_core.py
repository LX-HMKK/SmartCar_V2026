"""ROS-independent orchestration for QR and scene-description services."""
import math
import os
from pathlib import Path
import tempfile
import time
from typing import NamedTuple


HARD_MAX_VLM_TIMEOUT_SEC = 8.0
FALLBACK_TEXT = "检测到人物立牌"


class ReadQrOutcome(NamedTuple):
    success: bool
    content: str
    status: str


class DescribeOutcome(NamedTuple):
    success: bool
    fallback_used: bool
    description: str
    status: str


class _Deadline:
    def __init__(self, timeout_sec, maximum_sec, monotonic_ns):
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout must be finite and greater than zero")
        maximum = float(maximum_sec)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("maximum timeout must be finite and greater than zero")
        self._monotonic_ns = monotonic_ns
        bounded_timeout = min(timeout, maximum)
        self._deadline_ns = (
            self._monotonic_ns() + int(bounded_timeout * 1_000_000_000)
        )

    def remaining_sec(self):
        remaining_ns = self._deadline_ns - self._monotonic_ns()
        return max(0.0, remaining_ns / 1_000_000_000.0)

    def expired(self):
        return self.remaining_sec() <= 0.0


class VisionServiceCore:
    def __init__(
        self,
        barcode_buffer,
        image_buffer,
        backend,
        jpeg_writer,
        runtime_dir,
        max_vlm_timeout_sec=8.0,
        monotonic_ns=time.monotonic_ns,
    ):
        maximum = float(max_vlm_timeout_sec)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("max_vlm_timeout_sec must be finite and positive")
        self._barcode_buffer = barcode_buffer
        self._image_buffer = image_buffer
        self._backend = backend
        self._jpeg_writer = jpeg_writer
        self._runtime_dir = Path(runtime_dir)
        self._runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        self._max_vlm_timeout_sec = min(maximum, HARD_MAX_VLM_TIMEOUT_SEC)
        self._monotonic_ns = monotonic_ns

    def read_qr(self, not_before_ns, timeout_sec):
        try:
            timeout = float(timeout_sec)
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError
        except (TypeError, ValueError, OverflowError):
            return ReadQrOutcome(False, "", "invalid_timeout")
        try:
            content = self._barcode_buffer.wait_for(not_before_ns, timeout)
        except (TypeError, ValueError, OverflowError):
            return ReadQrOutcome(False, "", "invalid_request")

        if content is None:
            return ReadQrOutcome(False, "", "qr_timeout")
        content = str(content)
        if not content:
            return ReadQrOutcome(False, "", "qr_empty")
        return ReadQrOutcome(True, content, "ok")

    def describe_scene(self, not_before_ns, timeout_sec, prompt):
        try:
            deadline = _Deadline(
                timeout_sec,
                self._max_vlm_timeout_sec,
                self._monotonic_ns,
            )
        except (TypeError, ValueError, OverflowError):
            return DescribeOutcome(False, False, "", "invalid_timeout")

        try:
            image = self._image_buffer.wait_for(
                not_before_ns, deadline.remaining_sec())
        except (TypeError, ValueError, OverflowError):
            return DescribeOutcome(False, False, "", "invalid_request")
        if image is None:
            return DescribeOutcome(False, False, "", "image_timeout")
        if deadline.expired():
            return self._fallback("vlm_timeout")

        image_path = None
        try:
            try:
                with tempfile.NamedTemporaryFile(
                    dir=self._runtime_dir,
                    prefix="scene-",
                    suffix=".jpg",
                    delete=False,
                ) as temporary_file:
                    image_path = Path(temporary_file.name)
                    if hasattr(os, "fchmod"):
                        os.fchmod(temporary_file.fileno(), 0o600)
                    self._jpeg_writer(image, temporary_file)
            except Exception as error:
                return self._fallback(f"jpeg_error:{type(error).__name__}")

            if deadline.expired():
                return self._fallback("vlm_timeout")

            try:
                result = self._backend.describe(
                    str(image_path),
                    str(prompt),
                    deadline.remaining_sec(),
                )
            except Exception as error:
                return self._fallback(f"backend_error:{type(error).__name__}")

            if deadline.expired():
                return self._fallback("vlm_timeout")
            if not result.ok:
                return self._fallback(result.status)
            description = str(result.text).strip()
            if not description:
                return self._fallback("backend_empty_output")
            return DescribeOutcome(True, False, description, result.status)
        finally:
            if image_path is not None:
                try:
                    image_path.unlink()
                except FileNotFoundError:
                    pass

    def _fallback(self, status):
        return DescribeOutcome(True, True, FALLBACK_TEXT, str(status))
