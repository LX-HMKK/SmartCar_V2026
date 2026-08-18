"""ROS-independent QR and image-to-text service orchestration."""
import math
import time
from typing import NamedTuple


HARD_MAX_VLM_TIMEOUT_SEC = 30.0


class ReadQrOutcome(NamedTuple):
    success: bool
    content: str
    status: str


class DescribeOutcome(NamedTuple):
    success: bool
    description: str
    status: str


class _Deadline:
    def __init__(self, timeout_sec, maximum_sec, monotonic_ns):
        timeout = float(timeout_sec)
        maximum = float(maximum_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout must be finite and greater than zero")
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("maximum timeout must be finite and greater than zero")
        self._monotonic_ns = monotonic_ns
        self._deadline_ns = monotonic_ns() + int(
            min(timeout, maximum) * 1_000_000_000)

    def remaining_sec(self):
        return max(0.0, (self._deadline_ns - self._monotonic_ns()) / 1e9)


class VisionServiceCore:
    def __init__(
        self,
        barcode_buffer,
        image_buffer,
        describe_jpeg,
        jpeg_encoder,
        max_vlm_timeout_sec=30.0,
        monotonic_ns=time.monotonic_ns,
    ):
        maximum = float(max_vlm_timeout_sec)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("max_vlm_timeout_sec must be finite and positive")
        self._barcode_buffer = barcode_buffer
        self._image_buffer = image_buffer
        self._describe_jpeg = describe_jpeg
        self._jpeg_encoder = jpeg_encoder
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

    def describe_scene(self, not_before_ns, timeout_sec):
        try:
            deadline = _Deadline(
                timeout_sec, self._max_vlm_timeout_sec, self._monotonic_ns)
        except (TypeError, ValueError, OverflowError):
            return DescribeOutcome(False, "", "invalid_timeout")
        try:
            image = self._image_buffer.wait_for(
                not_before_ns, deadline.remaining_sec())
        except (TypeError, ValueError, OverflowError):
            return DescribeOutcome(False, "", "invalid_request")
        if image is None:
            return DescribeOutcome(False, "", "image_timeout")

        try:
            jpeg = self._jpeg_encoder(image)
        except Exception as error:
            return self._failure(f"jpeg_error:{type(error).__name__}")
        if not isinstance(jpeg, (bytes, bytearray)) or not jpeg:
            return self._failure("jpeg_empty")
        remaining = deadline.remaining_sec()
        if remaining <= 0.0:
            return self._failure("vlm_timeout")

        try:
            description = self._describe_jpeg(bytes(jpeg), remaining)
        except Exception as error:
            status = str(error).strip() or f"backend_error:{type(error).__name__}"
            return self._failure(status)
        if deadline.remaining_sec() <= 0.0:
            return self._failure("vlm_timeout")
        description = str(description).strip()
        if not description:
            return self._failure("backend_empty_output")
        return DescribeOutcome(True, description, "ok")

    @staticmethod
    def _failure(status):
        return DescribeOutcome(False, "", str(status))
