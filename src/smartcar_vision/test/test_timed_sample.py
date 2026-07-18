"""Tests for receipt-time sample buffering without ROS dependencies."""
import math
from pathlib import Path
import sys
import threading
import time
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_vision.timed_sample import TimedSampleBuffer  # noqa: E402


class TimedSampleBufferTests(unittest.TestCase):
    def test_returns_only_samples_at_or_after_requested_time(self):
        buffer = TimedSampleBuffer()
        buffer.put("stale", 99)
        self.assertIsNone(buffer.wait_for(100, 0.0))

        buffer.put("equal", 100)
        self.assertEqual(buffer.wait_for(100, 0.0), "equal")

        buffer.put("fresh", 101)
        self.assertEqual(buffer.wait_for(100, 0.0), "fresh")

    def test_older_out_of_order_samples_do_not_replace_latest(self):
        buffer = TimedSampleBuffer()
        buffer.put("latest", 200)
        buffer.put("older", 199)
        self.assertEqual(buffer.wait_for(0, 0.0), "latest")

    def test_equal_timestamp_may_replace_value(self):
        buffer = TimedSampleBuffer()
        buffer.put("first", 200)
        buffer.put("second", 200)
        self.assertEqual(buffer.wait_for(200, 0.0), "second")

    def test_waiter_is_woken_by_fresh_sample(self):
        buffer = TimedSampleBuffer()
        result = []

        thread = threading.Thread(
            target=lambda: result.append(buffer.wait_for(500, 1.0)),
            daemon=True,
        )
        thread.start()
        time.sleep(0.02)
        buffer.put("ready", 500)
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(result, ["ready"])

    def test_timeout_uses_monotonic_deadline(self):
        buffer = TimedSampleBuffer()
        started = time.monotonic()
        self.assertIsNone(buffer.wait_for(1, 0.03))
        elapsed = time.monotonic() - started
        self.assertGreaterEqual(elapsed, 0.02)
        self.assertLess(elapsed, 0.5)

    def test_invalid_timestamps_and_timeouts_are_rejected(self):
        buffer = TimedSampleBuffer()
        for timestamp in (-1, 1.5, True):
            with self.subTest(timestamp=timestamp):
                with self.assertRaises((TypeError, ValueError)):
                    buffer.put("value", timestamp)
        for timeout in (-0.1, math.nan, math.inf, -math.inf):
            with self.subTest(timeout=timeout):
                with self.assertRaises(ValueError):
                    buffer.wait_for(0, timeout)


if __name__ == "__main__":
    unittest.main()
