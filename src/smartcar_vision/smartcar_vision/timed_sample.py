"""Thread-safe receipt-time sample buffering."""
from dataclasses import dataclass
import math
import operator
import threading
import time
from typing import Generic, Optional, TypeVar


T = TypeVar("T")


def _nonnegative_nanoseconds(name, value):
    if isinstance(value, bool):
        raise TypeError(f"{name} must be an integer nanosecond timestamp")
    try:
        result = operator.index(value)
    except TypeError as error:
        raise TypeError(
            f"{name} must be an integer nanosecond timestamp") from error
    if result < 0:
        raise ValueError(f"{name} must be nonnegative")
    return result


def _nonnegative_timeout(value):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("timeout_sec must be finite and nonnegative") from error
    if not math.isfinite(result) or result < 0.0:
        raise ValueError("timeout_sec must be finite and nonnegative")
    return result


@dataclass(frozen=True)
class TimedSample(Generic[T]):
    received_ns: int
    value: T


class TimedSampleBuffer(Generic[T]):
    """Keep the newest sample and wait against a monotonic deadline."""

    def __init__(self, monotonic_ns=time.monotonic_ns):
        self._monotonic_ns = monotonic_ns
        self._condition = threading.Condition()
        self._latest: Optional[TimedSample[T]] = None

    def put(self, value: T, received_ns: int) -> None:
        timestamp = _nonnegative_nanoseconds("received_ns", received_ns)
        with self._condition:
            if self._latest is not None and timestamp < self._latest.received_ns:
                return
            self._latest = TimedSample(timestamp, value)
            self._condition.notify_all()

    def wait_for(self, not_before_ns: int, timeout_sec: float) -> Optional[T]:
        threshold = _nonnegative_nanoseconds("not_before_ns", not_before_ns)
        timeout = _nonnegative_timeout(timeout_sec)
        deadline_ns = self._monotonic_ns() + int(timeout * 1_000_000_000)

        with self._condition:
            while True:
                if (
                    self._latest is not None
                    and self._latest.received_ns >= threshold
                ):
                    return self._latest.value

                remaining_ns = deadline_ns - self._monotonic_ns()
                if remaining_ns <= 0:
                    return None
                self._condition.wait(remaining_ns / 1_000_000_000.0)
