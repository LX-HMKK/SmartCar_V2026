"""Shared finite/positive/nonnegative scalar validators.

These exist as one copy because the same checks are repeated across several
tool modules; callers inject their own error type so each module keeps its
distinct exception semantics.
"""
from __future__ import annotations

import math
from typing import Any


def finite_number(value: Any, label: str, error: type[Exception]) -> float:
    if isinstance(value, bool):
        raise error(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise error(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise error(f"{label} must be a finite number")
    return result


def positive_number(value: Any, label: str, error: type[Exception]) -> float:
    result = finite_number(value, label, error)
    if result <= 0.0:
        raise error(f"{label} must be positive")
    return result


def nonnegative_number(
    value: Any, label: str, error: type[Exception]
) -> float:
    result = finite_number(value, label, error)
    if result < 0.0:
        raise error(f"{label} must be nonnegative")
    return result
