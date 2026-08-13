#!/usr/bin/env python3
"""Analyze a recorded constant-steering circle without ROS dependencies.

The input is the CSV written by the steering-circle drive tool.  Complete rows
must provide: t, vx, gyro_wz, odom_wz, x, y, and yaw.  The report retains the
radius and effective-steering values needed for manual calibration while
rejecting incomplete, stationary, and geometrically degenerate recordings.
"""

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = ("t", "vx", "gyro_wz", "odom_wz", "x", "y", "yaw")
_MIN_SPEED_MPS = 1.0e-4
_MIN_ANGULAR_RATE_RADPS = 1.0e-4
_PIVOT_EPSILON = 1.0e-12


@dataclass(frozen=True)
class CircleSample:
    """One complete odometry and gyro sample from a circle drive."""

    time_sec: float
    vx_mps: float
    gyro_wz_radps: float
    odom_wz_radps: float
    x_m: float
    y_m: float
    yaw_rad: float
    ekf_x_m: Optional[float] = None
    ekf_y_m: Optional[float] = None
    ekf_yaw_rad: Optional[float] = None


@dataclass(frozen=True)
class CircleAnalysis:
    """Structured calibration result, including non-fatal data warnings."""

    sample_count: int
    skipped_rows: int
    start_time_sec: Optional[float]
    end_time_sec: Optional[float]
    steady_sample_count: int
    mean_vx_mps: Optional[float]
    mean_gyro_wz_radps: Optional[float]
    gyro_wz_stddev_radps: Optional[float]
    mean_odom_wz_radps: Optional[float]
    gyro_radius_m: Optional[float]
    gyro_effective_steering_rad: Optional[float]
    odom_radius_m: Optional[float]
    odom_effective_steering_rad: Optional[float]
    circle_center_m: Optional[Tuple[float, float]]
    circle_radius_m: Optional[float]
    circle_fit_rms_error_m: Optional[float]
    yaw_delta_rad: Optional[float]
    gyro_yaw_delta_rad: Optional[float]
    ekf_yaw_delta_rad: Optional[float]
    gyro_vs_odom_yaw_delta_rad: Optional[float]
    ekf_vs_odom_yaw_delta_rad: Optional[float]
    vx_sign_flips: int
    gyro_sign_flips: int
    errors: Tuple[str, ...]
    warnings: Tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors


def _empty_analysis(
    errors: Sequence[str], skipped_rows: int = 0
) -> CircleAnalysis:
    return CircleAnalysis(
        sample_count=0,
        skipped_rows=skipped_rows,
        start_time_sec=None,
        end_time_sec=None,
        steady_sample_count=0,
        mean_vx_mps=None,
        mean_gyro_wz_radps=None,
        gyro_wz_stddev_radps=None,
        mean_odom_wz_radps=None,
        gyro_radius_m=None,
        gyro_effective_steering_rad=None,
        odom_radius_m=None,
        odom_effective_steering_rad=None,
        circle_center_m=None,
        circle_radius_m=None,
        circle_fit_rms_error_m=None,
        yaw_delta_rad=None,
        gyro_yaw_delta_rad=None,
        ekf_yaw_delta_rad=None,
        gyro_vs_odom_yaw_delta_rad=None,
        ekf_vs_odom_yaw_delta_rad=None,
        vx_sign_flips=0,
        gyro_sign_flips=0,
        errors=tuple(errors),
        warnings=(),
    )


def _parse_sample(row: dict) -> Optional[CircleSample]:
    try:
        values = [float(row[name]) for name in REQUIRED_COLUMNS]
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    ekf_values = []
    for name in ("ekf_x", "ekf_y", "ekf_yaw"):
        value = row.get(name)
        if value in (None, ""):
            ekf_values.append(None)
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        ekf_values.append(value)
    if any(value is None for value in ekf_values):
        ekf_values = [None, None, None]
    return CircleSample(*values, *ekf_values)


def load_samples(
    csv_path: Path,
) -> Tuple[List[CircleSample], int, Tuple[str, ...]]:
    """Load complete finite samples and report skipped rows without raising."""

    try:
        with csv_path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            headers = reader.fieldnames
            if not headers:
                return [], 0, ("input CSV has no header",)
            missing = [
                name for name in REQUIRED_COLUMNS if name not in headers
            ]
            if missing:
                return [], 0, (
                    "input CSV is missing columns: " + ", ".join(missing),
                )

            samples: List[CircleSample] = []
            skipped_rows = 0
            for row in reader:
                sample = _parse_sample(row)
                if sample is None:
                    skipped_rows += 1
                else:
                    samples.append(sample)
    except OSError as error:
        return [], 0, (f"cannot read CSV: {error}",)

    if not samples:
        return [], skipped_rows, (
            "no complete finite samples (need "
            + ", ".join(REQUIRED_COLUMNS)
            + ")",
        )
    return samples, skipped_rows, ()


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values)


def _standard_deviation(values: Sequence[float], mean_value: float) -> float:
    variance = sum((value - mean_value) ** 2 for value in values) / len(values)
    return math.sqrt(variance)


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * fraction
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def _sign_flips(values: Sequence[float]) -> int:
    signs = []
    for value in values:
        if value > _MIN_ANGULAR_RATE_RADPS:
            signs.append(1)
        elif value < -_MIN_ANGULAR_RATE_RADPS:
            signs.append(-1)
    return sum(
        previous != current for previous, current in zip(signs, signs[1:])
    )


def _solve_3x3(
    matrix: Sequence[Sequence[float]], vector: Sequence[float]
) -> Optional[List[float]]:
    """Solve a small linear system with pivoting, or return None if singular.

    The input is intentionally limited to the normal-equation matrix used by
    ``fit_circle``.
    """

    augmented = [list(row) + [value] for row, value in zip(matrix, vector)]
    scale = max(
        (abs(value) for row in augmented for value in row), default=0.0
    )
    if scale == 0.0:
        return None
    threshold = _PIVOT_EPSILON * scale

    for column in range(3):
        pivot_row = max(
            range(column, 3), key=lambda row: abs(augmented[row][column])
        )
        if abs(augmented[pivot_row][column]) <= threshold:
            return None
        augmented[column], augmented[pivot_row] = (
            augmented[pivot_row],
            augmented[column],
        )
        pivot = augmented[column][column]
        for row in range(column + 1, 3):
            factor = augmented[row][column] / pivot
            for index in range(column, 4):
                augmented[row][index] -= factor * augmented[column][index]

    solution = [0.0, 0.0, 0.0]
    for row in range(2, -1, -1):
        rhs = augmented[row][3] - sum(
            augmented[row][column] * solution[column]
            for column in range(row + 1, 3)
        )
        pivot = augmented[row][row]
        if abs(pivot) <= threshold:
            return None
        solution[row] = rhs / pivot
    if not all(math.isfinite(value) for value in solution):
        return None
    return solution


def fit_circle(
    samples: Sequence[CircleSample],
) -> Optional[Tuple[float, float, float, float]]:
    """Return center x/y, radius, and radial RMS error, or None if invalid."""

    if len(samples) < 3:
        return None

    sum_x = sum(sample.x_m for sample in samples)
    sum_y = sum(sample.y_m for sample in samples)
    sum_xx = sum(sample.x_m * sample.x_m for sample in samples)
    sum_yy = sum(sample.y_m * sample.y_m for sample in samples)
    sum_xy = sum(sample.x_m * sample.y_m for sample in samples)
    sum_r2 = sum(
        sample.x_m * sample.x_m + sample.y_m * sample.y_m
        for sample in samples
    )
    sum_xr2 = sum(
        sample.x_m * (sample.x_m * sample.x_m + sample.y_m * sample.y_m)
        for sample in samples
    )
    sum_yr2 = sum(
        sample.y_m * (sample.x_m * sample.x_m + sample.y_m * sample.y_m)
        for sample in samples
    )

    coefficients = _solve_3x3(
        (
            (sum_xx, sum_xy, sum_x),
            (sum_xy, sum_yy, sum_y),
            (sum_x, sum_y, float(len(samples))),
        ),
        (sum_xr2, sum_yr2, sum_r2),
    )
    if coefficients is None:
        return None

    center_x = coefficients[0] / 2.0
    center_y = coefficients[1] / 2.0
    radius_squared = (
        coefficients[2] + center_x * center_x + center_y * center_y
    )
    if (
        not math.isfinite(radius_squared)
        or radius_squared <= _MIN_SPEED_MPS ** 2
    ):
        return None
    radius = math.sqrt(radius_squared)
    residuals = [
        math.hypot(sample.x_m - center_x, sample.y_m - center_y) - radius
        for sample in samples
    ]
    rms_error = math.sqrt(
        sum(residual * residual for residual in residuals) / len(residuals)
    )
    values = (center_x, center_y, radius, rms_error)
    if not all(math.isfinite(value) for value in values):
        return None
    return center_x, center_y, radius, rms_error


def _unwrap_yaw_delta(yaws: Sequence[float]) -> float:
    total = 0.0
    for previous, current in zip(yaws, yaws[1:]):
        step = (current - previous + math.pi) % (2.0 * math.pi) - math.pi
        total += step
    return total


def _integrate_rate(samples: Sequence[CircleSample]) -> float:
    """Integrate gyro samples with their command-recording timestamps."""

    total = 0.0
    for previous, current in zip(samples, samples[1:]):
        elapsed = current.time_sec - previous.time_sec
        if elapsed <= 0.0:
            continue
        total += 0.5 * (previous.gyro_wz_radps + current.gyro_wz_radps) * elapsed
    return total


def _normalized_delta(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def analyze_samples(
    samples: Sequence[CircleSample],
    wheelbase_m: float = 0.144,
    skipped_rows: int = 0,
) -> CircleAnalysis:
    """Compute steady-state turning metrics from valid recorded samples."""

    if not math.isfinite(wheelbase_m) or wheelbase_m <= 0.0:
        raise ValueError("wheelbase must be a positive finite value")
    if not samples:
        return _empty_analysis(("no complete finite samples",), skipped_rows)

    timestamps = [sample.time_sec for sample in samples]
    lower_time = _percentile(timestamps, 0.10)
    upper_time = _percentile(timestamps, 0.90)
    steady = [
        sample
        for sample in samples
        if lower_time <= sample.time_sec <= upper_time
    ]
    if not steady:
        steady = list(samples)

    steady_vx = [sample.vx_mps for sample in steady]
    steady_gyro = [sample.gyro_wz_radps for sample in steady]
    steady_odom = [sample.odom_wz_radps for sample in steady]
    mean_vx = _mean(steady_vx)
    mean_gyro = _mean(steady_gyro)
    mean_odom = _mean(steady_odom)
    warnings = []

    gyro_radius = None
    gyro_delta = None
    odom_radius = None
    odom_delta = None
    if abs(mean_vx) <= _MIN_SPEED_MPS:
        warnings.append(
            "steady-state vx is near zero; gyro and odom turning radii "
            "are unavailable"
        )
    else:
        if abs(mean_gyro) <= _MIN_ANGULAR_RATE_RADPS:
            warnings.append(
                "steady-state gyro_wz is near zero; "
                "gyro turning radius is unavailable"
            )
        else:
            gyro_radius = abs(mean_vx / mean_gyro)
            gyro_delta = math.atan(wheelbase_m / gyro_radius)
        if abs(mean_odom) <= _MIN_ANGULAR_RATE_RADPS:
            warnings.append(
                "steady-state odom_wz is near zero; "
                "odom turning radius is unavailable"
            )
        else:
            odom_radius = abs(mean_vx / mean_odom)
            odom_delta = math.atan(wheelbase_m / odom_radius)

    circle = fit_circle(samples)
    if circle is None:
        warnings.append(
            "odom path circle fit is degenerate or has insufficient "
            "geometric variation"
        )
        center = None
        circle_radius = None
        circle_rms = None
    else:
        center = (circle[0], circle[1])
        circle_radius = circle[2]
        circle_rms = circle[3]

    yaws = [sample.yaw_rad for sample in samples]
    odom_yaw_delta = _unwrap_yaw_delta(yaws)
    gyro_yaw_delta = _integrate_rate(samples)
    ekf_yaws = [sample.ekf_yaw_rad for sample in samples]
    if all(yaw is not None for yaw in ekf_yaws):
        ekf_yaw_delta = _unwrap_yaw_delta(ekf_yaws)
    else:
        ekf_yaw_delta = None
        warnings.append(
            "EKF pose samples are unavailable; cannot verify fused heading"
        )

    gyro_vs_odom = _normalized_delta(gyro_yaw_delta, odom_yaw_delta)
    ekf_vs_odom = (
        _normalized_delta(ekf_yaw_delta, odom_yaw_delta)
        if ekf_yaw_delta is not None else None
    )
    if (
        abs(gyro_yaw_delta) > 0.10
        and abs(odom_yaw_delta) > 0.10
        and gyro_yaw_delta * odom_yaw_delta < 0.0
    ):
        warnings.append(
            "IMU and wheel odometry yaw signs disagree; do not run navigation"
        )
    if (
        ekf_yaw_delta is not None
        and abs(ekf_yaw_delta) > 0.10
        and abs(odom_yaw_delta) > 0.10
        and ekf_yaw_delta * odom_yaw_delta < 0.0
    ):
        warnings.append(
            "EKF and wheel odometry yaw signs disagree; correct localization before navigation"
        )
    return CircleAnalysis(
        sample_count=len(samples),
        skipped_rows=skipped_rows,
        start_time_sec=min(timestamps),
        end_time_sec=max(timestamps),
        steady_sample_count=len(steady),
        mean_vx_mps=mean_vx,
        mean_gyro_wz_radps=mean_gyro,
        gyro_wz_stddev_radps=_standard_deviation(steady_gyro, mean_gyro),
        mean_odom_wz_radps=mean_odom,
        gyro_radius_m=gyro_radius,
        gyro_effective_steering_rad=gyro_delta,
        odom_radius_m=odom_radius,
        odom_effective_steering_rad=odom_delta,
        circle_center_m=center,
        circle_radius_m=circle_radius,
        circle_fit_rms_error_m=circle_rms,
        yaw_delta_rad=odom_yaw_delta,
        gyro_yaw_delta_rad=gyro_yaw_delta,
        ekf_yaw_delta_rad=ekf_yaw_delta,
        gyro_vs_odom_yaw_delta_rad=gyro_vs_odom,
        ekf_vs_odom_yaw_delta_rad=ekf_vs_odom,
        vx_sign_flips=_sign_flips(steady_vx),
        gyro_sign_flips=_sign_flips(steady_gyro),
        errors=(),
        warnings=tuple(warnings),
    )


def analyze_csv(csv_path: str, wheelbase_m: float = 0.144) -> CircleAnalysis:
    """Load and analyze a CSV without propagating malformed-input errors."""

    samples, skipped_rows, errors = load_samples(Path(csv_path))
    if errors:
        return _empty_analysis(errors, skipped_rows)
    try:
        return analyze_samples(samples, wheelbase_m, skipped_rows)
    except ValueError as error:
        return _empty_analysis((str(error),), skipped_rows)


def format_report(analysis: CircleAnalysis) -> str:
    """Format a stable, human-readable report for a calibration log."""

    if not analysis.ok:
        return "\n".join(f"ERROR: {error}" for error in analysis.errors)

    lines = [
        "samples: "
        f"{analysis.sample_count} "
        f"(skipped invalid rows: {analysis.skipped_rows})",
        "t range "
        f"{analysis.start_time_sec:.2f}..{analysis.end_time_sec:.2f} s; "
        f"steady-state samples: {analysis.steady_sample_count}",
        "steady-state: "
        f"mean vx={analysis.mean_vx_mps:.4f} m/s  "
        f"gyro_wz={analysis.mean_gyro_wz_radps:+.4f} "
        f"(std {analysis.gyro_wz_stddev_radps:.4f})  "
        f"odom_wz={analysis.mean_odom_wz_radps:+.4f}",
        "sign flips in steady state: "
        f"vx={analysis.vx_sign_flips}  gyro={analysis.gyro_sign_flips}",
    ]
    if analysis.gyro_radius_m is not None:
        lines.append(
            f"R_gyro = {analysis.gyro_radius_m:.4f} m   "
            "delta_eff = atan(L/R) = "
            f"{analysis.gyro_effective_steering_rad:.4f} rad "
            f"({math.degrees(analysis.gyro_effective_steering_rad):.2f} deg)"
        )
    if analysis.odom_radius_m is not None:
        lines.append(
            f"R_odom(wheel_wz) = {analysis.odom_radius_m:.4f} m   "
            "delta_eff = "
            f"{math.degrees(analysis.odom_effective_steering_rad):.2f} deg"
        )
    if analysis.circle_center_m is not None:
        lines.append(
            "odom path circle fit: "
            f"c=({analysis.circle_center_m[0]:.3f},"
            f"{analysis.circle_center_m[1]:.3f}) "
            f"R={analysis.circle_radius_m:.4f} m "
            f"rms={analysis.circle_fit_rms_error_m:.4f} m"
        )
    lines.append(
        f"total yaw turned: {math.degrees(analysis.yaw_delta_rad):.1f} deg "
        f"({analysis.yaw_delta_rad / (2.0 * math.pi):.2f} revs)"
    )
    lines.append(
        "yaw integration: "
        f"gyro={math.degrees(analysis.gyro_yaw_delta_rad):+.1f} deg "
        f"wheel={math.degrees(analysis.yaw_delta_rad):+.1f} deg "
        f"difference={math.degrees(analysis.gyro_vs_odom_yaw_delta_rad):+.1f} deg"
    )
    if analysis.ekf_yaw_delta_rad is not None:
        lines.append(
            "fused yaw: "
            f"EKF={math.degrees(analysis.ekf_yaw_delta_rad):+.1f} deg "
            f"EKF-wheel difference="
            f"{math.degrees(analysis.ekf_vs_odom_yaw_delta_rad):+.1f} deg"
        )
    lines.extend(f"WARNING: {warning}" for warning in analysis.warnings)
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze a steering-circle CSV for turning-radius calibration."
        )
    )
    parser.add_argument(
        "--csv", required=True, help="CSV recorded during the circle drive"
    )
    parser.add_argument(
        "--wheelbase", type=float, default=0.144, help="wheelbase in metres"
    )
    args = parser.parse_args(argv)

    analysis = analyze_csv(args.csv, args.wheelbase)
    print(format_report(analysis), flush=True)
    return 0 if analysis.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
