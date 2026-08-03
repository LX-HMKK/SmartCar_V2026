"""Pure unit tests for the offline steering-circle CSV analyzer."""

import csv
import math
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.steering_circle_analyze import (  # noqa: E402
    analyze_csv,
    format_report,
)


FIELDS = ("t", "vx", "gyro_wz", "odom_wz", "x", "y", "yaw")


def write_csv(path, rows, fieldnames=FIELDS):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def circle_rows(radius_m=0.5, speed_mps=0.15, count=41):
    angular_rate = speed_mps / radius_m
    rows = []
    for index in range(count):
        time_sec = index * 0.1
        theta = angular_rate * time_sec
        rows.append({
            "t": time_sec,
            "vx": speed_mps,
            "gyro_wz": angular_rate,
            "odom_wz": angular_rate,
            "x": 1.0 + radius_m * math.cos(theta),
            "y": -0.4 + radius_m * math.sin(theta),
            "yaw": theta,
        })
    return rows


class SteeringCircleAnalyzeTests(unittest.TestCase):
    def test_valid_circle_reports_calibration_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "circle.csv"
            write_csv(path, circle_rows())
            analysis = analyze_csv(str(path))

        self.assertTrue(analysis.ok)
        self.assertAlmostEqual(analysis.gyro_radius_m, 0.5, places=6)
        self.assertAlmostEqual(analysis.odom_radius_m, 0.5, places=6)
        self.assertAlmostEqual(analysis.circle_radius_m, 0.5, places=6)
        self.assertAlmostEqual(analysis.circle_center_m[0], 1.0, places=6)
        self.assertAlmostEqual(analysis.circle_center_m[1], -0.4, places=6)
        self.assertAlmostEqual(
            analysis.gyro_effective_steering_rad,
            math.atan(0.189 / 0.5),
            places=6,
        )
        self.assertIn("R_gyro = 0.5000 m", format_report(analysis))

    def test_empty_or_invalid_csv_returns_readable_error(self):
        with tempfile.TemporaryDirectory() as directory:
            empty_path = Path(directory) / "empty.csv"
            empty_path.write_text("", encoding="utf-8")
            empty = analyze_csv(str(empty_path))

            invalid_path = Path(directory) / "invalid.csv"
            write_csv(
                invalid_path,
                [{name: "not-a-number" for name in FIELDS}],
            )
            invalid = analyze_csv(str(invalid_path))

        self.assertFalse(empty.ok)
        self.assertIn("no header", format_report(empty))
        self.assertFalse(invalid.ok)
        self.assertIn("no complete finite samples", format_report(invalid))

    def test_near_zero_speed_does_not_divide_by_zero(self):
        rows = circle_rows(speed_mps=0.0)
        for row in rows:
            row["gyro_wz"] = 0.3
            row["odom_wz"] = 0.3
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stationary.csv"
            write_csv(path, rows)
            analysis = analyze_csv(str(path))

        self.assertTrue(analysis.ok)
        self.assertIsNone(analysis.gyro_radius_m)
        self.assertIsNone(analysis.odom_radius_m)
        self.assertTrue(
            any("near zero" in warning for warning in analysis.warnings)
        )

    def test_degenerate_path_skips_circle_fit(self):
        rows = []
        for index in range(25):
            time_sec = index * 0.1
            rows.append({
                "t": time_sec,
                "vx": 0.15,
                "gyro_wz": 0.3,
                "odom_wz": 0.3,
                "x": time_sec,
                "y": 0.0,
                "yaw": 0.3 * time_sec,
            })
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "line.csv"
            write_csv(path, rows)
            analysis = analyze_csv(str(path))

        self.assertTrue(analysis.ok)
        self.assertIsNone(analysis.circle_radius_m)
        self.assertTrue(
            any("degenerate" in warning for warning in analysis.warnings)
        )


if __name__ == "__main__":
    unittest.main()
