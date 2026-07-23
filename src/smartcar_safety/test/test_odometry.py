"""Pure tests for validating odometry messages before safety decisions."""
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_safety.odometry import odometry_is_finite


def _vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _make_odometry():
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=_vector(),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(linear=_vector(), angular=_vector()),
            covariance=[0.0] * 36,
        ),
    )


class OdometryValidationTests(unittest.TestCase):
    def test_finite_odometry_is_accepted(self):
        self.assertTrue(odometry_is_finite(_make_odometry()))

    def test_each_non_finite_scalar_field_is_rejected(self):
        scalar_fields = (
            ("position", "x"),
            ("position", "y"),
            ("position", "z"),
            ("orientation", "x"),
            ("orientation", "y"),
            ("orientation", "z"),
            ("orientation", "w"),
            ("linear", "x"),
            ("linear", "y"),
            ("linear", "z"),
            ("angular", "x"),
            ("angular", "y"),
            ("angular", "z"),
        )
        for group_name, field_name in scalar_fields:
            for invalid in (math.nan, math.inf, -math.inf):
                with self.subTest(
                    group=group_name, field=field_name, invalid=invalid
                ):
                    message = _make_odometry()
                    if group_name in ("position", "orientation"):
                        group = getattr(message.pose.pose, group_name)
                    else:
                        group = getattr(message.twist.twist, group_name)
                    setattr(group, field_name, invalid)
                    self.assertFalse(odometry_is_finite(message))

    def test_non_finite_pose_or_twist_covariance_is_rejected(self):
        for covariance_owner in ("pose", "twist"):
            for invalid in (math.nan, math.inf, -math.inf):
                with self.subTest(owner=covariance_owner, invalid=invalid):
                    message = _make_odometry()
                    getattr(message, covariance_owner).covariance[17] = invalid
                    self.assertFalse(odometry_is_finite(message))

    def test_non_numeric_value_fails_closed(self):
        message = _make_odometry()
        message.pose.pose.position.x = "invalid"
        self.assertFalse(odometry_is_finite(message))

        message = _make_odometry()
        message.twist.covariance[0] = "invalid"
        self.assertFalse(odometry_is_finite(message))


if __name__ == "__main__":
    unittest.main()
