"""Pure tests for the decoded odometry validation helper."""
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_safety.odometry import odometry_is_finite  # noqa: E402


class OdometryValidationTests(unittest.TestCase):
    def test_decoded_helper_checks_only_pose_and_twist(self):
        class Values:
            pass

        message = Values()
        message.pose = Values()
        message.pose.pose = Values()
        message.pose.pose.position = Values()
        message.pose.pose.orientation = Values()
        message.twist = Values()
        message.twist.twist = Values()
        message.twist.twist.linear = Values()
        message.twist.twist.angular = Values()
        for field in ("x", "y", "z"):
            setattr(message.pose.pose.position, field, 0.0)
            setattr(message.twist.twist.linear, field, 0.0)
            setattr(message.twist.twist.angular, field, 0.0)
        for field in ("x", "y", "z"):
            setattr(message.pose.pose.orientation, field, 0.0)
        message.pose.pose.orientation.w = 1.0
        self.assertTrue(odometry_is_finite(message))


if __name__ == "__main__":
    unittest.main()
