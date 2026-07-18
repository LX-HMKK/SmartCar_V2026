"""Tests for action-result and localization-reset protocol checks."""
from pathlib import Path
from types import SimpleNamespace
import math
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.mission import OperationResult  # noqa: E402
from smartcar_task.protocols import (  # noqa: E402
    classify_follow_waypoints_result,
    odometry_matches_origin,
    run_reset_sequence,
)


def odometry(x=0.0, y=0.0, yaw=0.0):
    orientation = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )
    vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="odom_combined"),
        child_frame_id="base_footprint",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=0.0),
                orientation=orientation,
            ),
            covariance=[0.1] * 36,
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(linear=vector, angular=vector),
            covariance=[0.1] * 36,
        ),
    )


class ProtocolTests(unittest.TestCase):
    def test_follow_waypoints_requires_success_status_and_no_misses(self):
        self.assertTrue(classify_follow_waypoints_result(4, []).success)
        self.assertFalse(classify_follow_waypoints_result(4, [0]).success)
        self.assertEqual(
            classify_follow_waypoints_result(5, []).status,
            "navigation_canceled",
        )
        self.assertFalse(classify_follow_waypoints_result(6, []).success)

    def test_origin_verification_rejects_wrong_frame_nonfinite_and_far_pose(self):
        self.assertTrue(odometry_matches_origin(odometry()))
        wrong_frame = odometry()
        wrong_frame.header.frame_id = "map"
        self.assertFalse(odometry_matches_origin(wrong_frame))
        nonfinite = odometry()
        nonfinite.pose.covariance[3] = math.nan
        self.assertFalse(odometry_matches_origin(nonfinite))
        self.assertFalse(odometry_matches_origin(odometry(x=0.5)))
        self.assertFalse(odometry_matches_origin(odometry(yaw=0.5)))

    def test_reset_sequence_is_ordered_and_stops_on_each_failure(self):
        calls = []

        def success(name):
            def callback():
                calls.append(name)
                return OperationResult(True, "ok")
            return callback

        result = run_reset_sequence(
            lambda: True,
            success("set_pose"),
            success("verify_odom"),
            success("clear_fault"),
        )
        self.assertTrue(result.success)
        self.assertEqual(calls, ["set_pose", "verify_odom", "clear_fault"])

        calls.clear()
        result = run_reset_sequence(
            lambda: True,
            success("set_pose"),
            lambda: OperationResult(False, "odom_timeout"),
            success("clear_fault"),
        )
        self.assertFalse(result.success)
        self.assertEqual(calls, ["set_pose"])


if __name__ == "__main__":
    unittest.main()
