"""Tests for forward-only action and motion-lease protocol helpers."""

import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.mission import OperationResult  # noqa: E402
from smartcar_task.protocols import (  # noqa: E402
    MOTION_FORWARD,
    MOTION_FORWARD_RECOVERY,
    MotionDirectionProtocol,
    MotionLease,
    classify_follow_waypoints_result,
    classify_navigate_to_pose_result,
    motion_direction,
    navigation_behavior_tree,
    odometry_matches_origin,
    run_reset_sequence,
    twist_is_stopped,
)


def odometry(x=0.0, y=0.0, yaw=0.0):
    orientation = SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
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
    def test_action_result_classification_fails_closed(self):
        self.assertTrue(classify_follow_waypoints_result(4, []).success)
        self.assertEqual(
            classify_follow_waypoints_result(4, [1]).status,
            "navigation_missed_waypoints:1",
        )
        self.assertEqual(
            classify_navigate_to_pose_result(5).status,
            "navigation_canceled",
        )
        self.assertEqual(
            classify_navigate_to_pose_result(6).status,
            "navigation_status_6",
        )

    def test_behavior_tree_selection_uses_bounded_forward_recovery_profile(self):
        self.assertEqual(motion_direction(), MOTION_FORWARD_RECOVERY)
        self.assertEqual(
            navigation_behavior_tree("standard", "precise.xml", "transit.xml"),
            "",
        )
        self.assertEqual(
            navigation_behavior_tree("precise", "precise.xml", "transit.xml"),
            "precise.xml",
        )
        self.assertEqual(
            navigation_behavior_tree(
                "standard", "precise.xml", "transit.xml", heading_locked=False),
            "transit.xml",
        )
        with self.assertRaisesRegex(ValueError, "unknown goal profile"):
            navigation_behavior_tree("reverse_handoff", "precise.xml", "transit.xml")
        with self.assertRaisesRegex(ValueError, "must not be empty"):
            navigation_behavior_tree("precise", "", "transit.xml")

    def test_motion_lease_allows_only_forward_and_bounded_recovery(self):
        self.assertEqual(
            MotionLease(MOTION_FORWARD_RECOVERY, 1, object()).direction,
            MOTION_FORWARD_RECOVERY,
        )
        with self.assertRaisesRegex(ValueError, "unknown motion direction"):
            MotionLease(2, 1, object())

    def test_motion_protocol_stops_before_preparing_forward_lease(self):
        calls = []

        class Guard:
            def stop(self, lease):
                calls.append(("stop", lease))
                return OperationResult(True, "stopped")

            def prepare(self, lease):
                calls.append(("prepare", lease))
                return OperationResult(True, "prepared"), 3, 5

            def activate(self, lease):
                calls.append(("activate", lease))
                return OperationResult(True, "active")

            def renew(self, lease):
                calls.append(("renew", lease))
                return OperationResult(True, "renewed")

        protocol = MotionDirectionProtocol(
            Guard(), lambda: OperationResult(True, "stopped"))
        result, lease = protocol.prepare(MOTION_FORWARD, 9, "action")

        self.assertTrue(result.success)
        self.assertEqual([name for name, _lease in calls], ["stop", "prepare"])
        self.assertEqual(lease.direction, MOTION_FORWARD)
        self.assertEqual((lease.boot_epoch, lease.lease_id), (3, 5))
        self.assertTrue(protocol.activate(lease).success)
        self.assertTrue(protocol.renew(lease).success)
        self.assertEqual(calls[-2][1], lease)
        self.assertEqual(calls[-1][1], lease)

    def test_motion_protocol_retries_only_stop_barrier(self):
        now = [0.0]
        responses = [
            OperationResult(False, "stop_barrier_not_ready"),
            OperationResult(True, "prepared"),
        ]

        class Guard:
            @staticmethod
            def stop(_lease):
                return OperationResult(True, "stopped")

            @staticmethod
            def prepare(_lease):
                return responses.pop(0), 1, 2

        protocol = MotionDirectionProtocol(
            Guard(),
            lambda: OperationResult(True, "stopped"),
            prepare_timeout_sec=0.2,
            prepare_retry_period_sec=0.1,
            monotonic=lambda: now[0],
            sleep=lambda seconds: now.__setitem__(0, now[0] + seconds),
        )
        result, lease = protocol.prepare(MOTION_FORWARD, 1, object())
        self.assertTrue(result.success)
        self.assertEqual(lease.lease_id, 2)

    def test_stop_and_origin_helpers_validate_full_messages(self):
        twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.001, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.001),
        )
        self.assertTrue(twist_is_stopped(twist, 0.01, 0.01))
        self.assertTrue(odometry_matches_origin(odometry()))
        self.assertFalse(odometry_matches_origin(odometry(x=0.3)))

    def test_reset_sequence_orders_navigation_pose_and_proof(self):
        calls = []
        result = run_reset_sequence(
            lambda: calls.append("stopped") or True,
            lambda: calls.append("set_pose") or OperationResult(True, "ok"),
            lambda: calls.append("verified") or OperationResult(True, "ok"),
        )
        self.assertTrue(result.success)
        self.assertEqual(calls, ["stopped", "set_pose", "verified"])


if __name__ == "__main__":
    unittest.main()
