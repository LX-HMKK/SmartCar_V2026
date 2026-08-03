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
    MOTION_FORWARD,
    MOTION_REVERSE,
    MotionDirectionProtocol,
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

    def test_navigate_to_pose_result_and_behavior_tree_selection(self):
        self.assertTrue(classify_navigate_to_pose_result(4).success)
        self.assertEqual(
            classify_navigate_to_pose_result(5).status,
            "navigation_canceled",
        )
        self.assertEqual(
            classify_navigate_to_pose_result(6).status,
            "navigation_status_6",
        )
        self.assertEqual(motion_direction(False), MOTION_FORWARD)
        self.assertEqual(motion_direction(True), MOTION_REVERSE)
        self.assertEqual(navigation_behavior_tree(False, "ignored.xml"), "")
        self.assertEqual(
            navigation_behavior_tree(
                False,
                "ignored.xml",
                goal_profile="precise",
                precise_forward_behavior_tree="precise.xml",
            ),
            "precise.xml",
        )
        self.assertEqual(
            navigation_behavior_tree(True, "reverse.xml"), "reverse.xml")
        self.assertEqual(
            navigation_behavior_tree(
                True,
                "reverse.xml",
                goal_profile="reverse_handoff",
                reverse_handoff_behavior_tree="reverse-handoff.xml",
            ),
            "reverse-handoff.xml",
        )
        with self.assertRaises(ValueError):
            navigation_behavior_tree(True, "")
        with self.assertRaises(ValueError):
            navigation_behavior_tree(
                True,
                "reverse.xml",
                goal_profile="reverse_handoff",
            )
        with self.assertRaises(ValueError):
            navigation_behavior_tree(
                False,
                "reverse.xml",
                goal_profile="reverse_handoff",
                reverse_handoff_behavior_tree="reverse-handoff.xml",
            )
        with self.assertRaises(ValueError):
            navigation_behavior_tree(
                False, "reverse.xml", goal_profile="precise")
        with self.assertRaises(ValueError):
            navigation_behavior_tree(
                True,
                "reverse.xml",
                goal_profile="precise",
                precise_forward_behavior_tree="precise.xml",
            )

    def test_motion_direction_protocol_binds_direction_generation_and_uuid(self):
        calls = []
        action_uuid = object()

        class Guard:
            @staticmethod
            def stop(lease):
                calls.append(("stop", lease))
                return OperationResult(True, "stopped")

            @staticmethod
            def prepare(lease):
                calls.append(("prepare", lease))
                return OperationResult(True, "prepared"), 17, 23

            @staticmethod
            def activate(lease):
                calls.append(("activate", lease))
                return OperationResult(True, "active")

            @staticmethod
            def renew(lease):
                calls.append(("renew", lease))
                return OperationResult(True, "renewed")

        def wait_stopped():
            calls.append(("settled", None))
            return OperationResult(True, "stopped")

        protocol = MotionDirectionProtocol(Guard(), wait_stopped)
        result, lease = protocol.prepare(
            MOTION_REVERSE, 9, action_uuid)
        self.assertTrue(result.success, result.status)
        self.assertEqual(
            [name for name, _value in calls],
            ["stop", "prepare"],
        )
        provisional = calls[0][1]
        self.assertEqual(provisional.direction, MOTION_REVERSE)
        self.assertEqual(provisional.generation, 9)
        self.assertIs(provisional.action_uuid, action_uuid)
        self.assertEqual(provisional.boot_epoch, 0)
        self.assertEqual(provisional.lease_id, 0)
        self.assertEqual(lease.boot_epoch, 17)
        self.assertEqual(lease.lease_id, 23)

        self.assertTrue(protocol.activate(lease).success)
        self.assertTrue(protocol.renew(lease).success)
        self.assertTrue(protocol.stop(lease).success)
        self.assertEqual(
            [name for name, _value in calls],
            [
                "stop", "prepare", "activate", "renew", "stop",
            ],
        )
        for name, identity in calls[3:6]:
            self.assertIn(name, {"activate", "renew", "stop"})
            self.assertEqual(identity, lease)

    def test_motion_direction_protocol_never_prepares_before_stop_barrier(self):
        calls = []

        class Guard:
            @staticmethod
            def stop(_lease):
                calls.append("stop")
                return OperationResult(False, "transport_timeout")

            @staticmethod
            def prepare(_lease):
                calls.append("prepare")
                return OperationResult(True, "prepared"), 1, 1

        protocol = MotionDirectionProtocol(
            Guard(),
            lambda: calls.append("settled") or OperationResult(True, "ok"),
        )
        result, lease = protocol.prepare(MOTION_FORWARD, 1, object())
        self.assertFalse(result.success)
        self.assertEqual(result.status, "direction_stop:transport_timeout")
        self.assertIsNone(lease)
        self.assertEqual(calls, ["stop"])

    def test_motion_direction_protocol_only_retries_stop_barrier_response(self):
        calls = []
        now = [0.0]

        class Guard:
            prepare_results = [
                OperationResult(False, "stop_barrier_not_ready"),
                OperationResult(False, "stop_barrier_not_ready"),
                OperationResult(True, "prepared"),
            ]

            @staticmethod
            def stop(_lease):
                calls.append("stop")
                return OperationResult(True, "stopped")

            @classmethod
            def prepare(cls, lease):
                calls.append((
                    "prepare", lease.generation, lease.action_uuid))
                return cls.prepare_results.pop(0), 4, 8

        def sleep(seconds):
            calls.append(("sleep", seconds))
            now[0] += seconds

        protocol = MotionDirectionProtocol(
            Guard(),
            lambda: calls.append("settled") or OperationResult(True, "ok"),
            prepare_timeout_sec=0.2,
            prepare_retry_period_sec=0.05,
            monotonic=lambda: now[0],
            sleep=sleep,
        )
        action_uuid = object()
        result, lease = protocol.prepare(MOTION_REVERSE, 3, action_uuid)

        self.assertTrue(result.success, result.status)
        prepare_calls = [call for call in calls if isinstance(call, tuple)
                         and call[0] == "prepare"]
        self.assertEqual(len(prepare_calls), 3)
        self.assertTrue(all(call[1] == 3 for call in prepare_calls))
        self.assertTrue(all(call[2] is action_uuid for call in prepare_calls))
        self.assertEqual(lease.boot_epoch, 4)
        self.assertEqual(lease.lease_id, 8)

    def test_cancel_order_can_revoke_before_action_cancel_then_settle(self):
        calls = []

        class Guard:
            @staticmethod
            def stop(_lease):
                calls.append("revoke")
                return OperationResult(True, "stopped")

        protocol = MotionDirectionProtocol(
            Guard(),
            lambda: calls.append("settle") or OperationResult(True, "ok"),
        )
        identity = protocol.provisional(MOTION_REVERSE, 5, object())

        self.assertTrue(protocol.revoke(identity).success)
        calls.append("action_cancel")
        self.assertTrue(protocol.settle().success)

        self.assertEqual(calls, ["revoke", "action_cancel", "settle"])

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

    def test_stop_barrier_checks_all_six_raw_odom_twist_axes(self):
        def twist(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
            return SimpleNamespace(
                linear=SimpleNamespace(
                    x=linear[0], y=linear[1], z=linear[2]),
                angular=SimpleNamespace(
                    x=angular[0], y=angular[1], z=angular[2]),
            )

        self.assertTrue(twist_is_stopped(twist(), 0.01, 0.05))
        for value in (
            twist(linear=(0.011, 0.0, 0.0)),
            twist(linear=(0.0, -0.011, 0.0)),
            twist(linear=(0.0, 0.0, 0.011)),
            twist(angular=(0.051, 0.0, 0.0)),
            twist(angular=(0.0, -0.051, 0.0)),
            twist(angular=(0.0, 0.0, 0.051)),
            twist(linear=(math.nan, 0.0, 0.0)),
        ):
            with self.subTest(value=value):
                self.assertFalse(twist_is_stopped(value, 0.01, 0.05))

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
        )
        self.assertTrue(result.success)
        self.assertEqual(calls, ["set_pose", "verify_odom"])

        calls.clear()
        result = run_reset_sequence(
            lambda: True,
            success("set_pose"),
            lambda: OperationResult(False, "odom_timeout"),
        )
        self.assertFalse(result.success)
        self.assertEqual(calls, ["set_pose"])


if __name__ == "__main__":
    unittest.main()
