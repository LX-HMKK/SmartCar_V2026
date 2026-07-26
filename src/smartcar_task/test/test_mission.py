"""Tests for the ROS-independent mission state machine."""
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_task.mission import (  # noqa: E402
    Mission,
    MissionConfig,
    MissionState,
    OperationResult,
)
from smartcar_task.waypoints import Waypoint  # noqa: E402


def waypoint(task, x=0.0):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        task=task,
    )


class FakeClock:
    def __init__(self):
        self.now = 1_000_000_000
        self.sleeps = []
        self.on_sleep = None

    def now_ns(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += int(seconds * 1_000_000_000)
        if self.on_sleep is not None:
            callback, self.on_sleep = self.on_sleep, None
            callback()


class FakeOutput:
    def __init__(self):
        self.states = []
        self.text = []
        self.speech = []

    def publish_state(self, state):
        self.states.append(state)

    def publish_text(self, value):
        self.text.append(value)

    def publish_speech(self, value):
        self.speech.append(value)


class FakeNavigator:
    def __init__(self, results=None, ready=True):
        self.results = list(results or [])
        self.ready = ready
        self.calls = []
        self.cancel_calls = 0
        self.active = False
        self.on_navigate = None

    def wait_ready(self, _timeout_sec):
        return self.ready

    def navigate(self, waypoint, reverse_direction=False):
        self.calls.append((waypoint, bool(reverse_direction)))
        self.active = True
        if self.on_navigate is not None:
            self.on_navigate()
        self.active = False
        if self.results:
            return self.results.pop(0)
        return OperationResult(True, "ok")

    def cancel(self):
        self.cancel_calls += 1
        self.active = False
        return True

    def is_active(self):
        return self.active


class UnconfirmedNavigator(FakeNavigator):
    def navigate(self, waypoint, reverse_direction=False):
        self.calls.append((waypoint, bool(reverse_direction)))
        self.active = True
        return OperationResult(False, "cancel_unconfirmed")


class FakeVision:
    def __init__(self, qr_results=None, vlm_results=None, ready=True):
        self.qr_results = list(qr_results or [])
        self.vlm_results = list(vlm_results or [])
        self.ready = ready
        self.ready_calls = []
        self.qr_calls = []
        self.vlm_calls = []
        self.on_qr = None
        self.on_vlm = None

    def wait_ready(self, require_qr, require_vlm, _timeout_sec):
        self.ready_calls.append((require_qr, require_vlm))
        return self.ready

    def read_qr(self, not_before_ns, timeout_sec):
        self.qr_calls.append((not_before_ns, timeout_sec))
        if self.on_qr is not None:
            self.on_qr()
        return self.qr_results.pop(0)

    def describe_scene(self, not_before_ns, timeout_sec, prompt):
        self.vlm_calls.append((not_before_ns, timeout_sec, prompt))
        if self.on_vlm is not None:
            self.on_vlm()
        return self.vlm_results.pop(0)


class FakeLocalization:
    def __init__(self, result=None):
        self.result = result or OperationResult(True, "ok")
        self.calls = 0

    def reset_origin(self):
        self.calls += 1
        return self.result


class MissionTests(unittest.TestCase):
    def make_mission(
        self,
        navigator=None,
        vision=None,
        localization=None,
        config=None,
    ):
        self.clock = FakeClock()
        self.output = FakeOutput()
        self.navigator = navigator or FakeNavigator()
        self.vision = vision or FakeVision()
        self.localization = localization or FakeLocalization()
        self.mission = Mission(
            navigator=self.navigator,
            vision=self.vision,
            localization=self.localization,
            clock=self.clock,
            output=self.output,
            config=config or MissionConfig(
                navigation_retry_delay_sec=0.0,
                qr_settle_sec=0.0,
                qr_retry_delay_sec=0.0,
            ),
        )
        return self.mission

    def test_success_path_navigates_three_segments_and_runs_endpoint_tasks(self):
        vision = FakeVision(
            qr_results=[OperationResult(True, "ok", "WARD-A")],
            vlm_results=[OperationResult(True, "ok", "person description")],
        )
        mission = self.make_mission(vision=vision)
        items = [
            waypoint("start", 0.0),
            waypoint("qr", 1.0),
            waypoint("corridor", 2.0),
            waypoint("vlm", 3.0),
            waypoint("loop", 4.0),
            waypoint("loop", 5.0),
            waypoint("loop", 6.0),
            waypoint("corridor", 7.0),
            waypoint("return", 8.0),
        ]

        result = mission.execute(items)

        self.assertTrue(result.success, result.status)
        self.assertEqual(mission.state, MissionState.COMPLETED)
        self.assertEqual(
            self.navigator.calls,
            [(item, False) for item in items[1:]],
        )
        self.assertEqual(vision.ready_calls, [(True, True)])
        self.assertEqual(self.output.text, ["WARD-A", "person description"])
        self.assertEqual(self.output.speech, ["WARD-A", "person description"])
        self.assertIn(MissionState.NAVIGATING.value, self.output.states)
        self.assertIn(MissionState.RUNNING_QR.value, self.output.states)
        self.assertIn(MissionState.RUNNING_VLM.value, self.output.states)

    def test_endpoint_task_waits_for_its_navigation_segment(self):
        items = [
            waypoint("start"),
            waypoint("qr", 1.0),
            waypoint("corridor", 2.0),
            waypoint("vlm", 3.0),
        ]
        navigator = FakeNavigator(results=[
            OperationResult(True, "ok"),
            OperationResult(False, "planner_failed"),
            OperationResult(False, "planner_failed"),
        ])
        vision = FakeVision(
            qr_results=[OperationResult(True, "ok", "WARD-A")],
            vlm_results=[OperationResult(True, "ok", "must-not-run")],
        )
        mission = self.make_mission(navigator=navigator, vision=vision)

        result = mission.execute(items)

        self.assertFalse(result.success)
        self.assertEqual(result.status, "navigation_failed:planner_failed")
        self.assertEqual(
            navigator.calls,
            [
                (items[1], False),
                (items[2], False),
                (items[2], False),
            ],
        )
        self.assertEqual(len(vision.qr_calls), 1)
        self.assertEqual(vision.vlm_calls, [])

    def test_qr_task_is_not_run_when_its_navigation_segment_fails(self):
        items = [waypoint("start"), waypoint("qr", 1.0)]
        navigator = FakeNavigator(results=[
            OperationResult(False, "planner_failed"),
            OperationResult(False, "planner_failed"),
        ])
        vision = FakeVision(
            qr_results=[OperationResult(True, "ok", "must-not-run")],
        )
        mission = self.make_mission(navigator=navigator, vision=vision)

        result = mission.execute(items)

        self.assertFalse(result.success)
        self.assertEqual(
            navigator.calls,
            [(items[1], False), (items[1], False)],
        )
        self.assertEqual(vision.qr_calls, [])

    def test_navigation_retries_once_before_continuing(self):
        navigator = FakeNavigator(results=[
            OperationResult(False, "planner_failed"),
            OperationResult(True, "ok"),
        ])
        mission = self.make_mission(navigator=navigator)

        result = mission.execute([waypoint("return")])

        self.assertTrue(result.success)
        self.assertEqual(len(navigator.calls), 2)
        self.assertEqual(
            navigator.calls,
            [(waypoint("return"), False), (waypoint("return"), False)],
        )

    def test_qr_settle_and_single_retry_use_fresh_request_times(self):
        vision = FakeVision(qr_results=[
            OperationResult(False, "qr_timeout"),
            OperationResult(True, "ok", "WARD-B"),
        ])
        config = MissionConfig(
            navigation_retry_delay_sec=0.0,
            qr_settle_sec=2.0,
            qr_retry_delay_sec=0.25,
        )
        mission = self.make_mission(vision=vision, config=config)

        result = mission.execute([waypoint("qr")])

        self.assertTrue(result.success)
        self.assertEqual(len(vision.qr_calls), 2)
        self.assertGreater(vision.qr_calls[1][0], vision.qr_calls[0][0])
        self.assertAlmostEqual(sum(self.clock.sleeps), 2.25, places=6)
        self.assertEqual(self.output.text, ["WARD-B"])

    def test_vlm_fallback_is_published_and_mission_completes(self):
        fallback = "检测到人物立牌"
        vision = FakeVision(vlm_results=[
            OperationResult(True, "backend_timeout", fallback, True),
        ])
        mission = self.make_mission(vision=vision)

        result = mission.execute([waypoint("vlm")])

        self.assertTrue(result.success)
        self.assertEqual(mission.state, MissionState.COMPLETED)
        self.assertEqual(self.output.text, [fallback])
        self.assertEqual(self.output.speech, [fallback])
        self.assertEqual(vision.vlm_calls[0][1], 8.0)

    def test_vlm_service_failure_also_uses_fixed_fallback(self):
        vision = FakeVision(vlm_results=[
            OperationResult(False, "image_timeout"),
        ])
        mission = self.make_mission(vision=vision)

        result = mission.execute([waypoint("vlm")])

        self.assertTrue(result.success)
        self.assertEqual(self.output.text, ["检测到人物立牌"])
        self.assertEqual(mission.state, MissionState.COMPLETED)

    def test_stop_request_cancels_navigation_and_reaches_stopped(self):
        mission = self.make_mission()
        self.navigator.on_navigate = mission.request_stop

        result = mission.execute([waypoint("loop")])

        self.assertFalse(result.success)
        self.assertEqual(result.status, "mission_stopped")
        self.assertEqual(mission.state, MissionState.STOPPED)
        self.assertEqual(self.navigator.cancel_calls, 1)

    def test_stop_interrupts_settle_and_discards_late_vlm_result(self):
        vision = FakeVision(
            qr_results=[OperationResult(True, "ok", "must-not-publish")],
        )
        config = MissionConfig(qr_settle_sec=2.0)
        mission = self.make_mission(vision=vision, config=config)
        self.clock.on_sleep = mission.request_stop

        result = mission.execute([waypoint("qr")])

        self.assertEqual(result.status, "mission_stopped")
        self.assertEqual(vision.qr_calls, [])
        self.assertEqual(self.output.speech, [])

        vision = FakeVision(vlm_results=[
            OperationResult(True, "ok", "late description"),
        ])
        mission = self.make_mission(vision=vision)
        vision.on_vlm = mission.request_stop

        result = mission.execute([waypoint("vlm")])

        self.assertEqual(result.status, "mission_stopped")
        self.assertEqual(self.output.speech, [])

    def test_reset_is_only_allowed_after_terminal_state_and_navigation_stop(self):
        mission = self.make_mission()
        rejected = mission.reset()
        self.assertFalse(rejected.success)
        self.assertEqual(self.localization.calls, 0)

        self.assertTrue(mission.execute([waypoint("return")]).success)
        reset = mission.reset()

        self.assertTrue(reset.success)
        self.assertEqual(self.localization.calls, 1)
        self.assertEqual(mission.state, MissionState.IDLE)

    def test_repeated_start_and_reset_while_navigation_active_are_rejected(self):
        mission = self.make_mission()
        generation = mission.reserve_start()
        self.assertIsNotNone(generation)
        self.assertIsNone(mission.reserve_start())
        mission.request_stop()
        mission.run_reserved(generation, [waypoint("return")])

        self.navigator.active = True
        reset = mission.reset()
        self.assertFalse(reset.success)
        self.assertEqual(reset.status, "navigation_not_stopped")
        self.assertEqual(self.localization.calls, 0)

    def test_exhausted_navigation_or_qr_retry_fails_closed(self):
        navigator = FakeNavigator(results=[
            OperationResult(False, "failed_1"),
            OperationResult(False, "failed_2"),
        ])
        mission = self.make_mission(navigator=navigator)
        result = mission.execute([waypoint("return")])
        self.assertFalse(result.success)
        self.assertEqual(mission.state, MissionState.FAILED)

        vision = FakeVision(qr_results=[
            OperationResult(False, "timeout_1"),
            OperationResult(False, "timeout_2"),
        ])
        mission = self.make_mission(vision=vision)
        result = mission.execute([waypoint("qr")])
        self.assertFalse(result.success)
        self.assertEqual(mission.state, MissionState.FAILED)

    def test_failed_mission_can_still_cancel_an_unconfirmed_active_goal(self):
        navigator = UnconfirmedNavigator()
        mission = self.make_mission(navigator=navigator)

        result = mission.execute([waypoint("return")])

        self.assertFalse(result.success)
        self.assertTrue(navigator.is_active())
        self.assertTrue(mission.request_stop())
        self.assertFalse(navigator.is_active())
        self.assertEqual(navigator.cancel_calls, 1)

    def test_direction_boundary_splits_navigation_segments(self):
        """Every waypoint reaches the navigator with its declared direction."""
        def wp(
            task,
            x=0.0,
            direction="forward",
            goal_profile="standard",
        ):
            return Waypoint(
                frame_id="odom_combined",
                position=(x, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
                task=task,
                direction=direction,
                goal_profile=goal_profile,
            )

        items = [
            wp("start", 0.0),
            wp("qr", 1.0),                       # forward (default)
            wp("corridor", 2.0, "reverse"),       # reverse start
            wp("corridor", 3.0, "reverse"),       # still reverse
            wp("vlm", 4.0, "reverse", "reverse_handoff"),
            wp("loop", 5.0),                      # back to forward (default)
            wp("loop", 6.0),
            wp("loop", 7.0),
            wp("corridor", 8.0),
            wp("return", 9.0),
        ]

        vision = FakeVision(
            qr_results=[OperationResult(True, "ok", "WARD-C")],
            vlm_results=[OperationResult(True, "ok", "person")],
        )
        mission = self.make_mission(vision=vision)
        result = mission.execute(items)

        self.assertTrue(result.success, result.status)
        self.assertEqual(len(self.navigator.calls), len(items) - 1)
        self.assertEqual(
            self.navigator.calls,
            [
                (items[1], False),
                (items[2], True),
                (items[3], True),
                (items[4], True),
                (items[5], False),
                (items[6], False),
                (items[7], False),
                (items[8], False),
                (items[9], False),
            ],
        )
        self.assertEqual(items[4].goal_profile, "reverse_handoff")
        self.assertEqual(mission.state, MissionState.COMPLETED)

    def test_consecutive_forward_waypoints_are_submitted_individually(self):
        items = [
            Waypoint("odom_combined", (0.0, 0.0, 0.0), (0., 0., 0., 1.), "start", "forward"),
            Waypoint("odom_combined", (1.0, 0.0, 0.0), (0., 0., 0., 1.), "qr", "forward"),
        ]
        vision = FakeVision(
            qr_results=[OperationResult(True, "ok", "WARD-D")],
        )
        mission = self.make_mission(vision=vision)
        result = mission.execute(items)

        self.assertTrue(result.success, result.status)
        self.assertEqual(len(self.navigator.calls), 1)
        self.assertEqual(self.navigator.calls[0], (items[1], False))


if __name__ == "__main__":
    unittest.main()
