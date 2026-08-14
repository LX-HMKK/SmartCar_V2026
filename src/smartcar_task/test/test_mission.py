"""Forward-only mission state-machine tests."""

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
    QR_UNRECOGNIZED_TEXT,
    VLM_FALLBACK_TEXT,
)
from smartcar_task.waypoints import Waypoint  # noqa: E402


def waypoint(waypoint_id, task, x, goal_profile="standard", heading_mode=None, direction="forward"):
    return Waypoint(
        frame_id="odom_combined",
        position=(x, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        task=task,
        direction=direction,
        id=waypoint_id,
        goal_profile=goal_profile,
        heading_mode=heading_mode,
    )


class FakeNavigator:
    def __init__(self, results=()):
        self.results = list(results)
        self.calls = []
        self.through_calls = []
        self.active = False
        self.cancel_calls = 0

    def wait_ready(self, _timeout_sec):
        return True

    def is_active(self):
        return self.active

    def navigate(self, goal):
        self.calls.append(goal)
        return self.results.pop(0) if self.results else OperationResult(True, "ok")

    def navigate_through(self, goals):
        self.through_calls.append(tuple(goals))
        return self.results.pop(0) if self.results else OperationResult(True, "ok")

    def cancel(self):
        self.cancel_calls += 1
        self.active = False
        return True


class FakeVision:
    def __init__(self, qr=None, vlm=None):
        self.qr = qr or OperationResult(True, "ok", "WARD-A")
        self.vlm = vlm or OperationResult(True, "ok", "person")
        self.ready_calls = []
        self.qr_calls = []
        self.vlm_calls = []

    def wait_ready(self, qr, vlm, _timeout_sec):
        self.ready_calls.append((qr, vlm))
        return True

    def read_qr(self, _not_before, _timeout_sec):
        self.qr_calls.append((_not_before, _timeout_sec))
        return self.qr

    def describe_scene(self, _not_before, _timeout_sec, _prompt):
        self.vlm_calls.append((_not_before, _timeout_sec, _prompt))
        return self.vlm


class FakeClock:
    def __init__(self):
        self.now = 0

    def now_ns(self):
        self.now += 1
        return self.now

    def sleep(self, _seconds):
        pass


class FakeOutput:
    def __init__(self):
        self.states = []
        self.text = []
        self.speech = []
        self.qr = []
        self.c_zone_direction = []
        self.vlm = []

    def publish_state(self, value):
        self.states.append(value)

    def publish_text(self, value):
        self.text.append(value)

    def publish_speech(self, value):
        self.speech.append(value)

    def publish_qr(self, value):
        self.qr.append(value)

    def publish_c_zone_direction(self, value):
        self.c_zone_direction.append(value)

    def publish_vlm(self, value):
        self.vlm.append(value)


class FakeLocalization:
    def __init__(self):
        self.calls = 0

    def reset_origin(self):
        self.calls += 1
        return OperationResult(True, "ok")


class MissionTests(unittest.TestCase):
    def make_mission(self, navigator=None, vision=None, config=None):
        self.navigator = navigator or FakeNavigator()
        self.vision = vision or FakeVision()
        self.output = FakeOutput()
        self.localization = FakeLocalization()
        return Mission(
            self.navigator,
            self.vision,
            self.localization,
            FakeClock(),
            self.output,
            config or MissionConfig(qr_settle_sec=0.0, navigation_retry_delay_sec=0.0),
        )

    def test_executes_forward_segments_with_native_single_and_through_actions(self):
        mission = self.make_mission()
        p = waypoint("p_start", "start", 0.0)
        a = waypoint("a_task_observe", "qr", 1.0, "precise", "locked")
        via_1 = waypoint("via_1", "via", 2.0)
        c = waypoint("c_corner_1", "vlm", 3.0, "precise", "locked")
        via_2 = waypoint("via_2", "via", 2.0)
        finish = waypoint("p_finish", "return", 0.0)

        result = mission.execute(
            (p, a, via_1, c, via_2, finish),
            navigation_segments=((a,), (via_1, c), (via_2, finish)),
        )

        self.assertTrue(result.success, result.status)
        self.assertEqual(self.navigator.calls, [a])
        self.assertEqual(self.navigator.through_calls, [(via_1, c), (via_2, finish)])
        self.assertEqual(self.vision.ready_calls, [(True, True)])
        self.assertEqual(self.output.text, ["WARD-A", "person"])
        self.assertEqual(mission.state, MissionState.COMPLETED)

    def test_qr_result_selects_the_remaining_c_zone_variant_once(self):
        a = waypoint("a_task_observe", "qr", 1.0, "precise", "locked")
        counterclockwise_via = waypoint("via_2", "via", 2.0)
        counterclockwise_c = waypoint(
            "c_corner_1", "vlm", 3.0, "precise", "locked")
        counterclockwise_return = waypoint("via_4", "via", 2.0)
        finish = waypoint("p_finish", "return", 0.0)
        clockwise_via = waypoint("via_2", "via", 20.0)
        clockwise_c = waypoint(
            "c_corner_1", "vlm", 21.0, "precise", "locked")
        clockwise_return = waypoint("via_4", "via", 22.0)
        counterclockwise_segments = (
            (a,),
            (counterclockwise_via, counterclockwise_c),
            (counterclockwise_return, finish),
        )
        clockwise_segments = (
            (a,),
            (clockwise_via, clockwise_c),
            (clockwise_return, finish),
        )

        mission = self.make_mission(vision=FakeVision(
            qr=OperationResult(True, "ok", "24"),
            vlm=OperationResult(True, "ok", "person"),
        ))
        result = mission.execute(
            (waypoint("p_start", "start", 0.0), a, counterclockwise_via,
             counterclockwise_c, counterclockwise_return, finish),
            counterclockwise_segments,
            {
                "counterclockwise": counterclockwise_segments,
                "clockwise": clockwise_segments,
            },
        )

        self.assertTrue(result.success, result.status)
        self.assertEqual(self.navigator.calls, [a])
        self.assertEqual(
            self.navigator.through_calls,
            [
                (clockwise_via, clockwise_c),
                (clockwise_return, finish),
            ],
        )
        self.assertEqual(self.output.c_zone_direction, ["顺时针"])
        self.assertEqual(self.output.text, ["24", "C区方向：顺时针", "person"])

    def test_odd_or_unrecognized_qr_keeps_the_authored_c_zone_variant(self):
        a = waypoint("a_task_observe", "qr", 1.0, "precise", "locked")
        via = waypoint("via_2", "via", 2.0)
        c = waypoint("c_corner_1", "vlm", 3.0, "precise", "locked")
        return_via = waypoint("via_4", "via", 2.0)
        finish = waypoint("p_finish", "return", 0.0)
        authored_segments = ((a,), (via, c), (return_via, finish))
        variants = {
            "counterclockwise": authored_segments,
            "clockwise": (
                (a,),
                (waypoint("via_2", "via", 20.0), waypoint(
                    "c_corner_1", "vlm", 21.0, "precise", "locked")),
                (waypoint("via_4", "via", 22.0), finish),
            ),
        }
        for payload in ("13", "奇数或偶数"):
            with self.subTest(payload=payload):
                mission = self.make_mission(vision=FakeVision(
                    qr=OperationResult(True, "ok", payload),
                ))
                result = mission.execute(
                    (waypoint("p_start", "start", 0.0), a, via, c,
                     return_via, finish),
                    authored_segments,
                    variants,
                )
                self.assertTrue(result.success, result.status)
                self.assertEqual(
                    self.navigator.through_calls,
                    [(via, c), (return_via, finish)],
                )
                self.assertEqual(self.output.c_zone_direction, ["逆时针"])

    def test_c_zone_variants_fail_closed_when_they_change_nav2_topology(self):
        a = waypoint("a_task_observe", "qr", 1.0, "precise", "locked")
        via = waypoint("via_2", "via", 2.0)
        c = waypoint("c_corner_1", "vlm", 3.0, "precise", "locked")
        return_via = waypoint("via_4", "via", 2.0)
        finish = waypoint("p_finish", "return", 0.0)
        baseline = ((a,), (via, c), (return_via, finish))
        mission = self.make_mission()

        result = mission.execute(
            (waypoint("p_start", "start", 0.0), a, via, c,
             return_via, finish),
            baseline,
            {
                "counterclockwise": baseline,
                "clockwise": ((a,), (via, c), (finish,)),
            },
        )

        self.assertFalse(result.success)
        self.assertEqual(result.status, "mission_exception:ValueError")
        self.assertEqual(self.navigator.calls, [])
        self.assertEqual(self.navigator.through_calls, [])

    def test_rejects_reverse_and_semantic_boundary_segments_before_navigation(self):
        mission = self.make_mission()
        reverse = waypoint("bad", "nav", 1.0, direction="reverse")
        result = mission.execute((waypoint("p", "start", 0.0), reverse), ((reverse,),))
        self.assertEqual(result.status, "navigation_segment_direction_not_forward")
        self.assertEqual(self.navigator.calls, [])

        mission = self.make_mission()
        a = waypoint("a", "qr", 1.0)
        via = waypoint("via", "via", 2.0)
        finish = waypoint("p_finish", "return", 0.0)
        result = mission.execute((waypoint("p", "start", 0.0), a, via, finish), ((a, via, finish),))
        self.assertEqual(result.status, "navigation_segment_semantic_boundary")
        self.assertEqual(self.navigator.calls, [])

    def test_rejects_non_via_intermediates_and_via_endpoints_before_navigation(self):
        mission = self.make_mission()
        nav = waypoint("nav", "nav", 1.0)
        finish = waypoint("p_finish", "return", 2.0)
        result = mission.execute(
            (waypoint("p", "start", 0.0), nav, finish), ((nav, finish),))
        self.assertEqual(result.status, "navigation_segment_intermediate_not_via")
        self.assertEqual(self.navigator.through_calls, [])

        mission = self.make_mission()
        via = waypoint("via", "via", 1.0)
        result = mission.execute((waypoint("p", "start", 0.0), via), ((via,),))
        self.assertEqual(result.status, "navigation_segment_endpoint_is_via")
        self.assertEqual(self.navigator.calls, [])

    def test_navigation_failure_does_not_reissue_the_same_goal(self):
        navigator = FakeNavigator([
            OperationResult(False, "planner_failed"),
        ])
        mission = self.make_mission(
            navigator=navigator,
            config=MissionConfig(qr_settle_sec=0.0, navigation_retry_delay_sec=0.0),
        )
        target = waypoint("p_finish", "return", 0.0)
        result = mission.execute((waypoint("p", "start", 0.0), target), ((target,),))
        self.assertEqual(result.status, "navigation_failed:planner_failed")
        self.assertEqual(len(navigator.calls), 1)
        self.assertEqual(mission.state, MissionState.FAILED)

    def test_navigation_failure_remains_terminal_when_qr_failure_continues(self):
        navigator = FakeNavigator([
            OperationResult(False, "planner_failed"),
        ])
        mission = self.make_mission(
            navigator=navigator,
            config=MissionConfig(
                qr_settle_sec=0.0,
                navigation_retry_delay_sec=0.0,
                continue_after_qr_failure=True,
            ),
        )
        a = waypoint("a_task_observe", "qr", 1.0, "precise", "locked")
        via_1 = waypoint("via_1", "via", 2.0)
        c = waypoint("c_corner_1", "vlm", 3.0, "precise", "locked")
        via_4 = waypoint("via_4", "via", 2.0)
        finish = waypoint("p_finish", "return", 0.0)

        counterclockwise_segments = ((a,), (via_1, c), (via_4, finish))
        clockwise_segments = (
            (a,),
            (via_1, waypoint("c_corner_1", "vlm", 30.0, "precise", "locked")),
            (waypoint("via_4", "via", 31.0), finish),
        )
        result = mission.execute(
            (waypoint("p_start", "start", 0.0), a, via_1, c, via_4, finish),
            counterclockwise_segments,
            {
                "counterclockwise": counterclockwise_segments,
                "clockwise": clockwise_segments,
            },
        )

        self.assertFalse(result.success)
        self.assertEqual(result.status, "navigation_failed:planner_failed")
        self.assertEqual(navigator.calls, [a])
        self.assertEqual(navigator.through_calls, [])
        self.assertEqual(self.vision.qr_calls, [])
        self.assertEqual(self.vision.vlm_calls, [])
        self.assertEqual(mission.state, MissionState.FAILED)

    def test_reset_after_navigation_failure_allows_a_fresh_mission(self):
        navigator = FakeNavigator([
            OperationResult(False, "planner_failed"),
            OperationResult(True, "ok"),
        ])
        mission = self.make_mission(
            navigator=navigator,
            config=MissionConfig(qr_settle_sec=0.0, navigation_retry_delay_sec=0.0),
        )
        target = waypoint("p_finish", "return", 0.0)

        first = mission.execute(
            (waypoint("p", "start", 0.0), target), ((target,),))
        self.assertEqual(first.status, "navigation_failed:planner_failed")
        self.assertEqual(mission.state, MissionState.FAILED)

        reset = mission.reset()
        self.assertTrue(reset.success)
        self.assertEqual(mission.state, MissionState.IDLE)

        second = mission.execute(
            (waypoint("p", "start", 0.0), target), ((target,),))
        self.assertTrue(second.success)
        self.assertEqual(self.localization.calls, 1)
        self.assertEqual(len(navigator.calls), 2)

    def test_vlm_failure_uses_fixed_fallback(self):
        mission = self.make_mission(vision=FakeVision(
            vlm=OperationResult(False, "image_timeout")))
        target = waypoint("c", "vlm", 1.0, "precise", "locked")
        result = mission.execute((waypoint("p", "start", 0.0), target), ((target,),))
        self.assertTrue(result.success)
        self.assertEqual(self.output.text, [VLM_FALLBACK_TEXT])

    def test_qr_failure_can_continue_through_vlm_and_return(self):
        mission = self.make_mission(
            vision=FakeVision(
                qr=OperationResult(False, "qr_timeout"),
                vlm=OperationResult(True, "ok", "person"),
            ),
            config=MissionConfig(
                qr_settle_sec=0.0,
                qr_retries=0,
                navigation_retry_delay_sec=0.0,
                continue_after_qr_failure=True,
            ),
        )
        a = waypoint("a_task_observe", "qr", 1.0, "precise", "locked")
        via_1 = waypoint("via_1", "via", 2.0)
        c = waypoint("c_corner_1", "vlm", 3.0, "precise", "locked")
        via_4 = waypoint("via_4", "via", 2.0)
        finish = waypoint("p_finish", "return", 0.0)

        counterclockwise_segments = ((a,), (via_1, c), (via_4, finish))
        clockwise_segments = (
            (a,),
            (via_1, waypoint("c_corner_1", "vlm", 30.0, "precise", "locked")),
            (waypoint("via_4", "via", 31.0), finish),
        )
        result = mission.execute(
            (waypoint("p_start", "start", 0.0), a, via_1, c, via_4, finish),
            counterclockwise_segments,
            {
                "counterclockwise": counterclockwise_segments,
                "clockwise": clockwise_segments,
            },
        )

        self.assertTrue(result.success, result.status)
        self.assertEqual(result.status, "mission_completed")
        self.assertEqual(self.navigator.calls, [a])
        self.assertEqual(
            self.navigator.through_calls,
            [(via_1, c), (via_4, finish)],
        )
        self.assertEqual(len(self.vision.qr_calls), 1)
        self.assertEqual(len(self.vision.vlm_calls), 1)
        self.assertEqual(self.output.qr, [QR_UNRECOGNIZED_TEXT])
        self.assertEqual(self.output.c_zone_direction, ["逆时针"])
        self.assertEqual(self.output.vlm, ["person"])
        self.assertEqual(
            self.output.text,
            [QR_UNRECOGNIZED_TEXT, "C区方向：逆时针", "person"],
        )
        self.assertEqual(mission.state, MissionState.COMPLETED)

    def test_qr_failure_is_terminal_without_competition_continuation(self):
        mission = self.make_mission(
            vision=FakeVision(qr=OperationResult(False, "qr_timeout")),
            config=MissionConfig(
                qr_settle_sec=0.0,
                qr_retries=0,
                navigation_retry_delay_sec=0.0,
            ),
        )
        a = waypoint("a_task_observe", "qr", 1.0, "precise", "locked")
        finish = waypoint("p_finish", "return", 0.0)

        result = mission.execute(
            (waypoint("p_start", "start", 0.0), a, finish),
            ((a,), (finish,)),
        )

        self.assertFalse(result.success)
        self.assertEqual(result.status, "qr_failed:qr_timeout")
        self.assertEqual(self.navigator.calls, [a])
        self.assertEqual(self.navigator.through_calls, [])
        self.assertEqual(mission.state, MissionState.FAILED)

    def test_continue_after_qr_failure_requires_a_boolean(self):
        with self.assertRaisesRegex(ValueError, "continue_after_qr_failure"):
            MissionConfig(continue_after_qr_failure="true")

    def test_reset_is_allowed_after_terminal_mission(self):
        mission = self.make_mission()
        target = waypoint("p_finish", "return", 0.0)
        self.assertTrue(mission.execute((waypoint("p", "start", 0.0), target), ((target,),)).success)
        result = mission.reset()
        self.assertTrue(result.success)
        self.assertEqual(self.localization.calls, 1)
        self.assertEqual(mission.state, MissionState.IDLE)


if __name__ == "__main__":
    unittest.main()
