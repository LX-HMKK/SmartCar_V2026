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

    def wait_ready(self, qr, vlm, _timeout_sec):
        self.ready_calls.append((qr, vlm))
        return True

    def read_qr(self, _not_before, _timeout_sec):
        return self.qr

    def describe_scene(self, _not_before, _timeout_sec, _prompt):
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

    def publish_state(self, value):
        self.states.append(value)

    def publish_text(self, value):
        self.text.append(value)

    def publish_speech(self, value):
        self.speech.append(value)


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
