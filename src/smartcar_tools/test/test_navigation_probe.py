"""ROS-independent contracts for the fast navigation probe."""
import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_tools.navigation_probe import (  # noqa: E402
    SEQUENCE_FAILED,
    START_STEPS,
    SUCCESS,
    StepResult,
    build_parser,
    run_start_sequence,
)


class NavigationProbeTests(unittest.TestCase):
    def test_start_is_explicit_and_run_is_an_alias(self):
        self.assertEqual(build_parser().parse_args(["start"]).command, "start")
        self.assertEqual(build_parser().parse_args(["run"]).command, "run")
        with self.assertRaises(SystemExit):
            build_parser().parse_args([])

    def test_success_uses_one_strict_sequence_without_stop(self):
        calls = []

        def call(step):
            calls.append(step)
            return StepResult(True, "ok")

        code = run_start_sequence(call, lambda: calls.append("stop"))
        self.assertEqual(code, SUCCESS)
        self.assertEqual(calls, list(START_STEPS))

    def test_failure_short_circuits_and_stops(self):
        calls = []

        def call(step):
            calls.append(step)
            return StepResult(step != "arm", "rejected")

        code = run_start_sequence(call, lambda: calls.append("stop"))
        self.assertEqual(code, SEQUENCE_FAILED)
        self.assertEqual(calls, ["prepare", "arm", "stop"])

    def test_exception_stops_and_propagates(self):
        calls = []

        def call(step):
            calls.append(step)
            if step == "start":
                raise KeyboardInterrupt
            return StepResult(True, "ok")

        with self.assertRaises(KeyboardInterrupt):
            run_start_sequence(call, lambda: calls.append("stop"))
        self.assertEqual(calls, ["prepare", "arm", "start", "stop"])


if __name__ == "__main__":
    unittest.main()
