"""Behavioral tests for complete-route simulation result validation."""

import importlib.util
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = (
    ROOT / "src" / "smartcar_sim" / "scripts"
    / "validate_sim_results.py"
)
SIM_TUNE = ROOT / "src" / "smartcar_sim" / "scripts" / "sim_tune.sh"
TUNE_PARAMS = (
    ROOT / "src" / "smartcar_sim" / "scripts" / "tune_params.py"
)
NAV2_PARAMS = (
    ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
)

SPEC = importlib.util.spec_from_file_location(
    "validate_sim_results", VALIDATOR)
VALIDATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATION)


def valid_manifest():
    results = []
    for waypoint_id, direction, goal_profile in VALIDATION.EXPECTED_ROUTE:
        if direction == "reverse":
            minimum, maximum = -0.1, -0.02
        else:
            minimum, maximum = 0.02, 0.1
        results.append({
            "id": waypoint_id,
            "direction": direction,
            "goal_profile": goal_profile,
            "outcome": "succeeded",
            "status": VALIDATION.SUCCEEDED_STATUS,
            "duration_sec": 1.0,
            "goal_error_m": 0.01,
            "goal_yaw_error_rad": 0.01,
            "path_messages": 1,
            "cmd_linear_min": minimum,
            "cmd_linear_max": maximum,
            "contract_errors": [],
        })
    inputs = {
        name: {
            "path": (
                "/tmp/nav2_params_fixed.yaml"
                if name == "nav2_params_file"
                else f"/tmp/{name}.xml"
            ),
            "sha256": "a" * 64,
        }
        for name in VALIDATION.REQUIRED_INPUTS
    }
    return {
        "overall_outcome": "completed",
        "expected_goal_count": len(VALIDATION.EXPECTED_ROUTE),
        "results": results,
        "inputs": inputs,
        "timestamp": 200.0,
    }


class SimResultValidationTests(unittest.TestCase):
    def test_complete_current_run_is_accepted(self):
        self.assertEqual(
            VALIDATION.validate_manifest(valid_manifest(), 199.0), [])

    def test_route_order_and_action_status_are_strict(self):
        manifest = valid_manifest()
        manifest["results"][1], manifest["results"][2] = (
            manifest["results"][2], manifest["results"][1])
        manifest["results"][3]["status"] = 5

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("route mismatch" in error for error in errors))
        self.assertTrue(any("status must be 4" in error for error in errors))

    def test_velocity_sign_is_checked_per_direction(self):
        manifest = valid_manifest()
        manifest["results"][1]["cmd_linear_max"] = 0.05
        manifest["results"][4]["cmd_linear_min"] = -0.05

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(
            any("contains a forward command" in error for error in errors))
        self.assertTrue(
            any("contains a reverse command" in error for error in errors))

    def test_stale_or_untraceable_manifest_is_rejected(self):
        manifest = valid_manifest()
        manifest["timestamp"] = 100.0
        manifest["inputs"]["waypoints_file"]["sha256"] = "invalid"
        manifest["inputs"]["nav2_params_file"]["path"] = (
            "/tmp/nav2_params.yaml")

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertIn("timestamp predates this simulation run", errors)
        self.assertTrue(any("SHA256" in error for error in errors))
        self.assertTrue(any("nav2_params_fixed.yaml" in error for error in errors))

    def test_tuning_runner_uses_the_shared_validator(self):
        source = SIM_TUNE.read_text(encoding="utf-8")
        self.assertIn("validate_sim_results.py", source)
        self.assertIn('--started-after "$run_started_epoch"', source)
        self.assertNotIn("overall_outcome=", source)

    def test_handoff_speed_and_fixed_lookahead_are_tunable(self):
        tuner = TUNE_PARAMS.read_text(encoding="utf-8")
        params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        handoff = params["controller_server"]["ros__parameters"][
            "ReverseHandoff"]

        self.assertIn('"reverse_handoff_desired_linear_vel"', tuner)
        self.assertIn('"reverse_handoff_lookahead_dist"', tuner)
        self.assertFalse(handoff["use_velocity_scaled_lookahead_dist"])
        self.assertAlmostEqual(handoff["desired_linear_vel"], 0.09)
        self.assertAlmostEqual(handoff["lookahead_dist"], 0.25)


if __name__ == "__main__":
    unittest.main()
