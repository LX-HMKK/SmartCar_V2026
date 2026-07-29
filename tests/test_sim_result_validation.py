"""Behavioral tests for complete-route simulation result validation."""

import copy
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
        if goal_profile == "reverse_handoff":
            minimum, maximum = -0.09, -0.02
        checker, _xy_range, _yaw_range = (
            VALIDATION.EXPECTED_GOAL_CONTRACTS[waypoint_id])
        if checker == "precise_goal_checker":
            xy_tolerance, yaw_tolerance = 0.12, 0.15
        elif checker == "reverse_goal_checker":
            xy_tolerance, yaw_tolerance = 0.12, 0.25
        else:
            xy_tolerance, yaw_tolerance = 0.25, 0.50
        entry_yaw_error = 0.40 if goal_profile == "reverse_handoff" else 0.01
        results.append({
            "id": waypoint_id,
            "direction": direction,
            "goal_profile": goal_profile,
            "behavior_tree": VALIDATION.EXPECTED_BEHAVIOR_TREES[waypoint_id],
            "outcome": "succeeded",
            "status": VALIDATION.SUCCEEDED_STATUS,
            "duration_sec": 1.0,
            "goal_error_m": 0.01,
            "goal_yaw_error_rad": 0.01,
            "goal_checker": checker,
            "xy_goal_tolerance_m": xy_tolerance,
            "yaw_goal_tolerance_rad": yaw_tolerance,
            "position_observer_margin_m": (
                VALIDATION.POSITION_OBSERVER_MARGIN_M),
            "yaw_observer_margin_rad": VALIDATION.YAW_OBSERVER_MARGIN_RAD,
            "signed_plan_goal_yaw_error_rad": 0.0,
            "xy_tolerance_entry_yaw_error_rad": entry_yaw_error,
            "post_xy_elapsed_sec": 5.0,
            "post_xy_max_goal_error_m": 0.20,
            "post_xy_travel_m": 0.40,
            "post_xy_controller_cmd_sample_count": 5,
            "post_xy_controller_angular_sample_count": 4,
            "post_xy_cmd_sample_count": 5,
            "post_xy_angular_sample_count": 4,
            "post_xy_yaw_error_reduction_rad": entry_yaw_error - 0.01,
            "path_messages": 1,
            "handoff_speed_cap_mps": (
                0.09 if goal_profile == "reverse_handoff" else None),
            "handoff_wz_cap_radps": (
                0.20 if goal_profile == "reverse_handoff" else None),
            "handoff_min_turning_radius_m": (
                0.55 if goal_profile == "reverse_handoff" else None),
            "handoff_controller_plugin": (
                VALIDATION.REVERSE_HANDOFF_CONTROLLER
                if goal_profile == "reverse_handoff" else None),
            "handoff_internal_vx_min_mps": (
                0.02 if goal_profile == "reverse_handoff" else None),
            "handoff_internal_vx_max_mps": (
                0.09 if goal_profile == "reverse_handoff" else None),
            "velocity_smoother_scale_velocities": (
                True if goal_profile == "reverse_handoff" else None),
            "controller_cmd_linear_min": minimum,
            "controller_cmd_linear_max": maximum,
            "controller_cmd_angular_abs_max": 0.10,
            "controller_cmd_min_turning_radius_m": 0.55,
            "controller_cmd_kinematic_violation_count": 0,
            "cmd_linear_min": minimum,
            "cmd_linear_max": maximum,
            "cmd_angular_abs_max": 0.10,
            "cmd_min_turning_radius_m": 0.55,
            "cmd_kinematic_violation_count": 0,
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


def dynamic_manifest():
    """A user-edited two-segment route with one ThroughPoses stage."""
    source = valid_manifest()
    single_goal = copy.deepcopy(source["results"][4])
    single_goal.update({
        "id": "entry",
        "direction": "forward",
        "goal_profile": "standard",
        "behavior_tree": "navigate_to_pose_w_replanning_and_recovery.xml",
        "goal_checker": "goal_checker",
    })
    through_result = {
        "id": "through_poses[reverse_a, reverse_b]",
        "mode": "through_poses",
        "segment_id": "reverse_loop",
        "direction": "reverse",
        "goal_ids": ["reverse_a", "reverse_b"],
        "goal_profiles": ["standard", "reverse_handoff"],
        "behavior_tree": (
            "navigate_through_poses_reverse_w_replanning_and_recovery.xml"),
        "waypoint_count": 2,
        "outcome": "succeeded",
        "status": VALIDATION.SUCCEEDED_STATUS,
        "duration_sec": 2.0,
        "travel_m": 0.8,
        "path_messages": 1,
        "waypoints_passed": [
            {"id": "reverse_a", "min_distance_m": 0.10},
            {"id": "reverse_b", "min_distance_m": 0.12},
        ],
    }
    source.update({
        "expected_goal_count": 3,
        "results": [single_goal, through_result],
        "route": {
            "segments": [
                {
                    "id": "entry",
                    "direction": "forward",
                    "goals": [
                        {
                            "id": "entry",
                            "direction": "forward",
                            "goal_profile": "standard",
                        },
                    ],
                },
                {
                    "id": "reverse_loop",
                    "direction": "reverse",
                    "goals": [
                        {
                            "id": "reverse_a",
                            "direction": "reverse",
                            "goal_profile": "standard",
                        },
                        {
                            "id": "reverse_b",
                            "direction": "reverse",
                            "goal_profile": "reverse_handoff",
                        },
                    ],
                },
            ],
        },
    })
    return source


class SimResultValidationTests(unittest.TestCase):
    def test_complete_current_run_is_accepted(self):
        self.assertEqual(
            VALIDATION.validate_manifest(valid_manifest(), 199.0), [])

    def test_saved_dynamic_segments_and_through_poses_are_accepted(self):
        manifest = dynamic_manifest()
        self.assertEqual(VALIDATION.validate_manifest(manifest, 199.0), [])

        manifest["results"][1]["goal_ids"] = ["reverse_b", "reverse_a"]
        manifest["results"][1]["waypoints_passed"][1]["min_distance_m"] = 0.8
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("goal_ids" in error for error in errors))
        self.assertTrue(any("min_distance_m" in error for error in errors))

    def test_route_order_and_action_status_are_strict(self):
        manifest = valid_manifest()
        manifest["results"][1], manifest["results"][2] = (
            manifest["results"][2], manifest["results"][1])
        manifest["results"][3]["status"] = 5
        manifest["results"][4]["behavior_tree"] = "wrong.xml"

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("route mismatch" in error for error in errors))
        self.assertTrue(any("status must be 4" in error for error in errors))
        self.assertTrue(any("behavior_tree must be" in error for error in errors))

    def test_velocity_sign_is_checked_per_direction(self):
        manifest = valid_manifest()
        manifest["results"][1]["cmd_linear_max"] = 0.05
        manifest["results"][4]["cmd_linear_min"] = -0.05
        manifest["results"][2]["controller_cmd_linear_max"] = 0.05
        manifest["results"][5]["controller_cmd_linear_min"] = -0.05

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(
            any("contains a forward command" in error for error in errors))
        self.assertTrue(
            any("contains a reverse command" in error for error in errors))
        self.assertTrue(
            any("controller contains a forward command" in error for error in errors))
        self.assertTrue(
            any("controller contains a reverse command" in error for error in errors))

    def test_pose_tolerances_and_planned_yaw_are_strict(self):
        manifest = valid_manifest()
        manifest["results"][0]["goal_yaw_error_rad"] = 0.31
        manifest["results"][1]["goal_error_m"] = 0.20
        manifest["results"][2]["signed_plan_goal_yaw_error_rad"] = 0.20
        manifest["results"][3]["yaw_goal_tolerance_rad"] = 0.50
        manifest["results"][4]["position_observer_margin_m"] = 0.05

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("goal_yaw_error_rad" in error for error in errors))
        self.assertTrue(any("goal_error_m" in error for error in errors))
        self.assertTrue(
            any("planned terminal yaw" in error for error in errors))
        self.assertTrue(
            any("yaw_goal_tolerance_rad must be within" in error for error in errors))
        self.assertTrue(
            any("position observer margin" in error for error in errors))

    def test_tuned_goal_tolerance_is_accepted_within_safe_range(self):
        manifest = valid_manifest()
        precise = manifest["results"][0]
        precise["yaw_goal_tolerance_rad"] = 0.20
        precise["goal_yaw_error_rad"] = 0.19

        self.assertEqual(
            VALIDATION.validate_manifest(manifest, 199.0), [])

    def test_reverse_handoff_proves_post_position_yaw_control(self):
        manifest = valid_manifest()
        handoff = manifest["results"][3]
        handoff["post_xy_controller_cmd_sample_count"] = 0
        handoff["post_xy_controller_angular_sample_count"] = 0
        handoff["post_xy_cmd_sample_count"] = 0
        handoff["post_xy_angular_sample_count"] = 0
        handoff["post_xy_yaw_error_reduction_rad"] = 0.0

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(
            any("controller stopped after XY entry" in error for error in errors))
        self.assertTrue(
            any("controller lacks post-XY steering" in error for error in errors))
        self.assertTrue(
            any("continue controlling after XY entry" in error for error in errors))
        self.assertTrue(
            any("post-XY steering" in error for error in errors))
        self.assertTrue(
            any("yaw did not converge" in error for error in errors))

    def test_reverse_handoff_rejects_terminal_loops(self):
        manifest = valid_manifest()
        handoff = manifest["results"][3]
        handoff["post_xy_max_goal_error_m"] = 1.50
        handoff["post_xy_travel_m"] = 8.0
        handoff["post_xy_elapsed_sec"] = 119.0

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("left the terminal area" in error for error in errors))
        self.assertTrue(any("traveled too far" in error for error in errors))
        self.assertTrue(any("took too long" in error for error in errors))

    def test_reverse_handoff_enforces_both_command_layers(self):
        manifest = valid_manifest()
        handoff = manifest["results"][3]
        handoff["controller_cmd_linear_min"] = -0.12
        handoff["cmd_kinematic_violation_count"] = 1
        handoff["controller_cmd_min_turning_radius_m"] = 0.40

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("exceeds speed cap" in error for error in errors))
        self.assertTrue(
            any("violates Ackermann curvature" in error for error in errors))
        self.assertTrue(
            any("observed turning radius is too small" in error for error in errors))

    def test_reverse_handoff_requires_virtual_forward_runtime_config(self):
        manifest = valid_manifest()
        handoff = manifest["results"][3]
        handoff["handoff_controller_plugin"] = (
            "nav2_mppi_controller::MPPIController")
        handoff["handoff_internal_vx_min_mps"] = -0.09
        handoff["velocity_smoother_scale_velocities"] = False

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("virtual-forward wrapper" in error for error in errors))
        self.assertTrue(any("vx bounds are invalid" in error for error in errors))
        self.assertTrue(any("scale velocities together" in error for error in errors))

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

    def test_handoff_mppi_speed_and_goal_angle_are_tunable(self):
        tuner = TUNE_PARAMS.read_text(encoding="utf-8")
        params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        handoff = params["controller_server"]["ros__parameters"][
            "ReverseHandoff"]

        self.assertIn('"reverse_handoff_vx_max"', tuner)
        self.assertIn('"reverse_handoff_goal_angle_weight"', tuner)
        self.assertNotIn('"reverse_handoff_desired_linear_vel"', tuner)
        self.assertNotIn('"reverse_handoff_lookahead_dist"', tuner)
        self.assertEqual(
            handoff["plugin"], "smartcar_nav2::ReverseOnlyMPPIController"
        )
        self.assertAlmostEqual(handoff["vx_min"], 0.02)
        self.assertAlmostEqual(handoff["vx_max"], 0.09)
        self.assertLess(handoff["vx_min"], handoff["vx_max"])
        self.assertIs(
            params["velocity_smoother"]["ros__parameters"]["scale_velocities"],
            True,
        )
        self.assertAlmostEqual(
            handoff["GoalAngleCritic"]["cost_weight"], 12.0
        )


if __name__ == "__main__":
    unittest.main()
