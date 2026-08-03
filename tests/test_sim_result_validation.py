"""Behavioral tests for complete-route simulation result validation."""

import copy
from contextlib import contextmanager
import hashlib
import importlib.util
import unittest
from unittest import mock
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
NAV_ONLY_WAYPOINTS = (
    ROOT / "src" / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml"
)

SPEC = importlib.util.spec_from_file_location(
    "validate_sim_results", VALIDATOR)
VALIDATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATION)


def planned_path_fields(chord=1.0, length=1.0):
    return {
        "planned_path_max_length_m": length,
        "planned_path_max_chord_m": chord,
        "planned_path_max_detour_ratio": length / max(chord, 1.0e-3),
    }


def executed_travel_fields(baseline=1.0, travel=1.0):
    limit = max(
        baseline * VALIDATION.MAX_EXECUTED_TRAVEL_DETOUR_RATIO,
        baseline + VALIDATION.MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M,
    )
    if baseline > 1.0e-3:
        ratio = travel / baseline
    elif travel > VALIDATION.MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M:
        ratio = VALIDATION.MAX_EXECUTED_TRAVEL_DETOUR_RATIO + 1.0
    else:
        ratio = 0.0
    return {
        "travel_m": travel,
        "executed_travel_m": travel,
        "executed_travel_baseline_m": baseline,
        "executed_travel_detour_ratio": ratio,
        "executed_travel_limit_m": limit,
        "executed_travel_detour_violation": False,
    }


def forward_ackermann_fields(direction, goal_profile="standard"):
    if direction != "forward":
        return {
            "forward_speed_cap_mps": None,
            "forward_wz_cap_radps": None,
            "forward_min_turning_radius_m": None,
            "forward_path_max_cross_track_error_m": None,
            "forward_controller_plugin": None,
            "forward_velocity_smoother_scale_velocities": None,
        }
    return {
        "forward_speed_cap_mps": VALIDATION.SIMULATION_FORWARD_SPEED_CAP_MPS,
        "forward_wz_cap_radps": VALIDATION.SIMULATION_FORWARD_WZ_CAP_RADPS,
        "forward_min_turning_radius_m": (
            VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M),
        "forward_path_max_cross_track_error_m": (
            None if goal_profile == "precise"
            else VALIDATION.SIMULATION_FORWARD_PATH_MAX_CROSS_TRACK_ERROR_M),
        "forward_controller_plugin": (
            VALIDATION.NATIVE_RPP_CONTROLLER
            if goal_profile == "precise"
            else VALIDATION.FORWARD_AVOIDANCE_CONTROLLER
        ),
        "forward_velocity_smoother_scale_velocities": True,
    }


def valid_perception():
    return {
        "schema_version": VALIDATION.PERCEPTION_STATUS_SCHEMA_VERSION,
        "topic": VALIDATION.PERCEPTION_READY_TOPIC,
        "ready": True,
        "checks": {
            name: True for name in VALIDATION.PERCEPTION_REQUIRED_CHECKS
        },
        "valid_beams": 101,
        "scan_stamp_ns": 104_900_000_000,
        "odom_stamp_ns": 104_965_000_000,
        "tf_odom_position_error_m": 0.001,
        "tf_odom_yaw_error_rad": 0.002,
        "tf_odom_bracket_span_sec": 0.033,
        "landmark_required_ids": 3,
        "landmark_matched_ids": ["cone_a1", "cone_a2", "cone_a3"],
        "landmark_matched_points": 28,
        "landmark_max_residual_m": 0.012,
        "landmark_current_expected_ids": ["cone_a1", "cone_a2", "cone_a3"],
        "landmark_current_matched_ids": ["cone_a1", "cone_a2", "cone_a3"],
        "landmark_current_matched_points": 28,
        "landmark_current_max_residual_m": 0.012,
        "landmark_current_valid": True,
        "local_costmap_stamp_ns": 104_980_000_000,
        "global_costmap_stamp_ns": 104_095_000_000,
        "received_monotonic_sec": 100.0,
        "status_age_sec": 0.2,
    }


def valid_tracking_trace():
    return {
        "schema_version": VALIDATION.EXECUTION_TRACE_SCHEMA_VERSION,
        "sample_limit": VALIDATION.MAX_EXECUTION_TRACE_SAMPLES,
        "accepted_path_count": 1,
        "odom_combined": [
            {
                "t_sec": 0.1,
                "x": 0.1,
                "y": 0.0,
                "yaw_rad": 0.0,
                "accepted_path_sequence": 1,
                "station_m": 0.1,
                "cross_track_m": 0.0,
                "path_heading_error_rad": 0.0,
                "path_segment_index": 0,
            },
        ],
        "cmd_vel_nav": [
            {"t_sec": 0.1, "linear_x": 0.1, "angular_z": 0.0},
        ],
        "cmd_vel_candidate": [
            {"t_sec": 0.1, "linear_x": 0.1, "angular_z": 0.0},
        ],
    }


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
            **planned_path_fields(),
            **executed_travel_fields(),
            **forward_ackermann_fields(direction, goal_profile),
            "handoff_speed_cap_mps": (
                VALIDATION.SIMULATION_HANDOFF_SPEED_CAP_MPS
                if goal_profile == "reverse_handoff" else None),
            "handoff_wz_cap_radps": (
                VALIDATION.SIMULATION_HANDOFF_WZ_CAP_RADPS
                if goal_profile == "reverse_handoff" else None),
            "handoff_min_turning_radius_m": (
                VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M
                if goal_profile == "reverse_handoff" else None),
            "handoff_controller_plugin": (
                VALIDATION.REVERSE_HANDOFF_CONTROLLER
                if goal_profile == "reverse_handoff" else None),
            "handoff_internal_vx_min_mps": (
                0.02 if goal_profile == "reverse_handoff" else None),
            "handoff_internal_vx_max_mps": (
                VALIDATION.SIMULATION_HANDOFF_SPEED_CAP_MPS
                if goal_profile == "reverse_handoff" else None),
            "velocity_smoother_scale_velocities": (
                True if goal_profile == "reverse_handoff" else None),
            "controller_cmd_linear_min": minimum,
            "controller_cmd_linear_max": maximum,
            "controller_cmd_angular_abs_max": 0.10,
            "controller_cmd_min_turning_radius_m": (
                VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M),
            "controller_cmd_kinematic_violation_count": 0,
            "cmd_linear_min": minimum,
            "cmd_linear_max": maximum,
            "cmd_angular_abs_max": 0.10,
            "cmd_min_turning_radius_m": (
                VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M),
            "cmd_kinematic_violation_count": 0,
            "tracking_trace": (
                valid_tracking_trace()
                if direction == "forward" and goal_profile == "precise"
                else None
            ),
            "contract_errors": [],
        })
    inputs = {
        name: {
            "path": (
                "/tmp/nav2_params_fixed.yaml"
                if name == "nav2_params_file"
                else f"/tmp/{name}.xml"
            ),
            "realpath": (
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
        "perception": valid_perception(),
        "timestamp": 200.0,
    }


@contextmanager
def reverse_handoff_manifest():
    """Yield one isolated legacy handoff result without changing C1's route."""
    waypoint_id = "reverse_handoff_fixture"
    route = ((waypoint_id, "reverse", "reverse_handoff"),)
    goal_contracts = dict(VALIDATION.EXPECTED_GOAL_CONTRACTS)
    goal_contracts[waypoint_id] = (
        "reverse_goal_checker", (0.08, 0.20), (0.10, 0.30)
    )
    behavior_trees = dict(VALIDATION.EXPECTED_BEHAVIOR_TREES)
    behavior_trees[waypoint_id] = (
        "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml"
    )
    with mock.patch.object(VALIDATION, "EXPECTED_ROUTE", route), \
            mock.patch.object(
                VALIDATION, "EXPECTED_GOAL_CONTRACTS", goal_contracts
            ), \
            mock.patch.object(
                VALIDATION, "EXPECTED_BEHAVIOR_TREES", behavior_trees
            ):
        yield valid_manifest()


def dynamic_manifest():
    """A user-edited two-segment route with one ThroughPoses stage."""
    source = valid_manifest()
    single_goal = copy.deepcopy(source["results"][0])
    single_goal.update({
        "id": "entry",
        "direction": "forward",
        "goal_profile": "standard",
        "heading_mode": "locked",
        "behavior_tree": "navigate_to_pose_w_replanning_and_recovery.xml",
        "goal_checker": "goal_checker",
        "controller_cmd_linear_min": 0.02,
        "controller_cmd_linear_max": 0.10,
        "cmd_linear_min": 0.02,
        "cmd_linear_max": 0.10,
    })
    single_goal.update(forward_ackermann_fields("forward", "standard"))
    through_result = {
        "id": "through_poses[reverse_a, reverse_b]",
        "mode": "through_poses",
        "segment_id": "reverse_loop",
        "direction": "reverse",
        "heading_mode": "locked",
        "goal_ids": ["reverse_a", "reverse_b"],
        "goal_profiles": ["standard", "standard"],
        "behavior_tree": (
            "navigate_through_poses_reverse_locked_sim_"
            "w_replanning_and_recovery.xml"),
        "waypoint_count": 2,
        "outcome": "succeeded",
        "status": VALIDATION.SUCCEEDED_STATUS,
        "duration_sec": 2.0,
        "path_messages": 1,
        **planned_path_fields(),
        **executed_travel_fields(baseline=0.8, travel=0.8),
        "goal_checker": "reverse_goal_checker",
        "xy_goal_tolerance_m": 0.12,
        "yaw_goal_tolerance_rad": 0.25,
        "position_observer_margin_m": (
            VALIDATION.POSITION_OBSERVER_MARGIN_M),
        "yaw_observer_margin_rad": VALIDATION.YAW_OBSERVER_MARGIN_RAD,
        "goal_error_m": 0.01,
        "goal_yaw_error_rad": 0.01,
        "controller_cmd_linear_min": -0.10,
        "controller_cmd_linear_max": -0.02,
        "cmd_linear_min": -0.10,
        "cmd_linear_max": -0.02,
        "contract_errors": [],
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
                            "task": "nav",
                            "direction": "forward",
                            "goal_profile": "standard",
                            "heading_mode": "locked",
                            "frame_id": "odom_combined",
                            "pose": {
                                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "orientation": {
                                    "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0,
                                },
                            },
                        },
                    ],
                },
                {
                    "id": "reverse_loop",
                    "direction": "reverse",
                    "goals": [
                        {
                            "id": "reverse_a",
                            "task": "via",
                            "direction": "reverse",
                            "goal_profile": "standard",
                            "heading_mode": "locked",
                            "frame_id": "odom_combined",
                            "pose": {
                                "position": {"x": 1.0, "y": 1.0, "z": 0.0},
                                "orientation": {
                                    "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0,
                                },
                            },
                        },
                        {
                            "id": "reverse_b",
                            "task": "loop",
                            "direction": "reverse",
                            "goal_profile": "standard",
                            "heading_mode": "locked",
                            "frame_id": "odom_combined",
                            "pose": {
                                "position": {"x": 2.0, "y": 2.0, "z": 0.0},
                                "orientation": {
                                    "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0,
                                },
                            },
                        },
                    ],
                },
            ],
        },
        "execution": {"use_through_poses": True},
    })
    return source


def _single_goal_result(goal):
    direction = goal["direction"]
    goal_profile = goal["goal_profile"]
    heading_mode = goal["heading_mode"]
    if direction == "reverse":
        minimum, maximum = -0.10, -0.02
    else:
        minimum, maximum = 0.02, 0.10
    if heading_mode == "free":
        xy_tolerance, yaw_tolerance = 0.35, None
    elif goal_profile == "precise":
        xy_tolerance, yaw_tolerance = 0.12, 0.15
    else:
        xy_tolerance, yaw_tolerance = 0.12, 0.25
    return {
        "id": goal["id"],
        "direction": direction,
        "goal_profile": goal_profile,
        "heading_mode": heading_mode,
        "behavior_tree": VALIDATION._goal_behavior_tree(
            direction, goal_profile),
        "outcome": "succeeded",
        "status": VALIDATION.SUCCEEDED_STATUS,
        "goal_checker": VALIDATION._goal_checker(
            direction, goal_profile, heading_mode
        ),
        "xy_goal_tolerance_m": xy_tolerance,
        "yaw_goal_tolerance_rad": yaw_tolerance,
        "position_observer_margin_m": (
            VALIDATION.POSITION_OBSERVER_MARGIN_M),
        "yaw_observer_margin_rad": VALIDATION.YAW_OBSERVER_MARGIN_RAD,
        "goal_error_m": 0.01,
        "target_yaw_rad": 0.0 if heading_mode == "locked" else None,
        "goal_yaw_error_rad": 0.01 if heading_mode == "locked" else None,
        "signed_goal_yaw_error_rad": (
            0.01 if heading_mode == "locked" else None
        ),
        "signed_plan_goal_yaw_error_rad": (
            0.0 if heading_mode == "locked" else None
        ),
        "path_messages": 1,
        **planned_path_fields(),
        **executed_travel_fields(),
        **forward_ackermann_fields(direction, goal_profile),
        "controller_cmd_linear_min": minimum,
        "controller_cmd_linear_max": maximum,
        "controller_cmd_angular_abs_max": 0.10,
        "controller_cmd_min_turning_radius_m": (
            VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M),
        "controller_cmd_kinematic_violation_count": 0,
        "cmd_linear_min": minimum,
        "cmd_linear_max": maximum,
        "cmd_angular_abs_max": 0.10,
        "cmd_min_turning_radius_m": (
            VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M),
        "cmd_kinematic_violation_count": 0,
        "tracking_trace": (
            valid_tracking_trace()
            if direction == "forward" and goal_profile == "precise"
            else None
        ),
        "contract_errors": [],
    }


def _through_poses_result(stage):
    segment_id, direction, goals = stage
    heading_mode = goals[-1]["heading_mode"]
    terminal_task = goals[-1]["task"]
    reverse_return = VALIDATION._is_reverse_return_terminal(
        direction, heading_mode, terminal_task)
    minimum, maximum = (-0.10, -0.02) if direction == "reverse" else (0.02, 0.10)
    return {
        "id": "through_poses[{}]".format(
            ", ".join(goal["id"] for goal in goals)),
        "mode": "through_poses",
        "segment_id": segment_id,
        "direction": direction,
        "heading_mode": heading_mode,
        "goal_ids": [goal["id"] for goal in goals],
        "goal_profiles": [goal["goal_profile"] for goal in goals],
        "behavior_tree": VALIDATION._through_poses_behavior_tree(
            direction, heading_mode, terminal_task),
        "waypoint_count": len(goals),
        "outcome": "succeeded",
        "status": VALIDATION.SUCCEEDED_STATUS,
        "goal_checker": VALIDATION._goal_checker(
            direction, "standard", heading_mode, reverse_return
        ),
        "xy_goal_tolerance_m": (
            0.15 if reverse_return else (
                0.35 if heading_mode == "free" else 0.12)),
        "yaw_goal_tolerance_rad": (
            0.15 if reverse_return else (
                None if heading_mode == "free" else 0.25)),
        "position_observer_margin_m": (
            VALIDATION.POSITION_OBSERVER_MARGIN_M),
        "yaw_observer_margin_rad": VALIDATION.YAW_OBSERVER_MARGIN_RAD,
        "goal_error_m": 0.01,
        "target_yaw_rad": 0.0 if heading_mode == "locked" else None,
        "goal_yaw_error_rad": 0.01 if heading_mode == "locked" else None,
        "signed_goal_yaw_error_rad": (
            0.01 if heading_mode == "locked" else None
        ),
        "path_messages": 1,
        **planned_path_fields(),
        **executed_travel_fields(),
        **forward_ackermann_fields(direction, "standard"),
        "controller_cmd_linear_min": minimum,
        "controller_cmd_linear_max": maximum,
        "controller_cmd_angular_abs_max": 0.10,
        "controller_cmd_min_turning_radius_m": (
            VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M),
        "controller_cmd_kinematic_violation_count": 0,
        "cmd_linear_min": minimum,
        "cmd_linear_max": maximum,
        "cmd_angular_abs_max": 0.10,
        "cmd_min_turning_radius_m": (
            VALIDATION.SIMULATION_MINIMUM_TURNING_RADIUS_M),
        "cmd_kinematic_violation_count": 0,
        "waypoints_passed": [
            {"id": goal["id"], "min_distance_m": 0.10}
            for goal in goals
        ],
        "contract_errors": [],
    }


def manifest_from_waypoint_snapshot(snapshot):
    errors = []
    stages = VALIDATION._reconstructed_action_stages(snapshot, errors)
    if errors or stages is None:
        raise AssertionError(f"cannot materialize test snapshot: {errors}")
    results = []
    for stage in stages:
        if len(stage[2]) == 1:
            results.append(_single_goal_result(stage[2][0]))
        else:
            results.append(_through_poses_result(stage))
    manifest = {
        "overall_outcome": "completed",
        "expected_goal_count": sum(len(stage[2]) for stage in stages),
        "results": results,
        "route": {
            "segments": [
                {
                    "id": segment_id,
                    "direction": direction,
                    "goals": [
                        {
                            key: copy.deepcopy(value)
                            for key, value in goal.items()
                        }
                        for goal in goals
                    ],
                }
                for segment_id, direction, goals in stages
            ],
        },
        "execution": {"use_through_poses": True},
        "inputs": valid_manifest()["inputs"],
        "perception": valid_perception(),
        "timestamp": 200.0,
    }
    manifest["inputs"]["waypoints_file"].update({
        "path": "/run/source/nav_only.yaml",
        "realpath": "/run/source/nav_only.yaml",
        "sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    })
    return manifest, stages


class SimResultValidationTests(unittest.TestCase):
    def test_complete_current_run_is_accepted(self):
        self.assertEqual(
            VALIDATION.validate_manifest(valid_manifest(), 199.0), [])

    def test_precise_forward_result_requires_bounded_linked_tracking_evidence(self):
        manifest = valid_manifest()
        precise = manifest["results"][0]
        precise.pop("tracking_trace")
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any(
            "lacks forward path tracking evidence" in error
            for error in errors
        ))

        precise["tracking_trace"] = valid_tracking_trace()
        precise["tracking_trace"]["sample_limit"] = (
            VALIDATION.MAX_EXECUTION_TRACE_SAMPLES + 1)
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any(
            "tracking trace sample_limit" in error for error in errors
        ))

        precise["tracking_trace"] = valid_tracking_trace()
        precise["tracking_trace"]["odom_combined"][0][
            "accepted_path_sequence"
        ] = None
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any(
            "cannot associate odometry to a path" in error
            for error in errors
        ))

        precise["tracking_trace"] = valid_tracking_trace()
        precise["forward_path_max_cross_track_error_m"] = 0.12
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any(
            "must not use a custom path guard" in error for error in errors
        ))

    def test_saved_dynamic_segments_and_through_poses_are_accepted(self):
        manifest = dynamic_manifest()
        self.assertEqual(VALIDATION.validate_manifest(manifest, 199.0), [])

        manifest["results"][1]["goal_ids"] = ["reverse_b", "reverse_a"]
        manifest["results"][1]["waypoints_passed"][1]["min_distance_m"] = 0.8
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("goal_ids" in error for error in errors))
        self.assertTrue(any("min_distance_m" in error for error in errors))

    def test_multi_goal_fallback_cannot_pass_as_a_through_poses_run(self):
        manifest = dynamic_manifest()
        first = copy.deepcopy(manifest["results"][0])
        reverse_a = copy.deepcopy(first)
        reverse_b = copy.deepcopy(first)
        for result, waypoint_id in (
            (reverse_a, "reverse_a"),
            (reverse_b, "reverse_b"),
        ):
            result.update({
                "id": waypoint_id,
                "direction": "reverse",
                "goal_profile": "standard",
                "behavior_tree": (
                    "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
                "goal_checker": "reverse_goal_checker",
                "controller_cmd_linear_min": -0.10,
                "controller_cmd_linear_max": -0.02,
                "cmd_linear_min": -0.10,
                "cmd_linear_max": -0.02,
            })
        manifest["results"] = [first, reverse_a, reverse_b]

        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any("must use NavigateThroughPoses" in error for error in errors))

    def test_waypoint_snapshot_reconstructs_the_complete_action_route(self):
        manifest, stages = manifest_from_waypoint_snapshot(NAV_ONLY_WAYPOINTS)

        self.assertEqual(
            [stage[0] for stage in stages],
            [
                "action_1_a_task_observe_to_a_task_observe",
                "action_2_via_2_to_c_corner_1",
                "action_3_via_1_to_p_finish",
            ],
        )
        self.assertEqual(
            VALIDATION.validate_manifest(
                manifest, 199.0, NAV_ONLY_WAYPOINTS), [])

        self.assertEqual(
            manifest["results"][1]["behavior_tree"],
            "navigate_through_poses_reverse_locked_sim_"
            "w_replanning_and_recovery.xml",
        )
        returned = manifest["results"][2]
        self.assertEqual(
            returned["behavior_tree"],
            "navigate_through_poses_reverse_return_sim_"
            "w_replanning_and_recovery.xml",
        )
        self.assertEqual(returned["goal_checker"], "return_goal_checker")
        self.assertEqual(returned["xy_goal_tolerance_m"], 0.15)
        self.assertEqual(returned["yaw_goal_tolerance_rad"], 0.15)

        returned["goal_checker"] = "reverse_goal_checker"
        returned["xy_goal_tolerance_m"] = 0.35
        returned["yaw_goal_tolerance_rad"] = 0.50
        errors = VALIDATION.validate_manifest(manifest, 199.0, NAV_ONLY_WAYPOINTS)
        self.assertTrue(any("goal_checker" in error for error in errors))
        self.assertTrue(any("xy_goal_tolerance_m" in error for error in errors))
        self.assertTrue(any("yaw_goal_tolerance_rad" in error for error in errors))
        returned["goal_checker"] = "return_goal_checker"
        returned["xy_goal_tolerance_m"] = 0.15
        returned["yaw_goal_tolerance_rad"] = 0.15

        manifest["inputs"]["waypoints_file"]["sha256"] = "b" * 64
        hash_errors = VALIDATION.validate_manifest(
            manifest, 199.0, NAV_ONLY_WAYPOINTS)
        self.assertTrue(any(
            "SHA256 does not match" in error for error in hash_errors))
        manifest["inputs"]["waypoints_file"]["sha256"] = (
            hashlib.sha256(NAV_ONLY_WAYPOINTS.read_bytes()).hexdigest())

        # A start_goal_id/end_goal_id trial writes a truncated manifest.  It
        # cannot become evidence for a complete-route run.
        manifest["route"]["segments"] = manifest["route"]["segments"][:1]
        manifest["results"] = manifest["results"][:1]
        manifest["expected_goal_count"] = 1
        errors = VALIDATION.validate_manifest(
            manifest, 199.0, NAV_ONLY_WAYPOINTS)

        self.assertTrue(any(
            "every reconstructed action" in error for error in errors))
        self.assertTrue(any(
            "expected_goal_count must be" in error for error in errors))

    def test_waypoint_snapshot_requires_locked_semantic_quaternions(self):
        manifest, stages = manifest_from_waypoint_snapshot(NAV_ONLY_WAYPOINTS)
        reverse_stage = stages[1]
        self.assertEqual(len(reverse_stage[2]), 2)
        source = yaml.safe_load(NAV_ONLY_WAYPOINTS.read_text(encoding="utf-8"))
        source_goal = next(
            waypoint for waypoint in source["waypoints"]
            if waypoint["id"] == "c_corner_1")
        self.assertIn("orientation", source_goal["pose"])
        first_reverse_goal = manifest["route"]["segments"][1]["goals"][-1]
        self.assertEqual(first_reverse_goal["id"], "c_corner_1")
        self.assertEqual(first_reverse_goal["heading_mode"], "locked")
        self.assertEqual(
            first_reverse_goal["pose"]["orientation"],
            source_goal["pose"]["orientation"],
        )
        finish_goal = manifest["route"]["segments"][-1]["goals"][-1]
        self.assertEqual(finish_goal["id"], "p_finish")
        self.assertAlmostEqual(
            sum(
                component * component
                for component in finish_goal["pose"]["orientation"].values()
            ),
            1.0,
        )

        first_reverse_goal["pose"]["orientation"] = {
            "x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0,
        }
        errors = VALIDATION.validate_manifest(
            manifest, 199.0, NAV_ONLY_WAYPOINTS)

        self.assertTrue(any(
            "must be a unit quaternion" in error
            for error in errors
        ))

    def test_heading_mode_controls_yaw_evidence_contract(self):
        # The production nav-only snapshot intentionally uses a locked
        # `task: nav` proxy at the QR position.  Exercise the free-terminal
        # evidence rule with an isolated dynamic route instead of assuming
        # the first production action remains position-only.
        manifest = dynamic_manifest()
        free_goal = manifest["route"]["segments"][0]["goals"][0]
        free_goal["heading_mode"] = "free"
        free_goal["pose"]["orientation"] = {
            "x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0,
        }
        free_result = manifest["results"][0]
        free_result["heading_mode"] = "free"
        free_result["goal_checker"] = "transit_goal_checker"
        free_result["xy_goal_tolerance_m"] = 0.35
        free_result["yaw_goal_tolerance_rad"] = None
        free_result["target_yaw_rad"] = None
        free_result["goal_yaw_error_rad"] = None
        free_result["signed_goal_yaw_error_rad"] = None
        free_result["signed_plan_goal_yaw_error_rad"] = None
        self.assertEqual(free_result["heading_mode"], "free")
        self.assertIsNone(free_result["target_yaw_rad"])
        self.assertIsNone(free_result["goal_yaw_error_rad"])
        self.assertIsNone(free_result["signed_goal_yaw_error_rad"])
        self.assertEqual(
            VALIDATION.validate_manifest(
                manifest, 199.0), []
        )

        free_result["xy_goal_tolerance_m"] = 0.50
        self.assertEqual(VALIDATION.validate_manifest(manifest, 199.0), [])

        free_result["xy_goal_tolerance_m"] = 0.501
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any(
            "xy_goal_tolerance_m exceeds its safe contract" in error
            for error in errors
        ))

        free_result["xy_goal_tolerance_m"] = 0.35
        free_result["yaw_goal_tolerance_rad"] = 0.25
        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any(
            "must not report yaw_goal_tolerance_rad" in error
            for error in errors
        ))

        free_result["yaw_goal_tolerance_rad"] = None
        free_result["goal_yaw_error_rad"] = 0.10
        errors = VALIDATION.validate_manifest(
            manifest, 199.0)
        self.assertTrue(any(
            "free-heading goal must not report goal_yaw_error_rad" in error
            for error in errors
        ))

        manifest = dynamic_manifest()
        locked_result = manifest["results"][0]
        self.assertEqual(locked_result["heading_mode"], "locked")
        locked_result["goal_yaw_error_rad"] = 0.50
        errors = VALIDATION.validate_manifest(
            manifest, 199.0)
        self.assertTrue(any(
            "final goal yaw is outside tolerance" in error
            for error in errors
        ))

    def test_dynamic_through_poses_accepts_terminal_locked_reverse_handoff(self):
        manifest = dynamic_manifest()
        manifest["route"]["segments"][1]["goals"][-1]["goal_profile"] = (
            "reverse_handoff")
        manifest["results"][1]["goal_profiles"][1] = "reverse_handoff"

        self.assertEqual(VALIDATION.validate_manifest(manifest, 199.0), [])

        manifest["results"][1]["behavior_tree"] = (
            "navigate_through_poses_reverse_w_replanning_and_recovery.xml")
        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any(
            "navigate_through_poses_reverse_locked_sim_"
            "w_replanning_and_recovery.xml" in error
            for error in errors
        ))

    def test_dynamic_through_poses_rejects_nonterminal_or_unlocked_handoff(self):
        manifest = dynamic_manifest()
        manifest["route"]["segments"][1]["goals"][0]["goal_profile"] = (
            "reverse_handoff")
        manifest["results"][1]["goal_profiles"][0] = "reverse_handoff"

        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any("nonstandard goal profile" in error for error in errors))

        manifest = dynamic_manifest()
        terminal = manifest["route"]["segments"][1]["goals"][-1]
        terminal["goal_profile"] = "reverse_handoff"
        terminal["heading_mode"] = "free"
        terminal["pose"]["orientation"] = {
            "x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0,
        }
        through = manifest["results"][1]
        through.update({
            "heading_mode": "free",
            "goal_profiles": ["standard", "reverse_handoff"],
            "behavior_tree": (
                "navigate_through_poses_reverse_w_replanning_and_recovery.xml"),
            "goal_checker": "transit_goal_checker",
            "xy_goal_tolerance_m": 0.35,
            "yaw_goal_tolerance_rad": None,
            "target_yaw_rad": None,
            "goal_yaw_error_rad": None,
            "signed_goal_yaw_error_rad": None,
        })
        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any("nonstandard goal profile" in error for error in errors))

    def test_dynamic_route_records_a_planned_detour_without_rejecting_nav2(self):
        manifest = dynamic_manifest()
        result = manifest["results"][0]
        result["planned_path_max_length_m"] = 4.0
        result["planned_path_max_chord_m"] = 1.0
        result["planned_path_max_detour_ratio"] = 4.0

        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertFalse(any("planned path" in error for error in errors))

    def test_dynamic_route_rejects_an_executed_detour_before_it_can_pass(self):
        manifest = dynamic_manifest()
        manifest["results"][0].update(executed_travel_fields(
            baseline=1.0, travel=3.0))
        manifest["results"][0]["executed_travel_detour_violation"] = True

        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any("executed travel" in error for error in errors))

    def test_perception_evidence_is_a_hard_route_requirement(self):
        manifest = dynamic_manifest()
        manifest["perception"]["checks"]["local"] = False
        manifest["perception"]["global_costmap_stamp_ns"] = 0

        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any("perception check local" in error for error in errors))
        self.assertTrue(any(
            "global_costmap_stamp_ns" in error for error in errors))

    def test_perception_rejects_scan_odom_skew_above_simulator_limit(self):
        manifest = dynamic_manifest()
        manifest["perception"]["odom_stamp_ns"] = 104_976_000_000

        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any(
            "scan and odom timestamps are too far apart" in error
            for error in errors
        ))

    def test_route_order_and_action_status_are_strict(self):
        manifest = valid_manifest()
        manifest["results"][1], manifest["results"][2] = (
            manifest["results"][2], manifest["results"][1])
        manifest["results"][2]["status"] = 5
        manifest["results"][0]["behavior_tree"] = "wrong.xml"

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("route mismatch" in error for error in errors))
        self.assertTrue(any("status must be 4" in error for error in errors))
        self.assertTrue(any("behavior_tree must be" in error for error in errors))

    def test_velocity_sign_is_checked_per_direction(self):
        manifest = valid_manifest()
        manifest["results"][1]["cmd_linear_max"] = 0.05
        manifest["results"][0]["cmd_linear_min"] = -0.05
        manifest["results"][1]["controller_cmd_linear_max"] = 0.05
        manifest["results"][0]["controller_cmd_linear_min"] = -0.05
        manifest["results"][2]["controller_cmd_linear_max"] = 0.05

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(
            any("contains a forward command" in error for error in errors))
        self.assertTrue(
            any("contains a reverse command" in error for error in errors))
        self.assertTrue(
            any("controller contains a forward command" in error for error in errors))
        self.assertTrue(
            any("controller contains a reverse command" in error for error in errors))

    def test_forward_command_layers_must_remain_in_ackermann_envelope(self):
        manifest = valid_manifest()
        forward = manifest["results"][0]
        forward["controller_cmd_kinematic_violation_count"] = 1
        forward["cmd_min_turning_radius_m"] = 0.20

        errors = VALIDATION.validate_manifest(manifest, 199.0)

        self.assertTrue(any(
            "violates forward Ackermann curvature" in error
            for error in errors
        ))
        self.assertTrue(any(
            "forward observed turning radius is too small" in error
            for error in errors
        ))

    def test_pose_tolerances_and_planned_yaw_are_strict(self):
        manifest = valid_manifest()
        manifest["results"][0]["goal_yaw_error_rad"] = 0.31
        manifest["results"][1]["goal_error_m"] = 0.20
        manifest["results"][1]["signed_plan_goal_yaw_error_rad"] = 0.20
        manifest["results"][1]["yaw_goal_tolerance_rad"] = 0.51
        manifest["results"][2]["position_observer_margin_m"] = 0.05

        errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("goal_yaw_error_rad" in error for error in errors))
        self.assertTrue(any("goal_error_m" in error for error in errors))
        self.assertTrue(
            any("planned terminal yaw" in error for error in errors))
        self.assertTrue(
            any("yaw_goal_tolerance_rad must be within" in error for error in errors))
        self.assertTrue(
            any("position observer margin" in error for error in errors))

        with reverse_handoff_manifest() as handoff_manifest:
            handoff_manifest["results"][0]["yaw_goal_tolerance_rad"] = 0.50
            handoff_errors = VALIDATION.validate_manifest(
                handoff_manifest, 199.0
            )
        self.assertTrue(any(
            "yaw_goal_tolerance_rad must be within" in error
            for error in handoff_errors
        ))

    def test_tuned_goal_tolerance_is_accepted_within_safe_range(self):
        manifest = valid_manifest()
        precise = manifest["results"][0]
        precise["yaw_goal_tolerance_rad"] = 0.20
        precise["goal_yaw_error_rad"] = 0.19

        self.assertEqual(
            VALIDATION.validate_manifest(manifest, 199.0), [])

    def test_reverse_handoff_proves_post_position_yaw_control(self):
        with reverse_handoff_manifest() as manifest:
            handoff = manifest["results"][0]
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
        with reverse_handoff_manifest() as manifest:
            handoff = manifest["results"][0]
            handoff["post_xy_max_goal_error_m"] = 1.50
            handoff["post_xy_travel_m"] = 8.0
            handoff["post_xy_elapsed_sec"] = 119.0
            errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("left the terminal area" in error for error in errors))
        self.assertTrue(any("traveled too far" in error for error in errors))
        self.assertTrue(any("took too long" in error for error in errors))

    def test_reverse_handoff_enforces_both_command_layers(self):
        with reverse_handoff_manifest() as manifest:
            handoff = manifest["results"][0]
            handoff["controller_cmd_linear_min"] = (
                -VALIDATION.SIMULATION_HANDOFF_SPEED_CAP_MPS - 0.01)
            handoff["cmd_kinematic_violation_count"] = 1
            handoff["controller_cmd_min_turning_radius_m"] = 0.20
            errors = VALIDATION.validate_manifest(manifest, 199.0)
        self.assertTrue(any("exceeds speed cap" in error for error in errors))
        self.assertTrue(
            any("violates Ackermann curvature" in error for error in errors))
        self.assertTrue(
            any("observed turning radius is too small" in error for error in errors))

    def test_reverse_handoff_requires_virtual_forward_runtime_config(self):
        with reverse_handoff_manifest() as manifest:
            handoff = manifest["results"][0]
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
        tuner_spec = importlib.util.spec_from_file_location(
            "tune_params_handoff", TUNE_PARAMS)
        tuner_module = importlib.util.module_from_spec(tuner_spec)
        tuner_spec.loader.exec_module(tuner_module)
        params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        handoff = params["controller_server"]["ros__parameters"][
            "ReverseHandoff"]
        handoff_tuning = tuner_module.TUNABLE_PARAMS["8"]

        self.assertIn('"reverse_handoff_vx_max"', tuner)
        self.assertIn('"reverse_handoff_goal_angle_weight"', tuner)
        self.assertNotIn('"reverse_handoff_desired_linear_vel"', tuner)
        self.assertNotIn('"reverse_handoff_lookahead_dist"', tuner)
        self.assertEqual(
            handoff["plugin"], "smartcar_nav2::ReverseOnlyMPPIController"
        )
        self.assertAlmostEqual(handoff["vx_min"], 0.02)
        self.assertAlmostEqual(handoff["vx_max"], 0.30)
        self.assertLess(handoff["vx_min"], handoff["vx_max"])
        self.assertEqual(handoff_tuning["default"], 0.30)
        self.assertLessEqual(handoff_tuning["range"][0], 0.30)
        self.assertGreaterEqual(handoff_tuning["range"][1], 0.30)
        self.assertIs(
            params["velocity_smoother"]["ros__parameters"]["scale_velocities"],
            True,
        )
        self.assertAlmostEqual(
            handoff["GoalAngleCritic"]["cost_weight"], 12.0
        )


if __name__ == "__main__":
    unittest.main()
