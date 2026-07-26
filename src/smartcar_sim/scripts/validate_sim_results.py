#!/usr/bin/env python3
"""Validate one complete-route simulation result manifest."""

import argparse
import json
import math
import re
from pathlib import Path


SUCCEEDED_STATUS = 4
VELOCITY_EPSILON = 1.0e-3
CONFIG_TOLERANCE_EPSILON = 2.0e-3
POSITION_OBSERVER_MARGIN_M = 1.0e-2
YAW_OBSERVER_MARGIN_RAD = 1.0e-2
YAW_IMPROVEMENT_EPSILON = 1.0e-2
HANDOFF_POST_XY_MAX_POSITION_ERROR_M = 0.75
HANDOFF_POST_XY_MAX_TRAVEL_M = 1.00
HANDOFF_POST_XY_MAX_DURATION_SEC = 25.0
REVERSE_HANDOFF_CONTROLLER = "smartcar_nav2::ReverseOnlyMPPIController"
EXPECTED_ROUTE = (
    ("a_task_observe", "forward", "precise"),
    ("b_corridor_enter", "reverse", "standard"),
    ("b_corridor_out", "reverse", "standard"),
    ("c_corner_1", "reverse", "reverse_handoff"),
    ("c_corner_2", "forward", "standard"),
    ("c_corner_3", "forward", "standard"),
    ("c_corner_4", "forward", "standard"),
    ("b_corridor_return_enter", "forward", "standard"),
    ("b_corridor_return", "forward", "standard"),
    ("p_finish", "forward", "standard"),
)
EXPECTED_GOAL_CONTRACTS = {
    "a_task_observe": ("precise_goal_checker", (0.08, 0.25), (0.10, 0.30)),
    "b_corridor_enter": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "b_corridor_out": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "c_corner_1": ("reverse_goal_checker", (0.08, 0.20), (0.10, 0.30)),
    "c_corner_2": ("goal_checker", (0.10, 0.50), (0.25, 1.00)),
    "c_corner_3": ("goal_checker", (0.10, 0.50), (0.25, 1.00)),
    "c_corner_4": ("goal_checker", (0.10, 0.50), (0.25, 1.00)),
    "b_corridor_return_enter": (
        "goal_checker", (0.10, 0.50), (0.25, 1.00)),
    "b_corridor_return": ("goal_checker", (0.10, 0.50), (0.25, 1.00)),
    "p_finish": ("goal_checker", (0.10, 0.50), (0.25, 1.00)),
}
EXPECTED_BEHAVIOR_TREES = {
    "a_task_observe": (
        "navigate_to_pose_precise_w_replanning_and_recovery.xml"),
    "b_corridor_enter": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "b_corridor_out": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "c_corner_1": (
        "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml"),
    "c_corner_2": "navigate_to_pose_w_replanning_and_recovery.xml",
    "c_corner_3": "navigate_to_pose_w_replanning_and_recovery.xml",
    "c_corner_4": "navigate_to_pose_w_replanning_and_recovery.xml",
    "b_corridor_return_enter": (
        "navigate_to_pose_w_replanning_and_recovery.xml"),
    "b_corridor_return": "navigate_to_pose_w_replanning_and_recovery.xml",
    "p_finish": "navigate_to_pose_w_replanning_and_recovery.xml",
}
REQUIRED_INPUTS = (
    "waypoints_file",
    "forward_behavior_tree",
    "precise_behavior_tree",
    "reverse_behavior_tree",
    "reverse_handoff_behavior_tree",
    "nav2_params_file",
)
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


def _finite_number(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def validate_manifest(data, started_after=None):
    errors = []
    if not isinstance(data, dict):
        return ["manifest must be a JSON object"]

    results = data.get("results")
    if data.get("overall_outcome") != "completed":
        errors.append("overall_outcome must be completed")
    if data.get("expected_goal_count") != len(EXPECTED_ROUTE):
        errors.append(
            f"expected_goal_count must be {len(EXPECTED_ROUTE)}")
    if not isinstance(results, list):
        errors.append("results must be a list")
        results = []
    if len(results) != len(EXPECTED_ROUTE):
        errors.append(f"results must contain {len(EXPECTED_ROUTE)} goals")

    for index, expected in enumerate(EXPECTED_ROUTE):
        if index >= len(results):
            break
        result = results[index]
        label = f"results[{index}]"
        if not isinstance(result, dict):
            errors.append(f"{label} must be an object")
            continue

        actual = (
            result.get("id"),
            result.get("direction"),
            result.get("goal_profile"),
        )
        if actual != expected:
            errors.append(
                f"{label} route mismatch: expected {expected}, got {actual}")
        if result.get("outcome") != "succeeded":
            errors.append(f"{label} outcome must be succeeded")
        if result.get("status") != SUCCEEDED_STATUS:
            errors.append(
                f"{label} status must be {SUCCEEDED_STATUS}")
        if result.get("contract_errors") != []:
            errors.append(f"{label} contract_errors must be empty")
        expected_tree = EXPECTED_BEHAVIOR_TREES[expected[0]]
        if result.get("behavior_tree") != expected_tree:
            errors.append(f"{label} behavior_tree must be {expected_tree}")
        path_messages = result.get("path_messages")
        if (
            isinstance(path_messages, bool)
            or not isinstance(path_messages, int)
            or path_messages <= 0
        ):
            errors.append(f"{label} path_messages must be positive")

        checker_name, xy_range, yaw_range = (
            EXPECTED_GOAL_CONTRACTS[expected[0]])
        if result.get("goal_checker") != checker_name:
            errors.append(
                f"{label} goal_checker must be {checker_name}")
        reported_xy_tolerance = result.get("xy_goal_tolerance_m")
        reported_yaw_tolerance = result.get("yaw_goal_tolerance_rad")
        if (
            not _finite_number(reported_xy_tolerance)
            or not xy_range[0] <= float(reported_xy_tolerance) <= xy_range[1]
        ):
            errors.append(
                f"{label} xy_goal_tolerance_m must be within {xy_range}")
        if (
            not _finite_number(reported_yaw_tolerance)
            or not yaw_range[0] <= float(reported_yaw_tolerance) <= yaw_range[1]
        ):
            errors.append(
                f"{label} yaw_goal_tolerance_rad must be within {yaw_range}")

        xy_tolerance = (
            float(reported_xy_tolerance)
            if _finite_number(reported_xy_tolerance) else xy_range[1]
        )
        yaw_tolerance = (
            float(reported_yaw_tolerance)
            if _finite_number(reported_yaw_tolerance) else yaw_range[1]
        )

        if result.get("position_observer_margin_m") != POSITION_OBSERVER_MARGIN_M:
            errors.append(
                f"{label} position observer margin must be "
                f"{POSITION_OBSERVER_MARGIN_M}")
        if result.get("yaw_observer_margin_rad") != YAW_OBSERVER_MARGIN_RAD:
            errors.append(
                f"{label} yaw observer margin must be "
                f"{YAW_OBSERVER_MARGIN_RAD}")

        position_error = result.get("goal_error_m")
        yaw_error = result.get("goal_yaw_error_rad")
        if (
            not _finite_number(position_error)
            or float(position_error)
            > xy_tolerance + POSITION_OBSERVER_MARGIN_M
        ):
            errors.append(
                f"{label} goal_error_m exceeds {xy_tolerance}")
        if (
            not _finite_number(yaw_error)
            or float(yaw_error) > yaw_tolerance + YAW_OBSERVER_MARGIN_RAD
        ):
            errors.append(
                f"{label} goal_yaw_error_rad exceeds {yaw_tolerance}")
        plan_yaw_error = result.get("signed_plan_goal_yaw_error_rad")
        if (
            not _finite_number(plan_yaw_error)
            or abs(float(plan_yaw_error))
            > 0.15 + CONFIG_TOLERANCE_EPSILON
        ):
            errors.append(f"{label} planned terminal yaw is not constrained")

        minimum = result.get("cmd_linear_min")
        maximum = result.get("cmd_linear_max")
        controller_minimum = result.get("controller_cmd_linear_min")
        controller_maximum = result.get("controller_cmd_linear_max")
        if not _finite_number(minimum) or not _finite_number(maximum):
            errors.append(f"{label} command extrema must be finite")
            continue
        if (
            not _finite_number(controller_minimum)
            or not _finite_number(controller_maximum)
        ):
            errors.append(f"{label} controller command extrema must be finite")
            continue
        if expected[1] == "reverse":
            if float(minimum) >= -VELOCITY_EPSILON:
                errors.append(f"{label} has no reverse command")
            if float(maximum) > VELOCITY_EPSILON:
                errors.append(f"{label} contains a forward command")
            if float(controller_minimum) >= -VELOCITY_EPSILON:
                errors.append(f"{label} controller has no reverse command")
            if float(controller_maximum) > VELOCITY_EPSILON:
                errors.append(f"{label} controller contains a forward command")
        else:
            if float(maximum) <= VELOCITY_EPSILON:
                errors.append(f"{label} has no forward command")
            if float(minimum) < -VELOCITY_EPSILON:
                errors.append(f"{label} contains a reverse command")
            if float(controller_maximum) <= VELOCITY_EPSILON:
                errors.append(f"{label} controller has no forward command")
            if float(controller_minimum) < -VELOCITY_EPSILON:
                errors.append(f"{label} controller contains a reverse command")

        if expected[2] == "reverse_handoff":
            speed_cap = result.get("handoff_speed_cap_mps")
            angular_cap = result.get("handoff_wz_cap_radps")
            turning_radius = result.get("handoff_min_turning_radius_m")
            internal_vx_min = result.get("handoff_internal_vx_min_mps")
            internal_vx_max = result.get("handoff_internal_vx_max_mps")
            if result.get("handoff_controller_plugin") != REVERSE_HANDOFF_CONTROLLER:
                errors.append(
                    f"{label} handoff controller must be virtual-forward wrapper")
            if result.get("velocity_smoother_scale_velocities") is not True:
                errors.append(
                    f"{label} velocity smoother must scale velocities together")
            if (
                not _finite_number(internal_vx_min)
                or not _finite_number(internal_vx_max)
                or not 0.0 < float(internal_vx_min)
                <= float(internal_vx_max) <= 0.15
            ):
                errors.append(
                    f"{label} virtual-forward vx bounds are invalid")
            if (
                not _finite_number(speed_cap)
                or not 0.0 < float(speed_cap) <= 0.15
            ):
                errors.append(f"{label} has invalid handoff speed cap")
            elif (
                _finite_number(internal_vx_max)
                and abs(float(speed_cap) - float(internal_vx_max))
                > CONFIG_TOLERANCE_EPSILON
            ):
                errors.append(
                    f"{label} handoff speed cap must match virtual vx_max")
            if (
                not _finite_number(angular_cap)
                or abs(float(angular_cap) - 0.20) > CONFIG_TOLERANCE_EPSILON
            ):
                errors.append(f"{label} handoff angular cap must be 0.20")
            if (
                not _finite_number(turning_radius)
                or abs(float(turning_radius) - 0.55)
                > CONFIG_TOLERANCE_EPSILON
            ):
                errors.append(f"{label} handoff turning radius must be 0.55")
            if _finite_number(speed_cap):
                for field in ("cmd_linear_min", "cmd_linear_max",
                              "controller_cmd_linear_min",
                              "controller_cmd_linear_max"):
                    value = result.get(field)
                    if (
                        _finite_number(value)
                        and abs(float(value))
                        > float(speed_cap) + VELOCITY_EPSILON
                    ):
                        errors.append(f"{label} {field} exceeds speed cap")
            for prefix in ("controller_cmd", "cmd"):
                angular = result.get(f"{prefix}_angular_abs_max")
                violations = result.get(
                    f"{prefix}_kinematic_violation_count")
                observed_radius = result.get(
                    f"{prefix}_min_turning_radius_m")
                if (
                    not _finite_number(angular)
                    or (
                        _finite_number(angular_cap)
                        and float(angular)
                        > float(angular_cap) + VELOCITY_EPSILON
                    )
                ):
                    errors.append(f"{label} {prefix} exceeds angular cap")
                if (
                    isinstance(violations, bool)
                    or not isinstance(violations, int)
                    or violations != 0
                ):
                    errors.append(f"{label} {prefix} violates Ackermann curvature")
                if (
                    _finite_number(angular)
                    and float(angular) > VELOCITY_EPSILON
                    and (
                        not _finite_number(observed_radius)
                        or (
                            _finite_number(turning_radius)
                            and float(observed_radius)
                            < float(turning_radius) - CONFIG_TOLERANCE_EPSILON
                        )
                    )
                ):
                    errors.append(
                        f"{label} {prefix} observed turning radius is too small")
            entry_yaw_error = result.get(
                "xy_tolerance_entry_yaw_error_rad")
            if not _finite_number(entry_yaw_error):
                errors.append(
                    f"{label} lacks XY-entry yaw evidence")
            elif (
                float(entry_yaw_error)
                > yaw_tolerance + YAW_OBSERVER_MARGIN_RAD
            ):
                controller_command_count = result.get(
                    "post_xy_controller_cmd_sample_count")
                controller_angular_count = result.get(
                    "post_xy_controller_angular_sample_count")
                command_count = result.get("post_xy_cmd_sample_count")
                angular_count = result.get("post_xy_angular_sample_count")
                reduction = result.get("post_xy_yaw_error_reduction_rad")
                if (
                    isinstance(controller_command_count, bool)
                    or not isinstance(controller_command_count, int)
                    or controller_command_count <= 0
                ):
                    errors.append(
                        f"{label} controller stopped after XY entry")
                if (
                    isinstance(controller_angular_count, bool)
                    or not isinstance(controller_angular_count, int)
                    or controller_angular_count <= 0
                ):
                    errors.append(
                        f"{label} controller lacks post-XY steering")
                if (
                    isinstance(command_count, bool)
                    or not isinstance(command_count, int)
                    or command_count <= 0
                ):
                    errors.append(
                        f"{label} did not continue controlling after XY entry")
                if (
                    isinstance(angular_count, bool)
                    or not isinstance(angular_count, int)
                    or angular_count <= 0
                ):
                    errors.append(
                        f"{label} lacks post-XY steering commands")
                if (
                    not _finite_number(reduction)
                    or float(reduction) <= YAW_IMPROVEMENT_EPSILON
                ):
                    errors.append(
                        f"{label} yaw did not converge after XY entry")

            post_xy_max_error = result.get("post_xy_max_goal_error_m")
            post_xy_travel = result.get("post_xy_travel_m")
            post_xy_elapsed = result.get("post_xy_elapsed_sec")
            if (
                not _finite_number(post_xy_max_error)
                or not 0.0 <= float(post_xy_max_error)
                <= HANDOFF_POST_XY_MAX_POSITION_ERROR_M
            ):
                errors.append(
                    f"{label} left the terminal area after XY entry")
            if (
                not _finite_number(post_xy_travel)
                or not 0.0 <= float(post_xy_travel)
                <= HANDOFF_POST_XY_MAX_TRAVEL_M
            ):
                errors.append(
                    f"{label} traveled too far after XY entry")
            if (
                not _finite_number(post_xy_elapsed)
                or not 0.0 <= float(post_xy_elapsed)
                <= HANDOFF_POST_XY_MAX_DURATION_SEC
            ):
                errors.append(
                    f"{label} took too long to converge after XY entry")

    inputs = data.get("inputs")
    if not isinstance(inputs, dict):
        errors.append("inputs must be an object")
        inputs = {}
    for name in REQUIRED_INPUTS:
        item = inputs.get(name)
        if not isinstance(item, dict):
            errors.append(f"inputs.{name} must be an object")
            continue
        if not isinstance(item.get("path"), str) or not item["path"]:
            errors.append(f"inputs.{name}.path must be non-empty")
        if not SHA256_PATTERN.fullmatch(str(item.get("sha256", ""))):
            errors.append(f"inputs.{name}.sha256 must be a SHA256 digest")
    nav2_item = inputs.get("nav2_params_file")
    if isinstance(nav2_item, dict):
        nav2_path = str(nav2_item.get("path", ""))
        if Path(nav2_path).name != "nav2_params_fixed.yaml":
            errors.append(
                "inputs.nav2_params_file must reference nav2_params_fixed.yaml")

    timestamp = data.get("timestamp")
    if not _finite_number(timestamp):
        errors.append("timestamp must be finite")
    elif started_after is not None and float(timestamp) < float(started_after):
        errors.append("timestamp predates this simulation run")
    return errors


def main():
    parser = argparse.ArgumentParser(
        description="Validate a SmartCar complete-route simulation manifest")
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--started-after", type=float)
    args = parser.parse_args()

    try:
        data = json.loads(args.manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        parser.error(str(error))

    results = data.get("results", []) if isinstance(data, dict) else []
    print(f"[tune] overall_outcome={data.get('overall_outcome')}")
    for result in results:
        if not isinstance(result, dict):
            print(f"[tune] invalid result entry: {result!r}")
            continue
        print(
            "[tune] {id}: {outcome}, {duration}s, error={error}m, "
            "yaw_error={yaw_error}rad, xy_entry_yaw={entry_yaw}rad, "
            "cmd=[{minimum}, {maximum}]".format(
                id=result.get("id"),
                outcome=result.get("outcome"),
                duration=result.get("duration_sec"),
                error=result.get("goal_error_m"),
                yaw_error=result.get("goal_yaw_error_rad"),
                entry_yaw=result.get(
                    "xy_tolerance_entry_yaw_error_rad"),
                minimum=result.get("cmd_linear_min"),
                maximum=result.get("cmd_linear_max"),
            )
        )

    errors = validate_manifest(data, args.started_after)
    for error in errors:
        print(f"[tune] validation error: {error}")
    raise SystemExit(1 if errors else 0)


if __name__ == "__main__":
    main()
