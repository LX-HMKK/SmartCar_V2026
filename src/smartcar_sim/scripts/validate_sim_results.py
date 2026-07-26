#!/usr/bin/env python3
"""Validate one complete-route simulation result manifest."""

import argparse
import json
import math
import re
from pathlib import Path


SUCCEEDED_STATUS = 4
VELOCITY_EPSILON = 1.0e-3
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
        path_messages = result.get("path_messages")
        if (
            isinstance(path_messages, bool)
            or not isinstance(path_messages, int)
            or path_messages <= 0
        ):
            errors.append(f"{label} path_messages must be positive")

        minimum = result.get("cmd_linear_min")
        maximum = result.get("cmd_linear_max")
        if not _finite_number(minimum) or not _finite_number(maximum):
            errors.append(f"{label} command extrema must be finite")
            continue
        if expected[1] == "reverse":
            if float(minimum) >= -VELOCITY_EPSILON:
                errors.append(f"{label} has no reverse command")
            if float(maximum) > VELOCITY_EPSILON:
                errors.append(f"{label} contains a forward command")
        else:
            if float(maximum) <= VELOCITY_EPSILON:
                errors.append(f"{label} has no forward command")
            if float(minimum) < -VELOCITY_EPSILON:
                errors.append(f"{label} contains a reverse command")

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
            "yaw_error={yaw_error}rad, cmd=[{minimum}, {maximum}]".format(
                id=result.get("id"),
                outcome=result.get("outcome"),
                duration=result.get("duration_sec"),
                error=result.get("goal_error_m"),
                yaw_error=result.get("goal_yaw_error_rad"),
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
