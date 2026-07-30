#!/usr/bin/env python3
"""Validate one complete-route simulation result manifest."""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

import yaml


SUCCEEDED_STATUS = 4
VELOCITY_EPSILON = 1.0e-3
CONFIG_TOLERANCE_EPSILON = 2.0e-3
POSITION_OBSERVER_MARGIN_M = 2.0e-2
YAW_OBSERVER_MARGIN_RAD = 1.0e-2
YAW_IMPROVEMENT_EPSILON = 1.0e-2
HANDOFF_POST_XY_MAX_POSITION_ERROR_M = 0.75
HANDOFF_POST_XY_MAX_TRAVEL_M = 1.00
HANDOFF_POST_XY_MAX_DURATION_SEC = 25.0
REVERSE_HANDOFF_CONTROLLER = "smartcar_nav2::ReverseOnlyMPPIController"
# The reverse ThroughPoses BT removes a waypoint at 0.15 m.  Keep only the
# odometry observer margin beyond that runtime contract.
THROUGH_POSE_DISTANCE_TOLERANCE_M = 0.17
EXPECTED_ROUTE = (
    ("a_task_observe", "forward", "precise"),
    ("b_corridor_enter", "reverse", "standard"),
    ("c_corner_1", "reverse", "standard"),
    ("c_corner_2", "reverse", "standard"),
    ("c_corner_3", "reverse", "standard"),
    ("c_corner_4", "reverse", "standard"),
    ("b_corridor_return_enter", "reverse", "standard"),
    ("b_corridor_return", "reverse", "standard"),
    ("p_finish", "reverse", "standard"),
)
EXPECTED_GOAL_CONTRACTS = {
    "a_task_observe": ("precise_goal_checker", (0.08, 0.25), (0.10, 0.30)),
    "b_corridor_enter": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "c_corner_1": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "c_corner_2": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "c_corner_3": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "c_corner_4": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "b_corridor_return_enter": (
        "reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "b_corridor_return": (
        "reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
    "p_finish": ("reverse_goal_checker", (0.08, 0.25), (0.10, 0.50)),
}
EXPECTED_BEHAVIOR_TREES = {
    "a_task_observe": (
        "navigate_to_pose_precise_w_replanning_and_recovery.xml"),
    "b_corridor_enter": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "c_corner_1": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "c_corner_2": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "c_corner_3": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "c_corner_4": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "b_corridor_return_enter": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "b_corridor_return": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
    "p_finish": (
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"),
}
REQUIRED_INPUTS = (
    "waypoints_file",
    "forward_behavior_tree",
    "precise_behavior_tree",
    "reverse_behavior_tree",
    "reverse_handoff_behavior_tree",
    "nav2_params_file",
    "through_poses_behavior_tree",
    "through_poses_reverse_behavior_tree",
)
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
ALLOWED_DIRECTIONS = frozenset({"forward", "reverse"})
ALLOWED_GOAL_PROFILES = frozenset({
    "standard", "precise", "reverse_handoff",
})
MAX_DYNAMIC_GOAL_TOLERANCES = {
    ("forward", "standard"): (0.25, 0.50),
    ("forward", "precise"): (0.25, 0.30),
    ("reverse", "standard"): (0.25, 0.50),
    ("reverse", "reverse_handoff"): (0.20, 0.30),
}
POSE_POSITION_FIELDS = ("x", "y", "z")
POSE_ORIENTATION_FIELDS = ("x", "y", "z", "w")


def _finite_number(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _nonempty_string(value):
    return isinstance(value, str) and bool(value.strip())


def _route_goal_pose(goal, label, errors):
    """Validate and normalize one physical goal snapshot from auto_train."""
    frame_id = goal.get("frame_id")
    if not _nonempty_string(frame_id):
        errors.append(f"{label}.frame_id must be non-empty")
        return None
    pose = goal.get("pose")
    if not isinstance(pose, dict):
        errors.append(f"{label}.pose must be an object")
        return None
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict):
        errors.append(f"{label}.pose.position must be an object")
        return None
    if not isinstance(orientation, dict):
        errors.append(f"{label}.pose.orientation must be an object")
        return None
    normalized_position = {}
    normalized_orientation = {}
    valid = True
    for field in POSE_POSITION_FIELDS:
        value = position.get(field)
        if not _finite_number(value):
            errors.append(f"{label}.pose.position.{field} must be finite")
            valid = False
        else:
            normalized_position[field] = float(value)
    for field in POSE_ORIENTATION_FIELDS:
        value = orientation.get(field)
        if not _finite_number(value):
            errors.append(f"{label}.pose.orientation.{field} must be finite")
            valid = False
        else:
            normalized_orientation[field] = float(value)
    if not valid:
        return None
    return str(frame_id), normalized_position, normalized_orientation


def _dynamic_route_stages(data):
    """Load the route snapshot emitted by auto_train, if one is present."""
    route = data.get("route")
    if route is None:
        return None, []
    if not isinstance(route, dict):
        return (), ["route must be an object"]
    raw_stages = route.get("segments")
    if not isinstance(raw_stages, list) or not raw_stages:
        return (), ["route.segments must be a nonempty list"]

    errors = []
    stages = []
    segment_ids = set()
    goal_ids = set()
    for index, raw_stage in enumerate(raw_stages):
        label = f"route.segments[{index}]"
        if not isinstance(raw_stage, dict):
            errors.append(f"{label} must be an object")
            continue
        segment_id = raw_stage.get("id")
        direction = raw_stage.get("direction")
        goals = raw_stage.get("goals")
        if not _nonempty_string(segment_id):
            errors.append(f"{label}.id must be non-empty")
            continue
        if segment_id in segment_ids:
            errors.append(f"route has duplicate segment id {segment_id!r}")
        segment_ids.add(segment_id)
        if direction not in ALLOWED_DIRECTIONS:
            errors.append(f"{label}.direction must be forward or reverse")
        if not isinstance(goals, list) or not goals:
            errors.append(f"{label}.goals must be a nonempty list")
            continue
        parsed_goals = []
        for goal_index, raw_goal in enumerate(goals):
            goal_label = f"{label}.goals[{goal_index}]"
            if not isinstance(raw_goal, dict):
                errors.append(f"{goal_label} must be an object")
                continue
            goal_id = raw_goal.get("id")
            goal_direction = raw_goal.get("direction")
            goal_profile = raw_goal.get("goal_profile", "standard")
            if not _nonempty_string(goal_id):
                errors.append(f"{goal_label}.id must be non-empty")
                continue
            if goal_id in goal_ids:
                errors.append(f"route visits waypoint {goal_id!r} more than once")
            goal_ids.add(goal_id)
            if goal_direction != direction:
                errors.append(f"{goal_label}.direction must match its segment")
            if goal_profile not in ALLOWED_GOAL_PROFILES:
                errors.append(f"{goal_label}.goal_profile is invalid")
            if goal_profile == "precise" and direction != "forward":
                errors.append(f"{goal_label}.precise goal must be forward")
            if goal_profile == "reverse_handoff" and direction != "reverse":
                errors.append(f"{goal_label}.reverse_handoff goal must be reverse")
            _route_goal_pose(raw_goal, goal_label, errors)
            parsed_goals.append((goal_id, goal_direction, goal_profile))
        stages.append((segment_id, direction, tuple(parsed_goals)))
    return tuple(stages), errors


def _goal_behavior_tree(direction, goal_profile):
    if goal_profile == "reverse_handoff":
        return "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml"
    if direction == "reverse":
        return "navigate_to_pose_reverse_w_replanning_and_recovery.xml"
    if goal_profile == "precise":
        return "navigate_to_pose_precise_w_replanning_and_recovery.xml"
    return "navigate_to_pose_w_replanning_and_recovery.xml"


def _goal_checker(direction, goal_profile):
    if goal_profile == "precise":
        return "precise_goal_checker"
    if direction == "reverse":
        return "reverse_goal_checker"
    return "goal_checker"


def _validate_goal_completion(result, direction, goal_profile, label, errors):
    """Check observed terminal pose against the goal checker recorded at run time."""
    expected_checker = _goal_checker(direction, goal_profile)
    if result.get("goal_checker") != expected_checker:
        errors.append(f"{label} goal_checker does not match its direction/profile")
    xy_tolerance = result.get("xy_goal_tolerance_m")
    yaw_tolerance = result.get("yaw_goal_tolerance_rad")
    if not _finite_number(xy_tolerance) or not _finite_number(yaw_tolerance):
        errors.append(f"{label} goal tolerances must be finite")
        return
    max_xy, max_yaw = MAX_DYNAMIC_GOAL_TOLERANCES[(direction, goal_profile)]
    if float(xy_tolerance) <= 0.0 or float(xy_tolerance) > max_xy:
        errors.append(f"{label} xy_goal_tolerance_m exceeds its safe contract")
    if float(yaw_tolerance) <= 0.0 or float(yaw_tolerance) > max_yaw:
        errors.append(f"{label} yaw_goal_tolerance_rad exceeds its safe contract")
    if result.get("position_observer_margin_m") != POSITION_OBSERVER_MARGIN_M:
        errors.append(f"{label} position observer margin is invalid")
    if result.get("yaw_observer_margin_rad") != YAW_OBSERVER_MARGIN_RAD:
        errors.append(f"{label} yaw observer margin is invalid")
    position_error = result.get("goal_error_m")
    yaw_error = result.get("goal_yaw_error_rad")
    if (
        not _finite_number(position_error)
        or float(position_error) > float(xy_tolerance) + POSITION_OBSERVER_MARGIN_M
    ):
        errors.append(f"{label} final goal position is outside tolerance")
    if (
        not _finite_number(yaw_error)
        or float(yaw_error) > float(yaw_tolerance) + YAW_OBSERVER_MARGIN_RAD
    ):
        errors.append(f"{label} final goal yaw is outside tolerance")


def _validate_command_direction(result, direction, label, errors):
    for prefix in ("controller_cmd", "cmd"):
        minimum = result.get(f"{prefix}_linear_min")
        maximum = result.get(f"{prefix}_linear_max")
        if not _finite_number(minimum) or not _finite_number(maximum):
            errors.append(f"{label} {prefix} extrema must be finite")
            continue
        if direction == "reverse":
            if float(minimum) >= -VELOCITY_EPSILON:
                errors.append(f"{label} {prefix} has no reverse command")
            if float(maximum) > VELOCITY_EPSILON:
                errors.append(f"{label} {prefix} contains a forward command")
        else:
            if float(maximum) <= VELOCITY_EPSILON:
                errors.append(f"{label} {prefix} has no forward command")
            if float(minimum) < -VELOCITY_EPSILON:
                errors.append(f"{label} {prefix} contains a reverse command")


def _validate_dynamic_single_goal(result, expected, label, errors):
    waypoint_id, direction, goal_profile = expected
    if not isinstance(result, dict):
        errors.append(f"{label} must be an object")
        return
    actual = (result.get("id"), result.get("direction"), result.get("goal_profile"))
    if actual != expected:
        errors.append(f"{label} route mismatch: expected {expected}, got {actual}")
    if result.get("outcome") != "succeeded":
        errors.append(f"{label} outcome must be succeeded")
    if result.get("status") != SUCCEEDED_STATUS:
        errors.append(f"{label} status must be {SUCCEEDED_STATUS}")
    if result.get("contract_errors") != []:
        errors.append(f"{label} contract_errors must be empty")
    expected_tree = _goal_behavior_tree(direction, goal_profile)
    if result.get("behavior_tree") != expected_tree:
        errors.append(f"{label} behavior_tree must be {expected_tree}")
    _validate_goal_completion(result, direction, goal_profile, label, errors)
    path_messages = result.get("path_messages")
    if isinstance(path_messages, bool) or not isinstance(path_messages, int) or path_messages <= 0:
        errors.append(f"{label} path_messages must be positive")
    _validate_command_direction(result, direction, label, errors)


def _validate_dynamic_through_result(result, stage, label, errors):
    segment_id, direction, goals = stage
    expected_ids = [goal[0] for goal in goals]
    expected_profiles = [goal[2] for goal in goals]
    if not isinstance(result, dict):
        errors.append(f"{label} must be an object")
        return
    if len(goals) < 2:
        errors.append(f"{label} uses ThroughPoses for a one-goal segment")
    if any(profile != "standard" for profile in expected_profiles):
        errors.append(
            f"{label} uses ThroughPoses for a nonstandard goal profile")
    if result.get("segment_id") != segment_id:
        errors.append(f"{label} segment_id must be {segment_id!r}")
    if result.get("direction") != direction:
        errors.append(f"{label} direction must be {direction}")
    if result.get("goal_ids") != expected_ids:
        errors.append(f"{label} goal_ids do not match the saved segment")
    if result.get("goal_profiles") != expected_profiles:
        errors.append(f"{label} goal_profiles do not match the saved segment")
    if result.get("waypoint_count") != len(goals):
        errors.append(f"{label} waypoint_count must be {len(goals)}")
    expected_tree = (
        "navigate_through_poses_reverse_w_replanning_and_recovery.xml"
        if direction == "reverse"
        else "navigate_through_poses_w_replanning_and_recovery.xml"
    )
    if result.get("behavior_tree") != expected_tree:
        errors.append(f"{label} behavior_tree must be {expected_tree}")
    if result.get("outcome") != "succeeded":
        errors.append(f"{label} outcome must be succeeded")
    if result.get("status") != SUCCEEDED_STATUS:
        errors.append(f"{label} status must be {SUCCEEDED_STATUS}")
    if result.get("contract_errors") != []:
        errors.append(f"{label} contract_errors must be empty")
    _validate_goal_completion(result, direction, "standard", label, errors)
    _validate_command_direction(result, direction, label, errors)
    path_messages = result.get("path_messages")
    if isinstance(path_messages, bool) or not isinstance(path_messages, int) or path_messages <= 0:
        errors.append(f"{label} path_messages must be positive")
    passed = result.get("waypoints_passed")
    if not isinstance(passed, list) or len(passed) != len(expected_ids):
        errors.append(f"{label} waypoints_passed must cover every segment goal")
        return
    for goal_index, (passed_goal, expected_id) in enumerate(zip(passed, expected_ids)):
        point_label = f"{label}.waypoints_passed[{goal_index}]"
        if not isinstance(passed_goal, dict) or passed_goal.get("id") != expected_id:
            errors.append(f"{point_label} does not match {expected_id!r}")
            continue
        distance = passed_goal.get("min_distance_m")
        if (
            not _finite_number(distance)
            or float(distance) > THROUGH_POSE_DISTANCE_TOLERANCE_M
        ):
            errors.append(
                f"{point_label}.min_distance_m exceeds "
                f"{THROUGH_POSE_DISTANCE_TOLERANCE_M}")


def _validate_dynamic_results(results, stages, use_through_poses, errors):
    result_index = 0
    for stage in stages:
        if result_index >= len(results):
            errors.append(f"missing result for segment {stage[0]!r}")
            break
        result = results[result_index]
        label = f"results[{result_index}]"
        if isinstance(result, dict) and result.get("mode") == "through_poses":
            _validate_dynamic_through_result(result, stage, label, errors)
            result_index += 1
            continue
        if use_through_poses and len(stage[2]) > 1:
            errors.append(
                f"{label} must use NavigateThroughPoses for configured "
                f"multi-goal segment {stage[0]!r}")
        for expected_goal in stage[2]:
            if result_index >= len(results):
                errors.append(f"missing result for waypoint {expected_goal[0]!r}")
                break
            _validate_dynamic_single_goal(
                results[result_index], expected_goal, f"results[{result_index}]", errors
            )
            result_index += 1
    if result_index != len(results):
        errors.append("results contain entries outside the saved route")


def _validate_execution(data, errors):
    execution = data.get("execution")
    if not isinstance(execution, dict):
        errors.append("execution must be an object")
        return None
    use_through_poses = execution.get("use_through_poses")
    if not isinstance(use_through_poses, bool):
        errors.append("execution.use_through_poses must be a boolean")
        return None
    return use_through_poses


def _validate_traceability(data, started_after, errors):
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
        if not isinstance(item.get("realpath"), str) or not item["realpath"]:
            errors.append(f"inputs.{name}.realpath must be non-empty")
        if not SHA256_PATTERN.fullmatch(str(item.get("sha256", ""))):
            errors.append(f"inputs.{name}.sha256 must be a SHA256 digest")
    nav2_item = inputs.get("nav2_params_file")
    if isinstance(nav2_item, dict):
        nav2_path = str(nav2_item.get("path", ""))
        if Path(nav2_path).name != "nav2_params_fixed.yaml":
            errors.append("inputs.nav2_params_file must reference nav2_params_fixed.yaml")

    timestamp = data.get("timestamp")
    if not _finite_number(timestamp):
        errors.append("timestamp must be finite")
    elif started_after is not None and float(timestamp) < float(started_after):
        errors.append("timestamp predates this simulation run")


def _waypoint_snapshot_index(path, errors):
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        errors.append(f"cannot read waypoint snapshot {path}: {error}")
        return None
    if not isinstance(document, dict) or not isinstance(document.get("waypoints"), list):
        errors.append(f"waypoint snapshot {path} has no waypoint list")
        return None
    indexed = {}
    for index, waypoint in enumerate(document["waypoints"]):
        label = f"waypoint snapshot[{index}]"
        if not isinstance(waypoint, dict) or not _nonempty_string(waypoint.get("id")):
            errors.append(f"{label} must have a non-empty id")
            continue
        waypoint_id = waypoint["id"]
        if waypoint_id in indexed:
            errors.append(f"waypoint snapshot has duplicate id {waypoint_id!r}")
            continue
        pose = waypoint.get("pose")
        position = pose.get("position") if isinstance(pose, dict) else None
        orientation = pose.get("orientation") if isinstance(pose, dict) else None
        normalized = {
            "frame_id": waypoint.get("frame_id", "odom_combined"),
            "pose": {
                "position": {
                    "x": position.get("x") if isinstance(position, dict) else None,
                    "y": position.get("y") if isinstance(position, dict) else None,
                    "z": (
                        position.get("z", 0.0)
                        if isinstance(position, dict) else None
                    ),
                },
                "orientation": {
                    field: (
                        orientation.get(field, 0.0)
                        if isinstance(orientation, dict) else 0.0
                    )
                    for field in POSE_ORIENTATION_FIELDS
                },
            },
        }
        normalized = _route_goal_pose(normalized, label, errors)
        if normalized is not None:
            indexed[waypoint_id] = (
                normalized,
                waypoint.get("goal_profile", "standard"),
            )
    return indexed


def _poses_match(actual, expected):
    actual_frame, actual_position, actual_orientation = actual
    expected_frame, expected_position, expected_orientation = expected
    if actual_frame != expected_frame:
        return False
    return all(
        math.isclose(actual_position[field], expected_position[field], abs_tol=1.0e-9)
        for field in POSE_POSITION_FIELDS
    ) and all(
        math.isclose(
            actual_orientation[field], expected_orientation[field], abs_tol=1.0e-9)
        for field in POSE_ORIENTATION_FIELDS
    )


def _validate_waypoint_snapshot(data, path, errors):
    """Tie a result manifest to the exact waypoint snapshot given to CI."""
    snapshot = Path(path)
    if not snapshot.is_file():
        errors.append(f"waypoint snapshot is not a file: {snapshot}")
        return
    expected_hash = hashlib.sha256(snapshot.read_bytes()).hexdigest()
    inputs = data.get("inputs")
    source_hash = inputs.get("waypoints_file", {}).get("sha256") if isinstance(inputs, dict) else None
    if source_hash != expected_hash:
        errors.append("waypoints_file SHA256 does not match the validated snapshot")
    indexed = _waypoint_snapshot_index(snapshot, errors)
    route = data.get("route")
    if indexed is None or not isinstance(route, dict):
        return
    for stage_index, stage in enumerate(route.get("segments", [])):
        goals = stage.get("goals", []) if isinstance(stage, dict) else []
        if not isinstance(goals, list):
            continue
        for goal_index, goal in enumerate(goals):
            label = f"route.segments[{stage_index}].goals[{goal_index}]"
            if not isinstance(goal, dict):
                continue
            waypoint_id = goal.get("id")
            source = indexed.get(waypoint_id)
            if source is None:
                errors.append(f"{label}.id is absent from waypoint snapshot")
                continue
            actual = _route_goal_pose(goal, label, errors)
            expected, expected_profile = source
            if actual is not None and not _poses_match(actual, expected):
                errors.append(f"{label} pose differs from waypoint snapshot")
            if goal.get("goal_profile", "standard") != expected_profile:
                errors.append(f"{label}.goal_profile differs from waypoint snapshot")


def validate_manifest(data, started_after=None, waypoint_snapshot=None):
    errors = []
    if not isinstance(data, dict):
        return ["manifest must be a JSON object"]

    results = data.get("results")
    if data.get("overall_outcome") != "completed":
        errors.append("overall_outcome must be completed")
    dynamic_stages, route_errors = _dynamic_route_stages(data)
    if dynamic_stages is not None:
        errors.extend(route_errors)
        use_through_poses = _validate_execution(data, errors)
        expected_goal_count = sum(len(stage[2]) for stage in dynamic_stages)
        if data.get("expected_goal_count") != expected_goal_count:
            errors.append(
                f"expected_goal_count must be {expected_goal_count}")
        if not isinstance(results, list):
            errors.append("results must be a list")
            results = []
        if not route_errors and use_through_poses is not None:
            _validate_dynamic_results(
                results, dynamic_stages, use_through_poses, errors)
        _validate_traceability(data, started_after, errors)
        if waypoint_snapshot is not None:
            _validate_waypoint_snapshot(data, waypoint_snapshot, errors)
        return errors

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

    _validate_traceability(data, started_after, errors)
    if waypoint_snapshot is not None:
        _validate_waypoint_snapshot(data, waypoint_snapshot, errors)
    return errors


def main():
    parser = argparse.ArgumentParser(
        description="Validate a SmartCar complete-route simulation manifest")
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--started-after", type=float)
    parser.add_argument(
        "--waypoints-file",
        type=Path,
        help="immutable waypoint snapshot expected by this simulation run",
    )
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

    errors = validate_manifest(
        data, args.started_after, args.waypoints_file)
    for error in errors:
        print(f"[tune] validation error: {error}")
    raise SystemExit(1 if errors else 0)


if __name__ == "__main__":
    main()
