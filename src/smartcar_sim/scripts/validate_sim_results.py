#!/usr/bin/env python3
"""Validate evidence emitted by the forward-only local Gazebo route runner."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml


RESULT_SCHEMA_VERSION = 2
PERCEPTION_STATUS_SCHEMA_VERSION = 2
SUCCEEDED_STATUS = 4
VELOCITY_EPSILON = 1.0e-3
REQUIRED_PERCEPTION_CHECKS = (
    "scan",
    "odom",
    "tf",
    "tf_odom_alignment",
    "local",
    "global",
    "a_zone_probe",
    "landmarks",
)
TREE_PARAMETERS = (
    "forward_behavior_tree",
    "transit_behavior_tree",
    "precise_behavior_tree",
    "through_poses_behavior_tree",
    "through_poses_transit_behavior_tree",
    "through_poses_precise_behavior_tree",
    "through_poses_return_behavior_tree",
)
SINGLE_TREES = {
    "standard": "navigate_to_pose_w_replanning_and_recovery.xml",
    "precise": "navigate_to_pose_precise_w_replanning_and_recovery.xml",
    "transit": "navigate_to_pose_transit_w_replanning_and_recovery.xml",
}
THROUGH_TREES = {
    "standard": "navigate_through_poses_w_replanning_and_recovery.xml",
    "precise": "navigate_through_poses_precise_w_replanning_and_recovery.xml",
    "transit": "navigate_through_poses_transit_w_replanning_and_recovery.xml",
    "return": "navigate_through_poses_return_w_replanning_and_recovery.xml",
}


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _expected_route(path, errors):
    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        errors.append(f"cannot load waypoints file: {error}")
        return ()
    if not isinstance(document, dict):
        errors.append("waypoints file root must be an object")
        return ()
    waypoints = document.get("waypoints")
    segments = document.get("planning_segments")
    if not isinstance(waypoints, list) or not isinstance(segments, list):
        errors.append("waypoints file must contain waypoints and planning_segments lists")
        return ()
    by_id = {item.get("id"): item for item in waypoints if isinstance(item, dict)}
    expected = []
    for index, segment in enumerate(segments):
        label = f"planning_segments[{index}]"
        if not isinstance(segment, dict):
            errors.append(f"{label} must be an object")
            continue
        segment_id = segment.get("id")
        direction = segment.get("direction")
        if not isinstance(segment_id, str) or not segment_id:
            errors.append(f"{label}.id must be non-empty")
            continue
        if direction != "forward":
            errors.append(f"{label}.direction must be forward")
        goal_ids = [*segment.get("through_ids", []), segment.get("end_id")]
        if not all(isinstance(goal_id, str) and goal_id in by_id for goal_id in goal_ids):
            errors.append(f"{label} references an unknown waypoint")
            continue
        expected.append((segment_id, tuple(goal_ids)))
    return tuple(expected)


def _tree_for(goals, through):
    terminal = goals[-1]
    if through:
        if terminal.get("task") == "return":
            return THROUGH_TREES["return"]
        if terminal.get("goal_profile") == "precise":
            return THROUGH_TREES["precise"]
        if terminal.get("heading_mode") == "free":
            return THROUGH_TREES["transit"]
        return THROUGH_TREES["standard"]
    if terminal.get("goal_profile") == "precise":
        return SINGLE_TREES["precise"]
    if terminal.get("heading_mode") == "free":
        return SINGLE_TREES["transit"]
    return SINGLE_TREES["standard"]


def _validate_route(data, expected, errors):
    route = data.get("route")
    if not isinstance(route, dict) or not isinstance(route.get("segments"), list):
        errors.append("route.segments must be a list")
        return ()
    stages = route["segments"]
    observed_ids = []
    parsed = []
    for index, stage in enumerate(stages):
        label = f"route.segments[{index}]"
        if not isinstance(stage, dict):
            errors.append(f"{label} must be an object")
            continue
        segment_id = stage.get("id")
        if not isinstance(segment_id, str) or not segment_id:
            errors.append(f"{label}.id must be non-empty")
            continue
        observed_ids.append(segment_id)
        if stage.get("direction") != "forward":
            errors.append(f"{label}.direction must be forward")
        goals = stage.get("goals")
        if not isinstance(goals, list) or not goals:
            errors.append(f"{label}.goals must be non-empty")
            continue
        for goal_index, goal in enumerate(goals):
            goal_label = f"{label}.goals[{goal_index}]"
            if not isinstance(goal, dict):
                errors.append(f"{goal_label} must be an object")
                continue
            if goal.get("direction") != "forward":
                errors.append(f"{goal_label}.direction must be forward")
            if goal.get("goal_profile") not in {"standard", "precise"}:
                errors.append(f"{goal_label}.goal_profile is invalid")
            if goal.get("heading_mode") not in {"free", "locked"}:
                errors.append(f"{goal_label}.heading_mode is invalid")
            pose = goal.get("pose")
            orientation = pose.get("orientation") if isinstance(pose, dict) else None
            if not isinstance(orientation, dict):
                errors.append(f"{goal_label}.pose.orientation must be an object")
            else:
                values = [orientation.get(field) for field in ("x", "y", "z", "w")]
                if not all(_finite(value) for value in values):
                    errors.append(f"{goal_label}.pose.orientation must be finite")
                elif abs(math.sqrt(sum(float(value) ** 2 for value in values)) - 1.0) > 1e-3:
                    errors.append(f"{goal_label}.pose.orientation must be unit length")
        parsed.append((segment_id, goals))
    if expected:
        expected_ids = [item[0] for item in expected]
        if observed_ids != expected_ids:
            errors.append("route segment ids differ from the waypoint file")
        for (expected_id, expected_goals), (observed_id, goals) in zip(expected, parsed):
            if expected_id != observed_id or tuple(goal.get("id") for goal in goals) != expected_goals:
                errors.append(f"route segment {expected_id!r} differs from the waypoint file")
    return tuple(parsed)


def _validate_commands(label, metrics, errors):
    if not isinstance(metrics, dict):
        errors.append(f"{label} command metrics must be an object")
        return
    minimum = metrics.get("linear_min_mps")
    maximum = metrics.get("linear_max_mps")
    positive = metrics.get("positive_count")
    negative = metrics.get("negative_count")
    if not _finite(minimum) or not _finite(maximum):
        errors.append(f"{label} command extrema must be finite")
        return
    if float(minimum) < -VELOCITY_EPSILON:
        errors.append(f"{label} contains a reverse command")
    if not isinstance(positive, int) or positive <= 0 or float(maximum) <= VELOCITY_EPSILON:
        errors.append(f"{label} has no forward command")
    if not isinstance(negative, int) or negative != 0:
        errors.append(f"{label} reports reverse command samples")


def validate(data, waypoints_file=None, started_after=None):
    errors = []
    if not isinstance(data, dict):
        return ["result manifest must be an object"]
    if data.get("schema_version") != RESULT_SCHEMA_VERSION:
        errors.append("result schema version is invalid")
    if data.get("status") != "completed":
        errors.append(f"simulation status is not completed: {data.get('reason', '')}")
    created = data.get("created_at_epoch_sec")
    if not _finite(created):
        errors.append("created_at_epoch_sec must be finite")
    elif started_after is not None and float(created) < started_after:
        errors.append("result predates this simulation run")

    manifest = data.get("input_manifest")
    if not isinstance(manifest, dict):
        errors.append("input_manifest must be an object")
    else:
        trees = manifest.get("behavior_trees")
        if not isinstance(trees, dict) or set(trees) != set(TREE_PARAMETERS):
            errors.append("input_manifest must contain exactly the forward tree set")
        elif len({item.get("name") for item in trees.values() if isinstance(item, dict)}) != len(TREE_PARAMETERS):
            errors.append("input_manifest behavior tree names must be unique")
        if waypoints_file is not None and manifest.get("waypoints_sha256") != _sha256(waypoints_file):
            errors.append("waypoints file hash does not match the simulation input")

    expected = _expected_route(waypoints_file, errors) if waypoints_file else ()
    stages = _validate_route(data, expected, errors)
    results = data.get("results")
    if not isinstance(results, list) or len(results) != len(stages):
        errors.append("results must contain one action result per route segment")
    else:
        for index, ((segment_id, goals), result) in enumerate(zip(stages, results)):
            label = f"results[{index}]"
            if not isinstance(result, dict):
                errors.append(f"{label} must be an object")
                continue
            through = len(goals) > 1
            expected_action = "navigate_through_poses" if through else "navigate_to_pose"
            if result.get("segment_id") != segment_id:
                errors.append(f"{label}.segment_id does not match route")
            if result.get("direction") != "forward":
                errors.append(f"{label}.direction must be forward")
            if result.get("action") != expected_action:
                errors.append(f"{label}.action does not match route cardinality")
            if result.get("goal_ids") != [goal.get("id") for goal in goals]:
                errors.append(f"{label}.goal_ids do not match route")
            if result.get("behavior_tree") != _tree_for(goals, through):
                errors.append(f"{label}.behavior_tree does not match the native forward tree")
            if result.get("outcome") != "succeeded" or result.get("status") != SUCCEEDED_STATUS:
                errors.append(f"{label} did not succeed")
            _validate_commands(f"{label}.controller_commands", result.get("controller_commands"), errors)
            _validate_commands(f"{label}.candidate_commands", result.get("candidate_commands"), errors)
            if not isinstance(result.get("planner_path_count"), int) or result["planner_path_count"] <= 0:
                errors.append(f"{label} has no native planner path evidence")

    perception = data.get("perception")
    if not isinstance(perception, dict) or perception.get("schema_version") != PERCEPTION_STATUS_SCHEMA_VERSION:
        errors.append("perception evidence is missing or invalid")
    elif perception.get("ready") is not True:
        errors.append("perception was not ready before route execution")
    else:
        checks = perception.get("checks")
        if not isinstance(checks, dict) or any(checks.get(name) is not True for name in REQUIRED_PERCEPTION_CHECKS):
            errors.append("perception checks do not prove obstacle costmap readiness")
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results_file", type=Path)
    parser.add_argument("--waypoints-file", type=Path)
    parser.add_argument("--started-after", type=float)
    arguments = parser.parse_args()
    try:
        data = json.loads(arguments.results_file.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        print(f"invalid result file: {error}")
        raise SystemExit(1)
    errors = validate(data, arguments.waypoints_file, arguments.started_after)
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)
    print("simulation result validation passed")


if __name__ == "__main__":
    main()
