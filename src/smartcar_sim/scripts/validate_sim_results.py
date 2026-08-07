#!/usr/bin/env python3
"""Validate one complete-route simulation result manifest."""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path


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
FORWARD_AVOIDANCE_CONTROLLER = "smartcar_nav2::ForwardOnlyRPPController"
NATIVE_RPP_CONTROLLER = (
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
)
# These values are local-Gazebo contracts from nav2_keepout_filter.yaml. They
# deliberately do not describe the real vehicle's uncalibrated base settings.
SIMULATION_MINIMUM_TURNING_RADIUS_M = 0.22
SIMULATION_SPEED_CAP_MPS = 0.30
SIMULATION_FORWARD_SPEED_CAP_MPS = SIMULATION_SPEED_CAP_MPS
SIMULATION_FORWARD_WZ_CAP_RADPS = (
    SIMULATION_FORWARD_SPEED_CAP_MPS / SIMULATION_MINIMUM_TURNING_RADIUS_M
)
SIMULATION_FORWARD_PATH_MAX_CROSS_TRACK_ERROR_M = 0.12
SIMULATION_HANDOFF_SPEED_CAP_MPS = SIMULATION_SPEED_CAP_MPS
SIMULATION_HANDOFF_WZ_CAP_RADPS = (
    SIMULATION_HANDOFF_SPEED_CAP_MPS / SIMULATION_MINIMUM_TURNING_RADIUS_M
)
# Forward ThroughPoses keeps each explicit waypoint. Simulation evidence must
# retain the 0.20 m transit bound for every configured corridor constraint.
THROUGH_POSE_DISTANCE_TOLERANCE_M = 0.20
MAX_EXECUTED_TRAVEL_DETOUR_RATIO = 2.00
MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M = 0.80
MAX_EXECUTION_TRACE_SAMPLES = 128
EXECUTION_TRACE_SCHEMA_VERSION = 1
PERCEPTION_READY_TOPIC = "/smartcar/sim_perception_ready"
PERCEPTION_STATUS_SCHEMA_VERSION = 2
PERCEPTION_REQUIRED_CHECKS = (
    "scan",
    "odom",
    "tf",
    "tf_odom_alignment",
    "local",
    "global",
    "a_zone_probe",
    "landmarks",
)
MAX_PERCEPTION_SCAN_ODOM_SKEW_SEC = 0.075
MAX_PERCEPTION_COSTMAP_AGE_SEC = 1.5
MAX_PERCEPTION_TF_ODOM_POSITION_ERROR_M = 0.02
MAX_PERCEPTION_TF_ODOM_YAW_ERROR_RAD = 0.05
MIN_PERCEPTION_LANDMARK_IDS = 3
EXPECTED_ROUTE = (
    ("a_task_observe", "forward", "precise"),
    ("c_corner_1", "forward", "precise"),
    ("p_finish", "forward", "standard"),
)
EXPECTED_GOAL_CONTRACTS = {
    "a_task_observe": ("precise_goal_checker", (0.08, 0.25), (0.10, 0.30)),
    "c_corner_1": ("precise_goal_checker", (0.08, 0.25), (0.10, 0.30)),
    "p_finish": ("return_goal_checker", (0.08, 0.15), None),
}
EXPECTED_BEHAVIOR_TREES = {
    "a_task_observe": (
        "navigate_to_pose_precise_sim_w_replanning_and_recovery.xml"),
    "c_corner_1": (
        "navigate_to_pose_precise_sim_w_replanning_and_recovery.xml"),
    "p_finish": (
        "navigate_to_pose_w_replanning_and_recovery.xml"),
}
REQUIRED_INPUTS = (
    "waypoints_file",
    "forward_behavior_tree",
    "transit_behavior_tree",
    "precise_behavior_tree",
    "reverse_behavior_tree",
    "reverse_handoff_behavior_tree",
    "nav2_params_file",
    "through_poses_behavior_tree",
    "through_poses_transit_behavior_tree",
    "through_poses_precise_behavior_tree",
    "through_poses_return_behavior_tree",
    "through_poses_reverse_behavior_tree",
    "through_poses_reverse_locked_behavior_tree",
    "through_poses_reverse_return_behavior_tree",
)
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
ALLOWED_DIRECTIONS = frozenset({"forward", "reverse"})
ALLOWED_GOAL_PROFILES = frozenset({
    "standard", "precise", "reverse_handoff",
})
ALLOWED_TASKS = frozenset({
    "start", "qr", "vlm", "corridor", "loop", "return", "nav", "via",
})
ALLOWED_HEADING_MODES = frozenset({"free", "locked"})
MAX_DYNAMIC_GOAL_TOLERANCES = {
    ("forward", "standard"): (0.35, 0.50),
    ("forward", "precise"): (0.25, 0.30),
    ("reverse", "standard"): (0.35, 0.50),
    ("reverse", "reverse_handoff"): (0.20, 0.30),
}
RETURN_DYNAMIC_GOAL_TOLERANCES = (0.15, None)
# Position-only transit gates must place the vehicle inside a corridor before
# the next segment begins. Semantic task poses use their dedicated checkers.
MAX_TRANSIT_XY_GOAL_TOLERANCE_M = 0.20
POSE_POSITION_FIELDS = ("x", "y", "z")
POSE_ORIENTATION_FIELDS = ("x", "y", "z", "w")
QUATERNION_NORM_TOLERANCE = 1.0e-3
HEADING_LOCKED_TASKS = frozenset({"start", "qr", "vlm"})


def _finite_number(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _nonempty_string(value):
    return isinstance(value, str) and bool(value.strip())


def _route_goal_pose(goal, label, errors, heading_mode=None):
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
    quaternion_norm = math.sqrt(
        sum(component * component for component in normalized_orientation.values())
    )
    is_zero = quaternion_norm <= QUATERNION_NORM_TOLERANCE
    is_unit = abs(quaternion_norm - 1.0) <= QUATERNION_NORM_TOLERANCE
    if heading_mode == "free" and not is_zero:
        errors.append(
            f"{label}.pose.orientation must be the free-heading zero quaternion"
        )
        return None
    if heading_mode == "locked" and not is_unit:
        errors.append(f"{label}.pose.orientation must be a unit quaternion")
        return None
    if heading_mode is None and not (is_zero or is_unit):
        errors.append(
            f"{label}.pose.orientation must be a unit quaternion or "
            "the free-heading zero quaternion"
        )
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
            task = raw_goal.get("task")
            goal_direction = raw_goal.get("direction")
            goal_profile = raw_goal.get("goal_profile", "standard")
            heading_mode = raw_goal.get("heading_mode")
            if not _nonempty_string(goal_id):
                errors.append(f"{goal_label}.id must be non-empty")
                continue
            if task not in ALLOWED_TASKS:
                errors.append(f"{goal_label}.task is invalid")
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
            if heading_mode not in ALLOWED_HEADING_MODES:
                errors.append(f"{goal_label}.heading_mode must be free or locked")
            _route_goal_pose(raw_goal, goal_label, errors, heading_mode)
            parsed_goals.append(
                (goal_id, goal_direction, goal_profile, heading_mode, task)
            )
        stages.append((segment_id, direction, tuple(parsed_goals)))
    return tuple(stages), errors


def _goal_behavior_tree(direction, goal_profile, heading_mode="locked"):
    if goal_profile == "reverse_handoff":
        return "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml"
    if direction == "reverse":
        return "navigate_to_pose_reverse_w_replanning_and_recovery.xml"
    if goal_profile == "precise":
        return "navigate_to_pose_precise_sim_w_replanning_and_recovery.xml"
    if heading_mode == "free":
        return "navigate_to_pose_transit_w_replanning_and_recovery.xml"
    return "navigate_to_pose_w_replanning_and_recovery.xml"


def _allows_terminal_reverse_handoff_through_poses(direction, goals):
    """Return whether a through action has the one supported special goal.

    The reverse-locked through tree is the only multi-goal action that may
    drive a ``reverse_handoff`` profile.  It can do so only for its final,
    locked-heading goal; all pass-through goals remain standard reverse goals.
    """
    if direction != "reverse" or len(goals) < 2:
        return False
    _terminal_id, goal_direction, goal_profile, heading_mode, _task = goals[-1]
    if (goal_direction, goal_profile, heading_mode) != (
        "reverse", "reverse_handoff", "locked"
    ):
        return False
    return all(
        goal_direction == "reverse" and goal_profile == "standard"
        for _goal_id, goal_direction, goal_profile, _heading_mode, _task
        in goals[:-1]
    )


def _allows_terminal_precise_through_poses(direction, goals):
    """Return whether forward vias end at a locked precise task goal."""
    if direction != "forward" or len(goals) < 2:
        return False
    _terminal_id, goal_direction, goal_profile, heading_mode, _task = goals[-1]
    if (goal_direction, goal_profile, heading_mode) != (
        "forward", "precise", "locked"
    ):
        return False
    return all(
        goal_direction == "forward" and goal_profile == "standard"
        for _goal_id, goal_direction, goal_profile, _heading_mode, _task
        in goals[:-1]
    )


def _is_return_terminal(heading_mode, task):
    del heading_mode
    return task == "return"


def _is_reverse_return_terminal(direction, heading_mode, task):
    return (
        direction == "reverse"
        and _is_return_terminal(heading_mode, task)
    )


def _through_poses_behavior_tree(
    direction, terminal_heading_mode, terminal_task=None,
    terminal_goal_profile="standard",
):
    """Return the simulation BT selected by auto_train for a through action."""
    if direction == "reverse":
        if _is_reverse_return_terminal(
            direction, terminal_heading_mode, terminal_task
        ):
            return (
                "navigate_through_poses_reverse_return_sim_"
                "w_replanning_and_recovery.xml"
            )
        if terminal_heading_mode == "locked":
            return (
                "navigate_through_poses_reverse_locked_sim_"
                "w_replanning_and_recovery.xml"
            )
        return "navigate_through_poses_reverse_w_replanning_and_recovery.xml"
    if _is_return_terminal(terminal_heading_mode, terminal_task):
        return "navigate_through_poses_return_w_replanning_and_recovery.xml"
    if terminal_heading_mode == "free":
        return (
            "navigate_through_poses_transit_w_replanning_and_recovery.xml"
        )
    if terminal_goal_profile == "precise":
        return "navigate_through_poses_precise_w_replanning_and_recovery.xml"
    return "navigate_through_poses_w_replanning_and_recovery.xml"


def _goal_checker(direction, goal_profile, heading_mode, return_terminal=False):
    if return_terminal:
        return "return_goal_checker"
    if goal_profile == "precise":
        return "precise_goal_checker"
    if heading_mode == "free":
        return "transit_goal_checker"
    if direction == "reverse":
        return "reverse_goal_checker"
    return "goal_checker"


def _goal_tolerance_limits(
    direction, goal_profile, heading_mode, return_terminal=False
):
    """Return the maximum accepted execution tolerance for one goal."""
    if return_terminal:
        return RETURN_DYNAMIC_GOAL_TOLERANCES
    if goal_profile == "precise":
        return MAX_DYNAMIC_GOAL_TOLERANCES[(direction, goal_profile)]
    if heading_mode == "free":
        return MAX_TRANSIT_XY_GOAL_TOLERANCE_M, None
    return MAX_DYNAMIC_GOAL_TOLERANCES[(direction, goal_profile)]


def _validate_goal_completion(
    result,
    direction,
    goal_profile,
    heading_mode,
    label,
    errors,
    return_terminal=False,
):
    """Check observed terminal pose against the goal checker recorded at run time."""
    expected_checker = _goal_checker(
        direction, goal_profile, heading_mode, return_terminal)
    if result.get("goal_checker") != expected_checker:
        errors.append(
            f"{label} goal_checker does not match its direction/profile/heading"
        )
    xy_tolerance = result.get("xy_goal_tolerance_m")
    yaw_tolerance = result.get("yaw_goal_tolerance_rad")
    if not _finite_number(xy_tolerance):
        errors.append(f"{label} xy_goal_tolerance_m must be finite")
        return
    max_xy, max_yaw = _goal_tolerance_limits(
        direction, goal_profile, heading_mode, return_terminal)
    if float(xy_tolerance) <= 0.0 or float(xy_tolerance) > max_xy:
        errors.append(f"{label} xy_goal_tolerance_m exceeds its safe contract")
    if max_yaw is None:
        if yaw_tolerance is not None:
            errors.append(
                f"{label} free-heading goal must not report yaw_goal_tolerance_rad"
            )
    elif (
        not _finite_number(yaw_tolerance)
        or float(yaw_tolerance) <= 0.0
        or float(yaw_tolerance) > max_yaw
    ):
        errors.append(f"{label} yaw_goal_tolerance_rad exceeds its safe contract")
    if result.get("position_observer_margin_m") != POSITION_OBSERVER_MARGIN_M:
        errors.append(f"{label} position observer margin is invalid")
    if result.get("yaw_observer_margin_rad") != YAW_OBSERVER_MARGIN_RAD:
        errors.append(f"{label} yaw observer margin is invalid")
    position_error = result.get("goal_error_m")
    yaw_error = result.get("goal_yaw_error_rad")
    if result.get("heading_mode") != heading_mode:
        errors.append(f"{label} heading_mode does not match its saved route")
    if (
        not _finite_number(position_error)
        or float(position_error) > float(xy_tolerance) + POSITION_OBSERVER_MARGIN_M
    ):
        errors.append(f"{label} final goal position is outside tolerance")
    if heading_mode == "locked":
        if (
            not _finite_number(yaw_error)
            or float(yaw_error) > float(yaw_tolerance) + YAW_OBSERVER_MARGIN_RAD
        ):
            errors.append(f"{label} final goal yaw is outside tolerance")
        return
    for field in (
        "target_yaw_rad",
        "goal_yaw_error_rad",
        "signed_goal_yaw_error_rad",
    ):
        if field not in result or result[field] is not None:
            errors.append(f"{label} free-heading goal must not report {field}")
    if (
        "signed_plan_goal_yaw_error_rad" in result
        and result["signed_plan_goal_yaw_error_rad"] is not None
    ):
        errors.append(
            f"{label} free-heading goal must not report "
            "signed_plan_goal_yaw_error_rad"
        )


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


def _validate_forward_ackermann_contract(
    result, direction, goal_profile, label, errors
):
    """Require final forward commands to remain physically steerable."""
    if direction != "forward":
        return

    speed_cap = result.get("forward_speed_cap_mps")
    angular_cap = result.get("forward_wz_cap_radps")
    turning_radius = result.get("forward_min_turning_radius_m")
    path_max_cross_track_error = result.get(
        "forward_path_max_cross_track_error_m")
    # Every forward simulation tree, including the precise P-to-A tree, now
    # routes commands through the Ackermann-safe wrapper.  Keeping this
    # expectation identical to auto_train prevents a stale native-RPP result
    # from being accepted after the terminal-loop fix.
    expected_controller = FORWARD_AVOIDANCE_CONTROLLER
    if result.get("forward_controller_plugin") != expected_controller:
        errors.append(f"{label} forward controller does not match its Nav2 tree")
    if result.get("forward_velocity_smoother_scale_velocities") is not True:
        errors.append(f"{label} forward velocity smoother must scale velocities together")
    if (
        not _finite_number(speed_cap)
        or abs(float(speed_cap) - SIMULATION_FORWARD_SPEED_CAP_MPS)
        > CONFIG_TOLERANCE_EPSILON
    ):
        errors.append(f"{label} has invalid forward speed cap")
    if (
        not _finite_number(angular_cap)
        or abs(float(angular_cap) - SIMULATION_FORWARD_WZ_CAP_RADPS)
        > CONFIG_TOLERANCE_EPSILON
    ):
        errors.append(f"{label} has invalid forward angular cap")
    if (
        not _finite_number(turning_radius)
        or abs(float(turning_radius) - SIMULATION_MINIMUM_TURNING_RADIUS_M)
        > CONFIG_TOLERANCE_EPSILON
    ):
        errors.append(
            f"{label} forward turning radius must be "
            f"{SIMULATION_MINIMUM_TURNING_RADIUS_M:.2f}")
    if (
        not _finite_number(path_max_cross_track_error)
        or abs(
            float(path_max_cross_track_error) -
            SIMULATION_FORWARD_PATH_MAX_CROSS_TRACK_ERROR_M
        ) > CONFIG_TOLERANCE_EPSILON
    ):
        errors.append(
            f"{label} forward path tracking threshold must be "
            f"{SIMULATION_FORWARD_PATH_MAX_CROSS_TRACK_ERROR_M:.2f}")

    for prefix in ("controller_cmd", "cmd"):
        maximum = result.get(f"{prefix}_linear_max")
        angular = result.get(f"{prefix}_angular_abs_max")
        violations = result.get(f"{prefix}_kinematic_violation_count")
        observed_radius = result.get(f"{prefix}_min_turning_radius_m")
        if (
            _finite_number(maximum)
            and _finite_number(speed_cap)
            and float(maximum) > float(speed_cap) + VELOCITY_EPSILON
        ):
            errors.append(f"{label} {prefix} exceeds forward speed cap")
        if not _finite_number(angular):
            errors.append(f"{label} {prefix} angular maximum must be finite")
        elif _finite_number(angular_cap) and (
            float(angular) > float(angular_cap) + VELOCITY_EPSILON
        ):
            errors.append(f"{label} {prefix} exceeds forward angular cap")
        if (
            isinstance(violations, bool)
            or not isinstance(violations, int)
            or violations != 0
        ):
            errors.append(f"{label} {prefix} violates forward Ackermann curvature")
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
                f"{label} {prefix} forward observed turning radius is too small")


def _validate_planned_path_metrics(result, label, errors):
    """Require coherent recorded Nav2 path metrics without rejecting its route."""
    fields = (
        "planned_path_max_length_m",
        "planned_path_max_chord_m",
        "planned_path_max_detour_ratio",
    )
    values = {field: result.get(field) for field in fields}
    if any(not _finite_number(value) for value in values.values()):
        errors.append(f"{label} planned path metrics must be finite")
        return
    length = float(values["planned_path_max_length_m"])
    chord = float(values["planned_path_max_chord_m"])
    ratio = float(values["planned_path_max_detour_ratio"])
    if length < 0.0 or chord < 0.0 or ratio < 0.0:
        errors.append(f"{label} planned path metrics must be non-negative")
        return
    expected_ratio = length / max(chord, 1.0e-3)
    if abs(ratio - expected_ratio) > CONFIG_TOLERANCE_EPSILON:
        errors.append(f"{label} planned path ratio is inconsistent")


def _validate_executed_travel_detour(result, label, errors):
    """Reject an observed loop even when every individual replan is short."""
    fields = (
        "executed_travel_m",
        "executed_travel_baseline_m",
        "executed_travel_detour_ratio",
        "executed_travel_limit_m",
    )
    values = {field: result.get(field) for field in fields}
    if any(not _finite_number(value) for value in values.values()):
        errors.append(f"{label} executed travel metrics must be finite")
        return
    travel = float(values["executed_travel_m"])
    baseline = float(values["executed_travel_baseline_m"])
    ratio = float(values["executed_travel_detour_ratio"])
    limit = float(values["executed_travel_limit_m"])
    if travel < 0.0 or baseline < 0.0 or ratio < 0.0 or limit <= 0.0:
        errors.append(f"{label} executed travel metrics must be non-negative")
        return
    expected_limit = max(
        baseline * MAX_EXECUTED_TRAVEL_DETOUR_RATIO,
        baseline + MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M,
    )
    if abs(limit - expected_limit) > CONFIG_TOLERANCE_EPSILON:
        errors.append(f"{label} executed travel detour limit is invalid")
    if baseline > 1.0e-3:
        expected_ratio = travel / baseline
    elif travel > MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M:
        expected_ratio = MAX_EXECUTED_TRAVEL_DETOUR_RATIO + 1.0
    else:
        expected_ratio = 0.0
    if abs(ratio - expected_ratio) > 1.0e-2:
        errors.append(f"{label} executed travel detour ratio is invalid")
    legacy_travel = result.get("travel_m")
    if (
        not _finite_number(legacy_travel)
        or abs(float(legacy_travel) - travel) > CONFIG_TOLERANCE_EPSILON
    ):
        errors.append(f"{label} travel_m must match executed travel")
    if travel > limit + CONFIG_TOLERANCE_EPSILON:
        errors.append(f"{label} executed travel exceeds its detour limit")
    if result.get("executed_travel_detour_violation") is not False:
        errors.append(f"{label} executed travel detour must be clear")


def _validate_tracking_trace(result, label, errors, require_linked_evidence=False):
    """Validate bounded controller/odom evidence against acknowledged paths."""
    trace = result.get("tracking_trace")
    if not isinstance(trace, dict):
        if require_linked_evidence:
            errors.append(f"{label} lacks forward path tracking evidence")
        return
    if trace.get("schema_version") != EXECUTION_TRACE_SCHEMA_VERSION:
        errors.append(f"{label} tracking trace schema_version is invalid")
    sample_limit = trace.get("sample_limit")
    if (
        isinstance(sample_limit, bool)
        or not isinstance(sample_limit, int)
        or sample_limit <= 0
        or sample_limit > MAX_EXECUTION_TRACE_SAMPLES
    ):
        errors.append(f"{label} tracking trace sample_limit is invalid")
        sample_limit = MAX_EXECUTION_TRACE_SAMPLES
    accepted_path_count = trace.get("accepted_path_count")
    if (
        isinstance(accepted_path_count, bool)
        or not isinstance(accepted_path_count, int)
        or accepted_path_count < 0
    ):
        errors.append(f"{label} tracking trace accepted_path_count is invalid")
        accepted_path_count = 0

    odom = trace.get("odom_combined")
    if not isinstance(odom, list) or len(odom) > sample_limit:
        errors.append(f"{label} tracking trace odom_combined is unbounded or invalid")
        odom = []
    associated_odom_samples = 0
    for index, sample in enumerate(odom):
        sample_label = f"{label}.tracking_trace.odom_combined[{index}]"
        if not isinstance(sample, dict):
            errors.append(f"{sample_label} must be an object")
            continue
        for field in ("t_sec", "x", "y", "yaw_rad"):
            if not _finite_number(sample.get(field)):
                errors.append(f"{sample_label}.{field} must be finite")
        sequence = sample.get("accepted_path_sequence")
        tracking_fields = (
            "station_m", "cross_track_m", "path_heading_error_rad",
            "path_segment_index",
        )
        if sequence is None:
            if any(sample.get(field) is not None for field in tracking_fields):
                errors.append(
                    f"{sample_label} cannot report path metrics without an accepted path")
            continue
        if (
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence <= 0
            or sequence > accepted_path_count
        ):
            errors.append(f"{sample_label}.accepted_path_sequence is invalid")
            continue
        for field in ("station_m", "cross_track_m", "path_heading_error_rad"):
            value = sample.get(field)
            if not _finite_number(value):
                errors.append(f"{sample_label}.{field} must be finite")
            elif field != "path_heading_error_rad" and float(value) < 0.0:
                errors.append(f"{sample_label}.{field} must be non-negative")
        segment_index = sample.get("path_segment_index")
        if (
            isinstance(segment_index, bool)
            or not isinstance(segment_index, int)
            or segment_index < 0
        ):
            errors.append(f"{sample_label}.path_segment_index is invalid")
        associated_odom_samples += 1

    for topic in ("cmd_vel_nav", "cmd_vel_candidate"):
        samples = trace.get(topic)
        if not isinstance(samples, list) or len(samples) > sample_limit:
            errors.append(f"{label} tracking trace {topic} is unbounded or invalid")
            continue
        for index, sample in enumerate(samples):
            sample_label = f"{label}.tracking_trace.{topic}[{index}]"
            if not isinstance(sample, dict):
                errors.append(f"{sample_label} must be an object")
                continue
            for field in ("t_sec", "linear_x", "angular_z"):
                if not _finite_number(sample.get(field)):
                    errors.append(f"{sample_label}.{field} must be finite")

    if require_linked_evidence:
        if accepted_path_count <= 0:
            errors.append(f"{label} tracking trace has no accepted path")
        if not odom:
            errors.append(f"{label} tracking trace has no odometry")
        if associated_odom_samples <= 0:
            errors.append(f"{label} tracking trace cannot associate odometry to a path")
        for topic in ("cmd_vel_nav", "cmd_vel_candidate"):
            samples = trace.get(topic)
            if not isinstance(samples, list) or not samples:
                errors.append(f"{label} tracking trace has no {topic} samples")


def _positive_int(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, int)
        and value > 0
    )


def _validate_perception(data, errors):
    """Require scan-to-raw-costmap evidence before accepting a route run."""
    perception = data.get("perception")
    if not isinstance(perception, dict):
        errors.append("perception evidence must be an object")
        return
    if perception.get("schema_version") != PERCEPTION_STATUS_SCHEMA_VERSION:
        errors.append("perception schema_version is invalid")
    if perception.get("topic") != PERCEPTION_READY_TOPIC:
        errors.append("perception topic is invalid")
    if perception.get("ready") is not True:
        errors.append("perception ready must be true")
    checks = perception.get("checks")
    if not isinstance(checks, dict):
        errors.append("perception checks must be an object")
    else:
        for name in PERCEPTION_REQUIRED_CHECKS:
            if checks.get(name) is not True:
                errors.append(f"perception check {name} must be true")
        if checks.get("depth_points") is True:
            depth_stamp = perception.get("depth_points_stamp_ns")
            if not _positive_int(depth_stamp):
                errors.append(
                    "perception depth_points_stamp_ns must be a positive integer")
            depth_count = perception.get("depth_points_count")
            if (
                isinstance(depth_count, bool)
                or not isinstance(depth_count, int)
                or depth_count <= 0
            ):
                errors.append(
                    "perception depth_points_count must be positive")
    valid_beams = perception.get("valid_beams")
    if (
        isinstance(valid_beams, bool)
        or not isinstance(valid_beams, int)
        or valid_beams < 5
    ):
        errors.append("perception valid_beams must be at least 5")
    stamps = {}
    for name in (
        "scan_stamp_ns",
        "odom_stamp_ns",
        "local_costmap_stamp_ns",
        "global_costmap_stamp_ns",
    ):
        value = perception.get(name)
        if not _positive_int(value):
            errors.append(f"perception {name} must be a positive integer")
        else:
            stamps[name] = value
    if (
        "scan_stamp_ns" in stamps
        and "odom_stamp_ns" in stamps
        and abs(stamps["scan_stamp_ns"] - stamps["odom_stamp_ns"])
        > MAX_PERCEPTION_SCAN_ODOM_SKEW_SEC * 1.0e9
    ):
        errors.append("perception scan and odom timestamps are too far apart")
    for name in ("local_costmap_stamp_ns", "global_costmap_stamp_ns"):
        if (
            "scan_stamp_ns" in stamps
            and name in stamps
            and abs(stamps[name] - stamps["scan_stamp_ns"])
            > MAX_PERCEPTION_COSTMAP_AGE_SEC * 1.0e9
        ):
            errors.append(f"perception {name} is too far from scan timestamp")
    for name, maximum in (
        ("tf_odom_position_error_m", MAX_PERCEPTION_TF_ODOM_POSITION_ERROR_M),
        ("tf_odom_yaw_error_rad", MAX_PERCEPTION_TF_ODOM_YAW_ERROR_RAD),
    ):
        value = perception.get(name)
        if not _finite_number(value) or float(value) < 0.0:
            errors.append(f"perception {name} must be finite and non-negative")
        elif float(value) > maximum + CONFIG_TOLERANCE_EPSILON:
            errors.append(f"perception {name} exceeds alignment limit")
    required_landmark_ids = perception.get("landmark_required_ids")
    if (
        isinstance(required_landmark_ids, bool)
        or not isinstance(required_landmark_ids, int)
        or required_landmark_ids < MIN_PERCEPTION_LANDMARK_IDS
    ):
        errors.append(
            "perception landmark_required_ids must be at least "
            f"{MIN_PERCEPTION_LANDMARK_IDS}")
    matched_landmark_ids = perception.get("landmark_matched_ids")
    if (
        not isinstance(matched_landmark_ids, list)
        or len(set(matched_landmark_ids)) < MIN_PERCEPTION_LANDMARK_IDS
        or any(not _nonempty_string(value) for value in matched_landmark_ids)
    ):
        errors.append(
            "perception landmark_matched_ids must contain at least "
            f"{MIN_PERCEPTION_LANDMARK_IDS} unique ids")
    matched_points = perception.get("landmark_matched_points")
    if (
        isinstance(matched_points, bool)
        or not isinstance(matched_points, int)
        or matched_points <= 0
    ):
        errors.append("perception landmark_matched_points must be positive")
    landmark_residual = perception.get("landmark_max_residual_m")
    if not _finite_number(landmark_residual) or float(landmark_residual) < 0.0:
        errors.append(
            "perception landmark_max_residual_m must be finite and non-negative")
    if perception.get("landmark_current_valid") is not True:
        errors.append("perception current landmark registration must be valid")
    for name in ("received_monotonic_sec", "status_age_sec"):
        value = perception.get(name)
        if not _finite_number(value) or float(value) < 0.0:
            errors.append(f"perception {name} must be finite and non-negative")
    age = perception.get("status_age_sec")
    if _finite_number(age) and float(age) > 3.0:
        errors.append("perception status is stale")


def _validate_dynamic_single_goal(result, expected, label, errors):
    waypoint_id, direction, goal_profile, heading_mode, _task = expected
    if not isinstance(result, dict):
        errors.append(f"{label} must be an object")
        return
    actual = (result.get("id"), result.get("direction"), result.get("goal_profile"))
    if actual != expected[:3]:
        errors.append(
            f"{label} route mismatch: expected {expected[:3]}, got {actual}"
        )
    if result.get("outcome") != "succeeded":
        errors.append(f"{label} outcome must be succeeded")
    if result.get("status") != SUCCEEDED_STATUS:
        errors.append(f"{label} status must be {SUCCEEDED_STATUS}")
    if result.get("contract_errors") != []:
        errors.append(f"{label} contract_errors must be empty")
    expected_tree = _goal_behavior_tree(direction, goal_profile, heading_mode)
    if result.get("behavior_tree") != expected_tree:
        errors.append(f"{label} behavior_tree must be {expected_tree}")
    _validate_goal_completion(
        result, direction, goal_profile, heading_mode, label, errors
    )
    path_messages = result.get("path_messages")
    if isinstance(path_messages, bool) or not isinstance(path_messages, int) or path_messages <= 0:
        errors.append(f"{label} path_messages must be positive")
    _validate_planned_path_metrics(result, label, errors)
    _validate_executed_travel_detour(result, label, errors)
    _validate_tracking_trace(
        result, label, errors,
        require_linked_evidence=(
            direction == "forward" and goal_profile == "precise"),
    )
    _validate_command_direction(result, direction, label, errors)
    _validate_forward_ackermann_contract(
        result, direction, goal_profile, label, errors
    )


def _validate_dynamic_through_result(result, stage, label, errors):
    segment_id, direction, goals = stage
    expected_ids = [goal[0] for goal in goals]
    expected_profiles = [goal[2] for goal in goals]
    terminal_heading_mode = goals[-1][3]
    terminal_task = goals[-1][4]
    terminal_goal_profile = goals[-1][2]
    return_terminal = _is_return_terminal(
        terminal_heading_mode, terminal_task)
    if not isinstance(result, dict):
        errors.append(f"{label} must be an object")
        return
    if len(goals) < 2:
        errors.append(f"{label} uses ThroughPoses for a one-goal segment")
    if (
        any(profile != "standard" for profile in expected_profiles)
        and not (
            _allows_terminal_reverse_handoff_through_poses(direction, goals)
            or _allows_terminal_precise_through_poses(direction, goals)
        )
    ):
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
    expected_tree = _through_poses_behavior_tree(
        direction, terminal_heading_mode, terminal_task, terminal_goal_profile)
    if result.get("behavior_tree") != expected_tree:
        errors.append(f"{label} behavior_tree must be {expected_tree}")
    if result.get("outcome") != "succeeded":
        errors.append(f"{label} outcome must be succeeded")
    if result.get("status") != SUCCEEDED_STATUS:
        errors.append(f"{label} status must be {SUCCEEDED_STATUS}")
    if result.get("contract_errors") != []:
        errors.append(f"{label} contract_errors must be empty")
    _validate_goal_completion(
        result,
        direction,
        terminal_goal_profile,
        terminal_heading_mode,
        label,
        errors,
        return_terminal=return_terminal,
    )
    _validate_command_direction(result, direction, label, errors)
    _validate_forward_ackermann_contract(
        result, direction, terminal_goal_profile, label, errors
    )
    _validate_tracking_trace(
        result,
        label,
        errors,
        require_linked_evidence=(
            direction == "forward" and terminal_goal_profile == "precise"),
    )
    path_messages = result.get("path_messages")
    if isinstance(path_messages, bool) or not isinstance(path_messages, int) or path_messages <= 0:
        errors.append(f"{label} path_messages must be positive")
    _validate_planned_path_metrics(result, label, errors)
    _validate_executed_travel_detour(result, label, errors)
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


def _route_materializers():
    """Import the runtime route helpers, including when tests use source trees.

    ``validate_sim_results.py`` normally runs after ``install/setup.bash`` has
    made ``smartcar_task`` importable.  The repository-level unit tests load
    this script directly, so fall back to its adjacent source package only in
    that case.  The expected route always comes from the same helpers used by
    ``auto_train``.
    """
    try:
        from smartcar_task.planning_segments import (
            load_planning_segments,
            materialize_mission_route,
            materialize_navigation_segments,
        )
        from smartcar_task.route_geometry import materialize_free_yaws
        from smartcar_task.waypoints import is_heading_locked, load_waypoint_document
    except ModuleNotFoundError as error:
        if error.name != "smartcar_task":
            raise
        import sys

        source_package = Path(__file__).resolve().parents[2] / "smartcar_task"
        if not source_package.is_dir():
            raise
        sys.path.insert(0, str(source_package))
        from smartcar_task.planning_segments import (
            load_planning_segments,
            materialize_mission_route,
            materialize_navigation_segments,
        )
        from smartcar_task.route_geometry import materialize_free_yaws
        from smartcar_task.waypoints import is_heading_locked, load_waypoint_document
    return (
        load_planning_segments,
        materialize_mission_route,
        materialize_navigation_segments,
        materialize_free_yaws,
        is_heading_locked,
        load_waypoint_document,
    )


def _snapshot_goal(waypoint, is_heading_locked):
    """Serialize one materialized Waypoint exactly as auto_train does."""
    x, y, z = waypoint.position
    qx, qy, qz, qw = waypoint.orientation
    return {
        "id": waypoint.id,
        "task": waypoint.task,
        "heading_mode": "locked" if is_heading_locked(waypoint) else "free",
        "direction": waypoint.direction,
        "goal_profile": waypoint.goal_profile,
        "frame_id": waypoint.frame_id,
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": float(z)},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
    }


def _reconstructed_action_stages(path, errors):
    """Rebuild the complete, unfiltered action route from a YAML snapshot."""
    snapshot = Path(path)
    if not snapshot.is_file():
        errors.append(f"waypoint snapshot is not a file: {snapshot}")
        return None
    try:
        (
            load_planning_segments,
            materialize_mission_route,
            materialize_navigation_segments,
            materialize_free_yaws,
            is_heading_locked,
            load_waypoint_document,
        ) = _route_materializers()
        document, authored_waypoints = load_waypoint_document(snapshot)
        segments = load_planning_segments(document, authored_waypoints)
        ordered_waypoints = materialize_mission_route(
            authored_waypoints, segments)
        materialized_route = materialize_free_yaws(ordered_waypoints)
        action_segments = materialize_navigation_segments(
            authored_waypoints, segments)
    except (OSError, ValueError) as error:
        errors.append(
            f"waypoint snapshot cannot reconstruct complete route: {error}")
        return None

    materialized_by_id = {
        waypoint.id: waypoint for waypoint in materialized_route
    }
    stages = []
    for index, action_segment in enumerate(action_segments, start=1):
        goals = tuple(
            _snapshot_goal(materialized_by_id[source.id], is_heading_locked)
            for source in action_segment
        )
        if not goals:
            errors.append(
                f"waypoint snapshot generated an empty action {index}")
            return None
        stages.append((
            f"action_{index}_{goals[0]['id']}_to_{goals[-1]['id']}",
            goals[0]["direction"],
            goals,
        ))
    return tuple(stages)


def _goal_pose_matches(actual, expected):
    actual_frame, actual_position, actual_orientation = actual
    expected_frame, expected_position, expected_orientation = expected
    return (
        actual_frame == expected_frame
        and all(
            math.isclose(
                actual_position[field], expected_position[field],
                rel_tol=0.0, abs_tol=1.0e-9,
            )
            for field in POSE_POSITION_FIELDS
        )
        and all(
            math.isclose(
                actual_orientation[field], expected_orientation[field],
                rel_tol=0.0, abs_tol=1.0e-9,
            )
            for field in POSE_ORIENTATION_FIELDS
        )
    )


def _snapshot_heading_mode(goal):
    return goal.get(
        "heading_mode",
        "locked" if goal["task"] in HEADING_LOCKED_TASKS else "free",
    )


def _validate_reconstructed_route(data, path, errors):
    """Bind the manifest to the snapshot's full runtime-materialized route."""
    snapshot = Path(path)
    if not snapshot.is_file():
        errors.append(f"waypoint snapshot is not a file: {snapshot}")
        return None
    try:
        expected_hash = hashlib.sha256(snapshot.read_bytes()).hexdigest()
    except OSError as error:
        errors.append(f"cannot read waypoint snapshot {snapshot}: {error}")
        return None
    inputs = data.get("inputs")
    source_hash = (
        inputs.get("waypoints_file", {}).get("sha256")
        if isinstance(inputs, dict) else None
    )
    if source_hash != expected_hash:
        errors.append("waypoints_file SHA256 does not match the validated snapshot")

    expected_stages = _reconstructed_action_stages(snapshot, errors)
    if expected_stages is None:
        return None

    route = data.get("route")
    if not isinstance(route, dict):
        errors.append("route must record the complete reconstructed action route")
        return expected_stages
    actual_stages = route.get("segments")
    if not isinstance(actual_stages, list):
        errors.append("route.segments must record the complete reconstructed route")
        return expected_stages
    if len(actual_stages) != len(expected_stages):
        errors.append(
            "route.segments must contain every reconstructed action "
            f"({len(expected_stages)} expected, got {len(actual_stages)})"
        )

    for stage_index, expected_stage in enumerate(expected_stages):
        if stage_index >= len(actual_stages):
            break
        stage_label = f"route.segments[{stage_index}]"
        actual_stage = actual_stages[stage_index]
        if not isinstance(actual_stage, dict):
            continue
        expected_id, expected_direction, expected_goals = expected_stage
        if actual_stage.get("id") != expected_id:
            errors.append(
                f"{stage_label}.id must be reconstructed action {expected_id!r}")
        if actual_stage.get("direction") != expected_direction:
            errors.append(
                f"{stage_label}.direction must be {expected_direction}")
        actual_goals = actual_stage.get("goals")
        if not isinstance(actual_goals, list):
            continue
        if len(actual_goals) != len(expected_goals):
            errors.append(
                f"{stage_label}.goals must contain every reconstructed goal "
                f"({len(expected_goals)} expected, got {len(actual_goals)})"
            )
        for goal_index, expected_goal in enumerate(expected_goals):
            if goal_index >= len(actual_goals):
                break
            goal_label = f"{stage_label}.goals[{goal_index}]"
            actual_goal = actual_goals[goal_index]
            if not isinstance(actual_goal, dict):
                continue
            for field in (
                "id", "task", "direction", "goal_profile", "heading_mode"
            ):
                default = "standard" if field == "goal_profile" else None
                if actual_goal.get(field, default) != expected_goal[field]:
                    errors.append(
                        f"{goal_label}.{field} differs from the reconstructed route")
            heading_mode = _snapshot_heading_mode(expected_goal)
            actual_pose = _route_goal_pose(
                actual_goal, goal_label, errors, heading_mode)
            expected_pose = _route_goal_pose(
                expected_goal, "expected route goal", [], heading_mode)
            if (
                actual_pose is not None
                and expected_pose is not None
                and not _goal_pose_matches(actual_pose, expected_pose)
            ):
                errors.append(
                    f"{goal_label} pose differs from the reconstructed route")
    return expected_stages


def _result_stage_contract(stages):
    """Reduce reconstructed goals to the contract consumed by result checks."""
    return tuple(
        (
            segment_id,
            direction,
            tuple(
                (
                    goal["id"],
                    goal["direction"],
                    goal["goal_profile"],
                    goal["heading_mode"],
                    goal["task"],
                )
                for goal in goals
            ),
        )
        for segment_id, direction, goals in stages
    )


def validate_manifest(data, started_after=None, waypoint_snapshot=None):
    errors = []
    if not isinstance(data, dict):
        return ["manifest must be a JSON object"]

    results = data.get("results")
    if data.get("overall_outcome") != "completed":
        errors.append("overall_outcome must be completed")
    _validate_perception(data, errors)
    dynamic_stages, route_errors = _dynamic_route_stages(data)
    snapshot_stages = None
    if waypoint_snapshot is not None:
        snapshot_stages = _validate_reconstructed_route(
            data, waypoint_snapshot, errors)
    if dynamic_stages is not None:
        errors.extend(route_errors)
        use_through_poses = _validate_execution(data, errors)
        expected_stages = (
            _result_stage_contract(snapshot_stages)
            if snapshot_stages is not None else dynamic_stages
        )
        expected_goal_count = sum(len(stage[2]) for stage in expected_stages)
        if data.get("expected_goal_count") != expected_goal_count:
            errors.append(
                f"expected_goal_count must be {expected_goal_count}")
        if not isinstance(results, list):
            errors.append("results must be a list")
            results = []
        if not route_errors and use_through_poses is not None:
            _validate_dynamic_results(
                results, expected_stages, use_through_poses, errors)
        _validate_traceability(data, started_after, errors)
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
        _validate_planned_path_metrics(result, label, errors)
        _validate_executed_travel_detour(result, label, errors)
        _validate_tracking_trace(
            result, label, errors,
            require_linked_evidence=(
                expected[1] == "forward" and expected[2] == "precise"),
        )

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
        if yaw_range is None:
            if reported_yaw_tolerance is not None:
                errors.append(
                    f"{label} yaw_goal_tolerance_rad must be absent")
        elif (
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
            if _finite_number(reported_yaw_tolerance) else None
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
        if yaw_tolerance is not None and (
            not _finite_number(yaw_error)
            or float(yaw_error) > yaw_tolerance + YAW_OBSERVER_MARGIN_RAD
        ):
            errors.append(
                f"{label} goal_yaw_error_rad exceeds {yaw_tolerance}")
        plan_yaw_error = result.get("signed_plan_goal_yaw_error_rad")
        if yaw_tolerance is None:
            if plan_yaw_error is not None:
                errors.append(f"{label} free terminal must not report planned yaw")
        elif (
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

        _validate_forward_ackermann_contract(
            result, expected[1], expected[2], label, errors
        )

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
                <= float(internal_vx_max)
                or abs(
                    float(internal_vx_max)
                    - SIMULATION_HANDOFF_SPEED_CAP_MPS
                ) > CONFIG_TOLERANCE_EPSILON
            ):
                errors.append(
                    f"{label} virtual-forward vx bounds are invalid")
            if (
                not _finite_number(speed_cap)
                or abs(
                    float(speed_cap) - SIMULATION_HANDOFF_SPEED_CAP_MPS
                ) > CONFIG_TOLERANCE_EPSILON
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
                or abs(float(angular_cap) - SIMULATION_HANDOFF_WZ_CAP_RADPS)
                > CONFIG_TOLERANCE_EPSILON
            ):
                errors.append(
                    f"{label} handoff angular cap must be "
                    f"{SIMULATION_HANDOFF_WZ_CAP_RADPS:.2f}")
            if (
                not _finite_number(turning_radius)
                or abs(float(turning_radius) - SIMULATION_MINIMUM_TURNING_RADIUS_M)
                > CONFIG_TOLERANCE_EPSILON
            ):
                errors.append(
                    f"{label} handoff turning radius must be "
                    f"{SIMULATION_MINIMUM_TURNING_RADIUS_M:.2f}")
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
