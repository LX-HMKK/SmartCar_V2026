"""ROS-independent semantic waypoint loading and validation."""
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import tempfile

import yaml


ALLOWED_TASKS = frozenset({
    "start",
    "qr",
    "vlm",
    "corridor",
    "loop",
    "return",
    "nav",       # pure navigation pass-through (no vision/media subtask)
    "via",       # editor-created route constraint (no vision/media subtask)
})
ALLOWED_DIRECTIONS = frozenset({"forward", "reverse"})
ALLOWED_GOAL_PROFILES = frozenset({
    "standard",
    "precise",
    "reverse_handoff",
})
ORIGIN_TOLERANCE = 1e-9
QUATERNION_NORM_TOLERANCE = 1e-3
WAYPOINT_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")


@dataclass(frozen=True)
class Waypoint:
    frame_id: str
    position: tuple
    orientation: tuple
    task: str
    direction: str = "forward"
    id: str = ""
    goal_profile: str = "standard"


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _finite_number(value, label):
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label} must be a finite number") from error
    if not math.isfinite(result):
        raise ValueError(f"{label} must be a finite number")
    return result


def _components(mapping, names, label):
    return tuple(
        _finite_number(mapping.get(name), f"{label}.{name}")
        for name in names
    )


def _parse_waypoint(raw, index):
    item = _mapping(raw, f"waypoints[{index}]")
    waypoint_id = item.get("id", f"waypoint_{index}")
    if (
        not isinstance(waypoint_id, str)
        or WAYPOINT_ID_PATTERN.fullmatch(waypoint_id.strip()) is None
    ):
        raise ValueError(f"waypoints[{index}].id is invalid")
    frame_id = item.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError(f"waypoints[{index}].frame_id must be nonempty")
    if frame_id.strip() != "odom_combined":
        raise ValueError(
            f"waypoints[{index}].frame_id must be odom_combined")

    task = item.get("task")
    if not isinstance(task, str) or task not in ALLOWED_TASKS:
        raise ValueError(f"waypoints[{index}] has unknown task {task!r}")

    direction = item.get("direction", "forward")
    if not isinstance(direction, str) or direction not in ALLOWED_DIRECTIONS:
        raise ValueError(
            f"waypoints[{index}] has unknown direction {direction!r}")

    goal_profile = item.get("goal_profile", "standard")
    if (
        not isinstance(goal_profile, str)
        or goal_profile not in ALLOWED_GOAL_PROFILES
    ):
        raise ValueError(
            f"waypoints[{index}] has unknown goal_profile {goal_profile!r}")
    if direction == "reverse" and goal_profile not in {
        "standard",
        "reverse_handoff",
    }:
        raise ValueError(
            f"waypoints[{index}] reverse goals must use the standard "
            "or reverse_handoff profile")
    if direction != "reverse" and goal_profile == "reverse_handoff":
        raise ValueError(
            f"waypoints[{index}] reverse_handoff goals must be reverse")

    pose = _mapping(item.get("pose"), f"waypoints[{index}].pose")
    position = _components(
        _mapping(
            pose.get("position"),
            f"waypoints[{index}].pose.position",
        ),
        ("x", "y", "z"),
        f"waypoints[{index}].pose.position",
    )
    orientation_raw = pose.get("orientation")
    if orientation_raw is None:
        # Intermediate pass-through waypoint: no orientation at all.
        # Treated as all-zeros (no yaw constraint).
        orientation = (0.0, 0.0, 0.0, 0.0)
    else:
        orientation = _components(
            _mapping(
                orientation_raw,
                f"waypoints[{index}].pose.orientation",
            ),
            ("x", "y", "z", "w"),
            f"waypoints[{index}].pose.orientation",
        )
        norm = math.sqrt(sum(value * value for value in orientation))
        zero_norm = norm <= QUATERNION_NORM_TOLERANCE
        if not zero_norm and abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
            raise ValueError(
                f"waypoints[{index}] orientation must have unit length"
                f" or be all zeros (orientation unconstrained)")

    return Waypoint(
        frame_id=frame_id.strip(),
        position=position,
        orientation=orientation,
        task=task,
        direction=direction,
        id=waypoint_id.strip(),
        goal_profile=goal_profile,
    )


def is_zero_quaternion(orientation):
    """Check if orientation tuple is all zeros (Nav2 convention for orientation
    unconstrained pass-through waypoints in ThroughPoses segments)."""
    return all(abs(v) <= QUATERNION_NORM_TOLERANCE for v in orientation)


def _is_origin_position(waypoint):
    x, y, z = waypoint.position
    return (
        abs(x) <= ORIGIN_TOLERANCE
        and abs(y) <= ORIGIN_TOLERANCE
        and abs(z) <= ORIGIN_TOLERANCE
    )


def _faces_positive_x(waypoint):
    qx, qy, qz, qw = waypoint.orientation
    return (
        abs(qx) <= ORIGIN_TOLERANCE
        and abs(qy) <= ORIGIN_TOLERANCE
        and abs(qz) <= ORIGIN_TOLERANCE
        and abs(abs(qw) - 1.0) <= ORIGIN_TOLERANCE
    )


def _waypoint_mapping(waypoint, index=None):
    label = "waypoint" if index is None else f"waypoints[{index}]"
    if not isinstance(waypoint, Waypoint):
        raise ValueError(f"{label} must be a Waypoint")
    try:
        x, y, z = waypoint.position
        qx, qy, qz, qw = waypoint.orientation
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} has invalid pose dimensions") from error
    result = {
        "id": waypoint.id,
        "frame_id": waypoint.frame_id,
        "pose": {
            "position": {"x": x, "y": y, "z": z},
        },
        "task": waypoint.task,
        "direction": waypoint.direction,
        "goal_profile": waypoint.goal_profile,
    }
    # Only include orientation if it has a meaningful quaternion
    # (non-zero norm). Intermediate pass-through waypoints omit
    # orientation entirely.
    orient_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if orient_norm > 1e-9:
        result["pose"]["orientation"] = {"x": qx, "y": qy, "z": qz, "w": qw}
    return result


def validate_waypoints(waypoints):
    """Validate the ordered semantic mission without requiring ROS."""
    candidates = tuple(waypoints)
    if not candidates:
        raise ValueError("waypoints must be a nonempty sequence")
    waypoints = tuple(
        _parse_waypoint(_waypoint_mapping(waypoint, index), index)
        for index, waypoint in enumerate(candidates)
    )
    tasks = [waypoint.task for waypoint in waypoints]
    identifiers = [waypoint.id for waypoint in waypoints]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("waypoint ids must be unique")
    if tasks[0] != "start":
        raise ValueError("first waypoint task must be start")
    if tasks[-1] != "return":
        raise ValueError("last waypoint task must be return")
    for task in ("start", "return"):
        if tasks.count(task) != 1:
            raise ValueError(f"mission task {task} must occur exactly once")

    # validate sequence with a simple state machine:
    #   start → (qr|nav) → corridor* → (vlm|nav) → loop* → corridor* → return
    # ``via`` is a direction-preserving navigation constraint that may appear
    # anywhere between semantic endpoints.  It never starts a media task.
    # Loop corners are optional when C-zone ring walls constrain the path.
    loop_count = tasks.count("loop")

    state = "start"
    for i, task in enumerate(tasks):
        expected_direction = None
        if state == "start":
            if task != "start":
                raise ValueError(f"waypoint {i}: expected start, got {task}")
            expected_direction = "forward"
            state = "before_loop"
        elif state == "before_loop":
            if task == "via":
                expected_direction = "forward"
            elif task in ("qr", "nav"):
                expected_direction = "forward"
                state = "outbound_corridor"
            else:
                raise ValueError(
                    f"waypoint {i}: expected qr or nav, got {task}"
                )
        elif state == "outbound_corridor":
            if task == "via":
                expected_direction = "reverse"
            elif task == "corridor":
                expected_direction = "reverse"
            elif task in ("vlm", "nav"):
                expected_direction = "reverse"
                state = "loop_or_return"
            else:
                raise ValueError(
                    f"waypoint {i}: expected corridor or vlm/nav, got {task}"
                )
        elif state == "loop_or_return":
            if task == "via":
                expected_direction = "forward"
            elif task == "loop":
                expected_direction = "forward"
            elif task == "corridor":
                expected_direction = "forward"
                state = "return_corridor"
            else:
                raise ValueError(
                    f"waypoint {i}: expected loop or corridor, got {task}"
                )
        elif state == "return_corridor":
            if task == "via":
                expected_direction = "forward"
            elif task == "corridor":
                expected_direction = "forward"
            elif task == "return":
                expected_direction = "forward"
                state = "done"
            else:
                raise ValueError(
                    f"waypoint {i}: out-of-sequence task {task} "
                    f"(expected corridor or return in this segment)"
                )
        elif state == "done":
            raise ValueError(f"waypoint {i}: unexpected {task} after return")
        if waypoints[i].direction != expected_direction:
            raise ValueError(
                f"waypoint {i}: direction must be {expected_direction}, "
                f"got {waypoints[i].direction}"
            )
    if state != "done":
        raise ValueError(
            "mission order must be start, (qr|nav), corridor transit(s), "
            "(vlm|nav), optional loop corner(s), corridor transit(s), return"
        )
    if not _is_origin_position(waypoints[0]) or not _faces_positive_x(
        waypoints[0]
    ):
        raise ValueError("start must match the P-zone origin and face +X")
    if not _is_origin_position(waypoints[-1]):
        raise ValueError("return must match the P-zone origin")
    return waypoints


def load_waypoint_document(path):
    """Load the root mapping and validated waypoints for editor round-trips."""
    waypoint_path = Path(path)
    try:
        document = yaml.safe_load(waypoint_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot load waypoint file: {error}") from error

    root = _mapping(document, "waypoint document")
    raw_waypoints = root.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a nonempty list")
    waypoints = tuple(
        _parse_waypoint(raw, index)
        for index, raw in enumerate(raw_waypoints)
    )
    return dict(root), validate_waypoints(waypoints)


def load_waypoints(path):
    return load_waypoint_document(path)[1]


def write_waypoints_atomic(path, template, waypoints):
    """Validate and atomically replace one semantic waypoint YAML file."""
    validated = validate_waypoints(waypoints)
    root = dict(template)
    root["calibrated"] = False
    root["waypoints"] = [_waypoint_mapping(item) for item in validated]
    payload = yaml.safe_dump(
        root,
        allow_unicode=True,
        sort_keys=False,
        width=100,
    )
    destination = Path(path)
    try:
        destination.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=str(destination.parent),
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, destination)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
    except OSError as error:
        raise ValueError(f"cannot write waypoint file: {error}") from error
