"""ROS-independent semantic waypoint loading and validation."""
from dataclasses import dataclass
import math
from pathlib import Path

import yaml


ALLOWED_TASKS = frozenset({
    "start",
    "qr",
    "vlm",
    "corridor",
    "loop",
    "return",
})
ORIGIN_TOLERANCE = 1e-9
QUATERNION_NORM_TOLERANCE = 1e-3


@dataclass(frozen=True)
class Waypoint:
    frame_id: str
    position: tuple
    orientation: tuple
    task: str


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
    frame_id = item.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError(f"waypoints[{index}].frame_id must be nonempty")
    if frame_id.strip() != "odom_combined":
        raise ValueError(
            f"waypoints[{index}].frame_id must be odom_combined")

    task = item.get("task")
    if not isinstance(task, str) or task not in ALLOWED_TASKS:
        raise ValueError(f"waypoints[{index}] has unknown task {task!r}")

    pose = _mapping(item.get("pose"), f"waypoints[{index}].pose")
    position = _components(
        _mapping(
            pose.get("position"),
            f"waypoints[{index}].pose.position",
        ),
        ("x", "y", "z"),
        f"waypoints[{index}].pose.position",
    )
    orientation = _components(
        _mapping(
            pose.get("orientation"),
            f"waypoints[{index}].pose.orientation",
        ),
        ("x", "y", "z", "w"),
        f"waypoints[{index}].pose.orientation",
    )
    norm = math.sqrt(sum(value * value for value in orientation))
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise ValueError(
            f"waypoints[{index}] orientation must have unit length")

    return Waypoint(
        frame_id=frame_id.strip(),
        position=position,
        orientation=orientation,
        task=task,
    )


def _is_origin(waypoint):
    x, y, z = waypoint.position
    qx, qy, qz, qw = waypoint.orientation
    return (
        abs(x) <= ORIGIN_TOLERANCE
        and abs(y) <= ORIGIN_TOLERANCE
        and abs(z) <= ORIGIN_TOLERANCE
        and abs(qx) <= ORIGIN_TOLERANCE
        and abs(qy) <= ORIGIN_TOLERANCE
        and abs(qz) <= ORIGIN_TOLERANCE
        and abs(abs(qw) - 1.0) <= ORIGIN_TOLERANCE
    )


def _validate_sequence(waypoints):
    tasks = [waypoint.task for waypoint in waypoints]
    if tasks[0] != "start":
        raise ValueError("first waypoint task must be start")
    if tasks[-1] != "return":
        raise ValueError("last waypoint task must be return")
    if tasks.count("start") != 1 or tasks.count("return") != 1:
        raise ValueError("start and return tasks must each occur exactly once")
    if set(tasks) != ALLOWED_TASKS or len(tasks) != len(ALLOWED_TASKS):
        raise ValueError("mission must contain each competition task exactly once")
    if not _is_origin(waypoints[0]) or not _is_origin(waypoints[-1]):
        raise ValueError("start and return must match the P-zone origin")


def load_waypoints(path):
    waypoint_path = Path(path)
    try:
        document = yaml.safe_load(waypoint_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot load waypoint file: {error}") from error

    root = _mapping(document, "waypoint document")
    raw_waypoints = root.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a nonempty list")
    waypoints = tuple(
        _parse_waypoint(raw, index)
        for index, raw in enumerate(raw_waypoints)
    )
    _validate_sequence(waypoints)
    return waypoints
