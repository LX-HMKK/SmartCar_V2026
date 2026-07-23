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
    id: str = ""


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
        id=waypoint_id.strip(),
    )


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
    return {
        "id": waypoint.id,
        "frame_id": waypoint.frame_id,
        "pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
        "task": waypoint.task,
    }


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
    for task in ("start", "qr", "vlm", "return"):
        if tasks.count(task) != 1:
            raise ValueError(f"mission task {task} must occur exactly once")

    loop_count = tasks.count("loop")
    expected = ["start", "qr", "corridor", "vlm"]
    expected.extend(["loop"] * loop_count)
    expected.extend(["corridor", "return"])
    if loop_count < 3 or tasks != expected:
        raise ValueError(
            "mission order must be start, qr, outbound corridor, vlm corner, "
            "at least three loop corners, return corridor, return"
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
