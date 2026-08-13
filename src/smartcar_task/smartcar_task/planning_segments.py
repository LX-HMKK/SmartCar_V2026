"""Route-segment schema shared by task execution, editing, and simulation.

The semantic waypoint document keeps one ordered set of named constraints. A
``planning_segments`` section turns that set into an explicit sequence of
constant-direction Nav2 actions.  Keeping the schema in ``smartcar_task``
lets the runtime own the contract without creating a dependency on editor
tools.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import re
from typing import Any, Mapping, Sequence

from smartcar_task.waypoints import is_heading_locked



PLANNING_SEGMENTS_KEY = "planning_segments"
ALLOWED_DIRECTIONS = frozenset({"forward"})
SEGMENT_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
TERMINAL_TASKS = frozenset({"qr", "vlm"})


class PlanningSegmentError(ValueError):
    """Raised when a planning-segment document cannot form one mission route."""


@dataclass(frozen=True)
class PlanningSegment:
    """One ordered, constant-direction route portion."""

    id: str
    direction: str
    start_id: str
    end_id: str
    through_ids: tuple[str, ...] = ()

    @property
    def route_ids(self) -> tuple[str, ...]:
        return (self.start_id, *self.through_ids, self.end_id)


def _goal_field(goal: Any, name: str, default: Any = None) -> Any:
    """Read a route-goal field from either a Waypoint or manifest mapping."""
    if isinstance(goal, Mapping):
        return goal.get(name, default)
    return getattr(goal, name, default)


def allows_precise_terminal_through_poses(goals: Sequence[Any]) -> bool:
    """Return whether a forward ThroughPoses action ends at a precise goal.

    Standard ``via`` goals may constrain the approach to a QR or VLM task
    point.  The final precise goal remains heading-locked and uses a dedicated
    tree so Nav2 applies its strict pose/yaw completion check only there.
    """
    items = tuple(goals)
    if len(items) < 2:
        return False
    if any(_goal_field(goal, "direction") != "forward" for goal in items):
        return False
    if any(
        _goal_field(goal, "goal_profile", "standard") != "standard"
        for goal in items[:-1]
    ):
        return False
    terminal = items[-1]
    if _goal_field(terminal, "goal_profile", "standard") != "precise":
        return False
    if isinstance(terminal, Mapping):
        return _goal_field(terminal, "heading_mode") == "locked"
    try:
        return is_heading_locked(terminal)
    except (TypeError, ValueError):
        return False


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlanningSegmentError(f"{label} must be a mapping")
    return value


def _identifier(value: Any, label: str) -> str:
    if not isinstance(value, str) or SEGMENT_ID_PATTERN.fullmatch(value.strip()) is None:
        raise PlanningSegmentError(f"{label} is invalid")
    return value.strip()


def _direction(value: Any, label: str) -> str:
    if not isinstance(value, str) or value not in ALLOWED_DIRECTIONS:
        raise PlanningSegmentError(
            f"{label} must be one of {', '.join(sorted(ALLOWED_DIRECTIONS))}"
        )
    return value


def _through_ids(value: Any, label: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list):
        raise PlanningSegmentError(f"{label} must be a list")
    return tuple(_identifier(item, f"{label}[{index}]") for index, item in enumerate(value))


def _waypoint_index(waypoints: Sequence[Any]) -> dict[str, Any]:
    indexed: dict[str, Any] = {}
    for index, waypoint in enumerate(waypoints):
        waypoint_id = getattr(waypoint, "id", None)
        if not isinstance(waypoint_id, str) or not waypoint_id:
            raise PlanningSegmentError(f"waypoints[{index}] has no valid id")
        if waypoint_id in indexed:
            raise PlanningSegmentError(f"duplicate waypoint id {waypoint_id!r}")
        indexed[waypoint_id] = waypoint
    if not indexed:
        raise PlanningSegmentError("waypoints must not be empty")
    return indexed


def _parse_segment(raw: Any, index: int) -> PlanningSegment:
    item = _mapping(raw, f"{PLANNING_SEGMENTS_KEY}[{index}]")
    allowed = {"id", "direction", "start_id", "end_id", "through_ids"}
    unknown = sorted(set(item) - allowed)
    if unknown:
        raise PlanningSegmentError(
            f"{PLANNING_SEGMENTS_KEY}[{index}] has unknown fields: "
            + ", ".join(unknown)
        )
    return PlanningSegment(
        id=_identifier(item.get("id"), f"{PLANNING_SEGMENTS_KEY}[{index}].id"),
        direction=_direction(
            item.get("direction"), f"{PLANNING_SEGMENTS_KEY}[{index}].direction"
        ),
        start_id=_identifier(
            item.get("start_id"), f"{PLANNING_SEGMENTS_KEY}[{index}].start_id"
        ),
        end_id=_identifier(
            item.get("end_id"), f"{PLANNING_SEGMENTS_KEY}[{index}].end_id"
        ),
        through_ids=_through_ids(
            item.get("through_ids", []),
            f"{PLANNING_SEGMENTS_KEY}[{index}].through_ids",
        ),
    )


def validate_planning_segments(
    segments: Sequence[PlanningSegment], waypoints: Sequence[Any]
) -> tuple[PlanningSegment, ...]:
    """Validate continuity, coverage, semantic task boundaries, and directions."""
    items = tuple(waypoints)
    indexed = _waypoint_index(items)
    candidates = tuple(segments)
    if not candidates:
        raise PlanningSegmentError(f"{PLANNING_SEGMENTS_KEY} must not be empty")
    if any(not isinstance(item, PlanningSegment) for item in candidates):
        raise PlanningSegmentError(f"{PLANNING_SEGMENTS_KEY} must contain PlanningSegment values")

    segment_ids = [item.id for item in candidates]
    if len(segment_ids) != len(set(segment_ids)):
        raise PlanningSegmentError("planning segment ids must be unique")
    for index, segment in enumerate(candidates):
        _identifier(segment.id, f"{PLANNING_SEGMENTS_KEY}[{index}].id")
        _direction(segment.direction, f"{PLANNING_SEGMENTS_KEY}[{index}].direction")
        route_ids = segment.route_ids
        if len(route_ids) != len(set(route_ids)):
            raise PlanningSegmentError(
                f"{PLANNING_SEGMENTS_KEY}[{index}] repeats a waypoint"
            )
        for waypoint_id in route_ids:
            if waypoint_id not in indexed:
                raise PlanningSegmentError(
                    f"{PLANNING_SEGMENTS_KEY}[{index}] references unknown waypoint "
                    f"{waypoint_id!r}"
                )
        if segment.start_id == segment.end_id:
            raise PlanningSegmentError(
                f"{PLANNING_SEGMENTS_KEY}[{index}] start_id and end_id must differ"
            )
        if index and candidates[index - 1].end_id != segment.start_id:
            raise PlanningSegmentError(
                f"{PLANNING_SEGMENTS_KEY}[{index}] must start at the previous segment end"
            )
        if getattr(indexed[segment.end_id], "task", "") == "via":
            raise PlanningSegmentError(
                f"{PLANNING_SEGMENTS_KEY}[{index}].end_id must not be a via waypoint"
            )
        for through_index, waypoint_id in enumerate(segment.through_ids):
            if getattr(indexed[waypoint_id], "task", "") != "via":
                raise PlanningSegmentError(
                    f"{PLANNING_SEGMENTS_KEY}[{index}].through_ids["
                    f"{through_index}] must reference a via waypoint"
                )

    if candidates[0].start_id != items[0].id:
        raise PlanningSegmentError("first planning segment must start at the start waypoint")
    if candidates[-1].end_id != items[-1].id:
        raise PlanningSegmentError("last planning segment must end at the return waypoint")

    route_ids = [candidates[0].start_id]
    for segment in candidates:
        route_ids.extend(segment.through_ids)
        route_ids.append(segment.end_id)
    if len(route_ids) != len(set(route_ids)):
        raise PlanningSegmentError("planning segments visit a waypoint more than once")
    if set(route_ids) != set(indexed):
        missing = sorted(set(indexed) - set(route_ids))
        extra = sorted(set(route_ids) - set(indexed))
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if extra:
            details.append("unknown " + ", ".join(extra))
        raise PlanningSegmentError("planning segments must cover every waypoint: " + "; ".join(details))

    endpoint_ids = {segment.end_id for segment in candidates}
    for waypoint_id, waypoint in indexed.items():
        task = getattr(waypoint, "task", "")
        if task in TERMINAL_TASKS and waypoint_id not in endpoint_ids:
            raise PlanningSegmentError(
                f"semantic task waypoint {waypoint_id!r} must end a planning segment"
            )
        if task == "start" and waypoint_id != candidates[0].start_id:
            raise PlanningSegmentError("start waypoint may only begin the first planning segment")
        if task == "return" and waypoint_id != candidates[-1].end_id:
            raise PlanningSegmentError("return waypoint may only end the last planning segment")
    return candidates


def load_planning_segments(
    document: Mapping[str, Any], waypoints: Sequence[Any]
) -> tuple[PlanningSegment, ...]:
    """Load the route's required explicit navigation segments."""
    root = _mapping(document, "waypoint document")
    if PLANNING_SEGMENTS_KEY not in root:
        raise PlanningSegmentError(
            f"waypoint document must define {PLANNING_SEGMENTS_KEY}"
        )
    raw = root[PLANNING_SEGMENTS_KEY]
    if not isinstance(raw, list):
        raise PlanningSegmentError(f"{PLANNING_SEGMENTS_KEY} must be a list")
    return validate_planning_segments(
        tuple(_parse_segment(item, index) for index, item in enumerate(raw)),
        waypoints,
    )


def select_segment_prefix(
    segments: Sequence[PlanningSegment], end_segment_id: str = ""
) -> tuple[PlanningSegment, ...]:
    """Select a contiguous test prefix without permitting route skips.

    The caller validates the complete route before using this helper.  A field
    test may stop after a named segment, but it always starts at the route's
    first segment and keeps every preceding action.
    """
    items = tuple(segments)
    selected_id = str(end_segment_id).strip()
    if not selected_id:
        return items
    for index, segment in enumerate(items):
        if segment.id == selected_id:
            return items[:index + 1]
    raise PlanningSegmentError(
        f"navigation test segment {selected_id!r} is not in the route")


def materialize_route(
    waypoints: Sequence[Any], segments: Sequence[PlanningSegment]
) -> tuple[Any, ...]:
    """Return mission-order waypoints with travel direction set from segments."""
    indexed = _waypoint_index(waypoints)
    checked = validate_planning_segments(segments, waypoints)
    result = [indexed[checked[0].start_id]]
    for segment in checked:
        for waypoint_id in (*segment.through_ids, segment.end_id):
            result.append(replace(indexed[waypoint_id], direction=segment.direction))
    return tuple(result)


def materialize_mission_route(
    waypoints: Sequence[Any], segments: Sequence[PlanningSegment]
) -> tuple[Any, ...]:
    """Apply planning directions, then enforce the semantic mission contract.

    Segment direction is an execution detail, not authority to turn an
    otherwise invalid route into a mission.  Runtime, simulation, and editor
    callers use this strict variant; the lower-level ``materialize_route``
    remains useful to small geometry tests that intentionally model partial
    routes.
    """
    from smartcar_task.waypoints import validate_waypoints

    return validate_waypoints(materialize_route(waypoints, segments))


def materialize_navigation_segments(
    waypoints: Sequence[Any], segments: Sequence[PlanningSegment]
) -> tuple[tuple[Any, ...], ...]:
    """Return one bounded Nav2 action per explicit planning segment.

    A planning segment is both the editable route model and one costmap
    planning boundary. Combining same-direction regions into one long
    ``NavigateThroughPoses`` request produces one complete path from the
    current costmap. Its ordinary intermediate constraints are submitted only
    to ``ComputePathThroughPoses`` at segment start. ``FollowPath`` then
    tracks that fixed path without resubmitting ``via`` points; it uses the
    action's final endpoint for arrival detection, never a ``via`` point.
    """
    indexed = _waypoint_index(waypoints)
    checked = validate_planning_segments(segments, waypoints)
    actions = tuple(
        tuple(
            replace(indexed[waypoint_id], direction=segment.direction)
            for waypoint_id in (*segment.through_ids, segment.end_id)
        )
        for segment in checked
    )
    for action in actions:
        nonstandard_goal_ids = [
            waypoint.id
            for waypoint in action
            if getattr(waypoint, "goal_profile", "standard") != "standard"
        ]
        if (
            len(action) > 1
            and nonstandard_goal_ids
            and not allows_precise_terminal_through_poses(action)
        ):
            raise PlanningSegmentError(
                "NavigateThroughPoses cannot combine nonstandard goal "
                "profiles except a terminal locked precise goal; "
                "split into single-goal actions: "
                + ", ".join(nonstandard_goal_ids)
            )
    return actions


def planning_segments_document(
    template: Mapping[str, Any], segments: Sequence[PlanningSegment]
) -> dict[str, Any]:
    """Return a root YAML template with a stable, human-editable segment list."""
    root = dict(template)
    root[PLANNING_SEGMENTS_KEY] = [
        {
            "id": segment.id,
            "direction": segment.direction,
            "start_id": segment.start_id,
            "end_id": segment.end_id,
            "through_ids": list(segment.through_ids),
        }
        for segment in segments
    ]
    return root
