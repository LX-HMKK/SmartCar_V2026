"""Route-segment schema shared by the waypoint editor and simulation tools.

The mission still stores semantic waypoints in one YAML document.  A
``planning_segments`` section gives the editor an explicit, ordered route
view: each segment has a start, a direction, zero or more pass-through
constraints, and an end.  Saving a route materializes that order back into
the semantic waypoint list so existing task code continues to consume it.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import re
from typing import Any, Mapping, Sequence


PLANNING_SEGMENTS_KEY = "planning_segments"
ALLOWED_DIRECTIONS = frozenset({"forward", "reverse"})
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


def derive_planning_segments(waypoints: Sequence[Any]) -> tuple[PlanningSegment, ...]:
    """Create a useful editable baseline for legacy waypoint documents.

    Direction changes and QR/VLM/return task boundaries start a new segment.
    Users can then split, merge, or reorder pass-through constraints in the UI.
    """
    items = tuple(waypoints)
    _waypoint_index(items)
    if len(items) < 2:
        return ()

    segments: list[PlanningSegment] = []
    start_index = 0
    direction = getattr(items[1], "direction", "forward")
    through: list[str] = []
    for index in range(1, len(items)):
        item = items[index]
        next_direction = (
            getattr(items[index + 1], "direction", None)
            if index + 1 < len(items)
            else None
        )
        endpoint = (
            getattr(item, "task", "") in {*TERMINAL_TASKS, "return"}
            or next_direction is not None and next_direction != direction
        )
        if endpoint:
            segments.append(PlanningSegment(
                id=f"segment_{len(segments) + 1}",
                direction=direction,
                start_id=items[start_index].id,
                end_id=item.id,
                through_ids=tuple(through),
            ))
            start_index = index
            direction = next_direction or direction
            through.clear()
        else:
            through.append(item.id)
    return tuple(segments)


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
    """Load explicit segments or derive an editable baseline for legacy YAML."""
    root = _mapping(document, "waypoint document")
    raw = root.get(PLANNING_SEGMENTS_KEY)
    if raw is None:
        return derive_planning_segments(waypoints)
    if not isinstance(raw, list):
        raise PlanningSegmentError(f"{PLANNING_SEGMENTS_KEY} must be a list")
    return validate_planning_segments(
        tuple(_parse_segment(item, index) for index, item in enumerate(raw)),
        waypoints,
    )


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
