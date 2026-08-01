"""Compatibility import for the task-owned planning-segment schema.

The editor remains a consumer of the ROS-independent mission contract.  The
implementation lives in ``smartcar_task`` so the runtime, editor, and
simulation cannot diverge without a package-level dependency cycle.
"""

from smartcar_task.planning_segments import (
    ALLOWED_DIRECTIONS,
    PLANNING_SEGMENTS_KEY,
    PlanningSegment,
    PlanningSegmentError,
    derive_planning_segments,
    load_planning_segments,
    materialize_mission_route,
    materialize_navigation_segments,
    materialize_route,
    planning_segments_document,
    validate_planning_segments,
)


__all__ = [
    "ALLOWED_DIRECTIONS",
    "PLANNING_SEGMENTS_KEY",
    "PlanningSegment",
    "PlanningSegmentError",
    "derive_planning_segments",
    "load_planning_segments",
    "materialize_mission_route",
    "materialize_navigation_segments",
    "materialize_route",
    "planning_segments_document",
    "validate_planning_segments",
]
