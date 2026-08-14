"""Materialize valid Nav2 poses without changing the authored route.

Nav2 Humble requires every ``PoseStamped`` goal to carry a unit quaternion.
The YAML zero quaternion is only an authoring sentinel for a position-only
transit constraint.  Before a native Nav2 action is sent, it becomes a local
route tangent so Smac receives a valid pose; the position goal checker still
owns completion and Nav2 remains the sole planner.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Sequence

from smartcar_task.waypoints import (
    QUATERNION_NORM_TOLERANCE,
    Waypoint,
    is_heading_locked,
)


ZERO_QUATERNION = (0.0, 0.0, 0.0, 0.0)
_POSITION_EPSILON = 1.0e-6


class RouteGeometryError(ValueError):
    """Raised when a route cannot produce valid native Nav2 goal poses."""


def _require_unit_orientation(waypoint: Waypoint) -> None:
    orientation = waypoint.orientation
    if not all(math.isfinite(component) for component in orientation):
        raise RouteGeometryError(
            f"{waypoint.id}: heading-locked waypoint requires a finite unit orientation"
        )
    norm = math.sqrt(sum(component * component for component in orientation))
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise RouteGeometryError(
            f"{waypoint.id}: heading-locked waypoint requires a unit orientation"
        )


def _tangent_orientation(route: tuple[Waypoint, ...], index: int) -> tuple[float, ...]:
    """Return a unit yaw from the two-sided route tangent when available."""
    origin_x, origin_y, _origin_z = route[index].position
    previous = None
    for candidate in reversed(route[:index]):
        dx = origin_x - candidate.position[0]
        dy = origin_y - candidate.position[1]
        if math.hypot(dx, dy) > _POSITION_EPSILON:
            previous = candidate
            break

    following = None
    for candidate in route[index + 1:]:
        dx = candidate.position[0] - origin_x
        dy = candidate.position[1] - origin_y
        if math.hypot(dx, dy) > _POSITION_EPSILON:
            following = candidate
            break

    fallback_dx = fallback_dy = None
    if previous is not None and following is not None:
        dx = following.position[0] - previous.position[0]
        dy = following.position[1] - previous.position[1]
        # A U-turn can put the adjacent route points at the same position,
        # leaving no two-sided tangent. Preserve a valid forward pose from
        # the outgoing route chord for this degenerate geometry only.
        fallback_dx = following.position[0] - origin_x
        fallback_dy = following.position[1] - origin_y
    elif following is not None:
        dx = following.position[0] - origin_x
        dy = following.position[1] - origin_y
    elif previous is not None:
        dx = origin_x - previous.position[0]
        dy = origin_y - previous.position[1]
    else:
        raise RouteGeometryError(
            f"{route[index].id}: position-only waypoint has no route tangent")

    if math.hypot(dx, dy) <= _POSITION_EPSILON:
        if fallback_dx is None:
            raise RouteGeometryError(
                f"{route[index].id}: adjacent route points have no center tangent")
        dx, dy = fallback_dx, fallback_dy
    yaw = math.atan2(dy, dx)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def materialize_free_yaws(waypoints: Sequence[Waypoint]) -> tuple[Waypoint, ...]:
    """Return action inputs with authored headings or valid transit tangents.

    The derived orientation is not written back to YAML and is not a path or a
    controller directive.  It only lets Nav2's native Smac planner construct
    a legal pose for a semantic position-only route point.
    """
    route = tuple(waypoints)
    if not route:
        return ()
    identifiers = [waypoint.id for waypoint in route]
    if len(identifiers) != len(set(identifiers)):
        raise RouteGeometryError("position-only route has duplicate waypoint ids")

    normalized: list[Waypoint] = []
    for index, waypoint in enumerate(route):
        if waypoint.direction != "forward":
            raise RouteGeometryError(f"{waypoint.id}: invalid travel direction")
        if not all(math.isfinite(float(value)) for value in waypoint.position[:2]):
            raise RouteGeometryError(f"{waypoint.id}: position must be finite")
        if is_heading_locked(waypoint):
            _require_unit_orientation(waypoint)
            normalized.append(waypoint)
        else:
            normalized.append(replace(
                waypoint,
                orientation=_tangent_orientation(route, index),
            ))
    return tuple(normalized)
