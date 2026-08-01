"""Normalize semantic waypoints for Nav2's position-only transit contract.

Nav2 Humble's Smac planner has no orientation-free ``PoseStamped`` goal: a
zero quaternion is otherwise interpreted as a concrete yaw.  The task layer
therefore preserves zero quaternions for ordinary waypoints and the custom
Nav2 BT node resolves them against the live costmap immediately before
planning.  This module deliberately does *not* invent a route tangent: doing
so would turn an operator-free position into the hard yaw constraint that
causes Ackermann loops at tight corners.
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


class RouteGeometryError(ValueError):
    """Raised when a route cannot safely reach the Nav2 free-heading node."""


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


def materialize_free_yaws(waypoints: Sequence[Waypoint]) -> tuple[Waypoint, ...]:
    """Return Nav2 action inputs with nonsemantic headings explicitly free.

    Only P, QR, and VLM retain their authored base-frame yaw.  All other
    points become the all-zero sentinel consumed by
    ``Compute*FreeHeadingPath*``.  Any stale arrow in YAML is deliberately
    ignored so editing a transit point cannot accidentally change the route.
    """
    route = tuple(waypoints)
    if not route:
        return ()
    identifiers = [waypoint.id for waypoint in route]
    if len(identifiers) != len(set(identifiers)):
        raise RouteGeometryError("position-only route has duplicate waypoint ids")

    normalized: list[Waypoint] = []
    for waypoint in route:
        if waypoint.direction not in {"forward", "reverse"}:
            raise RouteGeometryError(f"{waypoint.id}: invalid travel direction")
        if not all(math.isfinite(float(value)) for value in waypoint.position[:2]):
            raise RouteGeometryError(f"{waypoint.id}: position must be finite")
        if is_heading_locked(waypoint):
            _require_unit_orientation(waypoint)
            normalized.append(waypoint)
        else:
            normalized.append(replace(waypoint, orientation=ZERO_QUATERNION))
    return tuple(normalized)
