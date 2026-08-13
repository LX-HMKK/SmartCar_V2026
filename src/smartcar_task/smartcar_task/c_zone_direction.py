"""Select the approved C-zone traversal orientation in memory.

The authored waypoint YAML remains the counterclockwise route.  Callers that
need the clockwise variant receive immutable ``Waypoint`` replacements; this
module never writes route documents or changes planning segments.
"""

from dataclasses import replace
from typing import Sequence

from smartcar_task.waypoints import Waypoint


C_ZONE_DIRECTION_COUNTERCLOCKWISE = "counterclockwise"
C_ZONE_DIRECTION_CLOCKWISE = "clockwise"
C_ZONE_DIRECTIONS = frozenset({
    C_ZONE_DIRECTION_COUNTERCLOCKWISE,
    C_ZONE_DIRECTION_CLOCKWISE,
})

# Short aliases keep the internal branch readable; callers should use the
# fully named constants to match the launch parameter contract.
COUNTERCLOCKWISE = C_ZONE_DIRECTION_COUNTERCLOCKWISE
CLOCKWISE = C_ZONE_DIRECTION_CLOCKWISE

# The approved C-zone mirror is the vertical field axis x = 2.0 m.
C_ZONE_MIRROR_X_SUM_M = 4.0
C_ZONE_MIRROR_WAYPOINT_IDS = (
    "via_2",
    "via_3",
    "via_6",
    "c_corner_1",
    "via_4",
    "via_5",
)
C_ZONE_MIRRORED_IDS = frozenset(C_ZONE_MIRROR_WAYPOINT_IDS)
C_ZONE_TERMINAL_ID = "c_corner_1"


class CZoneDirectionError(ValueError):
    """Raised when a C-zone direction cannot produce an approved route."""


def normalize_c_zone_direction(direction: str) -> str:
    """Validate and return one explicit C-zone traversal direction."""
    normalized = direction.strip().lower() if isinstance(direction, str) else ""
    if normalized not in C_ZONE_DIRECTIONS:
        allowed = ", ".join(sorted(C_ZONE_DIRECTIONS))
        raise CZoneDirectionError(
            f"c_zone_direction must be one of {allowed}, got {direction!r}"
        )
    return normalized


def _validated_waypoints(waypoints: Sequence[Waypoint]) -> tuple[Waypoint, ...]:
    """Require every approved mirror target exactly once before transforming."""
    items = tuple(waypoints)
    by_id: dict[str, Waypoint] = {}
    duplicate_ids: set[str] = set()
    for index, waypoint in enumerate(items):
        if not isinstance(waypoint, Waypoint):
            raise CZoneDirectionError(f"waypoints[{index}] must be a Waypoint")
        if waypoint.id in by_id:
            duplicate_ids.add(waypoint.id)
        by_id[waypoint.id] = waypoint

    missing = [
        waypoint_id
        for waypoint_id in C_ZONE_MIRROR_WAYPOINT_IDS
        if waypoint_id not in by_id
    ]
    duplicates = [
        waypoint_id
        for waypoint_id in C_ZONE_MIRROR_WAYPOINT_IDS
        if waypoint_id in duplicate_ids
    ]
    if missing or duplicates:
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if duplicates:
            details.append("duplicate " + ", ".join(duplicates))
        raise CZoneDirectionError(
            "C-zone direction requires each fixed waypoint exactly once: "
            + "; ".join(details)
        )
    return items


def _mirrored_waypoint(waypoint: Waypoint) -> Waypoint:
    """Reflect one approved C-zone waypoint across x = 2.0 m."""
    try:
        x, y, z = waypoint.position
    except (TypeError, ValueError) as error:
        raise CZoneDirectionError(
            f"{waypoint.id}: position must contain x, y, z"
        ) from error

    changes = {
        "position": (C_ZONE_MIRROR_X_SUM_M - x, y, z),
    }
    if waypoint.id == C_ZONE_TERMINAL_ID:
        try:
            qx, qy, qz, qw = waypoint.orientation
        except (TypeError, ValueError) as error:
            raise CZoneDirectionError(
                f"{waypoint.id}: orientation must contain x, y, z, w"
            ) from error
        # Reflection in the vertical C-zone axis maps a planar yaw theta to
        # pi - theta. For a planar quaternion this is exactly z/w exchange.
        changes["orientation"] = (qx, qy, qw, qz)
    return replace(waypoint, **changes)


def apply_c_zone_direction(
    waypoints: Sequence[Waypoint],
    direction: str = COUNTERCLOCKWISE,
) -> tuple[Waypoint, ...]:
    """Return the approved C-zone route variant without mutating YAML or input.

    ``counterclockwise`` is the authored/default route.  ``clockwise`` reflects
    only the approved C-zone IDs and flips C1's locked planar heading.  Both
    variants retain route IDs, order, task types, profiles, directions, and
    planning-segment topology.
    """
    selected = normalize_c_zone_direction(direction)
    items = _validated_waypoints(waypoints)
    if selected == COUNTERCLOCKWISE:
        return items

    return tuple(
        _mirrored_waypoint(waypoint)
        if waypoint.id in C_ZONE_MIRRORED_IDS
        else waypoint
        for waypoint in items
    )
