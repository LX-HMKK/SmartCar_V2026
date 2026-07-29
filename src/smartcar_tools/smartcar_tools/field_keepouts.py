"""Shared simulation keepout geometry used by route editing preflight.

The values deliberately mirror the simulation mask: the central C-zone core
forces a loop while the outer limits prevent plans from escaping the stadium.
They are display/planning constraints only and do not enable a map layer on
the real vehicle.
"""

from __future__ import annotations

from smartcar_tools.field_reference import Bounds2D, FieldReference
from smartcar_tools.route_planning import (
    RoutePlanningConfig,
    RoutePlanningConfigError,
    load_route_planning_config,
)


def central_c_keepout(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> Bounds2D:
    """Return the C-zone core that must be circumnavigated."""
    settings = config or load_route_planning_config()
    inner = reference.ring_inner
    keepout = Bounds2D(
        inner.x_min + settings.c_zone_keepout.horizontal_inset_m,
        inner.x_max - settings.c_zone_keepout.horizontal_inset_m,
        inner.y_min + settings.c_zone_keepout.vertical_inset_m,
        inner.y_max - settings.c_zone_keepout.vertical_inset_m,
    )
    if keepout.width <= 0.0 or keepout.height <= 0.0:
        raise RoutePlanningConfigError(
            "c_zone_keepout insets leave no central C-zone keepout"
        )
    return keepout


def keepout_bounds(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> tuple[Bounds2D, ...]:
    """Return all rectangle keepouts represented in the simulation mask."""
    settings = config or load_route_planning_config()
    field = reference.field
    outer = reference.ring_outer
    return (
        *reference.b_walls,
        central_c_keepout(reference, settings),
        Bounds2D(field.x_min, field.x_max, outer.y_max, field.y_max),
        Bounds2D(field.x_min, outer.x_min, outer.y_min, outer.y_max),
        Bounds2D(outer.x_max, field.x_max, outer.y_min, outer.y_max),
    )
