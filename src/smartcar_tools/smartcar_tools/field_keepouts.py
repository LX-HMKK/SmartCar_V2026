"""Shared simulation keepout geometry used by route editing preflight.

The values deliberately mirror the simulation mask: the central C-zone core
forces a loop while the outer limits prevent plans from escaping the stadium.
They are display/planning constraints only and do not enable a map layer on
the real vehicle.
"""

from __future__ import annotations

import math

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


def _rasterized_bounds(
    bounds: Bounds2D,
    field: Bounds2D,
    resolution: float,
) -> Bounds2D:
    """Return the full PGM cells occupied by ``generate_field_map.fill_rect``."""
    x_min = max(
        field.x_min,
        field.x_min
        + math.floor((bounds.x_min - field.x_min) / resolution) * resolution,
    )
    x_max = min(
        field.x_max,
        field.x_min
        + math.ceil((bounds.x_max - field.x_min) / resolution) * resolution,
    )
    y_min = max(
        field.y_min,
        field.y_min
        + math.floor((bounds.y_min - field.y_min) / resolution) * resolution,
    )
    y_max = min(
        field.y_max,
        field.y_min
        + math.ceil((bounds.y_max - field.y_min) / resolution) * resolution,
    )
    return Bounds2D(x_min, x_max, y_min, y_max)


def keepout_mask_bounds(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> tuple[Bounds2D, ...]:
    """Return occupied PGM-cell bounds used by simulation KeepoutFilter."""
    settings = config or load_route_planning_config()
    resolution = settings.simulation_keepout.map_resolution_m
    return tuple(
        _rasterized_bounds(bounds, reference.field, resolution)
        for bounds in keepout_bounds(reference, settings)
    )
