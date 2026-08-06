"""Shared field keepout geometry used by route preflight and Nav2 masks.

The values deliberately mirror the simulation mask: the central C-zone core
forces a loop while the outer limits prevent plans from escaping the stadium.
The generated PGM is a rule constraint for both Nav2 deployments; it is never
used to localize the physical vehicle.
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


def simulation_map_bounds(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> Bounds2D:
    """Return the finite PGM extent, including its lethal exterior ring."""
    settings = config or load_route_planning_config()
    field = reference.field
    padding = settings.simulation_keepout.boundary_padding_m
    return Bounds2D(
        field.x_min - padding,
        field.x_max + padding,
        field.y_min - padding,
        field.y_max + padding,
    )


def keepout_bounds(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> tuple[Bounds2D, ...]:
    """Return all rectangle keepouts represented in the simulation mask."""
    settings = config or load_route_planning_config()
    field = reference.field
    outer = reference.ring_outer
    map_bounds = simulation_map_bounds(reference, settings)
    return (
        *reference.b_walls,
        central_c_keepout(reference, settings),
        Bounds2D(field.x_min, field.x_max, outer.y_max, field.y_max),
        Bounds2D(field.x_min, outer.x_min, outer.y_min, outer.y_max),
        Bounds2D(outer.x_max, field.x_max, outer.y_min, outer.y_max),
        # The physical field boundary itself is not a Gazebo collision model.
        # Paint an exterior PGM ring instead so both KeepoutFilter and the
        # free-heading body sweep reject an apparent shortcut outside it.
        Bounds2D(map_bounds.x_min, map_bounds.x_max, map_bounds.y_min, field.y_min),
        Bounds2D(map_bounds.x_min, field.x_min, map_bounds.y_min, map_bounds.y_max),
        Bounds2D(field.x_max, map_bounds.x_max, map_bounds.y_min, map_bounds.y_max),
        Bounds2D(map_bounds.x_min, map_bounds.x_max, field.y_max, map_bounds.y_max),
    )


def _rasterized_bounds(
    bounds: Bounds2D,
    field: Bounds2D,
    resolution: float,
) -> Bounds2D:
    """Return the full PGM cells occupied by ``generate_field_map.fill_rect``."""
    # Decimal field dimensions such as 1.30 / 0.025 can land microscopically
    # below an integer in binary floating point. Preserve an already aligned
    # edge instead of expanding its lethal rectangle by one unintended cell.
    alignment_epsilon = 1.0e-9
    x_min = max(
        field.x_min,
        field.x_min
        + math.floor(
            (bounds.x_min - field.x_min) / resolution + alignment_epsilon
        ) * resolution,
    )
    x_max = min(
        field.x_max,
        field.x_min
        + math.ceil(
            (bounds.x_max - field.x_min) / resolution - alignment_epsilon
        ) * resolution,
    )
    y_min = max(
        field.y_min,
        field.y_min
        + math.floor(
            (bounds.y_min - field.y_min) / resolution + alignment_epsilon
        ) * resolution,
    )
    y_max = min(
        field.y_max,
        field.y_min
        + math.ceil(
            (bounds.y_max - field.y_min) / resolution - alignment_epsilon
        ) * resolution,
    )
    return Bounds2D(x_min, x_max, y_min, y_max)


def keepout_mask_bounds(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> tuple[Bounds2D, ...]:
    """Return occupied PGM-cell bounds used by simulation KeepoutFilter."""
    settings = config or load_route_planning_config()
    resolution = settings.simulation_keepout.map_resolution_m
    map_bounds = simulation_map_bounds(reference, settings)
    return tuple(
        _rasterized_bounds(bounds, map_bounds, resolution)
        for bounds in keepout_bounds(reference, settings)
    )
