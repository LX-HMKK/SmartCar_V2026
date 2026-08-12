"""B-zone-only field keepouts used by route preflight and Nav2 masks.

The generated PGM is a rule constraint, not a localization map. It paints
only the B-zone forbidden walls with the configured static clearance; all
other field geometry is left to live depth-camera obstacle observations.
"""

from __future__ import annotations

import math

from smartcar_tools.field_reference import Bounds2D, FieldReference
from smartcar_tools.route_planning import RoutePlanningConfig, load_route_planning_config


def simulation_map_bounds(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> Bounds2D:
    """Return the finite PGM extent around the traversable field."""
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
    """Return the inflated B-zone forbidden areas represented in the mask."""
    settings = config or load_route_planning_config()
    clearance = settings.simulation_keepout.b_zone_inflation_m
    return tuple(
        Bounds2D(
            wall.x_min - clearance,
            wall.x_max + clearance,
            wall.y_min - clearance,
            wall.y_max + clearance,
        )
        for wall in reference.b_walls
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
