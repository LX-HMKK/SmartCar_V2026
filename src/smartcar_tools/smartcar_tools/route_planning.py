"""Versioned tuning shared by route preflight and simulation keepouts.

The waypoint editor deliberately does not parse Nav2's large parameter file.
Instead this small document owns the geometry and motion constraints which both
the editor's local preflight and the simulation keepout mask can represent.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


SCHEMA_VERSION = 2
CONFIG_RELATIVE_PATH = Path("config") / "routes" / "route_planning.yaml"


try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:  # pragma: no cover - source-only unit tests.
    get_package_share_directory = None


class RoutePlanningConfigError(ValueError):
    """Raised when the shared route-planning YAML is invalid."""


@dataclass(frozen=True)
class CZoneKeepoutConfig:
    horizontal_inset_m: float
    vertical_inset_m: float


@dataclass(frozen=True)
class RuntimeFootprintConfig:
    """Nav2 footprint vertices and padding expressed as half extents."""

    half_length_m: float
    half_width_m: float
    padding_m: float

    @property
    def padded_half_length_m(self) -> float:
        return self.half_length_m + self.padding_m

    @property
    def padded_half_width_m(self) -> float:
        return self.half_width_m + self.padding_m


@dataclass(frozen=True)
class PreflightConfig:
    grid_resolution_m: float
    heading_bins: int
    step_length_m: float
    primitive_samples: int
    steering_fractions: tuple[float, ...]
    turning_penalty_m: float
    goal_position_tolerance_m: float
    goal_heading_tolerance_deg: float
    terminal_connector_max_distance_m: float
    terminal_connector_max_length_m: float
    terminal_connector_sample_step_m: float
    max_expansions_per_leg: int


@dataclass(frozen=True)
class SimulationKeepoutConfig:
    map_resolution_m: float
    costmap_inflation_radius_m: float


@dataclass(frozen=True)
class RoutePlanningConfig:
    minimum_turning_radius_m: float
    runtime_footprint: RuntimeFootprintConfig
    c_zone_keepout: CZoneKeepoutConfig
    preflight: PreflightConfig
    simulation_keepout: SimulationKeepoutConfig


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RoutePlanningConfigError(f"{label} must be a mapping")
    return value


def _exact_fields(raw: Mapping[str, Any], fields: set[str], label: str) -> None:
    unknown = sorted(set(raw) - fields)
    missing = sorted(fields - set(raw))
    if unknown:
        raise RoutePlanningConfigError(
            f"{label} has unknown fields: {', '.join(unknown)}"
        )
    if missing:
        raise RoutePlanningConfigError(
            f"{label} is missing fields: {', '.join(missing)}"
        )


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise RoutePlanningConfigError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise RoutePlanningConfigError(f"{label} must be a finite number") from error
    if not math.isfinite(result):
        raise RoutePlanningConfigError(f"{label} must be a finite number")
    return result


def _positive_number(value: Any, label: str) -> float:
    result = _finite_number(value, label)
    if result <= 0.0:
        raise RoutePlanningConfigError(f"{label} must be positive")
    return result


def _nonnegative_number(value: Any, label: str) -> float:
    result = _finite_number(value, label)
    if result < 0.0:
        raise RoutePlanningConfigError(f"{label} must be nonnegative")
    return result


def _positive_integer(value: Any, label: str, minimum: int = 1) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise RoutePlanningConfigError(f"{label} must be an integer")
    if value < minimum:
        raise RoutePlanningConfigError(f"{label} must be at least {minimum}")
    return value


def _steering_fractions(value: Any) -> tuple[float, ...]:
    if not isinstance(value, list) or not value:
        raise RoutePlanningConfigError(
            "preflight.steering_fractions must be a non-empty list"
        )
    result = tuple(
        _finite_number(item, f"preflight.steering_fractions[{index}]")
        for index, item in enumerate(value)
    )
    if any(abs(item) > 1.0 for item in result):
        raise RoutePlanningConfigError(
            "preflight.steering_fractions values must lie in [-1.0, 1.0]"
        )
    if len(set(result)) != len(result):
        raise RoutePlanningConfigError(
            "preflight.steering_fractions must not repeat values"
        )
    return result


def default_route_planning_file() -> Path:
    """Find the installed or source-tree shared planning configuration."""
    override = os.environ.get("SMARTCAR_ROUTE_PLANNING_CONFIG")
    if override:
        configured = Path(override).expanduser()
        if configured.is_file():
            return configured
        raise RoutePlanningConfigError(
            "SMARTCAR_ROUTE_PLANNING_CONFIG does not name a readable file: "
            f"{configured}"
        )
    source_candidate = Path(__file__).resolve().parents[1] / CONFIG_RELATIVE_PATH
    if source_candidate.is_file():
        return source_candidate
    if get_package_share_directory is not None:
        try:
            installed_candidate = (
                Path(get_package_share_directory("smartcar_tools"))
                / CONFIG_RELATIVE_PATH
            )
        except (LookupError, OSError):
            installed_candidate = None
        if installed_candidate is not None and installed_candidate.is_file():
            return installed_candidate
    raise RoutePlanningConfigError(
        "cannot locate smartcar_tools/config/routes/route_planning.yaml"
    )


def load_route_planning_config(path: str | Path | None = None) -> RoutePlanningConfig:
    """Load the strict, versioned shared route-planning configuration."""
    source = Path(path) if path is not None else default_route_planning_file()
    try:
        root = _mapping(
            yaml.safe_load(source.read_text(encoding="utf-8")),
            "route planning document",
        )
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RoutePlanningConfigError(
            f"cannot load route planning config: {error}"
        ) from error

    _exact_fields(
        root,
        {
            "schema_version",
            "name",
            "minimum_turning_radius_m",
            "runtime_footprint",
            "c_zone_keepout",
            "preflight",
            "simulation_keepout",
        },
        "route planning document",
    )
    if root["schema_version"] != SCHEMA_VERSION:
        raise RoutePlanningConfigError(
            f"route planning schema_version must be {SCHEMA_VERSION}"
        )
    if not isinstance(root["name"], str) or not root["name"].strip():
        raise RoutePlanningConfigError("route planning name must be a non-empty string")

    c_zone = _mapping(root["c_zone_keepout"], "c_zone_keepout")
    _exact_fields(
        c_zone,
        {"horizontal_inset_m", "vertical_inset_m"},
        "c_zone_keepout",
    )
    footprint = _mapping(root["runtime_footprint"], "runtime_footprint")
    _exact_fields(
        footprint,
        {"half_length_m", "half_width_m", "padding_m"},
        "runtime_footprint",
    )
    preflight = _mapping(root["preflight"], "preflight")
    _exact_fields(
        preflight,
        {
            "grid_resolution_m",
            "heading_bins",
            "step_length_m",
            "primitive_samples",
            "steering_fractions",
            "turning_penalty_m",
            "goal_position_tolerance_m",
            "goal_heading_tolerance_deg",
            "terminal_connector_max_distance_m",
            "terminal_connector_max_length_m",
            "terminal_connector_sample_step_m",
            "max_expansions_per_leg",
        },
        "preflight",
    )
    simulation = _mapping(root["simulation_keepout"], "simulation_keepout")
    _exact_fields(
        simulation,
        {"map_resolution_m", "costmap_inflation_radius_m"},
        "simulation_keepout",
    )

    result = RoutePlanningConfig(
        minimum_turning_radius_m=_positive_number(
            root["minimum_turning_radius_m"], "minimum_turning_radius_m"
        ),
        runtime_footprint=RuntimeFootprintConfig(
            half_length_m=_positive_number(
                footprint["half_length_m"], "runtime_footprint.half_length_m"
            ),
            half_width_m=_positive_number(
                footprint["half_width_m"], "runtime_footprint.half_width_m"
            ),
            padding_m=_nonnegative_number(
                footprint["padding_m"], "runtime_footprint.padding_m"
            ),
        ),
        c_zone_keepout=CZoneKeepoutConfig(
            horizontal_inset_m=_nonnegative_number(
                c_zone["horizontal_inset_m"],
                "c_zone_keepout.horizontal_inset_m",
            ),
            vertical_inset_m=_nonnegative_number(
                c_zone["vertical_inset_m"],
                "c_zone_keepout.vertical_inset_m",
            ),
        ),
        preflight=PreflightConfig(
            grid_resolution_m=_positive_number(
                preflight["grid_resolution_m"], "preflight.grid_resolution_m"
            ),
            heading_bins=_positive_integer(
                preflight["heading_bins"], "preflight.heading_bins", minimum=4
            ),
            step_length_m=_positive_number(
                preflight["step_length_m"], "preflight.step_length_m"
            ),
            primitive_samples=_positive_integer(
                preflight["primitive_samples"],
                "preflight.primitive_samples",
            ),
            steering_fractions=_steering_fractions(preflight["steering_fractions"]),
            turning_penalty_m=_nonnegative_number(
                preflight["turning_penalty_m"], "preflight.turning_penalty_m"
            ),
            goal_position_tolerance_m=_positive_number(
                preflight["goal_position_tolerance_m"],
                "preflight.goal_position_tolerance_m",
            ),
            goal_heading_tolerance_deg=_positive_number(
                preflight["goal_heading_tolerance_deg"],
                "preflight.goal_heading_tolerance_deg",
            ),
            terminal_connector_max_distance_m=_positive_number(
                preflight["terminal_connector_max_distance_m"],
                "preflight.terminal_connector_max_distance_m",
            ),
            terminal_connector_max_length_m=_positive_number(
                preflight["terminal_connector_max_length_m"],
                "preflight.terminal_connector_max_length_m",
            ),
            terminal_connector_sample_step_m=_positive_number(
                preflight["terminal_connector_sample_step_m"],
                "preflight.terminal_connector_sample_step_m",
            ),
            max_expansions_per_leg=_positive_integer(
                preflight["max_expansions_per_leg"],
                "preflight.max_expansions_per_leg",
            ),
        ),
        simulation_keepout=SimulationKeepoutConfig(
            map_resolution_m=_positive_number(
                simulation["map_resolution_m"],
                "simulation_keepout.map_resolution_m",
            ),
            costmap_inflation_radius_m=_positive_number(
                simulation["costmap_inflation_radius_m"],
                "simulation_keepout.costmap_inflation_radius_m",
            ),
        ),
    )
    if result.preflight.terminal_connector_sample_step_m > result.preflight.step_length_m:
        raise RoutePlanningConfigError(
            "preflight.terminal_connector_sample_step_m must not exceed "
            "preflight.step_length_m"
        )
    if result.preflight.goal_heading_tolerance_deg > 180.0:
        raise RoutePlanningConfigError(
            "preflight.goal_heading_tolerance_deg must not exceed 180"
        )
    return result
