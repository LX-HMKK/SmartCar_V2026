#!/usr/bin/env python3
"""Generate the shared Nav2 field keepout mask from field_geometry.yaml.

The generated PGM is consumed by nav2_map_server and then by Nav2's
KeepoutFilter. It is not a localization map. Its black cells are prohibited
areas and its white cells are traversable. The real Nav2 package installs the
same artifact as a rule constraint, never as a localization map. Keeping this
generator tied to the same FieldReference used by Gazebo and RViz prevents
coordinate drift between the representations.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path


SCRIPT = Path(__file__).resolve()
SIM_ROOT = SCRIPT.parents[1]
SOURCE_ROOT = SCRIPT.parents[2]
SOURCE_TOOLS_ROOT = SOURCE_ROOT / "smartcar_tools"
GEOMETRY_RELATIVE_PATH = Path("config") / "routes" / "field_geometry.yaml"
ROUTE_PLANNING_RELATIVE_PATH = Path("config") / "routes" / "route_planning.yaml"
# PGM values used with ``negate: 0`` in the generated YAML.
OCCUPIED = 0
FREE = 254


try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:
    get_package_share_directory = None


def resolve_tools_share(script_path: Path, package_share_lookup=None) -> Path:
    """Locate smartcar_tools in a source tree or isolated install layout."""
    source_candidate = script_path.resolve().parents[2] / "smartcar_tools"
    if (source_candidate / GEOMETRY_RELATIVE_PATH).is_file():
        return source_candidate
    if package_share_lookup is not None:
        try:
            installed_candidate = Path(package_share_lookup("smartcar_tools"))
        except (LookupError, OSError):
            installed_candidate = None
        if (
            installed_candidate is not None
            and (installed_candidate / GEOMETRY_RELATIVE_PATH).is_file()
        ):
            return installed_candidate
    raise RuntimeError(
        "cannot locate smartcar_tools/config/routes/field_geometry.yaml"
    )


TOOLS_SHARE = resolve_tools_share(SCRIPT, get_package_share_directory)
DEFAULT_GEOMETRY = TOOLS_SHARE / GEOMETRY_RELATIVE_PATH
DEFAULT_ROUTE_PLANNING_CONFIG = TOOLS_SHARE / ROUTE_PLANNING_RELATIVE_PATH
DEFAULT_MAPS_DIR = SIM_ROOT / "maps"


try:
    from smartcar_tools.field_reference import (  # type: ignore[import-not-found]
        Bounds2D,
        FieldReference,
        load_field_reference,
    )
    from smartcar_tools.field_keepouts import (  # type: ignore[import-not-found]
        keepout_bounds as _keepout_bounds,
        simulation_map_bounds as _simulation_map_bounds,
    )
    from smartcar_tools.route_planning import (  # type: ignore[import-not-found]
        RoutePlanningConfig,
        load_route_planning_config,
    )
except ModuleNotFoundError:
    sys.path.insert(0, str(SOURCE_TOOLS_ROOT))
    from smartcar_tools.field_reference import (  # type: ignore[no-redef]
        Bounds2D,
        FieldReference,
        load_field_reference,
    )
    from smartcar_tools.field_keepouts import (  # type: ignore[no-redef]
        keepout_bounds as _keepout_bounds,
        simulation_map_bounds as _simulation_map_bounds,
    )
    from smartcar_tools.route_planning import (  # type: ignore[no-redef]
        RoutePlanningConfig,
        load_route_planning_config,
    )


@dataclass(frozen=True)
class MapSpec:
    origin_x: float
    origin_y: float
    width_m: float
    height_m: float
    resolution: float

    @property
    def width_px(self) -> int:
        return int(round(self.width_m / self.resolution))

    @property
    def height_px(self) -> int:
        return int(round(self.height_m / self.resolution))


def map_spec(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> MapSpec:
    """Build the finite PGM extent around the B-zone-only keepout mask."""
    settings = config or load_route_planning_config()
    bounds = _simulation_map_bounds(reference, settings)
    return MapSpec(
        origin_x=bounds.x_min,
        origin_y=bounds.y_min,
        width_m=bounds.width,
        height_m=bounds.height,
        resolution=settings.simulation_keepout.map_resolution_m,
    )


def keepout_bounds(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> tuple[Bounds2D, ...]:
    """Return the B-zone-only static keepouts painted into the PGM."""
    return _keepout_bounds(reference, config)


def fill_rect(grid: bytearray, spec: MapSpec, bounds: Bounds2D, value: int) -> None:
    """Fill cells whose centres lie within an odom-frame rectangle."""
    x0 = max(spec.origin_x, bounds.x_min)
    x1 = min(spec.origin_x + spec.width_m, bounds.x_max)
    y0 = max(spec.origin_y, bounds.y_min)
    y1 = min(spec.origin_y + spec.height_m, bounds.y_max)
    if x0 >= x1 or y0 >= y1:
        return

    col0 = max(0, int(math.floor((x0 - spec.origin_x) / spec.resolution)))
    col1 = min(
        spec.width_px,
        int(math.ceil((x1 - spec.origin_x) / spec.resolution)),
    )
    row_top = max(
        0,
        int(math.floor((spec.origin_y + spec.height_m - y1) / spec.resolution)),
    )
    row_bottom = min(
        spec.height_px - 1,
        int(math.ceil((spec.origin_y + spec.height_m - y0) / spec.resolution)) - 1,
    )
    if col0 >= col1 or row_top > row_bottom:
        return

    for row in range(row_top, row_bottom + 1):
        offset = row * spec.width_px
        for col in range(col0, col1):
            grid[offset + col] = value


def make_grid(
    reference: FieldReference,
    config: RoutePlanningConfig | None = None,
) -> tuple[MapSpec, bytearray]:
    """Build the black=keepout, white=free occupancy mask."""
    settings = config or load_route_planning_config()
    spec = map_spec(reference, settings)
    grid = bytearray([FREE]) * (spec.width_px * spec.height_px)
    for bounds in keepout_bounds(reference, settings):
        fill_rect(grid, spec, bounds, OCCUPIED)
    return spec, grid


def pgm_bytes(spec: MapSpec, grid: bytearray) -> bytes:
    header = f"P5\n{spec.width_px} {spec.height_px}\n255\n".encode("ascii")
    return header + bytes(grid)


def yaml_text(spec: MapSpec, image_name: str) -> str:
    return (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {spec.resolution}\n"
        f"origin: [{spec.origin_x}, {spec.origin_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )


def render(
    geometry_file: Path,
    route_planning_file: Path = DEFAULT_ROUTE_PLANNING_CONFIG,
) -> tuple[bytes, str]:
    reference = load_field_reference(geometry_file)
    config = load_route_planning_config(route_planning_file)
    spec, grid = make_grid(reference, config)
    return pgm_bytes(spec, grid), yaml_text(spec, "field_map.pgm")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, default=DEFAULT_GEOMETRY)
    parser.add_argument(
        "--route-planning-config",
        type=Path,
        default=DEFAULT_ROUTE_PLANNING_CONFIG,
        help="shared editor/simulation planning constraints YAML",
    )
    parser.add_argument("--maps-dir", type=Path, default=DEFAULT_MAPS_DIR)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail when committed field_map artifacts differ from the generator",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        expected_pgm, expected_yaml = render(
            args.geometry,
            args.route_planning_config,
        )
    except (OSError, ValueError) as error:
        print(f"cannot generate field map: {error}", file=sys.stderr)
        return 2

    pgm_path = args.maps_dir / "field_map.pgm"
    yaml_path = args.maps_dir / "field_map.yaml"
    if args.check:
        try:
            actual_pgm = pgm_path.read_bytes()
            actual_yaml = yaml_path.read_text(encoding="utf-8")
        except OSError as error:
            print(f"cannot read generated field map: {error}", file=sys.stderr)
            return 1
        if actual_pgm == expected_pgm and actual_yaml == expected_yaml:
            print(f"field keepout mask is current: {pgm_path}")
            return 0
        print("field keepout mask is stale; run generate_field_map.py", file=sys.stderr)
        return 1

    args.maps_dir.mkdir(parents=True, exist_ok=True)
    pgm_path.write_bytes(expected_pgm)
    yaml_path.write_text(expected_yaml, encoding="utf-8", newline="\n")
    print(f"generated field keepout mask: {pgm_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
