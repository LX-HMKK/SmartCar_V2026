#!/usr/bin/env python3
"""Generate the Gazebo competition field from the authoritative rule geometry."""
from __future__ import annotations

import argparse
import difflib
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


SCRIPT = Path(__file__).resolve()
SIM_ROOT = SCRIPT.parents[1]
SOURCE_ROOT = SCRIPT.parents[2]
SOURCE_TOOLS_ROOT = SOURCE_ROOT / "smartcar_tools"
GEOMETRY_RELATIVE_PATH = Path("config") / "routes" / "field_geometry.yaml"
DEFAULT_CONFIG = SIM_ROOT / "config" / "competition_field_model.yaml"
DEFAULT_OUTPUT = SIM_ROOT / "models" / "competition_field" / "model.sdf"
CONFIG_SCHEMA_VERSION = 1

try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:
    get_package_share_directory = None


def resolve_tools_share(
    script_path: Path,
    package_share_lookup=None,
) -> Path:
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

try:
    from smartcar_tools.field_reference import (  # type: ignore[import-not-found]
        Bounds2D,
        FieldReference,
        load_field_reference,
    )
except ModuleNotFoundError:
    sys.path.insert(0, str(SOURCE_TOOLS_ROOT))
    from smartcar_tools.field_reference import (  # type: ignore[no-redef]
        Bounds2D,
        FieldReference,
        load_field_reference,
    )


class ModelConfigError(ValueError):
    """Raised when the simulation-only rendering policy is invalid."""


def _finite_positive(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ModelConfigError(f"{label} must be a positive finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ModelConfigError(
            f"{label} must be a positive finite number"
        ) from error
    if not math.isfinite(result) or result <= 0.0:
        raise ModelConfigError(f"{label} must be a positive finite number")
    return result


def _required_bool(mapping: dict, name: str) -> bool:
    value = mapping.get(name)
    if not isinstance(value, bool):
        raise ModelConfigError(f"collision.{name} must be a boolean")
    return value


def load_model_config(path: Path) -> dict:
    """Load the simulation-only appearance and collision policy."""
    try:
        root = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ModelConfigError(f"cannot load model config: {error}") from error
    if not isinstance(root, dict):
        raise ModelConfigError("model config must be a mapping")
    root_fields = {"schema_version", "model_name", "collision", "dimensions"}
    unknown_root = sorted(set(root) - root_fields)
    if unknown_root:
        raise ModelConfigError(
            "model config contains unknown fields: " + ", ".join(unknown_root)
        )
    if root.get("schema_version") != CONFIG_SCHEMA_VERSION:
        raise ModelConfigError(
            f"model config schema_version must be {CONFIG_SCHEMA_VERSION}"
        )
    model_name = root.get("model_name")
    if not isinstance(model_name, str) or not model_name.strip():
        raise ModelConfigError("model_name must be a non-empty string")
    collision = root.get("collision")
    dimensions = root.get("dimensions")
    if not isinstance(collision, dict) or not isinstance(dimensions, dict):
        raise ModelConfigError("collision and dimensions must be mappings")

    collision_fields = (
        "b_zone_walls",
        "diagnosis_room_inner_ring",
        "c_ring_outer_boundary",
        "field_outer_boundary",
    )
    unknown_collision = sorted(set(collision) - set(collision_fields))
    if unknown_collision:
        raise ModelConfigError(
            "collision contains unknown fields: " + ", ".join(unknown_collision)
        )
    dimension_fields = {"wall_height_m", "surface_thickness_m"}
    unknown_dimensions = sorted(set(dimensions) - dimension_fields)
    if unknown_dimensions:
        raise ModelConfigError(
            "dimensions contains unknown fields: " + ", ".join(unknown_dimensions)
        )

    policy = {
        name: _required_bool(collision, name)
        for name in collision_fields
    }
    if not policy["b_zone_walls"]:
        raise ModelConfigError("the official B-zone walls must remain physical")
    for forbidden in ("c_ring_outer_boundary", "field_outer_boundary"):
        if policy[forbidden]:
            raise ModelConfigError(
                f"collision.{forbidden} would block the validated route"
            )

    return {
        "model_name": model_name.strip(),
        "collision": policy,
        "wall_height_m": _finite_positive(
            dimensions.get("wall_height_m"), "dimensions.wall_height_m"
        ),
        "surface_thickness_m": _finite_positive(
            dimensions.get("surface_thickness_m"),
            "dimensions.surface_thickness_m",
        ),
    }


def _number(value: float) -> str:
    if abs(value) < 5e-10:
        return "0"
    return f"{value:.9f}".rstrip("0").rstrip(".")


def _pose(x: float, y: float, z: float) -> str:
    return f"{_number(x)} {_number(y)} {_number(z)} 0 0 0"


def _material(parent: ET.Element, rgba: tuple[float, float, float, float]) -> None:
    material = ET.SubElement(parent, "material")
    color = " ".join(_number(component) for component in rgba)
    ET.SubElement(material, "ambient").text = color
    ET.SubElement(material, "diffuse").text = color


def _box_geometry(parent: ET.Element, x: float, y: float, z: float) -> None:
    geometry = ET.SubElement(parent, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = (
        f"{_number(x)} {_number(y)} {_number(z)}"
    )


def _cylinder_geometry(parent: ET.Element, radius: float, length: float) -> None:
    geometry = ET.SubElement(parent, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = _number(radius)
    ET.SubElement(cylinder, "length").text = _number(length)


def _visual_box(
    link: ET.Element,
    name: str,
    bounds: Bounds2D,
    z: float,
    thickness: float,
    color: tuple[float, float, float, float],
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(visual, "pose").text = _pose(
        bounds.center.x, bounds.center.y, z
    )
    _box_geometry(visual, bounds.width, bounds.height, thickness)
    _material(visual, color)


def _collision_box(
    link: ET.Element,
    name: str,
    bounds: Bounds2D,
    height: float,
) -> None:
    collision = ET.SubElement(
        link, "collision", {"name": f"{name}_collision"}
    )
    ET.SubElement(collision, "pose").text = _pose(
        bounds.center.x, bounds.center.y, height / 2.0
    )
    _box_geometry(collision, bounds.width, bounds.height, height)


def _stadium_parts(bounds: Bounds2D) -> tuple[Bounds2D, float, tuple[float, float]]:
    radius = bounds.height / 2.0
    straight_width = bounds.width - bounds.height
    center = bounds.center
    center_box = Bounds2D(
        center.x - straight_width / 2.0,
        center.x + straight_width / 2.0,
        bounds.y_min,
        bounds.y_max,
    )
    cap_offset = straight_width / 2.0
    return center_box, radius, (center.x - cap_offset, center.x + cap_offset)


def _visual_stadium(
    link: ET.Element,
    name: str,
    bounds: Bounds2D,
    z: float,
    thickness: float,
    color: tuple[float, float, float, float],
) -> None:
    center_box, radius, cap_centers = _stadium_parts(bounds)
    _visual_box(link, f"{name}_center", center_box, z, thickness, color)
    for side, x in zip(("west", "east"), cap_centers):
        visual = ET.SubElement(link, "visual", {"name": f"{name}_{side}_cap"})
        ET.SubElement(visual, "pose").text = _pose(x, bounds.center.y, z)
        _cylinder_geometry(visual, radius, thickness)
        _material(visual, color)


def _collision_stadium(
    link: ET.Element,
    name: str,
    bounds: Bounds2D,
    height: float,
) -> None:
    center_box, radius, cap_centers = _stadium_parts(bounds)
    _collision_box(link, f"{name}_center", center_box, height)
    for side, x in zip(("west", "east"), cap_centers):
        collision = ET.SubElement(
            link,
            "collision",
            {"name": f"{name}_{side}_cap_collision"},
        )
        ET.SubElement(collision, "pose").text = _pose(
            x, bounds.center.y, height / 2.0
        )
        _cylinder_geometry(collision, radius, height)


def build_model(reference: FieldReference, config: dict) -> ET.Element:
    """Build a visual field with collisions only where explicitly authorized."""
    sdf = ET.Element("sdf", {"version": "1.8"})
    sdf.append(ET.Comment(
        " Generated by scripts/generate_competition_field.py; do not edit. "
    ))
    model = ET.SubElement(sdf, "model", {"name": config["model_name"]})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "field"})

    thickness = config["surface_thickness_m"]
    zone_z = thickness / 2.0
    overlay_z = zone_z + thickness
    inner_z = overlay_z + thickness
    zone_colors = {
        "A": (0.20, 0.38, 0.66, 1.0),
        "B": (0.37, 0.39, 0.43, 1.0),
        "C": (0.22, 0.45, 0.29, 1.0),
    }
    for zone_name in ("A", "B", "C"):
        _visual_box(
            link,
            f"zone_{zone_name.lower()}_surface",
            reference.zones[zone_name],
            zone_z,
            thickness,
            zone_colors[zone_name],
        )

    route_color = (0.95, 0.68, 0.08, 1.0)
    _visual_box(
        link,
        "b_corridor_surface",
        reference.corridor,
        overlay_z,
        thickness,
        route_color,
    )
    _visual_stadium(
        link,
        "c_ring_outer",
        reference.ring_outer,
        overlay_z,
        thickness,
        route_color,
    )
    _visual_stadium(
        link,
        "diagnosis_room_inner",
        reference.ring_inner,
        inner_z,
        thickness,
        (0.48, 0.50, 0.54, 1.0),
    )

    wall_height = config["wall_height_m"]
    for side, bounds in zip(("west", "east"), reference.b_walls):
        _visual_box(
            link,
            f"b_wall_{side}",
            bounds,
            wall_height / 2.0,
            wall_height,
            (0.56, 0.58, 0.62, 1.0),
        )
        if config["collision"]["b_zone_walls"]:
            _collision_box(link, f"b_wall_{side}", bounds, wall_height)

    if config["collision"]["diagnosis_room_inner_ring"]:
        _collision_stadium(
            link,
            "diagnosis_room_inner",
            reference.ring_inner,
            wall_height,
        )
    return sdf


def render_model(geometry_file: Path, config_file: Path) -> str:
    reference = load_field_reference(geometry_file)
    config = load_model_config(config_file)
    root = build_model(reference, config)
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(
        root, encoding="unicode", short_empty_elements=False
    ) + "\n"


def check_output(output: Path, expected: str) -> bool:
    try:
        current = output.read_text(encoding="utf-8")
    except OSError as error:
        print(f"cannot read generated model {output}: {error}", file=sys.stderr)
        return False
    if current == expected:
        print(f"competition field model is current: {output}")
        return True
    diff = difflib.unified_diff(
        current.splitlines(),
        expected.splitlines(),
        fromfile=str(output),
        tofile="regenerated model.sdf",
        lineterm="",
    )
    print("\n".join(diff), file=sys.stderr)
    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, default=DEFAULT_GEOMETRY)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail when the committed model differs from deterministic output",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        rendered = render_model(args.geometry, args.config)
    except (ModelConfigError, ValueError) as error:
        print(f"cannot generate competition field: {error}", file=sys.stderr)
        return 2
    if args.check:
        return 0 if check_output(args.output, rendered) else 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered, encoding="utf-8", newline="\n")
    print(f"generated competition field model: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
