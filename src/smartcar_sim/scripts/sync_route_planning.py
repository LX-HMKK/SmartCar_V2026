#!/usr/bin/env python3
"""Project shared route-planning tuning into simulation Nav2 inputs.

``route_planning.yaml`` is deliberately the human-edited source of truth for
the small set of constraints that the editor and simulation can share.  This
script writes only the corresponding simulation-overlay values:

* Smac Hybrid and native RPP minimum-turning-radius parameters;
* the padded Nav2 footprint and simulation-only cost inflation radius.

It never changes the real vehicle's Nav2 parameter file or obstacle-layer
inflation parameters.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any, Mapping

import yaml
from yaml.nodes import MappingNode, ScalarNode


SCRIPT = Path(__file__).resolve()
SOURCE_ROOT = Path(os.environ.get("SMARTCAR_SRC", str(SCRIPT.parents[2])))
DEFAULT_ROUTE_PLANNING = (
    SOURCE_ROOT / "smartcar_tools" / "config" / "routes" / "route_planning.yaml"
)
DEFAULT_SIMULATION_OVERLAY = SOURCE_ROOT / "smartcar_sim" / "config" / "nav2_simulation.yaml"


def _load_route_planning(path: Path):
    tools_source = SOURCE_ROOT / "smartcar_tools"
    if str(tools_source) not in sys.path:
        sys.path.insert(0, str(tools_source))
    from smartcar_tools.route_planning import load_route_planning_config

    return load_route_planning_config(path)


def _mapping_value(node: MappingNode, key: str) -> Any:
    for key_node, value_node in node.value:
        if isinstance(key_node, ScalarNode) and key_node.value == key:
            return value_node
    raise KeyError(key)


def _scalar_node(document: Any, path: str) -> ScalarNode:
    node = document
    for key in path.split("."):
        if not isinstance(node, MappingNode):
            raise KeyError(path)
        node = _mapping_value(node, key)
    if not isinstance(node, ScalarNode):
        raise TypeError(f"route-planning sync target is not a scalar: {path}")
    return node


def _nested_value(document: Mapping[str, Any], path: str) -> Any:
    value: Any = document
    for key in path.split("."):
        if not isinstance(value, Mapping) or key not in value:
            raise KeyError(path)
        value = value[key]
    return value


def _format_scalar(value: float | str) -> str:
    if isinstance(value, str):
        return json.dumps(value)
    return format(float(value), ".12g")


def render_updates(
    source_text: str,
    updates: Mapping[str, float | str],
) -> tuple[str, bool]:
    """Replace scalar values without discarding comments or unrelated layout."""
    parsed = yaml.safe_load(source_text)
    if not isinstance(parsed, Mapping):
        raise ValueError("sync target must be a YAML mapping")
    document = yaml.compose(source_text)
    if not isinstance(document, MappingNode):
        raise ValueError("sync target must be a YAML mapping")

    replacements: list[tuple[int, int, str]] = []
    for path, desired in updates.items():
        current = _nested_value(parsed, path)
        if isinstance(desired, str):
            if not isinstance(current, str):
                raise TypeError(f"route-planning sync target is not a string: {path}")
            if current == desired:
                continue
        else:
            if isinstance(current, bool) or not isinstance(current, (int, float)):
                raise TypeError(f"route-planning sync target is not numeric: {path}")
            if float(current) == float(desired):
                continue
        node = _scalar_node(document, path)
        replacements.append((node.start_mark.index, node.end_mark.index, _format_scalar(desired)))

    rendered = source_text
    for start, end, replacement in sorted(replacements, reverse=True):
        rendered = rendered[:start] + replacement + rendered[end:]
    yaml.safe_load(rendered)
    return rendered, bool(replacements)


def _write_atomic(path: Path, content: str) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        temporary.write_text(content, encoding="utf-8", newline="\n")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def synchronize(
    route_planning_file: Path,
    simulation_overlay_file: Path,
    *,
    check: bool = False,
) -> bool:
    """Synchronize all simulation consumers; return whether any file changed."""
    config = _load_route_planning(route_planning_file)
    radius = config.simulation_minimum_turning_radius_m
    footprint = config.runtime_footprint
    footprint_text = "[[{0}, {1}], [{0}, -{1}], [-{0}, -{1}], [-{0}, {1}]]".format(
        _format_scalar(footprint.half_length_m),
        _format_scalar(footprint.half_width_m),
    )
    targets = (
        (
            simulation_overlay_file,
            {
                "controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_radius": radius,
                "planner_server.ros__parameters.GridBased.minimum_turning_radius": radius,
                "smoother_server.ros__parameters.constrained_smoother.minimum_turning_radius": radius,
                "local_costmap.local_costmap.ros__parameters.footprint": footprint_text,
                "local_costmap.local_costmap.ros__parameters.footprint_padding": footprint.padding_m,
                "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius": config.simulation_costmap.inflation_radius_m,
                "global_costmap.global_costmap.ros__parameters.footprint": footprint_text,
                "global_costmap.global_costmap.ros__parameters.footprint_padding": footprint.padding_m,
                "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius": config.simulation_costmap.inflation_radius_m,
            },
        ),
    )
    changed = False
    for path, updates in targets:
        rendered, target_changed = render_updates(
            path.read_text(encoding="utf-8"), updates
        )
        changed = changed or target_changed
        if target_changed and not check:
            _write_atomic(path, rendered)
    return changed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--route-planning-config", type=Path, default=DEFAULT_ROUTE_PLANNING
    )
    parser.add_argument(
        "--simulation-overlay", type=Path, default=DEFAULT_SIMULATION_OVERLAY
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="return nonzero when generated simulation inputs are stale",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        changed = synchronize(
            args.route_planning_config,
            args.simulation_overlay,
            check=args.check,
        )
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        print(f"cannot synchronize route planning: {error}", file=sys.stderr)
        return 2
    if args.check:
        if changed:
            print("simulation route-planning inputs are stale", file=sys.stderr)
            return 1
        print("simulation route-planning inputs are current")
        return 0
    print("synchronized route planning into simulation Nav2 inputs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
