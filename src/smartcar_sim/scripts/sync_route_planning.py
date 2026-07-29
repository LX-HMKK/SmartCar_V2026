#!/usr/bin/env python3
"""Project shared route-planning tuning into simulation Nav2 inputs.

``route_planning.yaml`` is deliberately the human-edited source of truth for
the small set of constraints that the editor and simulation can share.  This
script writes only the corresponding simulation values:

* Smac Hybrid and reverse-MPPI minimum turning radii;
* the simulation-only KeepoutFilter inflation envelope.

It never changes the real vehicle's obstacle-layer inflation parameters.
"""

from __future__ import annotations

import argparse
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
DEFAULT_NAV2_PARAMS = SOURCE_ROOT / "smartcar_nav2" / "config" / "nav2_params.yaml"
DEFAULT_KEEPOUT_OVERLAY = SOURCE_ROOT / "smartcar_sim" / "config" / "nav2_keepout_filter.yaml"


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


def _format_scalar(value: float) -> str:
    return format(float(value), ".12g")


def render_updates(source_text: str, updates: Mapping[str, float]) -> tuple[str, bool]:
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
    nav2_params_file: Path,
    keepout_overlay_file: Path,
    *,
    check: bool = False,
) -> bool:
    """Synchronize all simulation consumers; return whether any file changed."""
    config = _load_route_planning(route_planning_file)
    radius = config.minimum_turning_radius_m
    envelope = config.footprint_envelope_radius_m
    targets = (
        (
            nav2_params_file,
            {
                "planner_server.ros__parameters.GridBased.minimum_turning_radius": radius,
                "controller_server.ros__parameters.ReverseHandoff.AckermannConstraints.min_turning_r": radius,
            },
        ),
        (
            keepout_overlay_file,
            {
                "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius": envelope,
                "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius": envelope,
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
    parser.add_argument("--nav2-params", type=Path, default=DEFAULT_NAV2_PARAMS)
    parser.add_argument(
        "--keepout-overlay", type=Path, default=DEFAULT_KEEPOUT_OVERLAY
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
            args.nav2_params,
            args.keepout_overlay,
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
