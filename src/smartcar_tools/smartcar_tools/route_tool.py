"""Command-line route generator, validator, and atomic point editor."""
from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys
from typing import Sequence

from smartcar_tools.route_model import (
    RouteDocument,
    RoutePoint,
    RouteValidationError,
    load_field_geometry,
    load_route,
    normalize_yaw_deg,
    replace_waypoints,
    validate_route,
    write_route_atomic,
    generate_baseline_route,
)


def _point_index(route: RouteDocument, point_id: str) -> int:
    for index, point in enumerate(route.waypoints):
        if point.id == point_id:
            return index
    raise RouteValidationError(f"unknown waypoint id: {point_id}")


def _write_modified(route: RouteDocument, path: Path) -> None:
    # Structural or coordinate edits invalidate prior field calibration.
    write_route_atomic(replace(route, calibrated=False), path)


def _cmd_generate(args: argparse.Namespace) -> None:
    destination = Path(args.route)
    if destination.exists() and not args.force:
        raise RouteValidationError(
            f"{destination} already exists; pass --force to replace it")
    geometry = load_field_geometry(args.geometry)
    route = generate_baseline_route(geometry)
    write_route_atomic(route, destination)
    print(f"generated {len(route.waypoints)} uncalibrated waypoints: {destination}")


def _cmd_validate(args: argparse.Namespace) -> None:
    route = load_route(args.route)
    state = "calibrated" if route.calibrated else "UNCALIBRATED"
    print(
        f"valid: {route.name}, {len(route.waypoints)} waypoints, "
        f"frame={route.frame_id}, {state}"
    )


def _cmd_list(args: argparse.Namespace) -> None:
    route = load_route(args.route)
    print(f"{'INDEX':>5}  {'ID':<24} {'ZONE':<4} {'X':>9} {'Y':>9} {'YAW_DEG':>9}")
    for index, point in enumerate(route.waypoints):
        print(
            f"{index:5d}  {point.id:<24} {point.zone:<4} "
            f"{point.x:9.3f} {point.y:9.3f} {point.yaw_deg:9.1f}"
        )


def _cmd_set(args: argparse.Namespace) -> None:
    if all(
        value is None
        for value in (args.zone, args.x, args.y, args.yaw_deg)
    ):
        raise RouteValidationError("set requires at least one field option")
    route = load_route(args.route)
    index = _point_index(route, args.id)
    current = route.waypoints[index]
    updated = replace(
        current,
        zone=current.zone if args.zone is None else args.zone.upper(),
        x=current.x if args.x is None else args.x,
        y=current.y if args.y is None else args.y,
        yaw_deg=(
            current.yaw_deg
            if args.yaw_deg is None
            else normalize_yaw_deg(args.yaw_deg)
        ),
    )
    points = list(route.waypoints)
    points[index] = updated
    _write_modified(replace_waypoints(route, points), Path(args.route))
    print(f"updated {updated.id}; route marked uncalibrated")


def _cmd_nudge(args: argparse.Namespace) -> None:
    if args.dx == 0.0 and args.dy == 0.0 and args.dyaw_deg == 0.0:
        raise RouteValidationError("nudge requires at least one nonzero delta")
    route = load_route(args.route)
    index = _point_index(route, args.id)
    current = route.waypoints[index]
    updated = replace(
        current,
        x=current.x + args.dx,
        y=current.y + args.dy,
        yaw_deg=normalize_yaw_deg(current.yaw_deg + args.dyaw_deg),
    )
    points = list(route.waypoints)
    points[index] = updated
    _write_modified(replace_waypoints(route, points), Path(args.route))
    print(f"nudged {updated.id}; route marked uncalibrated")


def _cmd_insert(args: argparse.Namespace) -> None:
    route = load_route(args.route)
    if any(point.id == args.id for point in route.waypoints):
        raise RouteValidationError(f"duplicate waypoint id: {args.id}")
    after_index = _point_index(route, args.after)
    inserted = RoutePoint(
        id=args.id,
        zone=args.zone.upper(),
        x=args.x,
        y=args.y,
        yaw_deg=normalize_yaw_deg(args.yaw_deg),
    )
    points = list(route.waypoints)
    points.insert(after_index + 1, inserted)
    _write_modified(replace_waypoints(route, points), Path(args.route))
    print(f"inserted {inserted.id} after {args.after}; route marked uncalibrated")


def _cmd_delete(args: argparse.Namespace) -> None:
    route = load_route(args.route)
    index = _point_index(route, args.id)
    points = list(route.waypoints)
    del points[index]
    _write_modified(replace_waypoints(route, points), Path(args.route))
    print(f"deleted {args.id}; route marked uncalibrated")


def _cmd_mark_calibrated(args: argparse.Namespace) -> None:
    route = load_route(args.route)
    target = not args.uncalibrated
    updated = replace(route, calibrated=target)
    validate_route(updated)
    write_route_atomic(updated, args.route)
    print("route marked calibrated" if target else "route marked uncalibrated")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate, validate, and micro-adjust SmartCar field routes")
    commands = parser.add_subparsers(dest="command", required=True)

    command = commands.add_parser("generate", help="generate the rule-diagram baseline")
    command.add_argument("route", help="output route YAML")
    command.add_argument("--geometry", required=True, help="field geometry YAML")
    command.add_argument("--force", action="store_true", help="replace an existing file")
    command.set_defaults(handler=_cmd_generate)

    command = commands.add_parser("validate", help="validate one route YAML")
    command.add_argument("route")
    command.set_defaults(handler=_cmd_validate)

    command = commands.add_parser("list", help="print route point IDs and coordinates")
    command.add_argument("route")
    command.set_defaults(handler=_cmd_list)

    command = commands.add_parser("set", help="replace selected fields on one point")
    command.add_argument("route")
    command.add_argument("id")
    command.add_argument("--zone", choices=("P", "A", "B", "C"))
    command.add_argument("--x", type=float)
    command.add_argument("--y", type=float)
    command.add_argument("--yaw-deg", type=float)
    command.set_defaults(handler=_cmd_set)

    command = commands.add_parser("nudge", help="apply coordinate deltas to one point")
    command.add_argument("route")
    command.add_argument("id")
    command.add_argument("--dx", type=float, default=0.0)
    command.add_argument("--dy", type=float, default=0.0)
    command.add_argument("--dyaw-deg", type=float, default=0.0)
    command.set_defaults(handler=_cmd_nudge)

    command = commands.add_parser("insert", help="insert one point after an existing ID")
    command.add_argument("route")
    command.add_argument("--after", required=True)
    command.add_argument("--id", required=True)
    command.add_argument("--zone", required=True, choices=("A", "B", "C"))
    command.add_argument("--x", required=True, type=float)
    command.add_argument("--y", required=True, type=float)
    command.add_argument("--yaw-deg", required=True, type=float)
    command.set_defaults(handler=_cmd_insert)

    command = commands.add_parser("delete", help="delete one point by ID")
    command.add_argument("route")
    command.add_argument("id")
    command.set_defaults(handler=_cmd_delete)

    command = commands.add_parser(
        "mark-calibrated", help="set or clear the explicit field-calibration flag")
    command.add_argument("route")
    command.add_argument(
        "--uncalibrated", action="store_true", help="clear instead of set the flag")
    command.set_defaults(handler=_cmd_mark_calibrated)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        args.handler(args)
    except RouteValidationError as error:
        print(f"route error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
