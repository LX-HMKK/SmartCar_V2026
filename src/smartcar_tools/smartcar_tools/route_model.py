"""ROS-independent field geometry, route generation, and YAML validation."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Iterable, Mapping, Sequence

import yaml


SCHEMA_VERSION = 1
ROUTE_FRAME = "odom_combined"
POINT_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
ZONE_NAMES = frozenset({"P", "A", "B", "C"})
ORIGIN_TOLERANCE_M = 1e-6
GEOMETRY_TOLERANCE_M = 1e-6
LOOP_EXTENT_TOLERANCE_M = 0.03
LOOP_CLOSURE_TOLERANCE_M = 0.03
MAX_LOOP_POINT_SPACING_M = 0.25
MAX_LOOP_YAW_STEP_DEG = 45.0


class RouteValidationError(ValueError):
    """Raised when field or route data violates the public route contract."""


@dataclass(frozen=True)
class FieldGeometry:
    """Measured rule-diagram geometry plus estimated landmark locations."""

    field_width_m: float
    field_height_m: float
    zone_a_height_m: float
    zone_b_height_m: float
    zone_c_height_m: float
    corridor_width_m: float
    corridor_center_x_from_west_m: float
    ring_outer_width_m: float
    ring_outer_height_m: float
    ring_inner_width_m: float
    ring_inner_height_m: float
    ring_center_x_from_west_m: float
    ring_center_y_from_south_m: float
    p_origin_x_from_west_m: float
    p_origin_y_from_south_m: float
    task_point_x_from_west_m: float
    task_point_y_from_south_m: float

    @property
    def x_min_m(self) -> float:
        return -self.p_origin_x_from_west_m

    @property
    def x_max_m(self) -> float:
        return self.field_width_m - self.p_origin_x_from_west_m

    @property
    def y_min_m(self) -> float:
        return -self.p_origin_y_from_south_m

    @property
    def y_max_m(self) -> float:
        return self.field_height_m - self.p_origin_y_from_south_m

    @property
    def zone_a_y_max_m(self) -> float:
        return self.zone_a_height_m - self.p_origin_y_from_south_m

    @property
    def zone_b_y_max_m(self) -> float:
        return (
            self.zone_a_height_m
            + self.zone_b_height_m
            - self.p_origin_y_from_south_m
        )

    @property
    def corridor_center_x_m(self) -> float:
        return (
            self.corridor_center_x_from_west_m
            - self.p_origin_x_from_west_m
        )

    @property
    def ring_center_x_m(self) -> float:
        return self.ring_center_x_from_west_m - self.p_origin_x_from_west_m

    @property
    def ring_center_y_m(self) -> float:
        return self.ring_center_y_from_south_m - self.p_origin_y_from_south_m

    @property
    def ring_centerline_radius_m(self) -> float:
        return (self.ring_outer_height_m + self.ring_inner_height_m) / 4.0

    @property
    def ring_cap_center_separation_m(self) -> float:
        outer = self.ring_outer_width_m - self.ring_outer_height_m
        inner = self.ring_inner_width_m - self.ring_inner_height_m
        return (outer + inner) / 2.0

    @property
    def ring_left_cap_x_m(self) -> float:
        return self.ring_center_x_m - self.ring_cap_center_separation_m / 2.0

    @property
    def ring_right_cap_x_m(self) -> float:
        return self.ring_center_x_m + self.ring_cap_center_separation_m / 2.0

    def zone_for(self, x_m: float, y_m: float) -> str:
        """Infer the diagram zone for a point expressed in the P-origin frame."""
        if math.hypot(x_m, y_m) <= ORIGIN_TOLERANCE_M:
            return "P"
        if y_m <= self.zone_a_y_max_m + GEOMETRY_TOLERANCE_M:
            return "A"
        if y_m <= self.zone_b_y_max_m + GEOMETRY_TOLERANCE_M:
            return "B"
        return "C"


@dataclass(frozen=True)
class RoutePoint:
    id: str
    zone: str
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class RouteDocument:
    schema_version: int
    name: str
    frame_id: str
    calibrated: bool
    geometry: FieldGeometry
    waypoints: tuple[RoutePoint, ...]
    source: Mapping[str, Any]


_GEOMETRY_FIELDS = tuple(FieldGeometry.__dataclass_fields__)


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RouteValidationError(f"{label} must be a mapping")
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise RouteValidationError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise RouteValidationError(f"{label} must be a finite number") from error
    if not math.isfinite(result):
        raise RouteValidationError(f"{label} must be a finite number")
    return result


def _required_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RouteValidationError(f"{label} must be a nonempty string")
    return value.strip()


def normalize_yaw_deg(value: float) -> float:
    """Normalize a finite heading to [-180, 180)."""
    yaw = _finite_number(value, "yaw_deg")
    normalized = (yaw + 180.0) % 360.0 - 180.0
    if abs(normalized) < 1e-12:
        return 0.0
    return normalized


def angular_distance_deg(first: float, second: float) -> float:
    return abs(normalize_yaw_deg(second - first))


def _read_yaml(path: Path, label: str) -> Mapping[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RouteValidationError(f"cannot load {label}: {error}") from error
    return _mapping(document, label)


def _parse_geometry(raw: Any) -> FieldGeometry:
    item = _mapping(raw, "geometry")
    unknown = sorted(set(item) - set(_GEOMETRY_FIELDS))
    missing = sorted(set(_GEOMETRY_FIELDS) - set(item))
    if unknown:
        raise RouteValidationError(
            "geometry contains unknown fields: " + ", ".join(unknown))
    if missing:
        raise RouteValidationError(
            "geometry is missing fields: " + ", ".join(missing))
    geometry = FieldGeometry(**{
        name: _finite_number(item[name], f"geometry.{name}")
        for name in _GEOMETRY_FIELDS
    })
    validate_geometry(geometry)
    return geometry


def validate_geometry(geometry: FieldGeometry) -> None:
    positive_fields = (
        "field_width_m", "field_height_m", "zone_a_height_m",
        "zone_b_height_m", "zone_c_height_m", "corridor_width_m",
        "ring_outer_width_m", "ring_outer_height_m",
        "ring_inner_width_m", "ring_inner_height_m",
    )
    for name in positive_fields:
        if getattr(geometry, name) <= 0.0:
            raise RouteValidationError(f"geometry.{name} must be positive")
    zone_sum = (
        geometry.zone_a_height_m
        + geometry.zone_b_height_m
        + geometry.zone_c_height_m
    )
    if abs(zone_sum - geometry.field_height_m) > GEOMETRY_TOLERANCE_M:
        raise RouteValidationError("zone heights must sum to field_height_m")
    if not geometry.ring_outer_width_m > geometry.ring_inner_width_m:
        raise RouteValidationError("ring outer width must exceed inner width")
    if not geometry.ring_outer_height_m > geometry.ring_inner_height_m:
        raise RouteValidationError("ring outer height must exceed inner height")
    lane_width_x = (
        geometry.ring_outer_width_m - geometry.ring_inner_width_m) / 2.0
    lane_width_y = (
        geometry.ring_outer_height_m - geometry.ring_inner_height_m) / 2.0
    if abs(lane_width_x - lane_width_y) > GEOMETRY_TOLERANCE_M:
        raise RouteValidationError("ring lane must have a uniform width")
    outer_straight = geometry.ring_outer_width_m - geometry.ring_outer_height_m
    inner_straight = geometry.ring_inner_width_m - geometry.ring_inner_height_m
    if abs(outer_straight - inner_straight) > GEOMETRY_TOLERANCE_M:
        raise RouteValidationError("ring outlines must be concentric stadiums")
    if geometry.corridor_width_m > geometry.field_width_m:
        raise RouteValidationError("corridor is wider than the field")

    def inside(value: float, lower: float, upper: float, label: str) -> None:
        if value < lower or value > upper:
            raise RouteValidationError(f"{label} must be inside the field")

    inside(
        geometry.p_origin_x_from_west_m, 0.0, geometry.field_width_m,
        "P origin x",
    )
    inside(
        geometry.p_origin_y_from_south_m, 0.0, geometry.zone_a_height_m,
        "P origin y",
    )
    inside(
        geometry.task_point_x_from_west_m, 0.0, geometry.field_width_m,
        "task point x",
    )
    inside(
        geometry.task_point_y_from_south_m, 0.0, geometry.zone_a_height_m,
        "task point y",
    )
    half_outer_width = geometry.ring_outer_width_m / 2.0
    half_outer_height = geometry.ring_outer_height_m / 2.0
    if (
        geometry.ring_center_x_from_west_m - half_outer_width < 0.0
        or geometry.ring_center_x_from_west_m + half_outer_width
        > geometry.field_width_m
    ):
        raise RouteValidationError("ring outer width exceeds field bounds")
    c_south = geometry.zone_a_height_m + geometry.zone_b_height_m
    if (
        geometry.ring_center_y_from_south_m - half_outer_height < c_south
        or geometry.ring_center_y_from_south_m + half_outer_height
        > geometry.field_height_m
    ):
        raise RouteValidationError("ring outer height exceeds zone C bounds")


def load_field_geometry(path: os.PathLike[str] | str) -> FieldGeometry:
    root = _read_yaml(Path(path), "field geometry")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise RouteValidationError(
            f"field geometry schema_version must be {SCHEMA_VERSION}")
    return _parse_geometry(root.get("geometry"))


def _parse_point(raw: Any, index: int) -> RoutePoint:
    item = _mapping(raw, f"waypoints[{index}]")
    point_id = _required_string(item.get("id"), f"waypoints[{index}].id")
    if POINT_ID_PATTERN.fullmatch(point_id) is None:
        raise RouteValidationError(
            f"waypoints[{index}].id contains unsupported characters")
    zone = _required_string(item.get("zone"), f"waypoints[{index}].zone").upper()
    if zone not in ZONE_NAMES:
        raise RouteValidationError(f"waypoints[{index}].zone must be P, A, B, or C")
    return RoutePoint(
        id=point_id,
        zone=zone,
        x=_finite_number(item.get("x"), f"waypoints[{index}].x"),
        y=_finite_number(item.get("y"), f"waypoints[{index}].y"),
        yaw_deg=normalize_yaw_deg(
            _finite_number(item.get("yaw_deg"), f"waypoints[{index}].yaw_deg")),
    )


def _signed_area(points: Sequence[RoutePoint]) -> float:
    return 0.5 * sum(
        first.x * second.y - second.x * first.y
        for first, second in zip(points, points[1:])
    )


def _validate_loop(route: RouteDocument) -> None:
    loop = tuple(point for point in route.waypoints if point.id.startswith("c_loop_"))
    if len(loop) < 12:
        raise RouteValidationError("route must contain a sampled C-zone loop")
    if math.hypot(loop[-1].x - loop[0].x, loop[-1].y - loop[0].y) > LOOP_CLOSURE_TOLERANCE_M:
        raise RouteValidationError("C-zone loop must close at its start point")
    if _signed_area(loop) >= -0.5:
        raise RouteValidationError("C-zone loop must cover one clockwise circuit")
    for index, (first, second) in enumerate(zip(loop, loop[1:])):
        spacing = math.hypot(second.x - first.x, second.y - first.y)
        if spacing <= 1e-9:
            raise RouteValidationError(
                f"C-zone loop segment {index} has duplicate points")
        if spacing > MAX_LOOP_POINT_SPACING_M + 1e-9:
            raise RouteValidationError(
                f"C-zone loop segment {index} exceeds {MAX_LOOP_POINT_SPACING_M:.2f} m")
        if angular_distance_deg(first.yaw_deg, second.yaw_deg) > MAX_LOOP_YAW_STEP_DEG:
            raise RouteValidationError(
                f"C-zone loop heading changes too abruptly at segment {index}")

    geometry = route.geometry
    expected = (
        geometry.ring_left_cap_x_m - geometry.ring_centerline_radius_m,
        geometry.ring_right_cap_x_m + geometry.ring_centerline_radius_m,
        geometry.ring_center_y_m - geometry.ring_centerline_radius_m,
        geometry.ring_center_y_m + geometry.ring_centerline_radius_m,
    )
    actual = (
        min(point.x for point in loop),
        max(point.x for point in loop),
        min(point.y for point in loop),
        max(point.y for point in loop),
    )
    if any(
        abs(observed - target) > LOOP_EXTENT_TOLERANCE_M
        for observed, target in zip(actual, expected)
    ):
        raise RouteValidationError("C-zone loop does not follow the rule-diagram centerline")


def validate_route(route: RouteDocument) -> None:
    if route.schema_version != SCHEMA_VERSION:
        raise RouteValidationError(
            f"route schema_version must be {SCHEMA_VERSION}")
    _required_string(route.name, "name")
    if route.frame_id != ROUTE_FRAME:
        raise RouteValidationError(f"frame_id must be {ROUTE_FRAME}")
    if not isinstance(route.calibrated, bool):
        raise RouteValidationError("calibrated must be true or false")
    validate_geometry(route.geometry)
    if len(route.waypoints) < 5:
        raise RouteValidationError("route must contain at least five waypoints")

    identifiers = [point.id for point in route.waypoints]
    if len(set(identifiers)) != len(identifiers):
        raise RouteValidationError("waypoint ids must be unique")
    for index, point in enumerate(route.waypoints):
        _parse_point({
            "id": point.id,
            "zone": point.zone,
            "x": point.x,
            "y": point.y,
            "yaw_deg": point.yaw_deg,
        }, index)
        if not (
            route.geometry.x_min_m - GEOMETRY_TOLERANCE_M
            <= point.x
            <= route.geometry.x_max_m + GEOMETRY_TOLERANCE_M
            and route.geometry.y_min_m - GEOMETRY_TOLERANCE_M
            <= point.y
            <= route.geometry.y_max_m + GEOMETRY_TOLERANCE_M
        ):
            raise RouteValidationError(f"waypoint {point.id} is outside the 5 m field")
        inferred = route.geometry.zone_for(point.x, point.y)
        if point.zone != inferred:
            raise RouteValidationError(
                f"waypoint {point.id} is labeled {point.zone} but lies in {inferred}")

    first = route.waypoints[0]
    last = route.waypoints[-1]
    if first.zone != "P" or last.zone != "P":
        raise RouteValidationError("first and last waypoints must be in the P zone")
    if (
        math.hypot(first.x, first.y) > ORIGIN_TOLERANCE_M
        or math.hypot(last.x, last.y) > ORIGIN_TOLERANCE_M
    ):
        raise RouteValidationError("first and last waypoints must be the P origin")
    if abs(normalize_yaw_deg(first.yaw_deg)) > 1e-9:
        raise RouteValidationError("start heading must face +X (0 degrees)")
    if sum(point.zone == "P" for point in route.waypoints) != 2:
        raise RouteValidationError("only the first and last waypoints may use zone P")
    if not any(point.id == "a_task" for point in route.waypoints):
        raise RouteValidationError("route must include the estimated task publication point")
    if not any(point.zone == "B" for point in route.waypoints):
        raise RouteValidationError("route must pass through the B-zone corridor")
    _validate_loop(route)


def load_route(path: os.PathLike[str] | str) -> RouteDocument:
    root = _read_yaml(Path(path), "route")
    raw_points = root.get("waypoints")
    if not isinstance(raw_points, list):
        raise RouteValidationError("waypoints must be a list")
    calibrated = root.get("calibrated")
    if not isinstance(calibrated, bool):
        raise RouteValidationError("calibrated must be true or false")
    source = root.get("source", {})
    source_mapping = dict(_mapping(source, "source"))
    route = RouteDocument(
        schema_version=root.get("schema_version"),
        name=_required_string(root.get("name"), "name"),
        frame_id=_required_string(root.get("frame_id"), "frame_id"),
        calibrated=calibrated,
        geometry=_parse_geometry(root.get("geometry")),
        waypoints=tuple(_parse_point(item, index) for index, item in enumerate(raw_points)),
        source=source_mapping,
    )
    validate_route(route)
    return route


def _geometry_mapping(geometry: FieldGeometry) -> dict[str, float]:
    return {name: getattr(geometry, name) for name in _GEOMETRY_FIELDS}


def route_to_mapping(route: RouteDocument) -> dict[str, Any]:
    validate_route(route)
    return {
        "schema_version": route.schema_version,
        "name": route.name,
        "frame_id": route.frame_id,
        "calibrated": route.calibrated,
        "source": dict(route.source),
        "geometry": _geometry_mapping(route.geometry),
        "waypoints": [
            {
                "id": point.id,
                "zone": point.zone,
                "x": round(point.x, 6),
                "y": round(point.y, 6),
                "yaw_deg": round(normalize_yaw_deg(point.yaw_deg), 6),
            }
            for point in route.waypoints
        ],
    }


def write_route_atomic(route: RouteDocument, path: os.PathLike[str] | str) -> None:
    """Validate and atomically replace one route YAML in its destination directory."""
    destination = Path(path)
    payload = yaml.safe_dump(
        route_to_mapping(route),
        allow_unicode=True,
        sort_keys=False,
        width=100,
    )
    try:
        destination.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=str(destination.parent),
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, destination)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
    except OSError as error:
        raise RouteValidationError(f"cannot write route: {error}") from error


def _append_line(
    points: list[tuple[float, float, float]],
    end_x: float,
    end_y: float,
    yaw_deg: float,
    max_spacing_m: float,
) -> None:
    start_x, start_y, _ = points[-1]
    distance = math.hypot(end_x - start_x, end_y - start_y)
    segments = max(1, math.ceil(distance / max_spacing_m))
    for index in range(1, segments + 1):
        fraction = index / segments
        points.append((
            start_x + (end_x - start_x) * fraction,
            start_y + (end_y - start_y) * fraction,
            normalize_yaw_deg(yaw_deg),
        ))


def _append_arc(
    points: list[tuple[float, float, float]],
    center_x: float,
    center_y: float,
    radius_m: float,
    start_angle_rad: float,
    end_angle_rad: float,
    max_spacing_m: float,
) -> None:
    arc_length = abs(end_angle_rad - start_angle_rad) * radius_m
    segments = max(2, math.ceil(arc_length / max_spacing_m))
    for index in range(1, segments + 1):
        fraction = index / segments
        angle = start_angle_rad + (end_angle_rad - start_angle_rad) * fraction
        points.append((
            center_x + radius_m * math.cos(angle),
            center_y + radius_m * math.sin(angle),
            normalize_yaw_deg(math.degrees(angle - math.pi / 2.0)),
        ))


def generate_clockwise_c_loop(
    geometry: FieldGeometry,
    max_spacing_m: float = 0.20,
) -> tuple[RoutePoint, ...]:
    if not 0.02 <= max_spacing_m <= MAX_LOOP_POINT_SPACING_M:
        raise RouteValidationError(
            f"loop spacing must be between 0.02 and {MAX_LOOP_POINT_SPACING_M:.2f} m")
    radius = geometry.ring_centerline_radius_m
    left_x = geometry.ring_left_cap_x_m
    right_x = geometry.ring_right_cap_x_m
    center_y = geometry.ring_center_y_m
    bottom_y = center_y - radius
    merge_x = geometry.corridor_center_x_m - 0.30
    if not left_x < merge_x < right_x:
        merge_x = (left_x + right_x) / 2.0

    samples: list[tuple[float, float, float]] = [(merge_x, bottom_y, -180.0)]
    _append_line(samples, left_x, bottom_y, -180.0, max_spacing_m)
    _append_arc(
        samples, left_x, center_y, radius,
        -math.pi / 2.0, -3.0 * math.pi / 2.0, max_spacing_m,
    )
    _append_line(samples, right_x, center_y + radius, 0.0, max_spacing_m)
    _append_arc(
        samples, right_x, center_y, radius,
        math.pi / 2.0, -math.pi / 2.0, max_spacing_m,
    )
    _append_line(samples, merge_x, bottom_y, -180.0, max_spacing_m)
    return tuple(
        RoutePoint(
            id=f"c_loop_{index:03d}",
            zone="C",
            x=round(x_m, 6),
            y=round(y_m, 6),
            yaw_deg=round(normalize_yaw_deg(yaw_deg), 6),
        )
        for index, (x_m, y_m, yaw_deg) in enumerate(samples)
    )


def generate_baseline_route(geometry: FieldGeometry) -> RouteDocument:
    """Generate the uncalibrated P-task-corridor-C-loop-P rule baseline."""
    cx = geometry.corridor_center_x_m
    task_x = geometry.task_point_x_from_west_m - geometry.p_origin_x_from_west_m
    task_y = geometry.task_point_y_from_south_m - geometry.p_origin_y_from_south_m
    ring_bottom_y = geometry.ring_center_y_m - geometry.ring_centerline_radius_m
    before = (
        RoutePoint("p_start", "P", 0.0, 0.0, 0.0),
        RoutePoint("a_depart", "A", 0.8, 0.05, 5.0),
        RoutePoint("a_sweep", "A", 2.1, 0.25, 15.0),
        RoutePoint("a_task_approach", "A", 3.45, 0.80, 30.0),
        RoutePoint("a_task", "A", task_x, task_y, 0.0),
        RoutePoint("a_turn_south", "A", 4.35, 1.05, -90.0),
        RoutePoint("a_turn_west", "A", 4.00, 0.65, -180.0),
        RoutePoint("a_cross", "A", 3.00, 0.65, -180.0),
        RoutePoint("a_corridor_curve", "A", cx + 0.25, 0.75, 150.0),
        RoutePoint("a_corridor_align", "A", cx - 0.25, 1.15, 90.0),
        RoutePoint("a_corridor_entry", "A", cx, 1.65, 90.0),
        RoutePoint("b_corridor_01", "B", cx, 1.90, 90.0),
        RoutePoint("b_corridor_02", "B", cx, 2.15, 90.0),
        RoutePoint("c_entry", "C", cx, 2.40, 90.0),
        RoutePoint("c_merge", "C", cx - 0.15, ring_bottom_y - 0.13, 135.0),
    )
    after = (
        RoutePoint("c_exit", "C", cx - 0.45, ring_bottom_y - 0.13, -135.0),
        RoutePoint("b_corridor_return", "B", cx - 0.30, 2.15, -90.0),
        RoutePoint("a_corridor_return", "A", cx - 0.20, 1.65, -90.0),
        RoutePoint("a_return_curve", "A", 1.65, 1.10, -110.0),
        RoutePoint("a_return_sweep", "A", 1.20, 0.55, -145.0),
        RoutePoint("a_return_approach", "A", 0.50, 0.15, -170.0),
        RoutePoint("p_finish", "P", 0.0, 0.0, -180.0),
    )
    route = RouteDocument(
        schema_version=SCHEMA_VERSION,
        name="full_course_rule_baseline",
        frame_id=ROUTE_FRAME,
        calibrated=False,
        geometry=geometry,
        waypoints=before + generate_clockwise_c_loop(geometry) + after,
        source={
            "kind": "competition_rule_diagram",
            "diagrams": [
                "../reference/competition_field_dimensions.png",
                "../reference/competition_field_route_example.png",
            ],
            "geometry_status": "rule dimensions with estimated P/task landmarks",
            "warning": "UNCALIBRATED: measure the real venue before enabling motion gates",
        },
    )
    validate_route(route)
    return route


def replace_waypoints(
    route: RouteDocument,
    waypoints: Iterable[RoutePoint],
    *,
    calibrated: bool | None = None,
) -> RouteDocument:
    result = replace(
        route,
        waypoints=tuple(waypoints),
        calibrated=route.calibrated if calibrated is None else calibrated,
    )
    validate_route(result)
    return result
