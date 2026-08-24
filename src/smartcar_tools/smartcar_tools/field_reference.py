"""ROS-independent geometry for the competition field reference overlay."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import yaml

from ._validators import finite_number


REFERENCE_FRAME = "odom_combined"
DEFAULT_ARC_SAMPLES = 24
SCHEMA_VERSION = 1


class FieldReferenceError(ValueError):
    """Raised when the rule-diagram geometry is missing or inconsistent."""


@dataclass(frozen=True)
class FieldGeometry:
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
        return self.corridor_center_x_from_west_m - self.p_origin_x_from_west_m

    @property
    def ring_center_x_m(self) -> float:
        return self.ring_center_x_from_west_m - self.p_origin_x_from_west_m

    @property
    def ring_center_y_m(self) -> float:
        return self.ring_center_y_from_south_m - self.p_origin_y_from_south_m


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Bounds2D:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def center(self) -> Point2D:
        return Point2D(
            x=(self.x_min + self.x_max) / 2.0,
            y=(self.y_min + self.y_max) / 2.0,
        )


@dataclass(frozen=True)
class ReferenceLabel:
    text: str
    position: Point2D


@dataclass(frozen=True)
class FieldReference:
    frame_id: str
    field: Bounds2D
    zones: dict[str, Bounds2D]
    corridor: Bounds2D
    b_walls: tuple[Bounds2D, Bounds2D]
    ring_outer: Bounds2D
    ring_inner: Bounds2D
    ring_outer_outline: tuple[Point2D, ...]
    ring_inner_outline: tuple[Point2D, ...]
    p_origin: Point2D
    task_point: Point2D
    labels: tuple[ReferenceLabel, ...]


_GEOMETRY_FIELDS = tuple(FieldGeometry.__dataclass_fields__)


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise FieldReferenceError(f"{label} must be a mapping")
    return value


def _finite_number(value: Any, label: str) -> float:
    return finite_number(value, label, FieldReferenceError)


def _within(value: float, lower: float, upper: float, label: str) -> None:
    if not lower <= value <= upper:
        raise FieldReferenceError(f"{label} lies outside the field")


def validate_field_geometry(geometry: FieldGeometry) -> None:
    positive = (
        "field_width_m",
        "field_height_m",
        "zone_a_height_m",
        "zone_b_height_m",
        "zone_c_height_m",
        "corridor_width_m",
        "ring_outer_width_m",
        "ring_outer_height_m",
        "ring_inner_width_m",
        "ring_inner_height_m",
    )
    for name in positive:
        if getattr(geometry, name) <= 0.0:
            raise FieldReferenceError(f"geometry.{name} must be positive")

    zone_height = (
        geometry.zone_a_height_m
        + geometry.zone_b_height_m
        + geometry.zone_c_height_m
    )
    if not math.isclose(zone_height, geometry.field_height_m, abs_tol=1e-9):
        raise FieldReferenceError("zone heights must cover the field")
    if geometry.corridor_width_m > geometry.field_width_m:
        raise FieldReferenceError("corridor is wider than the field")
    half_corridor = geometry.corridor_width_m / 2.0
    if not (
        half_corridor
        <= geometry.corridor_center_x_from_west_m
        <= geometry.field_width_m - half_corridor
    ):
        raise FieldReferenceError("corridor lies outside the field")

    if not geometry.ring_outer_width_m > geometry.ring_inner_width_m:
        raise FieldReferenceError("ring outer width must exceed inner width")
    if not geometry.ring_outer_height_m > geometry.ring_inner_height_m:
        raise FieldReferenceError("ring outer height must exceed inner height")
    outer_straight = geometry.ring_outer_width_m - geometry.ring_outer_height_m
    inner_straight = geometry.ring_inner_width_m - geometry.ring_inner_height_m
    if not math.isclose(outer_straight, inner_straight, abs_tol=1e-9):
        raise FieldReferenceError("ring boundaries must be concentric stadiums")
    if geometry.corridor_width_m > outer_straight:
        raise FieldReferenceError("corridor opening is wider than the ring straight")
    if not math.isclose(
        geometry.corridor_center_x_from_west_m,
        geometry.ring_center_x_from_west_m,
        abs_tol=1e-9,
    ):
        raise FieldReferenceError("corridor and ring centers must align")

    _within(
        geometry.p_origin_x_from_west_m,
        0.0,
        geometry.field_width_m,
        "P origin x",
    )
    _within(
        geometry.p_origin_y_from_south_m,
        0.0,
        geometry.zone_a_height_m,
        "P origin y",
    )
    _within(
        geometry.task_point_x_from_west_m,
        0.0,
        geometry.field_width_m,
        "task point x",
    )
    _within(
        geometry.task_point_y_from_south_m,
        0.0,
        geometry.zone_a_height_m,
        "task point y",
    )

    outer_half_width = geometry.ring_outer_width_m / 2.0
    outer_half_height = geometry.ring_outer_height_m / 2.0
    c_zone_south = geometry.zone_a_height_m + geometry.zone_b_height_m
    if not (
        outer_half_width
        <= geometry.ring_center_x_from_west_m
        <= geometry.field_width_m - outer_half_width
        and c_zone_south + outer_half_height
        <= geometry.ring_center_y_from_south_m
        <= geometry.field_height_m - outer_half_height
    ):
        raise FieldReferenceError("outer C ring lies outside zone C")


def load_field_geometry(path: str | Path) -> FieldGeometry:
    """Load and validate the versioned field geometry YAML document."""
    source = Path(path)
    try:
        root = _mapping(
            yaml.safe_load(source.read_text(encoding="utf-8")),
            "field geometry document",
        )
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise FieldReferenceError(f"cannot load field geometry: {error}") from error
    if root.get("schema_version") != SCHEMA_VERSION:
        raise FieldReferenceError(
            f"field geometry schema_version must be {SCHEMA_VERSION}"
        )

    raw = _mapping(root.get("geometry"), "geometry")
    unknown = sorted(set(raw) - set(_GEOMETRY_FIELDS))
    missing = sorted(set(_GEOMETRY_FIELDS) - set(raw))
    if unknown:
        raise FieldReferenceError(
            "geometry contains unknown fields: " + ", ".join(unknown)
        )
    if missing:
        raise FieldReferenceError(
            "geometry is missing fields: " + ", ".join(missing)
        )
    geometry = FieldGeometry(**{
        name: _finite_number(raw[name], f"geometry.{name}")
        for name in _GEOMETRY_FIELDS
    })
    validate_field_geometry(geometry)
    return geometry


def _stadium_outline(
    center_x: float,
    center_y: float,
    width: float,
    height: float,
    arc_samples: int = DEFAULT_ARC_SAMPLES,
    bottom_opening_width: float = 0.0,
) -> tuple[Point2D, ...]:
    """Return a stadium outline, optionally opened at the bottom center."""
    if not all(math.isfinite(value) for value in (
        width,
        height,
        bottom_opening_width,
    )):
        raise ValueError("stadium dimensions must be finite")
    if width < height or height <= 0.0:
        raise ValueError("stadium width must be at least its positive height")
    if not 0.0 <= bottom_opening_width <= width - height:
        raise ValueError("stadium bottom opening must fit within the straight edge")
    if arc_samples < 2:
        raise ValueError("stadium arcs require at least two samples")

    radius = height / 2.0
    cap_offset = (width - height) / 2.0
    left_x = center_x - cap_offset
    right_x = center_x + cap_offset
    bottom_y = center_y - radius
    if bottom_opening_width > 0.0:
        points = [Point2D(center_x + bottom_opening_width / 2.0, bottom_y)]
    else:
        points = [Point2D(left_x, bottom_y)]
    points.append(Point2D(right_x, bottom_y))

    for index in range(1, arc_samples + 1):
        angle = -math.pi / 2.0 + math.pi * index / arc_samples
        points.append(Point2D(
            right_x + radius * math.cos(angle),
            center_y + radius * math.sin(angle),
        ))

    points.append(Point2D(left_x, center_y + radius))
    for index in range(1, arc_samples + 1):
        angle = math.pi / 2.0 + math.pi * index / arc_samples
        if index == arc_samples and bottom_opening_width == 0.0:
            points.append(points[0])
        else:
            points.append(Point2D(
                left_x + radius * math.cos(angle),
                center_y + radius * math.sin(angle),
            ))
    if bottom_opening_width > 0.0:
        points.append(Point2D(center_x - bottom_opening_width / 2.0, bottom_y))
    return tuple(points)


def _bounds(center_x: float, center_y: float, width: float, height: float) -> Bounds2D:
    return Bounds2D(
        x_min=round(center_x - width / 2.0, 9),
        x_max=round(center_x + width / 2.0, 9),
        y_min=round(center_y - height / 2.0, 9),
        y_max=round(center_y + height / 2.0, 9),
    )


def build_field_reference(geometry: FieldGeometry) -> FieldReference:
    """Build the metric rule-diagram overlay in the P-origin coordinate frame."""
    field = Bounds2D(
        geometry.x_min_m,
        geometry.x_max_m,
        geometry.y_min_m,
        geometry.y_max_m,
    )
    zone_a_top = geometry.zone_a_y_max_m
    zone_b_top = geometry.zone_b_y_max_m
    zones = {
        "A": Bounds2D(field.x_min, field.x_max, field.y_min, zone_a_top),
        "B": Bounds2D(field.x_min, field.x_max, zone_a_top, zone_b_top),
        "C": Bounds2D(field.x_min, field.x_max, zone_b_top, field.y_max),
    }

    ring_outer = _bounds(
        geometry.ring_center_x_m,
        geometry.ring_center_y_m,
        geometry.ring_outer_width_m,
        geometry.ring_outer_height_m,
    )
    ring_inner = _bounds(
        geometry.ring_center_x_m,
        geometry.ring_center_y_m,
        geometry.ring_inner_width_m,
        geometry.ring_inner_height_m,
    )
    half_corridor = geometry.corridor_width_m / 2.0
    corridor = Bounds2D(
        geometry.corridor_center_x_m - half_corridor,
        geometry.corridor_center_x_m + half_corridor,
        zone_a_top,
        ring_outer.y_min,
    )
    b_walls = (
        Bounds2D(field.x_min, corridor.x_min, zone_a_top, zone_b_top),
        Bounds2D(corridor.x_max, field.x_max, zone_a_top, zone_b_top),
    )
    p_origin = Point2D(0.0, 0.0)
    task_point = Point2D(
        geometry.task_point_x_from_west_m - geometry.p_origin_x_from_west_m,
        geometry.task_point_y_from_south_m - geometry.p_origin_y_from_south_m,
    )
    labels = (
        ReferenceLabel("Zone A", zones["A"].center),
        ReferenceLabel("Zone B", b_walls[0].center),
        ReferenceLabel("Zone C", Point2D(field.center.x, field.y_max - 0.25)),
        ReferenceLabel("Corridor", corridor.center),
        ReferenceLabel("C Ring", Point2D(
            geometry.ring_center_x_m,
            geometry.ring_center_y_m,
        )),
    )

    return FieldReference(
        frame_id=REFERENCE_FRAME,
        field=field,
        zones=zones,
        corridor=corridor,
        b_walls=b_walls,
        ring_outer=ring_outer,
        ring_inner=ring_inner,
        ring_outer_outline=_stadium_outline(
            geometry.ring_center_x_m,
            geometry.ring_center_y_m,
            geometry.ring_outer_width_m,
            geometry.ring_outer_height_m,
            bottom_opening_width=geometry.corridor_width_m,
        ),
        ring_inner_outline=_stadium_outline(
            geometry.ring_center_x_m,
            geometry.ring_center_y_m,
            geometry.ring_inner_width_m,
            geometry.ring_inner_height_m,
        ),
        p_origin=p_origin,
        task_point=task_point,
        labels=labels,
    )


def load_field_reference(path: str | Path) -> FieldReference:
    """Load a geometry document and build its display-only reference model."""
    return build_field_reference(load_field_geometry(path))
