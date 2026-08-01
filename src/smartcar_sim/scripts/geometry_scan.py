#!/usr/bin/env python3
"""Offline geometry scanner for C-return and B-opening waypoint candidates.

The C mode enumerates return-path candidates. The B mode scans only ordinary
``b_corridor_gate`` and ``b_corridor_enter`` positions, verifies a reverse
chain through the B opening, and never turns its diagnostic headings into
semantic waypoint orientations.

Pure geometry — no ROS or Gazebo process is required.
"""

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from itertools import product
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import yaml

# ── constants ──────────────────────────────────────────────────────────
# The scanner is source-only, so load the same shared planning configuration
# and effective (rasterized) PGM keepouts that simulation Nav2 consumes.
SCRIPT = Path(__file__).resolve()
SOURCE_ROOT = SCRIPT.parents[2]
SOURCE_TOOLS_ROOT = SOURCE_ROOT / "smartcar_tools"
GEOMETRY_RELATIVE_PATH = Path("config") / "routes" / "field_geometry.yaml"
ROUTE_PLANNING_RELATIVE_PATH = Path("config") / "routes" / "route_planning.yaml"
DEFAULT_GEOMETRY = SOURCE_TOOLS_ROOT / GEOMETRY_RELATIVE_PATH
DEFAULT_ROUTE_PLANNING_CONFIG = SOURCE_TOOLS_ROOT / ROUTE_PLANNING_RELATIVE_PATH
DEFAULT_NAV_ONLY_WAYPOINTS = (
    SOURCE_ROOT / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml"
)

try:
    from smartcar_tools.field_keepouts import keepout_mask_bounds
    from smartcar_tools.field_reference import Bounds2D, FieldReference, load_field_reference
    from smartcar_tools.route_planning import (
        RoutePlanningConfig,
        load_route_planning_config,
    )
except ModuleNotFoundError:
    sys.path.insert(0, str(SOURCE_TOOLS_ROOT))
    from smartcar_tools.field_keepouts import keepout_mask_bounds  # type: ignore[no-redef]
    from smartcar_tools.field_reference import (  # type: ignore[no-redef]
        Bounds2D,
        FieldReference,
        load_field_reference,
    )
    from smartcar_tools.route_planning import (  # type: ignore[no-redef]
        RoutePlanningConfig,
        load_route_planning_config,
    )


# ``keepout_bounds`` has a stable order in smartcar_tools.field_keepouts:
# west/east B walls, central C core, then the three rasterized outer C limits.
KEEP_OUT_NAMES = (
    "b_wall_west",
    "b_wall_east",
    "c_core",
    "pgm_c_north",
    "pgm_c_west",
    "pgm_c_east",
)


@dataclass(frozen=True)
class GeometryContext:
    """Authoritative field, motion, and PGM keepout geometry for one scan."""

    reference: FieldReference
    config: RoutePlanningConfig
    keepouts: Tuple[Bounds2D, ...]
    keepout_names: Tuple[str, ...] = KEEP_OUT_NAMES

    @property
    def half_length_m(self) -> float:
        return self.config.runtime_footprint.padded_half_length_m

    @property
    def half_width_m(self) -> float:
        return self.config.runtime_footprint.padded_half_width_m


_DEFAULT_CONTEXT: Optional[GeometryContext] = None


def load_geometry_context(
    route_planning_file: str | Path | None = None,
    geometry_file: str | Path | None = None,
) -> GeometryContext:
    """Load the shared route constraints and effective PGM keepout bounds."""
    config = load_route_planning_config(
        route_planning_file or DEFAULT_ROUTE_PLANNING_CONFIG
    )
    reference = load_field_reference(geometry_file or DEFAULT_GEOMETRY)
    keepouts = tuple(keepout_mask_bounds(reference, config))
    if len(keepouts) != len(KEEP_OUT_NAMES):
        raise ValueError(
            "unexpected keepout geometry: expected "
            f"{len(KEEP_OUT_NAMES)} rectangles, got {len(keepouts)}"
        )
    return GeometryContext(reference, config, keepouts)


def default_geometry_context() -> GeometryContext:
    """Return the cached source-tree geometry used by direct helper calls."""
    global _DEFAULT_CONTEXT
    if _DEFAULT_CONTEXT is None:
        _DEFAULT_CONTEXT = load_geometry_context()
    return _DEFAULT_CONTEXT
GOAL_XY_TOLERANCE = 0.25           # standard goal checker XY tolerance (m)
GOAL_YAW_TOLERANCE = 0.50          # standard goal checker yaw tolerance (rad)
DUBINS_STEP = 0.02                 # path discretisation step (m)
LOOP_THRESHOLD = 5.0               # path longer than this (m) → likely loop
SUSPICIOUS_THRESHOLD = 3.5         # path longer than this → suspicious

# ── field geometry ─────────────────────────────────────────────────────
FIELD_W = 5.0
FIELD_H = 5.0

# B-zone walls from the official field geometry.
WEST_WALL = ((-0.5, 1.75), (1.5, 2.25))    # (x_min, x_max), (y_min, y_max)
EAST_WALL = ((2.5, 1.75), (4.5, 2.25))

# corridor (B-zone pass-through)
CORRIDOR_X_MIN = 2.0
CORRIDOR_X_MAX = 3.0
CORRIDOR_Y_MIN = 2.0
CORRIDOR_Y_MAX = 2.5

# ring (zone C)
RING_OUTER = ((0.5, 2.75), (4.5, 4.4))     # outer ring bounding box
RING_INNER = ((1.0, 3.25), (4.0, 3.9))     # inner ring bounding box


@dataclass
class Pose:
    x: float
    y: float
    yaw: float  # radians

    def offset(self, dx: float, dy: float, dyaw: float) -> "Pose":
        return Pose(self.x + dx, self.y + dy,
                    math.remainder(self.yaw + dyaw, 2 * math.pi))

    def dist_to(self, other: "Pose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


@dataclass
class Waypoint:
    wid: str
    x: float
    y: float
    yaw: float
    task: str = "nav"
    direction: str = "forward"

    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.yaw)


@dataclass
class SegmentResult:
    segment: str                     # e.g. "c2→c3", "c3→c4", "c4→return"
    start_pose: Pose
    end_pose: Pose
    dubins_type: str                 # "LSL", "RSR", etc.
    path_length: float
    path_points: List[Tuple[float, float]]
    straight_length: float
    total_turn_angle: float          # rad
    is_loop: bool
    wall_collisions: int             # B-wall OBB collisions
    core_collisions: int             # central C-core OBB collisions
    boundary_collisions: int         # outer PGM/field-boundary OBB collisions
    pgm_keepout_collisions: int      # all effective PGM keepout collisions
    feasible: bool


# ── Dubins path ────────────────────────────────────────────────────────

def _mod2pi(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.remainder(a, 2 * math.pi)


def _mod2pi_pos(a: float) -> float:
    """Wrap angle to [0, 2π) — used for turn-angle validity checks."""
    a = math.fmod(a, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a


def _range_sym(a: float, b: float):
    """Yield angles from a to b (both inclusive), shortest direction."""
    diff = _mod2pi(b - a)
    sign = 1.0 if diff >= 0 else -1.0
    steps = max(1, int(abs(diff) / 0.02))
    for i in range(steps + 1):
        yield _mod2pi(a + sign * abs(diff) * i / steps)


def _dubins_LSL(start: Pose, end: Pose, r: float):
    """Compute LSL Dubins path. Returns (L, alpha, beta, ok)."""
    dx = end.x - start.x
    dy = end.y - start.y
    D = math.hypot(dx, dy) / r

    if D < 1e-9:
        return None

    theta = math.atan2(dy, dx)
    alpha = _mod2pi(start.yaw - theta)
    beta = _mod2pi(end.yaw - theta)

    # LSL
    tmp = 2.0 + D * D - 2.0 * math.cos(alpha - beta) + 2.0 * D * (
        math.sin(alpha) - math.sin(beta))
    if tmp < 0:
        return None
    sa = math.asin((math.cos(beta) - math.cos(alpha)) / math.sqrt(max(tmp, 0)))
    # clamp
    sa = max(-math.pi / 2, min(math.pi / 2, sa))
    alpha_lsl = _mod2pi_pos(alpha - sa)
    beta_lsl = _mod2pi_pos(beta - sa)
    L = math.sqrt(max(tmp, 0)) * r
    return (L, alpha_lsl, beta_lsl)


def _dubins_RSR(start: Pose, end: Pose, r: float):
    """Compute RSR Dubins path."""
    dx = end.x - start.x
    dy = end.y - start.y
    D = math.hypot(dx, dy) / r

    if D < 1e-9:
        return None

    theta = math.atan2(dy, dx)
    alpha = _mod2pi(theta - start.yaw)
    beta = _mod2pi(theta - end.yaw)

    tmp = 2.0 + D * D - 2.0 * math.cos(alpha - beta) + 2.0 * D * (
        math.sin(beta) - math.sin(alpha))
    if tmp < 0:
        return None
    sa = math.asin((math.cos(alpha) - math.cos(beta)) / math.sqrt(max(tmp, 0)))
    sa = max(-math.pi / 2, min(math.pi / 2, sa))
    alpha_rsr = _mod2pi_pos(alpha - sa)
    beta_rsr = _mod2pi_pos(beta - sa)
    L = math.sqrt(max(tmp, 0)) * r
    return (L, alpha_rsr, beta_rsr)


def _dubins_LSR(start: Pose, end: Pose, r: float):
    """Compute LSR Dubins path."""
    dx = end.x - start.x
    dy = end.y - start.y
    D = math.hypot(dx, dy) / r

    if D < 1e-9:
        return None

    theta = math.atan2(dy, dx)
    alpha = _mod2pi(start.yaw - theta)
    beta = _mod2pi(end.yaw - theta)

    tmp = -2.0 + D * D + 2.0 * math.cos(alpha - beta) + 2.0 * D * (
        math.sin(alpha) + math.sin(beta))
    if tmp < 0:
        return None
    p = math.sqrt(max(tmp, 0))
    if p < 1e-9:
        return None
    sa = math.asin(2.0 / p) if p >= 2.0 else None
    if sa is None:
        sa = math.asin(min(1.0, max(-1.0,
            (math.cos(alpha) + math.cos(beta)) / p)))
    alpha_lsr = _mod2pi_pos(alpha - sa)
    beta_lsr = _mod2pi_pos(math.pi - beta - sa)
    L = p * r
    return (L, alpha_lsr, beta_lsr)


def _dubins_RSL(start: Pose, end: Pose, r: float):
    """Compute RSL Dubins path."""
    dx = end.x - start.x
    dy = end.y - start.y
    D = math.hypot(dx, dy) / r

    if D < 1e-9:
        return None

    theta = math.atan2(dy, dx)
    alpha = _mod2pi(theta - start.yaw)
    beta = _mod2pi(theta - end.yaw)

    tmp = -2.0 + D * D + 2.0 * math.cos(alpha - beta) - 2.0 * D * (
        math.sin(alpha) + math.sin(beta))
    if tmp < 0:
        return None
    p = math.sqrt(max(tmp, 0))
    if p < 1e-9:
        return None
    sa = math.asin(2.0 / p) if p >= 2.0 else None
    if sa is None:
        sa = math.asin(min(1.0, max(-1.0,
            (math.cos(alpha) + math.cos(beta)) / p)))
    alpha_rsl = _mod2pi_pos(alpha - sa)
    beta_rsl = _mod2pi_pos(math.pi - beta - sa)
    L = p * r
    return (L, alpha_rsl, beta_rsl)


def _dubins_LRL(start: Pose, end: Pose, r: float):
    """Compute LRL Dubins path."""
    dx = end.x - start.x
    dy = end.y - start.y
    D = math.hypot(dx, dy) / r

    if D < 1e-9 or D > 4.0:
        return None

    theta = math.atan2(dy, dx)
    alpha = _mod2pi(start.yaw - theta)
    beta = _mod2pi(end.yaw - theta)

    tmp = (6.0 - D * D + 2.0 * math.cos(alpha - beta)
           + 2.0 * D * (math.sin(alpha) + math.sin(beta))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = math.acos(tmp)
    sa = math.asin(min(1.0, max(-1.0,
        2.0 * math.sin(p) / D))) if D > 0 else 0.0
    alpha_lrl = _mod2pi_pos(alpha - sa + p)
    beta_lrl = _mod2pi_pos(alpha - beta - sa + p)
    L = r * (alpha_lrl + beta_lrl + 2.0 * p)
    return (L, alpha_lrl, beta_lrl)


def _dubins_RLR(start: Pose, end: Pose, r: float):
    """Compute RLR Dubins path."""
    dx = end.x - start.x
    dy = end.y - start.y
    D = math.hypot(dx, dy) / r

    if D < 1e-9 or D > 4.0:
        return None

    theta = math.atan2(dy, dx)
    alpha = _mod2pi(theta - start.yaw)
    beta = _mod2pi(theta - end.yaw)

    tmp = (6.0 - D * D + 2.0 * math.cos(alpha - beta)
           - 2.0 * D * (math.sin(alpha) + math.sin(beta))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = math.acos(tmp)
    sa = math.asin(min(1.0, max(-1.0,
        2.0 * math.sin(p) / D))) if D > 0 else 0.0
    alpha_rlr = _mod2pi_pos(alpha - sa - p)
    beta_rlr = _mod2pi_pos(alpha - beta - sa - p)
    L = r * (alpha_rlr + beta_rlr + 2.0 * p)
    return (L, alpha_rlr, beta_rlr)


DUBINS_SOLVERS = [
    ("LSL", _dubins_LSL),
    ("RSR", _dubins_RSR),
    ("LSR", _dubins_LSR),
    ("RSL", _dubins_RSL),
    ("LRL", _dubins_LRL),
    ("RLR", _dubins_RLR),
]


def _trace_arc(cx: float, cy: float, r: float, a0: float, a1: float,
               left: bool, step: float) -> List[Tuple[float, float]]:
    """Trace a circular arc from angle a0 to a1 (radians, CCW positive)."""
    da = _mod2pi(a1 - a0)
    if not left:
        da = -_mod2pi(-(a1 - a0))
    n = max(2, int(abs(da) * r / step))
    pts = []
    for i in range(n + 1):
        a = a0 + da * i / n
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _standard_dubins_candidates(
    start: Pose,
    end: Pose,
    radius: float,
) -> List[Tuple[str, Tuple[float, float, float]]]:
    """Return exact Dubins words in units of the turning radius.

    The previous scanner kept separately-derived turn angles, then recreated
    the second circle with the wrong preceding turn sign. Sampling the word
    directly from the standard normalized Dubins equations keeps collision
    checks aligned with the path length used for ranking.
    """
    dx = end.x - start.x
    dy = end.y - start.y
    distance = math.hypot(dx, dy)
    if distance <= 1.0e-9:
        return []
    d = distance / radius
    theta = math.atan2(dy, dx)
    alpha = _mod2pi_pos(start.yaw - theta)
    beta = _mod2pi_pos(end.yaw - theta)
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_alpha_beta = math.cos(alpha - beta)
    candidates: List[Tuple[str, Tuple[float, float, float]]] = []

    def append(word: str, first: float, middle: float, last: float) -> None:
        values = (first, middle, last)
        if not all(math.isfinite(value) for value in values):
            return
        if any(
            symbol == "S" and value < -1.0e-9
            for symbol, value in zip(word, values)
        ):
            return
        candidates.append((
            word,
            tuple(
                0.0 if abs(value) <= 1.0e-9 else _mod2pi_pos(value)
                if word[index] != "S" else max(0.0, value)
                for index, value in enumerate(values)
            ),
        ))

    # LSL
    squared = (
        2.0 + d * d - 2.0 * cos_alpha_beta
        + 2.0 * d * (sin_alpha - sin_beta)
    )
    if squared >= -1.0e-9:
        middle = math.sqrt(max(0.0, squared))
        temporary = math.atan2(
            cos_beta - cos_alpha, d + sin_alpha - sin_beta
        )
        append("LSL", -alpha + temporary, middle, beta - temporary)

    # RSR
    squared = (
        2.0 + d * d - 2.0 * cos_alpha_beta
        + 2.0 * d * (sin_beta - sin_alpha)
    )
    if squared >= -1.0e-9:
        middle = math.sqrt(max(0.0, squared))
        temporary = math.atan2(
            cos_alpha - cos_beta, d - sin_alpha + sin_beta
        )
        append("RSR", alpha - temporary, middle, -beta + temporary)

    # LSR
    squared = (
        -2.0 + d * d + 2.0 * cos_alpha_beta
        + 2.0 * d * (sin_alpha + sin_beta)
    )
    if squared >= -1.0e-9:
        middle = math.sqrt(max(0.0, squared))
        temporary = (
            math.atan2(-cos_alpha - cos_beta, d + sin_alpha + sin_beta)
            - math.atan2(-2.0, middle)
        )
        append("LSR", -alpha + temporary, middle, -beta + temporary)

    # RSL
    squared = (
        -2.0 + d * d + 2.0 * cos_alpha_beta
        - 2.0 * d * (sin_alpha + sin_beta)
    )
    if squared >= -1.0e-9:
        middle = math.sqrt(max(0.0, squared))
        temporary = (
            math.atan2(cos_alpha + cos_beta, d - sin_alpha - sin_beta)
            - math.atan2(2.0, middle)
        )
        append("RSL", alpha - temporary, middle, beta - temporary)

    # RLR
    temporary = (
        6.0 - d * d + 2.0 * cos_alpha_beta
        + 2.0 * d * (sin_alpha - sin_beta)
    ) / 8.0
    if -1.0 <= temporary <= 1.0:
        middle = _mod2pi_pos(2.0 * math.pi - math.acos(temporary))
        first = _mod2pi_pos(
            alpha
            - math.atan2(cos_alpha - cos_beta, d - sin_alpha + sin_beta)
            + middle / 2.0
        )
        append("RLR", first, middle, alpha - beta - first + middle)

    # LRL
    temporary = (
        6.0 - d * d + 2.0 * cos_alpha_beta
        + 2.0 * d * (-sin_alpha + sin_beta)
    ) / 8.0
    if -1.0 <= temporary <= 1.0:
        middle = _mod2pi_pos(2.0 * math.pi - math.acos(temporary))
        first = _mod2pi_pos(
            -alpha
            - math.atan2(cos_alpha - cos_beta, d + sin_alpha - sin_beta)
            + middle / 2.0
        )
        append("LRL", first, middle, beta - alpha - first + middle)

    return candidates


def _advance_dubins_pose(
    pose: Pose,
    symbol: str,
    distance: float,
    radius: float,
) -> Pose:
    """Advance one exact Dubins primitive by a nonnegative distance."""
    if symbol == "S":
        return Pose(
            pose.x + distance * math.cos(pose.yaw),
            pose.y + distance * math.sin(pose.yaw),
            pose.yaw,
        )
    curvature = 1.0 / radius if symbol == "L" else -1.0 / radius
    yaw = pose.yaw + curvature * distance
    return Pose(
        pose.x + (math.sin(yaw) - math.sin(pose.yaw)) / curvature,
        pose.y - (math.cos(yaw) - math.cos(pose.yaw)) / curvature,
        yaw,
    )


def _sample_dubins_word(
    start: Pose,
    word: str,
    normalized_lengths: Tuple[float, float, float],
    radius: float,
    step: float,
) -> Tuple[List[Tuple[float, float]], Pose]:
    """Sample an exact word without reconstructing circle centers manually."""
    current = start
    points = [(current.x, current.y)]
    for symbol, normalized_length in zip(word, normalized_lengths):
        distance = normalized_length * radius
        samples = max(1, int(math.ceil(distance / step)))
        primitive_start = current
        for index in range(1, samples + 1):
            current = _advance_dubins_pose(
                primitive_start, symbol, distance * index / samples, radius
            )
            points.append((current.x, current.y))
    return points, current


def compute_dubins(start: Pose, end: Pose, r: Optional[float] = None,
                   step: float = DUBINS_STEP) -> Optional[SegmentResult]:
    """Compute the shortest feasible Dubins path between two poses.

    Returns the shortest among all 6 Dubins types, with path discretisation.
    """
    if r is None:
        r = default_geometry_context().config.minimum_turning_radius_m
    if r <= 0.0:
        raise ValueError("turning radius must be positive")

    dist = start.dist_to(end)
    if dist < 1e-6:
        return None

    candidates = _standard_dubins_candidates(start, end, r)
    if not candidates:
        return None
    best = min(candidates, key=lambda item: sum(item[1]) * r)
    name, normalized_lengths = best
    points, endpoint = _sample_dubins_word(
        start, name, normalized_lengths, r, step
    )
    if (
        math.hypot(endpoint.x - end.x, endpoint.y - end.y) > 1.0e-5
        or abs(_mod2pi(endpoint.yaw - end.yaw)) > 1.0e-5
    ):
        return None
    path_length = sum(normalized_lengths) * r
    straight_length = (
        normalized_lengths[1] * r if name[1] == "S" else 0.0
    )
    total_turn_angle = sum(
        length
        for symbol, length in zip(name, normalized_lengths)
        if symbol != "S"
    )

    return SegmentResult(
        segment="",
        start_pose=start,
        end_pose=end,
        dubins_type=name,
        path_length=path_length,
        path_points=points,
        straight_length=straight_length,
        total_turn_angle=total_turn_angle,
        is_loop=False,       # filled in later
        wall_collisions=0,    # filled in later
        core_collisions=0,
        boundary_collisions=0,
        pgm_keepout_collisions=0,
        feasible=True,
    )


# ── collision checks ───────────────────────────────────────────────────

def _in_rect(px: float, py: float, rect) -> bool:
    (rx0, ry0), (rx1, ry1) = rect
    return rx0 <= px <= rx1 and ry0 <= py <= ry1


def _footprint_extents(pose: Pose, context: GeometryContext) -> Tuple[float, float]:
    """Return the world-axis extents of the padded runtime OBB."""
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return (
        context.half_length_m * abs(cosine) + context.half_width_m * abs(sine),
        context.half_length_m * abs(sine) + context.half_width_m * abs(cosine),
    )


def obb_intersects_keepout(
    pose: Pose,
    bounds: Bounds2D,
    context: Optional[GeometryContext] = None,
) -> bool:
    """Test the padded vehicle OBB against one effective PGM AABB.

    This is the same four-axis separating-axis test used by the waypoint
    editor preflight. ``bounds`` must be from ``keepout_mask_bounds`` rather
    than an idealized rule-diagram rectangle, so the scanner sees the final
    black PGM cells that Nav2's KeepoutFilter sees.
    """
    settings = context or default_geometry_context()
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    delta_x = bounds.center.x - pose.x
    delta_y = bounds.center.y - pose.y
    obstacle_half_x = bounds.width / 2.0
    obstacle_half_y = bounds.height / 2.0
    half_length = settings.half_length_m
    half_width = settings.half_width_m

    # AABB world X/Y axes.
    if abs(delta_x) > obstacle_half_x + half_length * abs(cosine) + half_width * abs(sine):
        return False
    if abs(delta_y) > obstacle_half_y + half_length * abs(sine) + half_width * abs(cosine):
        return False

    # Vehicle longitudinal/lateral axes.
    local_x = delta_x * cosine + delta_y * sine
    local_y = -delta_x * sine + delta_y * cosine
    if abs(local_x) > half_length + obstacle_half_x * abs(cosine) + obstacle_half_y * abs(sine):
        return False
    if abs(local_y) > half_width + obstacle_half_x * abs(sine) + obstacle_half_y * abs(cosine):
        return False
    return True


def _footprint_inside_field(pose: Pose, context: GeometryContext) -> bool:
    """Reject an OBB that projects outside the generated PGM extent."""
    extent_x, extent_y = _footprint_extents(pose, context)
    field = context.reference.field
    return (
        field.x_min <= pose.x - extent_x
        and pose.x + extent_x <= field.x_max
        and field.y_min <= pose.y - extent_y
        and pose.y + extent_y <= field.y_max
    )


def path_tangent_poses(
    points: Sequence[Tuple[float, float]],
    fallback_yaw: float = 0.0,
) -> List[Pose]:
    """Attach path-tangent yaws to discrete path points.

    At an arc sample the center-point heading is not enough: the OBB must be
    rotated along the actual travel tangent. Duplicate points are skipped when
    computing the finite difference, which also handles Smac-like angle-bin
    duplicates should they appear in a generated path.
    """
    poses: List[Pose] = []
    for index, (x, y) in enumerate(points):
        previous = None
        for candidate in range(index - 1, -1, -1):
            dx = x - points[candidate][0]
            dy = y - points[candidate][1]
            if dx * dx + dy * dy > 1.0e-12:
                previous = points[candidate]
                break
        following = None
        for candidate in range(index + 1, len(points)):
            dx = points[candidate][0] - x
            dy = points[candidate][1] - y
            if dx * dx + dy * dy > 1.0e-12:
                following = points[candidate]
                break

        if previous is not None and following is not None:
            yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
        elif following is not None:
            yaw = math.atan2(following[1] - y, following[0] - x)
        elif previous is not None:
            yaw = math.atan2(y - previous[1], x - previous[0])
        else:
            yaw = fallback_yaw
        poses.append(Pose(x, y, yaw))
    return poses


def check_path_collisions(
    points: Sequence[Tuple[float, float]],
    context: Optional[GeometryContext] = None,
) -> dict[str, int]:
    """Count padded-OBB collisions against every effective PGM keepout."""
    settings = context or default_geometry_context()
    counts = {name: 0 for name in settings.keepout_names}
    counts["field_boundary"] = 0
    for pose in path_tangent_poses(points):
        if not _footprint_inside_field(pose, settings):
            counts["field_boundary"] += 1
        for name, bounds in zip(settings.keepout_names, settings.keepouts):
            if obb_intersects_keepout(pose, bounds, settings):
                counts[name] += 1
    return counts


def check_wall_collision(
    points: Sequence[Tuple[float, float]],
    start_xy=None,
    end_xy=None,
    context: Optional[GeometryContext] = None,
) -> int:
    """Count B-wall collisions using tangent-aligned padded OBBs.

    ``start_xy`` and ``end_xy`` remain accepted for callers of the old helper,
    but are intentionally not exempted: a vehicle at a waypoint can still
    collide with a PGM keepout and must be rejected by the scanner.
    """
    del start_xy, end_xy
    counts = check_path_collisions(points, context)
    return counts["b_wall_west"] + counts["b_wall_east"]


def check_corridor_passable(
    points: Sequence[Tuple[float, float]],
    context: Optional[GeometryContext] = None,
) -> bool:
    """Require one tangent-aligned, B-wall-clear sample in the real corridor."""
    settings = context or default_geometry_context()
    corridor = settings.reference.corridor
    b_walls = settings.keepouts[:2]
    for pose in path_tangent_poses(points):
        if not (
            corridor.x_min <= pose.x <= corridor.x_max
            and corridor.y_min <= pose.y <= corridor.y_max
        ):
            continue
        if not any(obb_intersects_keepout(pose, wall, settings) for wall in b_walls):
            return True
    return False


# ── candidate generation ───────────────────────────────────────────────

def _yaw_from_deg(deg: float) -> float:
    return math.radians(deg)


def _deg_from_yaw(yaw: float) -> float:
    return math.degrees(yaw)


def generate_c3_candidates() -> List[Pose]:
    """Generate candidates for c_corner_3.

    Must stay in zone C (y > 2.5) and on the east side of the ring.
    Ring outer: x ∈ [0.5, 4.5], y ∈ [2.75, 4.4]
    Current: (3.175, 3.9), yaw -30°
    """
    candidates = []
    # Vary x: stay on east side but allow some adjustment
    for x in [2.9, 3.0, 3.1, 3.175, 3.25, 3.4]:
        # Vary y: stay near ring top edge
        for y in [3.75, 3.85, 3.9, 3.95, 4.05, 4.15]:
            if y <= 2.5:
                continue
            if not _in_rect(x, y, RING_OUTER):
                continue
            # Vary yaw: should guide toward c_corner_4
            for yaw_deg in [-60, -45, -30, -15, 0, 15, 30]:
                yaw = _yaw_from_deg(yaw_deg)
                candidates.append(Pose(x, y, yaw))
    return candidates


def generate_c4_candidates() -> List[Pose]:
    """Generate candidates for c_corner_4.

    This is the corner between zone C and B-zone corridor entry.
    Must stay in zone C (y > 2.5) but can approach the B boundary.
    Current: (3.175, 2.75), yaw 180°

    The car needs to turn from heading toward c4 to heading into the
    corridor gap between B-zone walls.
    """
    candidates = []
    for x in [2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.175, 3.25]:
        for y in [2.55, 2.6, 2.65, 2.7, 2.75, 2.8, 2.9]:
            if y <= 2.5:
                continue
            for yaw_deg in [-180, -165, -150, -135, -120, -90, 90, 120, 135,
                            150, 165, 180]:
                yaw = _yaw_from_deg(yaw_deg)
                candidates.append(Pose(x, y, yaw))
    return candidates


def generate_return_candidates() -> List[Pose]:
    """Generate candidates for b_corridor_return_enter.

    Must stay in corridor zone: x ∈ [2.0, 3.0], y ∈ [2.0, 2.5].
    Current: (2.132, 2.517), yaw -135°
    Note: current y=2.517 is slightly above corridor — needs to be ≤ 2.5.
    """
    candidates = []
    for x in [2.0, 2.1, 2.13, 2.2, 2.3, 2.4, 2.5]:
        for y in [2.05, 2.1, 2.15, 2.2, 2.25, 2.3, 2.35, 2.4, 2.45, 2.5]:
            if not (CORRIDOR_X_MIN <= x <= CORRIDOR_X_MAX):
                continue
            if not (CORRIDOR_Y_MIN <= y <= CORRIDOR_Y_MAX):
                continue
            for yaw_deg in [-180, -165, -150, -135, -120, -105, -90]:
                yaw = _yaw_from_deg(yaw_deg)
                candidates.append(Pose(x, y, yaw))
    return candidates


def generate_c2_candidates() -> List[Pose]:
    """Generate candidates for c_corner_2.

    Must stay in zone C. Current: (0.825, 3.9), yaw 38°
    """
    candidates = []
    for x in [0.7, 0.75, 0.8, 0.825, 0.85, 0.9, 0.95, 1.0]:
        for y in [3.7, 3.8, 3.85, 3.9, 3.95, 4.0, 4.1]:
            if y <= 2.5:
                continue
            for yaw_deg in [15, 25, 30, 38, 45, 50, 60]:
                yaw = _yaw_from_deg(yaw_deg)
                candidates.append(Pose(x, y, yaw))
    return candidates


WEST_B_CORNER = (1.50, 2.25)
DEFAULT_B_MIN_CORNER_CLEARANCE_M = 0.48


@dataclass
class BSegmentCandidate:
    """One collision-checked reverse route through the B opening."""

    gate: Pose
    enter: Pose
    start_to_gate: SegmentResult
    gate_to_enter: SegmentResult
    gate_heading_rank: int
    selected_gate_virtual_yaw: float
    selected_enter_virtual_yaw: float
    min_west_corner_clearance_m: float
    score: float

    @property
    def total_path_m(self) -> float:
        return self.start_to_gate.path_length + self.gate_to_enter.path_length


def generate_b_gate_positions() -> List[Pose]:
    """Return B-opening targets kept east and north of the west-wall corner."""
    return [
        Pose(x, y, 0.0)
        for x in (1.70, 1.80, 1.90, 2.00, 2.10)
        for y in (2.65, 2.75, 2.85, 2.95)
    ]


def generate_b_enter_positions() -> List[Pose]:
    """Return free targets in the lower-left C approach lane above B."""
    return [
        Pose(x, y, 0.0)
        for x in (0.90, 1.00, 1.10, 1.20)
        for y in (2.75, 2.85, 2.95)
    ]


def _free_goal_reference_yaw(
    start: Pose,
    target: Pose,
    next_target: Optional[Pose] = None,
) -> float:
    """Mirror the runtime free-goal bisector without prescribing a YAML yaw."""
    incoming_x = target.x - start.x
    incoming_y = target.y - start.y
    incoming_length = math.hypot(incoming_x, incoming_y)
    outgoing_x = 0.0
    outgoing_y = 0.0
    outgoing_length = 0.0
    if next_target is not None:
        outgoing_x = next_target.x - target.x
        outgoing_y = next_target.y - target.y
        outgoing_length = math.hypot(outgoing_x, outgoing_y)
    if incoming_length > 1.0e-9 and outgoing_length > 1.0e-9:
        bisector_x = incoming_x / incoming_length + outgoing_x / outgoing_length
        bisector_y = incoming_y / incoming_length + outgoing_y / outgoing_length
        if math.hypot(bisector_x, bisector_y) > 1.0e-9:
            return math.atan2(bisector_y, bisector_x)
    if outgoing_length > 1.0e-9:
        return math.atan2(outgoing_y, outgoing_x)
    if incoming_length > 1.0e-9:
        return math.atan2(incoming_y, incoming_x)
    return start.yaw


def free_goal_heading_candidates(
    start: Pose,
    target: Pose,
    next_target: Optional[Pose],
    heading_samples: int = 12,
    fallback_limit: int = 4,
) -> List[float]:
    """Generate the same reference and bounded fallback headings as Nav2."""
    if heading_samples < 2:
        raise ValueError("heading_samples must be at least 2")
    if fallback_limit < 0:
        raise ValueError("fallback_limit must not be negative")
    reference = _free_goal_reference_yaw(start, target, next_target)
    headings = [reference]
    step = 2.0 * math.pi / heading_samples
    proposed = [
        math.floor(reference / step) * step,
        math.ceil(reference / step) * step,
    ]
    if next_target is not None:
        proposed.append(math.atan2(
            next_target.y - target.y, next_target.x - target.x
        ))
    proposed.append(math.atan2(target.y - start.y, target.x - start.x))
    for heading in proposed:
        if len(headings) - 1 >= fallback_limit:
            break
        normalized = math.remainder(heading, 2.0 * math.pi)
        if any(
            abs(math.remainder(normalized - existing, 2.0 * math.pi)) <= 1.0e-9
            for existing in headings
        ):
            continue
        headings.append(normalized)
    return headings


def _west_corner_clearance(*segments: SegmentResult) -> float:
    """Return centerline clearance from the rasterized west-wall top corner."""
    return min(
        math.hypot(point_x - WEST_B_CORNER[0], point_y - WEST_B_CORNER[1])
        for segment in segments
        for point_x, point_y in segment.path_points
    )


def scan_b_segment_candidates(
    start_pose: Pose,
    gate_positions: Optional[Sequence[Pose]] = None,
    enter_positions: Optional[Sequence[Pose]] = None,
    *,
    heading_samples: int = 12,
    fallback_limit: int = 4,
    min_west_corner_clearance_m: float = DEFAULT_B_MIN_CORNER_CLEARANCE_M,
    collision_context: Optional[GeometryContext] = None,
) -> List[BSegmentCandidate]:
    """Find bounded reverse B-segment geometry with a protected west corner.

    ``start_pose`` is in Nav2's virtual-forward frame. The two returned point
    coordinates are still position-only semantic targets; selected headings
    are diagnostics used to verify that the runtime's bounded free-heading
    search has at least one collision-free chain.
    """
    if min_west_corner_clearance_m <= 0.0:
        raise ValueError("min_west_corner_clearance_m must be positive")
    context = collision_context or default_geometry_context()
    radius = context.config.minimum_turning_radius_m
    gate_options = tuple(gate_positions or generate_b_gate_positions())
    enter_options = tuple(enter_positions or generate_b_enter_positions())
    results: List[BSegmentCandidate] = []

    for gate_position in gate_options:
        gate_target = Pose(gate_position.x, gate_position.y, 0.0)
        for enter_position in enter_options:
            enter_target = Pose(enter_position.x, enter_position.y, 0.0)
            gate_headings = free_goal_heading_candidates(
                start_pose,
                gate_target,
                enter_target,
                heading_samples=heading_samples,
                fallback_limit=fallback_limit,
            )
            for gate_heading_rank, gate_yaw in enumerate(gate_headings):
                gate = Pose(gate_target.x, gate_target.y, gate_yaw)
                start_to_gate = _classify_segment(
                    compute_dubins(start_pose, gate, r=radius),
                    "a_task_observe->b_corridor_gate",
                    check_corridor=True,
                    context=context,
                )
                if (
                    start_to_gate is None
                    or not start_to_gate.feasible
                    or start_to_gate.is_loop
                ):
                    continue
                enter_yaw = _free_goal_reference_yaw(gate, enter_target)
                enter = Pose(enter_target.x, enter_target.y, enter_yaw)
                gate_to_enter = _classify_segment(
                    compute_dubins(gate, enter, r=radius),
                    "b_corridor_gate->b_corridor_enter",
                    context=context,
                )
                if (
                    gate_to_enter is None
                    or not gate_to_enter.feasible
                    or gate_to_enter.is_loop
                ):
                    continue
                clearance = _west_corner_clearance(start_to_gate, gate_to_enter)
                if clearance < min_west_corner_clearance_m:
                    continue
                total_path = start_to_gate.path_length + gate_to_enter.path_length
                total_turn = (
                    start_to_gate.total_turn_angle + gate_to_enter.total_turn_angle
                )
                results.append(BSegmentCandidate(
                    gate=gate,
                    enter=enter,
                    start_to_gate=start_to_gate,
                    gate_to_enter=gate_to_enter,
                    gate_heading_rank=gate_heading_rank,
                    selected_gate_virtual_yaw=gate_yaw,
                    selected_enter_virtual_yaw=enter_yaw,
                    min_west_corner_clearance_m=clearance,
                    score=total_path + 0.05 * total_turn - 0.75 * clearance,
                ))

    # No persisted ordinary-point yaw is allowed. Prefer the runtime's primary
    # heading before a fallback, then rank safety before shortness so a path
    # that merely grazes the top-left B corner cannot win.
    results.sort(key=lambda candidate: (
        candidate.gate_heading_rank,
        -candidate.min_west_corner_clearance_m,
        candidate.total_path_m,
        candidate.score,
    ))
    return results


def load_reverse_virtual_start_pose(
    waypoints_file: str | Path,
    waypoint_id: str = "a_task_observe",
) -> Pose:
    """Load one protected start pose and rotate it into reverse plan space."""
    source = Path(waypoints_file)
    root = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(root, dict) or not isinstance(root.get("waypoints"), list):
        raise ValueError("waypoints file has no waypoint list")
    waypoint = next(
        (item for item in root["waypoints"] if item.get("id") == waypoint_id),
        None,
    )
    if not isinstance(waypoint, dict):
        raise ValueError(f"waypoint {waypoint_id!r} was not found")
    pose = waypoint.get("pose")
    position = pose.get("position") if isinstance(pose, dict) else None
    orientation = pose.get("orientation") if isinstance(pose, dict) else None
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError(f"waypoint {waypoint_id!r} must have a pose orientation")
    try:
        x = float(position["x"])
        y = float(position["y"])
        qx = float(orientation["x"])
        qy = float(orientation["y"])
        qz = float(orientation["z"])
        qw = float(orientation["w"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"waypoint {waypoint_id!r} pose is invalid") from error
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return Pose(x, y, math.remainder(yaw + math.pi, 2.0 * math.pi))


def b_candidate_json(candidate: BSegmentCandidate) -> dict:
    """Serialize a B scan result in the format consumed by apply_candidate."""
    return {
        "score": round(candidate.score, 3),
        "total_path_m": round(candidate.total_path_m, 3),
        "min_west_corner_clearance_m": round(
            candidate.min_west_corner_clearance_m, 3
        ),
        "runtime_gate_heading_rank": candidate.gate_heading_rank,
        "b_corridor_gate": {
            "x": round(candidate.gate.x, 3),
            "y": round(candidate.gate.y, 3),
            "yaw_deg": round(_deg_from_yaw(candidate.selected_gate_virtual_yaw), 1),
        },
        "b_corridor_enter": {
            "x": round(candidate.enter.x, 3),
            "y": round(candidate.enter.y, 3),
            "yaw_deg": round(_deg_from_yaw(candidate.selected_enter_virtual_yaw), 1),
        },
        "segments": {
            "a_to_gate": {
                "type": candidate.start_to_gate.dubins_type,
                "length_m": round(candidate.start_to_gate.path_length, 3),
            },
            "gate_to_enter": {
                "type": candidate.gate_to_enter.dubins_type,
                "length_m": round(candidate.gate_to_enter.path_length, 3),
            },
        },
    }


def print_top_b(results: Sequence[BSegmentCandidate], count: int = 30) -> None:
    """Print concise diagnostics for B segment candidates."""
    print(f"\nTop {min(count, len(results))} B-segment candidates")
    for index, candidate in enumerate(results[:count], start=1):
        print(
            f"#{index} gate=({candidate.gate.x:.3f}, {candidate.gate.y:.3f}) "
            f"enter=({candidate.enter.x:.3f}, {candidate.enter.y:.3f}) "
            f"total={candidate.total_path_m:.3f}m "
            f"west-corner-clearance={candidate.min_west_corner_clearance_m:.3f}m "
            f"heading-rank={candidate.gate_heading_rank} "
            f"virtual-yaws=({_deg_from_yaw(candidate.selected_gate_virtual_yaw):.1f}, "
            f"{_deg_from_yaw(candidate.selected_enter_virtual_yaw):.1f}) deg"
        )


# ── main scan ───────────────────────────────────────────────────────────

@dataclass
class CandidateSet:
    c2: Pose
    c3: Pose
    c4: Pose
    ret: Pose
    c2_to_c3: Optional[SegmentResult] = None
    c3_to_c4: Optional[SegmentResult] = None
    c4_to_ret: Optional[SegmentResult] = None
    score: float = float("inf")

    def total_path(self) -> float:
        s = 0.0
        for seg in [self.c2_to_c3, self.c3_to_c4, self.c4_to_ret]:
            if seg:
                s += seg.path_length
        return s

    def is_viable(self) -> bool:
        for seg in [self.c2_to_c3, self.c3_to_c4, self.c4_to_ret]:
            if seg is None or not seg.feasible:
                return False
            if seg.is_loop:
                return False
            if seg.wall_collisions > 0:
                return False
            if seg.boundary_collisions > 0:
                return False
        return True


def _classify_segment(
    result: Optional[SegmentResult],
    name: str,
    check_corridor: bool = False,
    context: Optional[GeometryContext] = None,
) -> Optional[SegmentResult]:
    """Classify whether a path is a loop or suspicious, and set metadata.

    If check_corridor is True, also verifies the path crosses the B-zone
    corridor gap (used for segments that must pass through the B-zone).
    """
    if result is None:
        return None
    result.segment = name
    collisions = check_path_collisions(result.path_points, context)
    result.wall_collisions = (
        collisions["b_wall_west"] + collisions["b_wall_east"]
    )
    result.core_collisions = collisions["c_core"]
    result.pgm_keepout_collisions = sum(
        value for name, value in collisions.items() if name != "field_boundary"
    )
    result.boundary_collisions = (
        collisions["field_boundary"]
        + result.pgm_keepout_collisions
        - result.wall_collisions
        - result.core_collisions
    )
    result.is_loop = result.path_length > LOOP_THRESHOLD
    result.feasible = result.pgm_keepout_collisions == 0 and result.boundary_collisions == 0
    if check_corridor and not check_corridor_passable(result.path_points, context):
        result.feasible = False
    return result


def scan_candidates(
    c2_candidates: List[Pose],
    c3_candidates: List[Pose],
    c4_candidates: List[Pose],
    ret_candidates: List[Pose],
    c1_pose: Pose,
    max_candidates: int = 200,
    collision_context: Optional[GeometryContext] = None,
) -> List[CandidateSet]:
    """Staged scan: c3→c4 robust → c4→ret corridor → c2→c3 → c1→c2.

    This is more efficient than brute-force 4D product.
    """
    context = collision_context or default_geometry_context()
    turning_radius = context.config.minimum_turning_radius_m
    results: List[CandidateSet] = []

    # ── Stage 1: Find robust c3→c4 pairs ──
    print("Stage 1: Scanning c3→c4 with arrival sampling...")
    c3_c4_pairs = []
    for i, c3 in enumerate(c3_candidates):
        if i % 50 == 0:
            print(f"  c3 {i}/{len(c3_candidates)}")
        # Test robustness: sample arrivals around c3
        all_ok = True
        worst_len = 0.0
        worst_off = None
        for dx in [-0.25, 0, 0.25]:
            for dy in [-0.25, 0, 0.25]:
                for dyaw in [-0.50, 0, 0.50]:
                    if dx == 0 and dy == 0 and dyaw == 0:
                        continue
                    s = c3.offset(dx, dy, dyaw)
                    seg = compute_dubins(s, c4_candidates[0], r=turning_radius)  # placeholder
                    # Actually need to test each c4
        # This per-c3 sampling is independent of c4 — let me restructure

    # Better approach: pre-compute c3 arrival envelope, then test each c4
    print("Pre-computing c3 arrival envelope (26 perturbations × Dubins)...")
    # For each (c3, c4) pair, check nominal + worst-case from sampled arrivals
    c3_pool = c3_candidates[:40] if len(c3_candidates) > 40 else c3_candidates
    c4_pool = c4_candidates[:40] if len(c4_candidates) > 40 else c4_candidates

    for c3 in c3_pool:
        for c4 in c4_pool:
            # c3→c4 must be > 0.5m apart (meaningful segment)
            if c3.dist_to(c4) < 0.5:
                continue

            # Nominal path
            nominal = compute_dubins(c3, c4, r=turning_radius)
            if nominal is None:
                continue

            # Arrival robustness
            worst_len = nominal.path_length
            worst_loop = nominal.path_length > LOOP_THRESHOLD
            for dx in [-0.25, 0, 0.25]:
                for dy in [-0.25, 0, 0.25]:
                    for dyaw in [-0.50, 0, 0.50]:
                        if dx == 0 and dy == 0 and dyaw == 0:
                            continue
                        s = c3.offset(dx, dy, dyaw)
                        seg = compute_dubins(s, c4, r=turning_radius)
                        if seg is None:
                            worst_loop = True
                            break
                        if seg.path_length > worst_len:
                            worst_len = seg.path_length
                        if seg.path_length > LOOP_THRESHOLD:
                            worst_loop = True
                            break
                    if worst_loop:
                        break
                if worst_loop:
                    break
            if worst_loop:
                continue

            nominal = _classify_segment(nominal, "c3→c4", context=context)
            if not nominal.feasible:
                continue

            c3_c4_pairs.append((c3, c4, nominal, worst_len))

    c3_c4_pairs.sort(key=lambda x: x[3])  # sort by worst-case length
    print(f"  Found {len(c3_c4_pairs)} robust c3→c4 pairs (worst "
          f"{c3_c4_pairs[0][3]:.2f}m ~ {c3_c4_pairs[-1][3]:.2f}m)"
          if c3_c4_pairs else "  No robust c3→c4 pairs!")

    if not c3_c4_pairs:
        return []

    # ── Stage 2: For top c3→c4 pairs, find c4→ret that crosses corridor ──
    print("Stage 2: Matching c4→return with corridor crossing...")
    ret_pool = ret_candidates[:30] if len(ret_candidates) > 30 else ret_candidates
    top_c34 = c3_c4_pairs[:min(200, len(c3_c4_pairs))]
    c34_ret = []

    for c3, c4, c3c4_seg, worst_len in top_c34:
        best_ret = None
        best_ret_seg = None
        best_ret_len = float("inf")
        for ret in ret_pool:
            if c4.dist_to(ret) < 0.3:
                continue
            seg = compute_dubins(c4, ret, r=turning_radius)
            if seg is None:
                continue
            seg = _classify_segment(
                seg, "c4→return", check_corridor=True, context=context
            )
            if not seg.feasible:
                continue
            if seg.is_loop:
                continue
            if seg.path_length < best_ret_len:
                best_ret_len = seg.path_length
                best_ret = ret
                best_ret_seg = seg
        if best_ret is not None:
            c34_ret.append((c3, c4, c3c4_seg, best_ret, best_ret_seg, worst_len))

    c34_ret.sort(key=lambda x: x[5] + x[4].path_length)
    print(f"  Found {len(c34_ret)} c3→c4→ret chains")

    if not c34_ret:
        return []

    # ── Stage 3: For top chains, find c2→c3 ──
    print("Stage 3: Matching c2→c3...")
    c2_pool = c2_candidates[:30] if len(c2_candidates) > 30 else c2_candidates
    top_chains = c34_ret[:min(100, len(c34_ret))]

    for c3, c4, c3c4_seg, ret, c4ret_seg, worst_c3c4 in top_chains:
        best_c2 = None
        best_c2_seg = None
        best_c2_len = float("inf")
        for c2 in c2_pool:
            if c2.dist_to(c3) < 1.5:  # c2→c3 must be meaningful distance
                continue
            # Check c1→c2 first (quick filter)
            c1c2 = compute_dubins(c1_pose, c2, r=turning_radius)
            if c1c2 is None:
                continue
            c1c2 = _classify_segment(c1c2, "c1→c2", context=context)
            if not c1c2.feasible or c1c2.is_loop:
                continue

            seg = compute_dubins(c2, c3, r=turning_radius)
            if seg is None:
                continue
            seg = _classify_segment(seg, "c2→c3", context=context)
            if not seg.feasible or seg.is_loop:
                continue
            if seg.path_length < best_c2_len:
                best_c2_len = seg.path_length
                best_c2 = c2
                best_c2_seg = seg
                best_c1c2 = c1c2

        if best_c2 is not None:
            cs = CandidateSet(
                c2=best_c2, c3=c3, c4=c4, ret=ret,
                c2_to_c3=best_c2_seg,
                c3_to_c4=c3c4_seg,
                c4_to_ret=c4ret_seg,
            )
            cs.score = cs.total_path() + worst_c3c4 * 0.5
            results.append(cs)

    results.sort(key=lambda cs: cs.score)
    print(f"  Found {len(results)} complete candidate sets")
    return results


def _format_pose(p: Pose) -> str:
    return (f"({p.x:.3f}, {p.y:.3f}), "
            f"yaw {_deg_from_yaw(p.yaw):.0f}°")


def print_top(results: List[CandidateSet], n: int = 30):
    """Print the top N candidates with full diagnostic info."""
    print(f"\n{'─'*80}")
    print(f"Top {min(n, len(results))} candidates")
    print(f"{'─'*80}")

    for i, cs in enumerate(results[:n]):
        print(f"\n#{i+1}  score={cs.score:.2f}  total_path={cs.total_path():.2f}m")
        print(f"  c_corner_2:             {_format_pose(cs.c2)}")
        print(f"  c_corner_3:             {_format_pose(cs.c3)}")
        print(f"  c_corner_4:             {_format_pose(cs.c4)}")
        print(f"  b_corridor_return_enter:{_format_pose(cs.ret)}")
        for seg in [cs.c2_to_c3, cs.c3_to_c4, cs.c4_to_ret]:
            if seg:
                flag = ""
                if seg.is_loop:
                    flag = " ⚠LOOP"
                if seg.wall_collisions > 0:
                    flag += f" ⚠WALL({seg.wall_collisions})"
                if seg.core_collisions > 0:
                    flag += f" ⚠C-CORE({seg.core_collisions})"
                if seg.boundary_collisions > 0:
                    flag += f" ⚠PGM/BOUNDARY({seg.boundary_collisions})"
                print(f"  {seg.segment:8s} {seg.dubins_type:4s} "
                      f"length={seg.path_length:.2f}m "
                      f"L={seg.straight_length:.2f}m "
                      f"turn={math.degrees(seg.total_turn_angle):.0f}°{flag}")


def main():
    parser = argparse.ArgumentParser(description="Geometry scan for waypoint optimization")
    parser.add_argument(
        "--mode", choices=("c", "b"), default="c",
        help="Scan C return geometry (c) or the reverse B opening (b)",
    )
    parser.add_argument("--top", type=int, default=30,
                        help="Number of top candidates to print")
    parser.add_argument("--json", type=str, default=None,
                        help="Save top candidates as JSON")
    parser.add_argument("--c1-pose", type=str, default="0.387,2.682,60",
                        help="c_corner_1 pose: x,y,yaw_deg")
    parser.add_argument(
        "--waypoints", type=Path, default=DEFAULT_NAV_ONLY_WAYPOINTS,
        help="waypoint YAML used to load the protected B-segment start pose",
    )
    parser.add_argument(
        "--b-start-id", default="a_task_observe",
        help="locked waypoint used as the reverse B-segment start",
    )
    parser.add_argument(
        "--b-heading-samples", type=int, default=12,
        help="runtime free-heading bin count used by the B scan",
    )
    parser.add_argument(
        "--b-min-west-corner-clearance-m", type=float,
        default=DEFAULT_B_MIN_CORNER_CLEARANCE_M,
        help="minimum centerline clearance from the west B-wall top corner",
    )
    parser.add_argument(
        "--route-planning-config", type=Path,
        default=DEFAULT_ROUTE_PLANNING_CONFIG,
        help="shared route_planning.yaml used by Nav2 keepout generation",
    )
    parser.add_argument(
        "--geometry", type=Path, default=DEFAULT_GEOMETRY,
        help="shared field_geometry.yaml used by the simulation PGM",
    )
    args = parser.parse_args()

    try:
        context = load_geometry_context(args.route_planning_config, args.geometry)
    except (OSError, ValueError) as error:
        print(f"cannot load shared scan geometry: {error}", file=sys.stderr)
        return 2

    # Parse c1 (fixed, not optimized but used for c1→c2 check)
    parts = args.c1_pose.split(",")
    c1_pose = Pose(float(parts[0]), float(parts[1]),
                   _yaw_from_deg(float(parts[2])))

    print(
        "Generating candidates with padded OBB "
        f"{context.half_length_m * 2.0:.2f}x{context.half_width_m * 2.0:.2f} m "
        "against effective PGM keepouts..."
    )
    if args.mode == "b":
        try:
            start_pose = load_reverse_virtual_start_pose(
                args.waypoints, args.b_start_id
            )
            results = scan_b_segment_candidates(
                start_pose,
                heading_samples=args.b_heading_samples,
                min_west_corner_clearance_m=args.b_min_west_corner_clearance_m,
                collision_context=context,
            )
        except (OSError, ValueError) as error:
            print(f"cannot scan B geometry: {error}", file=sys.stderr)
            return 2
        if not results:
            print("No collision-free B-segment candidates found.")
            return 1
        print_top_b(results, args.top)
        if args.json:
            top_data = [b_candidate_json(candidate) for candidate in results[:args.top]]
            Path(args.json).write_text(
                json.dumps(top_data, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
            print(f"Saved top {len(top_data)} B candidates to {args.json}")
        return 0

    c2_cands = generate_c2_candidates()
    c3_cands = generate_c3_candidates()
    c4_cands = generate_c4_candidates()
    ret_cands = generate_return_candidates()

    print(f"  c_corner_2: {len(c2_cands)} candidates")
    print(f"  c_corner_3: {len(c3_cands)} candidates")
    print(f"  c_corner_4: {len(c4_cands)} candidates")
    print(f"  return_enter: {len(ret_cands)} candidates")

    results = scan_candidates(
        c2_cands, c3_cands, c4_cands, ret_cands, c1_pose,
        collision_context=context,
    )

    if not results:
        print("\nNo viable candidates found!")
        return 1

    print_top(results, args.top)

    if args.json:
        top_data = []
        for cs in results[:args.top]:
            top_data.append({
                "score": round(cs.score, 2),
                "total_path_m": round(cs.total_path(), 2),
                "c_corner_2": {
                    "x": round(cs.c2.x, 3), "y": round(cs.c2.y, 3),
                    "yaw_deg": round(_deg_from_yaw(cs.c2.yaw), 1),
                },
                "c_corner_3": {
                    "x": round(cs.c3.x, 3), "y": round(cs.c3.y, 3),
                    "yaw_deg": round(_deg_from_yaw(cs.c3.yaw), 1),
                },
                "c_corner_4": {
                    "x": round(cs.c4.x, 3), "y": round(cs.c4.y, 3),
                    "yaw_deg": round(_deg_from_yaw(cs.c4.yaw), 1),
                },
                "b_corridor_return_enter": {
                    "x": round(cs.ret.x, 3), "y": round(cs.ret.y, 3),
                    "yaw_deg": round(_deg_from_yaw(cs.ret.yaw), 1),
                },
                "segments": {
                    "c2_to_c3": {
                        "type": cs.c2_to_c3.dubins_type if cs.c2_to_c3 else None,
                        "length_m": round(cs.c2_to_c3.path_length, 2) if cs.c2_to_c3 else None,
                        "turn_deg": round(math.degrees(cs.c2_to_c3.total_turn_angle), 0) if cs.c2_to_c3 else None,
                    },
                    "c3_to_c4": {
                        "type": cs.c3_to_c4.dubins_type if cs.c3_to_c4 else None,
                        "length_m": round(cs.c3_to_c4.path_length, 2) if cs.c3_to_c4 else None,
                        "turn_deg": round(math.degrees(cs.c3_to_c4.total_turn_angle), 0) if cs.c3_to_c4 else None,
                    },
                    "c4_to_return": {
                        "type": cs.c4_to_ret.dubins_type if cs.c4_to_ret else None,
                        "length_m": round(cs.c4_to_ret.path_length, 2) if cs.c4_to_ret else None,
                        "turn_deg": round(math.degrees(cs.c4_to_ret.total_turn_angle), 0) if cs.c4_to_ret else None,
                    },
                },
            })
        Path(args.json).write_text(
            json.dumps(top_data, indent=2, ensure_ascii=False),
            encoding="utf-8")
        print(f"\nSaved top {len(top_data)} to {args.json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
