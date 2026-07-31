#!/usr/bin/env python3
"""Geometry scanner for c_corner waypoint optimization.

Enumerates candidate positions and yaws for c_corner_2/3/4 and
b_corridor_return_enter, computes Dubins paths for the three critical
segments, checks wall collisions, and ranks candidates by path quality.

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

# ── constants ──────────────────────────────────────────────────────────
MIN_TURNING_RADIUS = 0.55          # minimum turning radius (m)
FOOTPRINT_RADIUS = 0.40            # conservative circular footprint (m)
INFLATION_MARGIN = 0.05            # geometric margin for collision check (m)
TOTAL_CLEARANCE = FOOTPRINT_RADIUS + INFLATION_MARGIN  # 0.45 m
WAYPOINT_EXCLUSION_RADIUS = 0.30   # exclude waypoint vicinity from collision
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
    wall_collisions: int
    boundary_collisions: int
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


def compute_dubins(start: Pose, end: Pose, r: float = MIN_TURNING_RADIUS,
                   step: float = DUBINS_STEP) -> Optional[SegmentResult]:
    """Compute the shortest feasible Dubins path between two poses.

    Returns the shortest among all 6 Dubins types, with path discretisation."
    """
    dist = start.dist_to(end)
    if dist < 1e-6:
        return None

    best = None
    best_len = float("inf")

    for name, solver in DUBINS_SOLVERS:
        sol = solver(start, end, r)
        if sol is None:
            continue
        L, alpha, beta = sol
        total = r * (alpha + beta) + L
        if total < best_len:
            best_len = total
            best = (name, L, alpha, beta)

    if best is None:
        return None

    name, L, alpha, beta = best
    # Discretise the path
    points: List[Tuple[float, float]] = []
    pts = [(start.x, start.y)]

    for turn_idx, (turn_angle, is_left) in enumerate([
        (alpha, name[0] == "L"),
        (beta, name[2] == "L"),
    ]):
        if turn_angle < 0.001:
            continue
        # centre of turning circle
        sign = 1.0 if is_left else -1.0
        t0 = start.yaw if turn_idx == 0 else None  # computed after straight
        if turn_idx == 0:
            cx = start.x + sign * r * math.cos(start.yaw - sign * math.pi / 2)
            cy = start.y + sign * r * math.sin(start.yaw - sign * math.pi / 2)
            a0 = start.yaw - sign * math.pi / 2
        else:
            # After straight segment
            mid_yaw = _mod2pi(start.yaw - sign * alpha if name[0] == "R"
                              else start.yaw + alpha)
            mid_x = (start.x - sign * r * math.cos(start.yaw - sign * math.pi / 2)
                     + sign * r * math.cos(mid_yaw - sign * math.pi / 2))
            mid_y = (start.y - sign * r * math.sin(start.yaw - sign * math.pi / 2)
                     + sign * r * math.sin(mid_yaw - sign * math.pi / 2))
            # after straight of length L
            if L > 0.001:
                straight_dir = mid_yaw
                mid_x += L * math.cos(straight_dir)
                mid_y += L * math.sin(straight_dir)
                # Add straight segment points
                n_s = max(2, int(L / step))
                for i in range(1, n_s + 1):
                    t = i / n_s
                    pts.append((pts[-1][0] * (1 - t) + mid_x * t,
                                pts[-1][1] * (1 - t) + mid_y * t))
            mid_yaw_straight = mid_yaw
            next_sign = 1.0 if name[2] == "L" else -1.0
            cx = mid_x + next_sign * r * math.cos(
                mid_yaw_straight - next_sign * math.pi / 2)
            cy = mid_y + next_sign * r * math.sin(
                mid_yaw_straight - next_sign * math.pi / 2)
            a0 = mid_yaw_straight - next_sign * math.pi / 2

        a1 = a0 + (turn_angle if is_left else -turn_angle)
        arc_pts = _trace_arc(cx, cy, r, a0, a1, is_left, step)
        if turn_idx == 0 and L > 0.001:
            pts.extend(arc_pts)
        elif turn_idx == 0:
            pts.extend(arc_pts)
        elif L > 0.001:
            pts.extend(arc_pts)
        else:
            pts.extend(arc_pts[1:])

    points = pts

    return SegmentResult(
        segment="",
        start_pose=start,
        end_pose=end,
        dubins_type=name,
        path_length=best_len,
        path_points=points,
        straight_length=L,
        total_turn_angle=alpha + beta,
        is_loop=False,       # filled in later
        wall_collisions=0,    # filled in later
        boundary_collisions=0,
        feasible=True,
    )


# ── collision checks ───────────────────────────────────────────────────

def _rect_collision(px: float, py: float, rect) -> bool:
    """Check if point is inside or too close to a rectangular obstacle."""
    (rx0, ry0), (rx1, ry1) = rect
    # Expand by clearance
    mx0, my0 = rx0 - TOTAL_CLEARANCE, ry0 - TOTAL_CLEARANCE
    mx1, my1 = rx1 + TOTAL_CLEARANCE, ry1 + TOTAL_CLEARANCE
    return mx0 <= px <= mx1 and my0 <= py <= my1


def _in_rect(px: float, py: float, rect) -> bool:
    (rx0, ry0), (rx1, ry1) = rect
    return rx0 <= px <= rx1 and ry0 <= py <= ry1


def check_wall_collision(points: List[Tuple[float, float]],
                         start_xy=None, end_xy=None) -> int:
    """Count points that are INSIDE physical wall rectangles (no clearance).

    Points within WAYPOINT_EXCLUSION_RADIUS of start/end are skipped.
    Boundary violations are not checked — Nav2 handles field boundaries.
    """
    hits = 0
    excl2 = WAYPOINT_EXCLUSION_RADIUS ** 2
    for x, y in points:
        if start_xy is not None:
            dx = x - start_xy[0]; dy = y - start_xy[1]
            if dx*dx + dy*dy < excl2:
                continue
        if end_xy is not None:
            dx = x - end_xy[0]; dy = y - end_xy[1]
            if dx*dx + dy*dy < excl2:
                continue
        # Check physical wall interior (no expansion)
        for (rx0, ry0), (rx1, ry1) in [WEST_WALL, EAST_WALL]:
            if rx0 <= x <= rx1 and ry0 <= y <= ry1:
                hits += 1
                break
    return hits


def check_corridor_passable(points: List[Tuple[float, float]]) -> bool:
    """Check if the path crosses the B-zone corridor gap feasibly.

    The corridor gap is between the two walls: x ∈ [1.5, 2.5], y ∈ [2.0, 2.5].
    With footprint radius 0.40m, the minimum passable width is 0.20m.
    We check that at least one path point passes through the gap center region.
    """
    gap_x_min = 1.5 + FOOTPRINT_RADIUS
    gap_x_max = 2.5 - FOOTPRINT_RADIUS
    gap_y_min = 2.0
    gap_y_max = 2.5

    for x, y in points:
        if (gap_x_min <= x <= gap_x_max and gap_y_min <= y <= gap_y_max):
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


def _classify_segment(result: Optional[SegmentResult], name: str,
                      check_corridor: bool = False) -> Optional[SegmentResult]:
    """Classify whether a path is a loop or suspicious, and set metadata.

    If check_corridor is True, also verifies the path crosses the B-zone
    corridor gap (used for segments that must pass through the B-zone).
    """
    if result is None:
        return None
    result.segment = name
    sx, sy = result.start_pose.x, result.start_pose.y
    ex, ey = result.end_pose.x, result.end_pose.y
    result.wall_collisions = check_wall_collision(
        result.path_points, start_xy=(sx, sy), end_xy=(ex, ey))
    result.boundary_collisions = 0
    result.is_loop = result.path_length > LOOP_THRESHOLD
    result.feasible = True
    if result.wall_collisions > 0:
        result.feasible = False
    if check_corridor and not check_corridor_passable(result.path_points):
        result.feasible = False
    return result


def scan_candidates(
    c2_candidates: List[Pose],
    c3_candidates: List[Pose],
    c4_candidates: List[Pose],
    ret_candidates: List[Pose],
    c1_pose: Pose,
    max_candidates: int = 200,
) -> List[CandidateSet]:
    """Staged scan: c3→c4 robust → c4→ret corridor → c2→c3 → c1→c2.

    This is more efficient than brute-force 4D product.
    """
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
                    seg = compute_dubins(s, c4_candidates[0])  # placeholder
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
            nominal = compute_dubins(c3, c4)
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
                        seg = compute_dubins(s, c4)
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

            nominal = _classify_segment(nominal, "c3→c4")
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
            seg = compute_dubins(c4, ret)
            if seg is None:
                continue
            seg = _classify_segment(seg, "c4→return", check_corridor=True)
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
            c1c2 = compute_dubins(c1_pose, c2)
            if c1c2 is None:
                continue
            c1c2 = _classify_segment(c1c2, "c1→c2")
            if not c1c2.feasible or c1c2.is_loop:
                continue

            seg = compute_dubins(c2, c3)
            if seg is None:
                continue
            seg = _classify_segment(seg, "c2→c3")
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
                if seg.boundary_collisions > 0:
                    flag += f" ⚠BOUNDARY({seg.boundary_collisions})"
                print(f"  {seg.segment:8s} {seg.dubins_type:4s} "
                      f"length={seg.path_length:.2f}m "
                      f"L={seg.straight_length:.2f}m "
                      f"turn={math.degrees(seg.total_turn_angle):.0f}°{flag}")


def main():
    parser = argparse.ArgumentParser(description="Geometry scan for waypoint optimization")
    parser.add_argument("--top", type=int, default=30,
                        help="Number of top candidates to print")
    parser.add_argument("--json", type=str, default=None,
                        help="Save top candidates as JSON")
    parser.add_argument("--c1-pose", type=str, default="0.387,2.682,60",
                        help="c_corner_1 pose: x,y,yaw_deg")
    args = parser.parse_args()

    # Parse c1 (fixed, not optimized but used for c1→c2 check)
    parts = args.c1_pose.split(",")
    c1_pose = Pose(float(parts[0]), float(parts[1]),
                   _yaw_from_deg(float(parts[2])))

    print("Generating candidates...")
    c2_cands = generate_c2_candidates()
    c3_cands = generate_c3_candidates()
    c4_cands = generate_c4_candidates()
    ret_cands = generate_return_candidates()

    print(f"  c_corner_2: {len(c2_cands)} candidates")
    print(f"  c_corner_3: {len(c3_cands)} candidates")
    print(f"  c_corner_4: {len(c4_cands)} candidates")
    print(f"  return_enter: {len(ret_cands)} candidates")

    results = scan_candidates(
        c2_cands, c3_cands, c4_cands, ret_cands, c1_pose)

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
