"""Offline geometric preflight for the waypoint editor.

This is a small heading-aware lattice planner.  It runs entirely in the
editor process, honors the same B-wall/C-core keepouts as the simulation mask,
uses the vehicle's minimum turning radius, and treats every pass-through point
as an ordered waypoint constraint.  It intentionally does not invoke Nav2:
there is no live costmap, current robot pose, controller state, or guarantee
that the generated geometric candidate matches Nav2's ``/plan``.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Any, Iterable, Sequence

from smartcar_task.waypoints import is_zero_quaternion

from smartcar_tools.field_keepouts import keepout_bounds
from smartcar_tools.field_reference import Bounds2D, FieldReference, Point2D
from smartcar_tools.planning_segments import PlanningSegment
from smartcar_tools.route_planning import (
    RoutePlanningConfig,
    load_route_planning_config,
)


_DUBINS_EPSILON = 1.0e-8


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class _PoseConstraint:
    """A route waypoint whose heading may be intentionally unconstrained."""

    x: float
    y: float
    yaw: float | None


@dataclass(frozen=True)
class _ThroughPlan:
    """One continuous lattice result split at ordered through-pose targets."""

    legs: tuple[tuple[Point2D, ...], ...]
    expanded_states: tuple[int, ...]
    completed_goals: int


@dataclass(frozen=True)
class LegPreflight:
    start_id: str
    end_id: str
    points: tuple[Point2D, ...]
    length_m: float
    expanded_states: int
    feasible: bool
    message: str = ""


@dataclass(frozen=True)
class SegmentPreflight:
    segment_id: str
    direction: str
    legs: tuple[LegPreflight, ...]
    feasible: bool
    length_m: float
    message: str = ""

    @property
    def points(self) -> tuple[Point2D, ...]:
        result: list[Point2D] = []
        for leg in self.legs:
            if result and leg.points:
                result.extend(leg.points[1:])
            else:
                result.extend(leg.points)
        return tuple(result)


@dataclass(frozen=True)
class RoutePreflight:
    segments: tuple[SegmentPreflight, ...]
    warnings: tuple[str, ...]

    @property
    def feasible(self) -> bool:
        return bool(self.segments) and all(segment.feasible for segment in self.segments)


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _angle_distance(first: float, second: float) -> float:
    return abs(_normalize_angle(second - first))


def _yaw_from_waypoint(waypoint: Any) -> float:
    qx, qy, qz, qw = waypoint.orientation
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _point(waypoint: Any) -> Point2D:
    return Point2D(float(waypoint.position[0]), float(waypoint.position[1]))


def _distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def _polyline_length(points: Sequence[Point2D]) -> float:
    return sum(_distance(first, second) for first, second in zip(points, points[1:]))


class LatticePreflightPlanner:
    """A deterministic forward-only planner in a virtual vehicle frame."""

    def __init__(
        self,
        reference: FieldReference,
        config: RoutePlanningConfig | None = None,
    ):
        self._reference = reference
        self._field = reference.field
        self._config = config or load_route_planning_config()
        self._preflight = self._config.preflight
        self._keepouts = keepout_bounds(reference, self._config)

    @property
    def config(self) -> RoutePlanningConfig:
        """Return the exact shared configuration used for this preflight."""
        return self._config

    def _inside_rect(self, x: float, y: float, bounds: Bounds2D) -> bool:
        radius = self._config.footprint_envelope_radius_m
        return (
            bounds.x_min - radius <= x <= bounds.x_max + radius
            and bounds.y_min - radius <= y <= bounds.y_max + radius
        )

    def _is_free(self, x: float, y: float) -> bool:
        if not (
            self._field.x_min <= x <= self._field.x_max
            and self._field.y_min <= y <= self._field.y_max
        ):
            return False
        return not any(self._inside_rect(x, y, obstacle) for obstacle in self._keepouts)

    def _state_key(self, pose: Pose2D) -> tuple[int, int, int]:
        resolution = self._preflight.grid_resolution_m
        heading_bins = self._preflight.heading_bins
        x = int(round((pose.x - self._field.x_min) / resolution))
        y = int(round((pose.y - self._field.y_min) / resolution))
        heading = int(round(
            (pose.yaw % (2.0 * math.pi)) / (2.0 * math.pi) * heading_bins
        ))
        return x, y, heading % heading_bins

    @staticmethod
    def _advance(pose: Pose2D, curvature: float, distance: float) -> Pose2D:
        if abs(curvature) <= 1.0e-9:
            return Pose2D(
                pose.x + distance * math.cos(pose.yaw),
                pose.y + distance * math.sin(pose.yaw),
                pose.yaw,
            )
        next_yaw = pose.yaw + curvature * distance
        return Pose2D(
            pose.x + (math.sin(next_yaw) - math.sin(pose.yaw)) / curvature,
            pose.y - (math.cos(next_yaw) - math.cos(pose.yaw)) / curvature,
            _normalize_angle(next_yaw),
        )

    def _primitive_is_free(self, pose: Pose2D, curvature: float) -> tuple[bool, Pose2D]:
        samples = self._preflight.primitive_samples
        next_pose = pose
        for sample in range(1, samples + 1):
            next_pose = self._advance(
                pose,
                curvature,
                self._preflight.step_length_m * sample / samples,
            )
            if not self._is_free(next_pose.x, next_pose.y):
                return False, next_pose
        return True, next_pose

    @staticmethod
    def _heuristic(pose: Pose2D, goal: Pose2D) -> float:
        return math.hypot(goal.x - pose.x, goal.y - pose.y)

    def _is_goal(self, pose: Pose2D, goal: Pose2D) -> bool:
        return (
            self._heuristic(pose, goal)
            <= self._preflight.goal_position_tolerance_m
            and _angle_distance(pose.yaw, goal.yaw)
            <= math.radians(self._preflight.goal_heading_tolerance_deg)
        )

    @staticmethod
    def _mod2pi_positive(value: float) -> float:
        result = math.fmod(value, 2.0 * math.pi)
        return result + 2.0 * math.pi if result < 0.0 else result

    def _advance_dubins(
        self,
        start: Pose2D,
        word: str,
        normalized_lengths: tuple[float, float, float],
    ) -> Pose2D:
        """Advance a Dubins word expressed in units of the turning radius."""
        radius = self._config.minimum_turning_radius_m
        pose = start
        for symbol, normalized_length in zip(word, normalized_lengths):
            if normalized_length <= _DUBINS_EPSILON:
                continue
            curvature = {
                "L": 1.0 / radius,
                "R": -1.0 / radius,
                "S": 0.0,
            }[symbol]
            pose = self._advance(pose, curvature, normalized_length * radius)
        return pose

    def _dubins_candidates(
        self,
        start: Pose2D,
        goal: Pose2D,
    ) -> tuple[tuple[str, tuple[float, float, float]], ...]:
        """Return exact forward-only Dubins words that reach ``goal``.

        The lattice offers coarse global search.  These analytic expansions
        replace the former last-step straight-line patch, so the displayed
        route stays tangent-continuous all the way to its endpoint.
        """
        radius = self._config.minimum_turning_radius_m
        distance = self._heuristic(start, goal)
        if distance <= _DUBINS_EPSILON:
            return ()

        d = distance / radius
        theta = math.atan2(goal.y - start.y, goal.x - start.x)
        alpha = self._mod2pi_positive(start.yaw - theta)
        beta = self._mod2pi_positive(goal.yaw - theta)
        candidates: list[tuple[str, tuple[float, float, float]]] = []

        def append_candidate(
            word: str,
            first: float,
            straight_or_middle: float,
            last: float,
        ) -> None:
            values = (first, straight_or_middle, last)
            if not all(math.isfinite(value) and value >= -_DUBINS_EPSILON for value in values):
                return
            normalized = tuple(
                0.0 if abs(value) <= _DUBINS_EPSILON else value
                for value in values
            )
            endpoint = self._advance_dubins(start, word, normalized)
            if (
                self._heuristic(endpoint, goal) <= 1.0e-6
                and _angle_distance(endpoint.yaw, goal.yaw) <= 1.0e-6
            ):
                candidates.append((word, normalized))

        def root(value: float) -> float | None:
            if value < -_DUBINS_EPSILON:
                return None
            return math.sqrt(max(0.0, value))

        # LSL
        p = root(
            2.0 + d * d - 2.0 * math.cos(alpha - beta)
            + 2.0 * d * (math.sin(alpha) - math.sin(beta))
        )
        if p is not None:
            temporary = math.atan2(
                math.cos(beta) - math.cos(alpha),
                d + math.sin(alpha) - math.sin(beta),
            )
            append_candidate(
                "LSL",
                self._mod2pi_positive(temporary - alpha),
                p,
                self._mod2pi_positive(beta - temporary),
            )

        # RSR
        p = root(
            2.0 + d * d - 2.0 * math.cos(alpha - beta)
            + 2.0 * d * (math.sin(beta) - math.sin(alpha))
        )
        if p is not None:
            temporary = math.atan2(
                math.cos(alpha) - math.cos(beta),
                d - math.sin(alpha) + math.sin(beta),
            )
            append_candidate(
                "RSR",
                self._mod2pi_positive(alpha - temporary),
                p,
                self._mod2pi_positive(temporary - beta),
            )

        # LSR
        p = root(
            -2.0 + d * d + 2.0 * math.cos(alpha - beta)
            + 2.0 * d * (math.sin(alpha) + math.sin(beta))
        )
        if p is not None:
            temporary = (
                math.atan2(
                    -math.cos(alpha) - math.cos(beta),
                    d + math.sin(alpha) + math.sin(beta),
                )
                - math.atan2(-2.0, p)
            )
            append_candidate(
                "LSR",
                self._mod2pi_positive(temporary - alpha),
                p,
                self._mod2pi_positive(temporary - beta),
            )

        # RSL
        p = root(
            d * d - 2.0 + 2.0 * math.cos(alpha - beta)
            - 2.0 * d * (math.sin(alpha) + math.sin(beta))
        )
        if p is not None:
            temporary = (
                math.atan2(
                    math.cos(alpha) + math.cos(beta),
                    d - math.sin(alpha) - math.sin(beta),
                )
                - math.atan2(2.0, p)
            )
            append_candidate(
                "RSL",
                self._mod2pi_positive(alpha - temporary),
                p,
                self._mod2pi_positive(beta - temporary),
            )

        # RLR
        temporary = (
            6.0 - d * d + 2.0 * math.cos(alpha - beta)
            + 2.0 * d * (math.sin(alpha) - math.sin(beta))
        ) / 8.0
        if -1.0 - _DUBINS_EPSILON <= temporary <= 1.0 + _DUBINS_EPSILON:
            middle = self._mod2pi_positive(
                2.0 * math.pi - math.acos(max(-1.0, min(1.0, temporary)))
            )
            first = self._mod2pi_positive(
                alpha
                - math.atan2(
                    math.cos(alpha) - math.cos(beta),
                    d - math.sin(alpha) + math.sin(beta),
                )
                + middle / 2.0
            )
            append_candidate(
                "RLR",
                first,
                middle,
                self._mod2pi_positive(alpha - beta - first + middle),
            )

        # LRL
        temporary = (
            6.0 - d * d + 2.0 * math.cos(alpha - beta)
            + 2.0 * d * (-math.sin(alpha) + math.sin(beta))
        ) / 8.0
        if -1.0 - _DUBINS_EPSILON <= temporary <= 1.0 + _DUBINS_EPSILON:
            middle = self._mod2pi_positive(
                2.0 * math.pi - math.acos(max(-1.0, min(1.0, temporary)))
            )
            first = self._mod2pi_positive(
                -alpha
                - math.atan2(
                    math.cos(alpha) - math.cos(beta),
                    d + math.sin(alpha) - math.sin(beta),
                )
                + middle / 2.0
            )
            append_candidate(
                "LRL",
                first,
                middle,
                self._mod2pi_positive(beta - alpha - first + middle),
            )

        return tuple(candidates)

    def _sample_terminal_connector(
        self,
        start: Pose2D,
        goal: Pose2D,
    ) -> tuple[Point2D, ...] | None:
        """Return the shortest collision-free exact Dubins endpoint connection."""
        if self._heuristic(start, goal) > self._preflight.terminal_connector_max_distance_m:
            return None
        candidates = sorted(
            self._dubins_candidates(start, goal),
            key=lambda item: sum(item[1]),
        )
        radius = self._config.minimum_turning_radius_m
        for word, normalized_lengths in candidates:
            total_length = sum(normalized_lengths) * radius
            if total_length > self._preflight.terminal_connector_max_length_m:
                continue
            pose = start
            points = [Point2D(pose.x, pose.y)]
            collision_free = True
            for symbol, normalized_length in zip(word, normalized_lengths):
                distance = normalized_length * radius
                if distance <= _DUBINS_EPSILON:
                    continue
                curvature = {
                    "L": 1.0 / radius,
                    "R": -1.0 / radius,
                    "S": 0.0,
                }[symbol]
                samples = max(
                    1,
                    math.ceil(
                        distance / self._preflight.terminal_connector_sample_step_m
                    ),
                )
                for sample in range(1, samples + 1):
                    point_pose = self._advance(
                        pose, curvature, distance * sample / samples
                    )
                    if not self._is_free(point_pose.x, point_pose.y):
                        collision_free = False
                        break
                    points.append(Point2D(point_pose.x, point_pose.y))
                if not collision_free:
                    break
                pose = self._advance(pose, curvature, distance)
            if not collision_free:
                continue
            if (
                self._heuristic(pose, goal) > 1.0e-6
                or _angle_distance(pose.yaw, goal.yaw) > 1.0e-6
            ):
                continue
            # This replaces only numerical round-off from the verified curve;
            # it is not a separate line segment to the target.
            points[-1] = Point2D(goal.x, goal.y)
            return tuple(points)
        return None

    @staticmethod
    def _reconstruct_route(
        key: tuple[int, int, int],
        parent: dict[tuple[int, int, int], tuple[int, int, int] | None],
        poses: dict[tuple[int, int, int], Pose2D],
    ) -> list[Point2D]:
        route = [Point2D(poses[key].x, poses[key].y)]
        cursor = key
        while parent[cursor] is not None:
            cursor = parent[cursor]
            source = poses[cursor]
            route.append(Point2D(source.x, source.y))
        route.reverse()
        return route

    def plan(self, start: Pose2D, goal: Pose2D) -> tuple[tuple[Point2D, ...], int] | None:
        """Find one collision-free, minimum-turning-radius path or return None."""
        if not self._is_free(start.x, start.y):
            return None
        if not self._is_free(goal.x, goal.y):
            return None

        start_key = self._state_key(start)
        frontier: list[tuple[float, float, int, tuple[int, int, int], Pose2D]] = []
        heapq.heappush(frontier, (self._heuristic(start, goal), 0.0, 0, start_key, start))
        parent: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start_key: None}
        poses: dict[tuple[int, int, int], Pose2D] = {start_key: start}
        best_cost: dict[tuple[int, int, int], float] = {start_key: 0.0}
        serial = 0
        expanded = 0
        step_length = self._preflight.step_length_m
        radius = self._config.minimum_turning_radius_m
        curvatures = tuple(
            fraction / radius for fraction in self._preflight.steering_fractions
        )

        while frontier and expanded < self._preflight.max_expansions_per_leg:
            _estimate, cost, _order, key, pose = heapq.heappop(frontier)
            if cost != best_cost.get(key):
                continue
            expanded += 1
            connector = self._sample_terminal_connector(pose, goal)
            if connector is not None:
                route = self._reconstruct_route(key, parent, poses)
                route.extend(connector[1:])
                return tuple(route), expanded
            if self._is_goal(pose, goal):
                # The lattice endpoint is already inside the configured goal
                # acceptance region. Keep that physically valid endpoint
                # rather than fabricating a final straight segment to the
                # marker.
                return tuple(self._reconstruct_route(key, parent, poses)), expanded

            for curvature in curvatures:
                free, next_pose = self._primitive_is_free(pose, curvature)
                if not free:
                    continue
                next_key = self._state_key(next_pose)
                turning_cost = self._preflight.turning_penalty_m * abs(
                    curvature * radius
                )
                next_cost = cost + step_length + turning_cost
                if next_cost >= best_cost.get(next_key, math.inf):
                    continue
                best_cost[next_key] = next_cost
                parent[next_key] = key
                poses[next_key] = next_pose
                serial += 1
                heapq.heappush(
                    frontier,
                    (
                        next_cost + self._heuristic(next_pose, goal),
                        next_cost,
                        serial,
                        next_key,
                        next_pose,
                    ),
                )
        return None

    def _matches_constraint(self, pose: Pose2D, goal: _PoseConstraint) -> bool:
        if math.hypot(goal.x - pose.x, goal.y - pose.y) > (
            self._preflight.goal_position_tolerance_m
        ):
            return False
        return goal.yaw is None or _angle_distance(pose.yaw, goal.yaw) <= math.radians(
            self._preflight.goal_heading_tolerance_deg
        )

    @staticmethod
    def _reconstruct_staged_route(
        key: tuple[int, int, int, int],
        parent: dict[
            tuple[int, int, int, int],
            tuple[int, int, int, int] | None,
        ],
        poses: dict[tuple[int, int, int, int], Pose2D],
    ) -> list[tuple[int, Point2D]]:
        route: list[tuple[int, Point2D]] = []
        cursor = key
        while cursor is not None:
            pose = poses[cursor]
            route.append((cursor[0], Point2D(pose.x, pose.y)))
            cursor = parent[cursor]
        route.reverse()
        return route

    @staticmethod
    def _split_staged_route(
        route: Sequence[tuple[int, Point2D]], goal_count: int
    ) -> tuple[tuple[Point2D, ...], ...]:
        legs: list[list[Point2D]] = [[] for _ in range(goal_count)]
        for stage, point in route:
            legs[stage].append(point)
        return tuple(tuple(points) for points in legs)

    def plan_through(
        self,
        start: Pose2D,
        goals: Sequence[_PoseConstraint],
    ) -> _ThroughPlan:
        """Plan an ordered sequence while carrying heading through free-yaw goals.

        Nav2 represents a pass-through pose without an orientation as a zero
        quaternion.  The planner must therefore retain the actual heading at
        that position for the following leg; choosing a route tangent there
        would add a constraint that Nav2 never received.
        """
        targets = tuple(goals)
        if not targets:
            raise ValueError("through-pose planning requires at least one goal")
        if not self._is_free(start.x, start.y) or any(
            not self._is_free(goal.x, goal.y) for goal in targets
        ):
            return _ThroughPlan((), (0,) * len(targets), 0)

        start_key = (0, *self._state_key(start))
        frontier: list[
            tuple[float, float, int, tuple[int, int, int, int], Pose2D]
        ] = []
        first_goal = targets[0]
        heapq.heappush(
            frontier,
            (
                math.hypot(first_goal.x - start.x, first_goal.y - start.y),
                0.0,
                0,
                start_key,
                start,
            ),
        )
        parent: dict[
            tuple[int, int, int, int],
            tuple[int, int, int, int] | None,
        ] = {start_key: None}
        poses: dict[tuple[int, int, int, int], Pose2D] = {start_key: start}
        best_cost: dict[tuple[int, int, int, int], float] = {start_key: 0.0}
        expanded_by_goal = [0] * len(targets)
        completed_goals = 0
        serial = 0
        expanded = 0
        max_expansions = self._preflight.max_expansions_per_leg * len(targets)
        step_length = self._preflight.step_length_m
        radius = self._config.minimum_turning_radius_m
        curvatures = tuple(
            fraction / radius for fraction in self._preflight.steering_fractions
        )

        while frontier and expanded < max_expansions:
            _estimate, cost, _order, key, pose = heapq.heappop(frontier)
            if cost != best_cost.get(key):
                continue
            goal_index = key[0]
            goal = targets[goal_index]
            expanded += 1
            expanded_by_goal[goal_index] += 1

            if goal_index == len(targets) - 1 and goal.yaw is not None:
                connector = self._sample_terminal_connector(
                    pose, Pose2D(goal.x, goal.y, goal.yaw)
                )
                if connector is not None:
                    route = self._reconstruct_staged_route(key, parent, poses)
                    route.extend((goal_index, point) for point in connector[1:])
                    return _ThroughPlan(
                        self._split_staged_route(route, len(targets)),
                        tuple(expanded_by_goal),
                        len(targets),
                    )

            if self._matches_constraint(pose, goal):
                completed_goals = max(completed_goals, goal_index + 1)
                if goal_index == len(targets) - 1:
                    route = self._reconstruct_staged_route(key, parent, poses)
                    return _ThroughPlan(
                        self._split_staged_route(route, len(targets)),
                        tuple(expanded_by_goal),
                        len(targets),
                    )

                next_key = (goal_index + 1, *self._state_key(pose))
                if cost < best_cost.get(next_key, math.inf):
                    best_cost[next_key] = cost
                    parent[next_key] = key
                    poses[next_key] = pose
                    serial += 1
                    next_goal = targets[goal_index + 1]
                    heapq.heappush(
                        frontier,
                        (
                            cost + math.hypot(
                                next_goal.x - pose.x, next_goal.y - pose.y
                            ),
                            cost,
                            serial,
                            next_key,
                            pose,
                        ),
                    )
                continue

            for curvature in curvatures:
                free, next_pose = self._primitive_is_free(pose, curvature)
                if not free:
                    continue
                next_key = (goal_index, *self._state_key(next_pose))
                turning_cost = self._preflight.turning_penalty_m * abs(
                    curvature * radius
                )
                next_cost = cost + step_length + turning_cost
                if next_cost >= best_cost.get(next_key, math.inf):
                    continue
                best_cost[next_key] = next_cost
                parent[next_key] = key
                poses[next_key] = next_pose
                serial += 1
                heapq.heappush(
                    frontier,
                    (
                        next_cost + math.hypot(
                            goal.x - next_pose.x, goal.y - next_pose.y
                        ),
                        next_cost,
                        serial,
                        next_key,
                        next_pose,
                    ),
                )
        return _ThroughPlan((), tuple(expanded_by_goal), completed_goals)


def _segment_waypoints(
    segment: PlanningSegment, indexed: dict[str, Any]
) -> tuple[tuple[str, Any], ...]:
    return tuple((waypoint_id, indexed[waypoint_id]) for waypoint_id in segment.route_ids)


def _constraints_for_segment(
    items: Sequence[tuple[str, Any]], warnings: list[str]
) -> tuple[_PoseConstraint, ...]:
    result: list[_PoseConstraint] = []
    for waypoint_id, waypoint in items:
        point = _point(waypoint)
        if not is_zero_quaternion(waypoint.orientation):
            yaw = _yaw_from_waypoint(waypoint)
        else:
            yaw = None
            warnings.append(
                f"{waypoint_id}: orientation unconstrained; preflight selects "
                "a continuous heading"
            )
        result.append(_PoseConstraint(point.x, point.y, yaw))
    return tuple(result)


def preflight_route(
    reference: FieldReference,
    waypoints: Iterable[Any],
    segments: Sequence[PlanningSegment],
) -> RoutePreflight:
    """Check each segment against static geometry and report a blocked leg.

    This offline result is not a Nav2 planning or reachability result.
    """
    indexed = {waypoint.id: waypoint for waypoint in waypoints}
    planner = LatticePreflightPlanner(reference)
    warnings: list[str] = []
    reports: list[SegmentPreflight] = []

    for segment in segments:
        items = _segment_waypoints(segment, indexed)
        constraints = _constraints_for_segment(items, warnings)
        virtual_offset = math.pi if segment.direction == "reverse" else 0.0
        legs: list[LegPreflight] = []
        failed_message = ""

        if constraints[0].yaw is None:
            start_id, _start = items[0]
            end_id, _end = items[1]
            message = (
                f"{start_id} -> {end_id}: orientation-unconstrained segment starts "
                "cannot be checked offline"
            )
            legs.append(LegPreflight(start_id, end_id, (), 0.0, 0, False, message))
            reports.append(SegmentPreflight(
                segment_id=segment.id,
                direction=segment.direction,
                legs=tuple(legs),
                feasible=False,
                length_m=0.0,
                message=message,
            ))
            continue

        if any(constraint.yaw is None for constraint in constraints[1:]):
            virtual_start = Pose2D(
                constraints[0].x,
                constraints[0].y,
                constraints[0].yaw + virtual_offset,
            )
            virtual_goals = tuple(
                _PoseConstraint(
                    constraint.x,
                    constraint.y,
                    None if constraint.yaw is None else constraint.yaw + virtual_offset,
                )
                for constraint in constraints[1:]
            )
            planned_through = planner.plan_through(virtual_start, virtual_goals)
            if len(planned_through.legs) != len(virtual_goals):
                failed_leg_index = min(
                    planned_through.completed_goals,
                    len(items) - 2,
                )
                start_id, _start = items[failed_leg_index]
                end_id, _end = items[failed_leg_index + 1]
                message = (
                    f"{start_id} -> {end_id}: no collision-free "
                    "minimum-radius route"
                )
                legs.append(LegPreflight(
                    start_id,
                    end_id,
                    (),
                    0.0,
                    planned_through.expanded_states[failed_leg_index],
                    False,
                    message,
                ))
                failed_message = message
            else:
                for (
                    (start_id, _start),
                    (end_id, _end),
                    points,
                    expanded,
                ) in zip(
                    items,
                    items[1:],
                    planned_through.legs,
                    planned_through.expanded_states,
                ):
                    legs.append(LegPreflight(
                        start_id,
                        end_id,
                        points,
                        _polyline_length(points),
                        expanded,
                        True,
                    ))
            feasible = not failed_message and len(legs) == len(items) - 1
            reports.append(SegmentPreflight(
                segment_id=segment.id,
                direction=segment.direction,
                legs=tuple(legs),
                feasible=feasible,
                length_m=sum(leg.length_m for leg in legs),
                message=failed_message,
            ))
            continue

        poses = tuple(
            Pose2D(constraint.x, constraint.y, constraint.yaw)
            for constraint in constraints
        )
        for (start_id, _start), (end_id, _end), start_pose, end_pose in zip(
            items,
            items[1:],
            poses,
            poses[1:],
        ):
            virtual_start = Pose2D(start_pose.x, start_pose.y, start_pose.yaw + virtual_offset)
            virtual_end = Pose2D(end_pose.x, end_pose.y, end_pose.yaw + virtual_offset)
            planned = planner.plan(virtual_start, virtual_end)
            if planned is None:
                message = (
                    f"{start_id} -> {end_id}: no collision-free "
                    "minimum-radius route"
                )
                legs.append(LegPreflight(start_id, end_id, (), 0.0, 0, False, message))
                failed_message = message
                break
            points, expanded = planned
            legs.append(LegPreflight(
                start_id,
                end_id,
                points,
                _polyline_length(points),
                expanded,
                True,
            ))
        feasible = not failed_message and len(legs) == len(items) - 1
        reports.append(SegmentPreflight(
            segment_id=segment.id,
            direction=segment.direction,
            legs=tuple(legs),
            feasible=feasible,
            length_m=sum(leg.length_m for leg in legs),
            message=failed_message,
        ))
    return RoutePreflight(tuple(reports), tuple(dict.fromkeys(warnings)))
