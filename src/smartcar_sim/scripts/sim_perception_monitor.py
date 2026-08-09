#!/usr/bin/env python3
"""Read-only simulator perception and costmap health monitor.

The monitor deliberately does not publish motion commands. It correlates the
newest non-self LaserScan returns with the odometry TF and both raw costmaps,
including the default track's A-zone cone area, so a simulation run can
distinguish these cases:

* the bridge is publishing a stale or all-minimum scan;
* the scan frame cannot be transformed at the scan timestamp; and
* a valid return never reaches one of Nav2's layered costmaps.

It publishes a transient-local JSON status for launch/test tooling. AutoTrain
uses that status as a pre-goal evidence gate; it is not a safety command.
"""

from __future__ import annotations

import json
import math
import struct
import time
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(stamp: RosTime) -> int:
    """Return a ROS builtin time as nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def normalize_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi] without losing finite precision."""
    return math.remainder(angle, 2.0 * math.pi)


def quaternion_yaw(quaternion: object) -> float:
    """Extract the planar yaw from a ROS quaternion-like object."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def parse_track_landmarks(world_file: str | Path) -> tuple[Landmark, ...]:
    """Read physical A-zone lidar-slice cylinders from the active SDF.

    The raw costmaps alone cannot detect a shared, incorrect global frame: a
    scan transformed by the wrong TF can still agree with a costmap built from
    that same wrong scan.  Parsing the static cone collision geometry gives
    the monitor an independent world-frame reference without duplicating the
    positions in a second configuration file.
    """
    path = Path(world_file)
    if not path.is_file():
        raise ValueError(f"track world file does not exist: {path}")
    root = ET.parse(path).getroot()
    landmarks: list[Landmark] = []
    for model in root.findall(".//model"):
        landmark_id = str(model.attrib.get("name", ""))
        if not landmark_id.startswith("cone_a"):
            continue
        pose_text = (model.findtext("pose") or "").strip()
        pose_values = pose_text.split()
        if len(pose_values) != 6:
            raise ValueError(
                f"{landmark_id} must have a six-element world pose")
        try:
            x, y, _, roll, pitch, _ = (float(value) for value in pose_values)
            radius_text = (model.findtext(
                "./link/collision/geometry/cylinder/radius") or "").strip()
            radius = float(radius_text)
        except ValueError as error:
            raise ValueError(
                f"{landmark_id} has a non-numeric pose or cylinder radius") from error
        if abs(roll) > 1.0e-9 or abs(pitch) > 1.0e-9:
            raise ValueError(f"{landmark_id} must remain upright")
        if radius <= 0.0:
            raise ValueError(f"{landmark_id} must have a positive cylinder collision radius")
        landmarks.append(Landmark(
            landmark_id=landmark_id,
            x=x,
            y=y,
            radius=radius,
        ))
    if not landmarks:
        raise ValueError(f"no physical A-zone cones found in {path}")
    return tuple(sorted(landmarks, key=lambda landmark: landmark.landmark_id))


def point_to_landmark_boundary_distance(
    point: tuple[float, float], landmark: Landmark
) -> float:
    """Return Euclidean distance from a point to a cylindrical lidar slice."""
    dx = point[0] - landmark.x
    dy = point[1] - landmark.y
    return abs(math.hypot(dx, dy) - landmark.radius)


@dataclass
class ScanSample:
    stamp_ns: int
    frame_id: str
    points: list[tuple[float, float]]


@dataclass
class PointCloudSample:
    """Newest depth cloud points in the sensor frame."""

    stamp_ns: int
    frame_id: str
    points: list[tuple[float, float]]


@dataclass(frozen=True)
class OdomPoseSample:
    """One finite odometry pose, retained for scan-time interpolation."""

    stamp_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Landmark:
    """A physical A-zone cone's cylindrical lidar slice."""

    landmark_id: str
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class CostmapMatchSummary:
    """Scan-to-costmap evidence, with rolling-window eligibility separated."""

    valid: bool
    in_window_points: int
    matched_points: int


@dataclass(frozen=True)
class TfOdomAlignment:
    """Comparison of scan-time TF with interpolated relay odometry."""

    valid: bool
    position_error_m: float | None
    yaw_error_rad: float | None
    bracket_span_sec: float | None


class SimPerceptionMonitor(Node):
    """Correlate scan endpoints with the currently published raw costmaps."""

    def __init__(self) -> None:
        super().__init__("sim_perception_monitor")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("depth_points_topic", "/smartcar/depth/points")
        self.declare_parameter("require_depth_points", False)
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter(
            "local_costmap_topic", "/local_costmap/costmap_raw")
        self.declare_parameter(
            "global_costmap_topic", "/global_costmap/costmap_raw")
        self.declare_parameter("ready_topic", "/smartcar/sim_perception_ready")
        self.declare_parameter("min_obstacle_range_m", 0.25)
        self.declare_parameter("max_obstacle_range_m", 2.5)
        self.declare_parameter("min_valid_beams", 5)
        self.declare_parameter("max_scan_odom_skew_sec", 0.075)
        self.declare_parameter("max_sensor_age_sec", 0.35)
        self.declare_parameter("max_costmap_age_sec", 1.5)
        self.declare_parameter("costmap_match_radius_m", 0.12)
        self.declare_parameter("lethal_cost_threshold", 253)
        # The dynamic TF must represent the same vehicle pose as the odometry
        # relay at the scan timestamp, not merely exist in the TF buffer.
        self.declare_parameter("max_tf_odom_position_error_m", 0.02)
        self.declare_parameter("max_tf_odom_yaw_error_rad", 0.05)
        self.declare_parameter("max_odom_interpolation_span_sec", 0.12)
        # The independent Gazebo-world reference prevents a scan and costmap
        # from passing merely because both share the same stale/wrong TF.
        self.declare_parameter("track_world_file", "")
        self.declare_parameter("require_landmark_registration", True)
        self.declare_parameter("landmark_match_tolerance_m", 0.10)
        self.declare_parameter("min_landmark_ids", 3)
        self.declare_parameter("diagnostic_period_sec", 0.5)
        self.declare_parameter("require_a_zone_probe", True)
        # The default track keeps this area free in the keepout PGM and places
        # its movable cone obstacles inside it. A lethal match here therefore
        # proves a real scan return was marked by the obstacle layer.
        self.declare_parameter(
            "a_zone_probe_bounds", [0.45, 3.15, 0.10, 1.25])

        self._min_range = float(
            self.get_parameter("min_obstacle_range_m").value)
        self._max_range = float(
            self.get_parameter("max_obstacle_range_m").value)
        self._min_beams = int(self.get_parameter("min_valid_beams").value)
        self._max_odom_skew = float(
            self.get_parameter("max_scan_odom_skew_sec").value)
        self._max_sensor_age = float(
            self.get_parameter("max_sensor_age_sec").value)
        self._max_costmap_age = float(
            self.get_parameter("max_costmap_age_sec").value)
        self._match_radius = float(
            self.get_parameter("costmap_match_radius_m").value)
        self._lethal_threshold = int(
            self.get_parameter("lethal_cost_threshold").value)
        self._max_tf_odom_position_error = self._positive_parameter(
            "max_tf_odom_position_error_m")
        self._max_tf_odom_yaw_error = self._positive_parameter(
            "max_tf_odom_yaw_error_rad")
        self._max_odom_interpolation_span = self._positive_parameter(
            "max_odom_interpolation_span_sec")
        self._require_depth_points = bool(
            self.get_parameter("require_depth_points").value)
        self._require_landmark_registration = bool(
            self.get_parameter("require_landmark_registration").value)
        self._landmark_match_tolerance = self._positive_parameter(
            "landmark_match_tolerance_m")
        self._min_landmark_ids = int(
            self.get_parameter("min_landmark_ids").value)
        if self._min_landmark_ids <= 0:
            raise ValueError("min_landmark_ids must be positive")
        world_file = str(self.get_parameter("track_world_file").value).strip()
        self._landmarks: tuple[Landmark, ...] = ()
        self._landmark_error: str | None = None
        if self._require_landmark_registration:
            try:
                self._landmarks = parse_track_landmarks(world_file)
            except (OSError, ET.ParseError, ValueError) as error:
                self._landmark_error = str(error)
                self.get_logger().error(
                    "Simulator landmark registration is unavailable: %s",
                    self._landmark_error)
            if len(self._landmarks) < self._min_landmark_ids:
                self._landmark_error = (
                    self._landmark_error or
                    "track world has fewer physical landmark cones than required")
        elif world_file:
            try:
                self._landmarks = parse_track_landmarks(world_file)
            except (OSError, ET.ParseError, ValueError) as error:
                self.get_logger().warn(
                    "Ignoring optional simulator landmark registration: %s", error)
        self._require_a_zone_probe = bool(
            self.get_parameter("require_a_zone_probe").value)
        self._a_zone_probe_bounds = tuple(
            float(value)
            for value in self.get_parameter("a_zone_probe_bounds").value
        )
        if len(self._a_zone_probe_bounds) != 4:
            raise ValueError(
                "a_zone_probe_bounds must be [min_x, max_x, min_y, max_y]")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        costmap_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        scan_topic = str(self.get_parameter("scan_topic").value)
        depth_points_topic = str(
            self.get_parameter("depth_points_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        local_topic = str(self.get_parameter("local_costmap_topic").value)
        global_topic = str(self.get_parameter("global_costmap_topic").value)
        ready_topic = str(self.get_parameter("ready_topic").value)

        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, sensor_qos)
        self._depth_points_sub = self.create_subscription(
            PointCloud2,
            depth_points_topic,
            self._depth_points_callback,
            sensor_qos,
        )
        self._odom_sub = self.create_subscription(
            # Odometry uses the same reliable profile as the simulation relay.
            Odometry,
            odom_topic,
            self._odom_callback,
            reliable_qos,
        )
        self._local_sub = self.create_subscription(
            Costmap, local_topic,
            lambda msg: self._costmap_callback("local", msg), costmap_qos)
        self._global_sub = self.create_subscription(
            Costmap, global_topic,
            lambda msg: self._costmap_callback("global", msg), costmap_qos)
        self._ready_pub = self.create_publisher(
            String, ready_topic, QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._scan: ScanSample | None = None
        self._depth_points: PointCloudSample | None = None
        self._depth_points_stamp_ns: int | None = None
        self._depth_points_count = 0
        # Preserve a positive witness after the latest camera frame becomes
        # empty because all obstacles have left the forward FOV.
        self._depth_points_nonempty_count = 0
        self._odom_stamp_ns: int | None = None
        self._odom_history: deque[OdomPoseSample] = deque(maxlen=512)
        self._costmaps: dict[str, Costmap | None] = {
            "local": None,
            "global": None,
        }
        # Costmap evidence is a startup witness. Once the selected perception
        # source has been observed in each map, an empty forward camera FOV
        # later in the route is not a fusion failure.
        self._costmap_observation_latched = {
            "local": False,
            "global": False,
        }
        self._last_ready: bool | None = None
        self._last_report_wall = 0.0
        self._a_zone_probe_passed = not self._require_a_zone_probe
        self._landmark_registration_passed = not self._require_landmark_registration
        self._landmark_matched_ids: list[str] = []
        self._landmark_matched_points = 0
        self._landmark_max_residual_m: float | None = None
        self._timer = self.create_timer(
            float(self.get_parameter("diagnostic_period_sec").value),
            self._evaluate,
        )
        self.get_logger().info(
            "sim perception monitor: scan/odom/TF -> local+global costmap_raw "
            f"(read-only, require_depth_points={self._require_depth_points})"
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a positive finite number")
        return value

    def _scan_callback(self, msg: LaserScan) -> None:
        points: list[tuple[float, float]] = []
        lower = max(self._min_range, float(msg.range_min))
        upper = min(self._max_range, float(msg.range_max))
        for index, value in enumerate(msg.ranges):
            if not math.isfinite(value) or value <= lower or value >= upper:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            points.append((float(value * math.cos(angle)),
                           float(value * math.sin(angle))))
        self._scan = ScanSample(
            stamp_ns=stamp_ns(msg.header.stamp),
            frame_id=msg.header.frame_id,
            points=points,
        )

    def _depth_points_callback(self, msg: PointCloud2) -> None:
        point_count = int(msg.width) * int(msg.height)
        expected_bytes = int(msg.row_step) * int(msg.height)
        received_stamp_ns = stamp_ns(msg.header.stamp)
        if (
            received_stamp_ns <= 0
            or not str(msg.header.frame_id).strip()
            or msg.point_step <= 0
            or msg.row_step < msg.width * msg.point_step
            or len(msg.data) < expected_bytes
        ):
            return
        fields = {field.name: field for field in msg.fields}
        x_field = fields.get("x")
        y_field = fields.get("y")
        if x_field is None or y_field is None:
            return
        if (
            x_field.datatype != PointField.FLOAT32
            or y_field.datatype != PointField.FLOAT32
        ):
            return
        points: list[tuple[float, float]] = []
        endian = ">" if msg.is_bigendian else "<"
        for row in range(int(msg.height)):
            row_start = row * int(msg.row_step)
            for column in range(int(msg.width)):
                point_start = row_start + column * int(msg.point_step)
                x_offset = point_start + int(x_field.offset)
                y_offset = point_start + int(y_field.offset)
                if x_offset + 4 > len(msg.data) or y_offset + 4 > len(msg.data):
                    continue
                x = struct.unpack_from(endian + "f", msg.data, x_offset)[0]
                y = struct.unpack_from(endian + "f", msg.data, y_offset)[0]
                if math.isfinite(x) and math.isfinite(y):
                    points.append((float(x), float(y)))
        self._depth_points_stamp_ns = received_stamp_ns
        self._depth_points_count = point_count
        self._depth_points = PointCloudSample(
            stamp_ns=received_stamp_ns,
            frame_id=str(msg.header.frame_id),
            points=points,
        )
        if point_count > 0:
            self._depth_points_nonempty_count = max(
                self._depth_points_nonempty_count, point_count)

    def _odom_callback(self, msg: Odometry) -> None:
        received_stamp_ns = stamp_ns(msg.header.stamp)
        position = msg.pose.pose.position
        yaw = quaternion_yaw(msg.pose.pose.orientation)
        if (
            received_stamp_ns <= 0
            or not all(math.isfinite(value) for value in (
                position.x, position.y, yaw))
        ):
            return
        if self._odom_history and received_stamp_ns <= self._odom_history[-1].stamp_ns:
            return
        sample = OdomPoseSample(
            stamp_ns=received_stamp_ns,
            x=float(position.x),
            y=float(position.y),
            yaw=yaw,
        )
        self._odom_history.append(sample)
        self._odom_stamp_ns = received_stamp_ns

    def _costmap_callback(self, name: str, msg: Costmap) -> None:
        self._costmaps[name] = msg

    @staticmethod
    def _costmap_stamp_ns(msg: Costmap) -> int:
        header_stamp = stamp_ns(msg.header.stamp)
        update_stamp = stamp_ns(msg.metadata.update_time)
        # Some Nav2 versions leave the outer header at zero while updating the
        # metadata timestamp, so accept whichever source is newer.
        return max(header_stamp, update_stamp)

    @staticmethod
    def _transform_xy(
        point: tuple[float, float], transform: object
    ) -> tuple[float, float]:
        yaw = quaternion_yaw(transform.transform.rotation)
        x, y = point
        return (
            transform.transform.translation.x + math.cos(yaw) * x
            - math.sin(yaw) * y,
            transform.transform.translation.y + math.sin(yaw) * x
            + math.cos(yaw) * y,
        )

    def _interpolate_odom_pose(self, target_stamp_ns: int) -> tuple[
        OdomPoseSample, float
    ] | None:
        """Interpolate relay odometry only when it brackets the scan stamp."""
        history = self._odom_history
        if len(history) < 2:
            return None
        previous: OdomPoseSample | None = None
        for current in history:
            if current.stamp_ns < target_stamp_ns:
                previous = current
                continue
            if previous is None:
                return None
            span_ns = current.stamp_ns - previous.stamp_ns
            if span_ns <= 0:
                return None
            span_sec = span_ns / 1.0e9
            if span_sec > self._max_odom_interpolation_span:
                return None
            ratio = (target_stamp_ns - previous.stamp_ns) / span_ns
            if ratio < 0.0 or ratio > 1.0:
                return None
            yaw_delta = normalize_angle(current.yaw - previous.yaw)
            return OdomPoseSample(
                stamp_ns=target_stamp_ns,
                x=previous.x + (current.x - previous.x) * ratio,
                y=previous.y + (current.y - previous.y) * ratio,
                yaw=normalize_angle(previous.yaw + yaw_delta * ratio),
            ), span_sec
        return None

    def _tf_odom_alignment(self, sample: ScanSample) -> TfOdomAlignment:
        """Verify scan-time base TF and relayed odometry describe one pose."""
        interpolated = self._interpolate_odom_pose(sample.stamp_ns)
        if interpolated is None:
            return TfOdomAlignment(False, None, None, None)
        odom, bracket_span_sec = interpolated
        try:
            transform = self._tf_buffer.lookup_transform(
                "odom_combined",
                "base_footprint",
                Time(nanoseconds=sample.stamp_ns),
                timeout=Duration(seconds=0.05),
            )
        except (TransformException, ValueError):
            return TfOdomAlignment(False, None, None, bracket_span_sec)
        transform_yaw = quaternion_yaw(transform.transform.rotation)
        position_error = math.hypot(
            transform.transform.translation.x - odom.x,
            transform.transform.translation.y - odom.y,
        )
        yaw_error = abs(normalize_angle(transform_yaw - odom.yaw))
        valid = (
            math.isfinite(position_error)
            and math.isfinite(yaw_error)
            and position_error <= self._max_tf_odom_position_error
            and yaw_error <= self._max_tf_odom_yaw_error
        )
        return TfOdomAlignment(
            valid, position_error, yaw_error, bracket_span_sec)

    def _landmark_matches(
        self, points: list[tuple[float, float]]
    ) -> tuple[list[str], int, float | None]:
        """Find physical cones whose collision boundaries contain scan hits."""
        if not self._landmarks or not points:
            return [], 0, None
        matched_ids: set[str] = set()
        matched_points = 0
        residuals: list[float] = []
        for point in points:
            landmark, residual = min(
                (
                    (candidate, point_to_landmark_boundary_distance(
                        point, candidate))
                    for candidate in self._landmarks
                ),
                key=lambda item: item[1],
            )
            if residual <= self._landmark_match_tolerance:
                matched_ids.add(landmark.landmark_id)
                matched_points += 1
                residuals.append(residual)
        return (
            sorted(matched_ids),
            matched_points,
            max(residuals) if residuals else None,
        )

    def _landmarks_in_scan_range(self, sample: ScanSample) -> list[str]:
        """Return physical cones whose collision surface is inside scan range."""
        if not self._landmarks:
            return []
        try:
            transform = self._tf_buffer.lookup_transform(
                "odom_combined",
                sample.frame_id,
                Time(nanoseconds=sample.stamp_ns),
                timeout=Duration(seconds=0.05),
            )
        except (TransformException, ValueError):
            return []
        sensor_x = transform.transform.translation.x
        sensor_y = transform.transform.translation.y
        return [
            landmark.landmark_id
            for landmark in self._landmarks
            if math.hypot(landmark.x - sensor_x, landmark.y - sensor_y)
            <= self._max_range + landmark.radius
        ]

    def _world_points(
        self, sample: ScanSample | PointCloudSample
    ) -> list[tuple[float, float]] | None:
        if not sample.frame_id or not sample.points:
            return []
        try:
            transform = self._tf_buffer.lookup_transform(
                "odom_combined",
                sample.frame_id,
                Time(nanoseconds=sample.stamp_ns),
                timeout=Duration(seconds=0.05),
            )
        except (TransformException, ValueError):
            return None
        return [
            self._transform_xy(point, transform)
            for point in sample.points
        ]

    def _costmap_match_summary(
        self, msg: Costmap, points: list[tuple[float, float]]
    ) -> CostmapMatchSummary:
        resolution = float(msg.metadata.resolution)
        size_x = int(msg.metadata.size_x)
        size_y = int(msg.metadata.size_y)
        if resolution <= 0.0 or size_x <= 0 or size_y <= 0:
            return CostmapMatchSummary(False, 0, 0)
        if len(msg.data) < size_x * size_y:
            return CostmapMatchSummary(False, 0, 0)

        origin = msg.metadata.origin
        q = origin.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        radius = max(1, int(math.ceil(self._match_radius / resolution)))
        in_window_points = 0
        matched_points = 0
        for world_x, world_y in points:
            dx = world_x - origin.position.x
            dy = world_y - origin.position.y
            # Transform the world point into the costmap's cell frame.
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            mx = int(math.floor(local_x / resolution))
            my = int(math.floor(local_y / resolution))
            if mx < 0 or mx >= size_x or my < 0 or my >= size_y:
                continue
            in_window_points += 1
            matched = False
            for x in range(mx - radius, mx + radius + 1):
                if x < 0 or x >= size_x:
                    continue
                for y in range(my - radius, my + radius + 1):
                    if y < 0 or y >= size_y:
                        continue
                    if int(msg.data[y * size_x + x]) >= self._lethal_threshold:
                        matched = True
                        break
                if matched:
                    break
            if matched:
                matched_points += 1
        return CostmapMatchSummary(
            True, in_window_points, matched_points)

    def _costmap_has_match(
        self, msg: Costmap, points: list[tuple[float, float]]
    ) -> bool:
        """Compatibility predicate for callers that only need a lethal match."""
        return self._costmap_match_summary(msg, points).matched_points > 0

    def _a_zone_points(
        self, points: list[tuple[float, float]]
    ) -> list[tuple[float, float]]:
        min_x, max_x, min_y, max_y = self._a_zone_probe_bounds
        return [
            point for point in points
            if min_x <= point[0] <= max_x and min_y <= point[1] <= max_y
        ]

    def _evaluate(self) -> None:
        sample = self._scan
        odom_stamp = self._odom_stamp_ns
        now_ns = stamp_ns(self.get_clock().now().to_msg())
        scan_age_sec = (
            (now_ns - sample.stamp_ns) / 1e9
            if sample is not None and now_ns >= sample.stamp_ns else None
        )
        odom_age_sec = (
            (now_ns - odom_stamp) / 1e9
            if odom_stamp is not None and now_ns >= odom_stamp else None
        )
        scan_fresh = (
            scan_age_sec is not None
            and scan_age_sec <= self._max_sensor_age
        )
        odom_fresh = (
            odom_age_sec is not None
            and odom_age_sec <= self._max_sensor_age
        )
        scan_odom_skew_sec = (
            abs(sample.stamp_ns - odom_stamp) / 1e9
            if sample is not None and odom_stamp is not None else None
        )
        depth_points_age_sec = (
            (now_ns - self._depth_points_stamp_ns) / 1e9
            if (
                self._depth_points_stamp_ns is not None
                and now_ns >= self._depth_points_stamp_ns
            ) else None
        )
        depth_points_fresh = (
            depth_points_age_sec is not None
            and depth_points_age_sec <= self._max_sensor_age
        )
        checks = {
            "scan": sample is not None
            and len(sample.points) >= self._min_beams
            and scan_fresh,
            "odom": odom_stamp is not None and odom_fresh,
            "tf": False,
            "tf_odom_alignment": False,
            "local": self._costmap_observation_latched["local"],
            "global": self._costmap_observation_latched["global"],
            "a_zone_probe": self._a_zone_probe_passed,
            "landmarks": self._landmark_registration_passed,
        }
        if self._require_depth_points:
            checks["depth_points"] = (
                depth_points_fresh and self._depth_points_nonempty_count > 0
            )
        points: list[tuple[float, float]] = []
        probe_points: list[tuple[float, float]] = []
        landmark_ids: list[str] = []
        landmark_matched_points = 0
        landmark_max_residual_m: float | None = None
        current_landmark_ids: list[str] = []
        current_landmark_matched_points = 0
        current_landmark_max_residual_m: float | None = None
        current_landmark_expected_ids: list[str] = []
        current_landmark_valid = False
        tf_odom_alignment = TfOdomAlignment(False, None, None, None)
        costmap_summaries = {
            name: CostmapMatchSummary(False, 0, 0)
            for name in ("local", "global")
        }
        probe_summaries = {
            name: CostmapMatchSummary(False, 0, 0)
            for name in ("local", "global")
        }
        costmap_age_sec = {name: None for name in ("local", "global")}
        if sample is not None and checks["scan"]:
            if odom_stamp is not None:
                checks["odom"] = (
                    scan_odom_skew_sec is not None
                    and scan_odom_skew_sec <= self._max_odom_skew
                )
            world_points = self._world_points(sample)
            checks["tf"] = world_points is not None
            if world_points:
                points = world_points
                tf_odom_alignment = self._tf_odom_alignment(sample)
                checks["tf_odom_alignment"] = tf_odom_alignment.valid
                (
                    current_landmark_ids,
                    current_landmark_matched_points,
                    current_landmark_max_residual_m,
                ) = self._landmark_matches(points)
                current_landmark_expected_ids = self._landmarks_in_scan_range(sample)
                if not self._landmark_registration_passed:
                    self._landmark_matched_ids = current_landmark_ids
                    self._landmark_matched_points = current_landmark_matched_points
                    self._landmark_max_residual_m = current_landmark_max_residual_m
                    self._landmark_registration_passed = (
                        self._landmark_error is None
                        and len(current_landmark_ids) >= self._min_landmark_ids
                    )
                landmark_ids = self._landmark_matched_ids
                landmark_matched_points = self._landmark_matched_points
                landmark_max_residual_m = self._landmark_max_residual_m
                expected_ids = set(current_landmark_expected_ids)
                required_current_ids = min(
                    self._min_landmark_ids, len(expected_ids))
                current_landmark_valid = (
                    not expected_ids
                    or len(set(current_landmark_ids) & expected_ids)
                    >= required_current_ids
                    # Static landmark registration is a startup witness. A
                    # front camera/lidar may legitimately leave the A-zone
                    # cones out of its current field of view later on.
                    or self._a_zone_probe_passed
                )
                checks["landmarks"] = (
                    self._landmark_registration_passed
                    and current_landmark_valid
                )
                observation_sample: ScanSample | PointCloudSample = sample
                if self._require_depth_points and self._depth_points is not None:
                    observation_sample = self._depth_points
                observation_age_sec = (
                    (now_ns - observation_sample.stamp_ns) / 1e9
                    if now_ns >= observation_sample.stamp_ns else None
                )
                observation_world_points = (
                    self._world_points(observation_sample)
                    if observation_age_sec is not None
                    and observation_age_sec <= self._max_sensor_age
                    else []
                )
                for name in ("local", "global"):
                    costmap = self._costmaps[name]
                    if costmap is None:
                        continue
                    costmap_age_sec[name] = (
                        abs(self._costmap_stamp_ns(costmap) - observation_sample.stamp_ns)
                        / 1e9
                    )
                    if costmap_age_sec[name] > self._max_costmap_age:
                        continue
                    summary = self._costmap_match_summary(
                        costmap, observation_world_points)
                    costmap_summaries[name] = summary
                    if summary.matched_points > 0:
                        self._costmap_observation_latched[name] = True
                    if not summary.valid:
                        checks[name] = self._costmap_observation_latched[name]
                        continue
                    # Once a positive source-to-costmap observation has been
                    # established, an empty rolling window is not a failure.
                    checks[name] = (
                        self._costmap_observation_latched[name]
                        or summary.in_window_points == 0
                    )

                if not self._a_zone_probe_passed:
                    probe_points = self._a_zone_points(observation_world_points)
                    if probe_points:
                        for name in ("local", "global"):
                            costmap = self._costmaps[name]
                            if (
                                costmap is not None
                                and costmap_age_sec[name] is not None
                                and costmap_age_sec[name]
                                <= self._max_costmap_age
                            ):
                                probe_summaries[name] = (
                                    self._costmap_match_summary(
                                        costmap, probe_points))
                        local_probe = probe_summaries["local"]
                        global_probe = probe_summaries["global"]
                        # This startup-only latch is intentionally stricter
                        # than the rolling-window exemption above: the A-zone
                        # cones must be inside both maps at P, proving actual
                        # obstacle-layer marking in each one before a route can
                        # send its first motion goal.
                        self._a_zone_probe_passed = (
                            local_probe.valid
                            and local_probe.in_window_points > 0
                            and local_probe.matched_points > 0
                            and global_probe.valid
                            and global_probe.in_window_points > 0
                            and global_probe.matched_points > 0
                        )
                    checks["a_zone_probe"] = self._a_zone_probe_passed

        ready = all(checks.values())
        costmap_stamps = {
            name: (
                self._costmap_stamp_ns(costmap)
                if costmap is not None else None
            )
            for name, costmap in self._costmaps.items()
        }
        status = {
            "schema_version": 2,
            "ready": ready,
            "checks": checks,
            "valid_beams": len(sample.points) if sample is not None else 0,
            "clock_stamp_ns": now_ns,
            "scan_stamp_ns": sample.stamp_ns if sample is not None else None,
            "depth_points_stamp_ns": self._depth_points_stamp_ns,
            "depth_points_count": self._depth_points_nonempty_count,
            "depth_points_current_count": self._depth_points_count,
            "depth_points_age_sec": depth_points_age_sec,
            "odom_stamp_ns": odom_stamp,
            "scan_age_sec": scan_age_sec,
            "odom_age_sec": odom_age_sec,
            "scan_odom_skew_sec": scan_odom_skew_sec,
            "tf_odom_position_error_m": tf_odom_alignment.position_error_m,
            "tf_odom_yaw_error_rad": tf_odom_alignment.yaw_error_rad,
            "tf_odom_bracket_span_sec": tf_odom_alignment.bracket_span_sec,
            "landmark_required_ids": self._min_landmark_ids,
            "landmark_matched_ids": landmark_ids,
            "landmark_matched_points": landmark_matched_points,
            "landmark_max_residual_m": landmark_max_residual_m,
            "landmark_error": self._landmark_error,
            "landmark_current_expected_ids": current_landmark_expected_ids,
            "landmark_current_matched_ids": current_landmark_ids,
            "landmark_current_matched_points": current_landmark_matched_points,
            "landmark_current_max_residual_m": current_landmark_max_residual_m,
            "landmark_current_valid": current_landmark_valid,
            "local_costmap_stamp_ns": costmap_stamps["local"],
            "global_costmap_stamp_ns": costmap_stamps["global"],
            "local_costmap_age_sec": costmap_age_sec["local"],
            "global_costmap_age_sec": costmap_age_sec["global"],
            "local_in_window_points": (
                costmap_summaries["local"].in_window_points),
            "local_matched_points": (
                costmap_summaries["local"].matched_points),
            "local_window_exempt": (
                costmap_summaries["local"].valid
                and costmap_summaries["local"].in_window_points == 0),
            "global_in_window_points": (
                costmap_summaries["global"].in_window_points),
            "global_matched_points": (
                costmap_summaries["global"].matched_points),
            "a_zone_probe_points": len(probe_points),
            "a_zone_local_in_window_points": (
                probe_summaries["local"].in_window_points),
            "a_zone_local_matched_points": (
                probe_summaries["local"].matched_points),
            "a_zone_global_in_window_points": (
                probe_summaries["global"].in_window_points),
            "a_zone_global_matched_points": (
                probe_summaries["global"].matched_points),
            "published_ros_stamp_ns": stamp_ns(
                self.get_clock().now().to_msg()),
        }
        message = String()
        message.data = json.dumps(
            status, sort_keys=True, separators=(",", ":"))
        self._ready_pub.publish(message)
        now_wall = time.monotonic()
        if ready != self._last_ready:
            if ready:
                self.get_logger().info(
                    "sim perception READY: %s (valid_beams=%d)" %
                    (checks, len(points)))
            else:
                self.get_logger().warn(
                    "sim perception NOT READY: %s (valid_beams=%d)" %
                    (checks, len(points)))
            self._last_ready = ready
            self._last_report_wall = now_wall
        elif not ready and now_wall - self._last_report_wall >= 5.0:
            self.get_logger().warn(
                "sim perception still not ready: %s (valid_beams=%d)" %
                (checks, len(points))
            )
            self._last_report_wall = now_wall


def main() -> None:
    rclpy.init()
    node = SimPerceptionMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
