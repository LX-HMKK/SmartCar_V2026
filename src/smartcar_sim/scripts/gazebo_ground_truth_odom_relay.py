#!/usr/bin/env python3
"""Publish simulation odometry from Gazebo's physical model pose.

The Fortress Ackermann system publishes an integrated ``/odom`` estimate.
That estimate can drift away from the DART model pose while the laser scan,
costmaps, and RViz remain internally consistent with the bad estimate. For
local Gazebo only, consume the model-local continuous ``PosePublisher`` stream
instead and make this node the sole owner of
``odom_combined -> base_footprint``. SceneBroadcaster's world
``dynamic_pose`` topic is event-driven, so its initial sample can disappear
before ROS DDS discovery and never be repeated while a parked vehicle stays
still.

The bridge converts ``gz.msgs.Pose_V`` to ``TFMessage`` and retains the source
stamp from each inner model pose. Dynamic TF must retain that exact simulation
time: substituting the latest ``/clock`` time or replaying an old pose makes
stale laser returns appear to be current in RViz and Nav2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def stamp_to_nanoseconds(stamp: object) -> int:
    """Convert a ROS time-like object to a non-negative integer nanosecond stamp."""
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    return max(0, seconds * 1_000_000_000 + nanoseconds)


def stamp_from_nanoseconds(stamp_ns: int):
    """Build a builtin ROS time message without mixing wall and simulation time."""
    from builtin_interfaces.msg import Time

    stamp = Time()
    stamp.sec = int(stamp_ns // 1_000_000_000)
    stamp.nanosec = int(stamp_ns % 1_000_000_000)
    return stamp


def normalize_angle(angle: float) -> float:
    """Normalize one planar yaw angle into [-pi, pi]."""
    return math.remainder(angle, 2.0 * math.pi)


@dataclass(frozen=True)
class PlanarPose:
    """Finite world pose of the simulated rear-axle model origin."""

    x: float
    y: float
    yaw: float


def planar_pose_from_transform(transform: TransformStamped) -> PlanarPose | None:
    """Project a Gazebo model transform onto Nav2's planar odometry frame."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    values = (
        translation.x,
        translation.y,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    norm = math.sqrt(
        rotation.x * rotation.x + rotation.y * rotation.y +
        rotation.z * rotation.z + rotation.w * rotation.w
    )
    if norm <= 1.0e-9:
        return None
    x = rotation.x / norm
    y = rotation.y / norm
    z = rotation.z / norm
    w = rotation.w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return PlanarPose(float(translation.x), float(translation.y), yaw)


class GazeboGroundTruthOdomRelay(Node):
    """Translate Gazebo's model world pose into Nav2 odometry and dynamic TF."""

    def __init__(self) -> None:
        super().__init__("gazebo_ground_truth_odom_relay")
        self.declare_parameter(
            "physical_pose_topic", "/model/origincar/pose")
        # The model-local PosePublisher topic contains exactly the physical
        # OriginCar model pose. Reject a changed Gazebo publisher shape rather
        # than silently selecting a different entity.
        self.declare_parameter("expected_pose_count", 1)
        self.declare_parameter("model_name", "origincar")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("odom_frame", "odom_combined")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("max_twist_interval_sec", 0.25)

        self._expected_pose_count = int(
            self.get_parameter("expected_pose_count").value)
        self._model_name = str(self.get_parameter("model_name").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._max_twist_interval_sec = float(
            self.get_parameter("max_twist_interval_sec").value)
        if not self._model_name or not self._odom_frame or not self._base_frame:
            raise ValueError(
                "model_name, odom_frame, and base_frame must be non-empty")
        if self._expected_pose_count <= 0:
            raise ValueError("expected_pose_count must be positive")
        if not math.isfinite(self._max_twist_interval_sec) or self._max_twist_interval_sec <= 0.0:
            raise ValueError("max_twist_interval_sec must be positive and finite")

        physical_pose_topic = str(
            self.get_parameter("physical_pose_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._last_published_stamp_ns = -1
        self._last_measured_pose: PlanarPose | None = None
        self._last_measured_stamp_ns: int | None = None

        self._pose_sub = self.create_subscription(
            TFMessage, physical_pose_topic, self._pose_callback, 20)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 30)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            "Gazebo physical pose -> "
            f"{odom_topic} + {self._odom_frame} -> {self._base_frame} "
            f"(source={physical_pose_topic}, model={self._model_name}, "
            f"transforms={self._expected_pose_count})")

    def _pose_callback(self, message: TFMessage) -> None:
        if len(message.transforms) != self._expected_pose_count:
            self.get_logger().warn(
                "Ignoring Gazebo transform array with "
                f"{len(message.transforms)} transforms; expected "
                f"{self._expected_pose_count}")
            return
        model_transform = next(
            (
                transform for transform in message.transforms
                if transform.child_frame_id == self._model_name
            ),
            None,
        )
        if model_transform is None:
            self.get_logger().warn(
                "Ignoring Gazebo pose stream without model transform "
                f"{self._model_name}")
            return
        stamp_ns = stamp_to_nanoseconds(model_transform.header.stamp)
        if stamp_ns <= 0:
            self.get_logger().warn(
                "Ignoring Gazebo physical pose without a simulation timestamp")
            return
        if stamp_ns <= self._last_published_stamp_ns:
            self.get_logger().warn(
                "Ignoring non-monotonic Gazebo physical pose timestamp "
                f"{stamp_ns} after {self._last_published_stamp_ns}")
            return
        pose = planar_pose_from_transform(model_transform)
        if pose is None:
            self.get_logger().warn("Ignoring non-finite Gazebo model pose")
            return
        self._publish_pose(pose, stamp_ns, estimate_twist=True)

    def _publish_pose(
        self,
        pose: PlanarPose,
        stamp_ns: int,
        *,
        estimate_twist: bool,
    ) -> None:
        """Publish a physical pose using a strictly increasing sim timestamp."""
        if stamp_ns <= self._last_published_stamp_ns:
            return

        odom = Odometry()
        odom.header.stamp = stamp_from_nanoseconds(stamp_ns)
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        odom.pose.covariance[0] = 1.0e-6
        odom.pose.covariance[7] = 1.0e-6
        odom.pose.covariance[35] = 1.0e-6
        if estimate_twist:
            self._fill_measured_twist(odom, pose, stamp_ns)
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        self._last_published_stamp_ns = stamp_ns

    def _fill_measured_twist(
        self, odom: Odometry, pose: PlanarPose, stamp_ns: int
    ) -> None:
        """Estimate a body-frame twist by finite difference of physical poses."""
        if self._last_measured_pose is None or self._last_measured_stamp_ns is None:
            self._last_measured_pose = pose
            self._last_measured_stamp_ns = stamp_ns
            return
        interval_sec = (
            stamp_ns - self._last_measured_stamp_ns
        ) / 1_000_000_000.0
        if not 1.0e-4 <= interval_sec <= self._max_twist_interval_sec:
            self._last_measured_pose = pose
            self._last_measured_stamp_ns = stamp_ns
            return
        dx = pose.x - self._last_measured_pose.x
        dy = pose.y - self._last_measured_pose.y
        world_vx = dx / interval_sec
        world_vy = dy / interval_sec
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        odom.twist.twist.linear.x = cos_yaw * world_vx + sin_yaw * world_vy
        odom.twist.twist.linear.y = -sin_yaw * world_vx + cos_yaw * world_vy
        odom.twist.twist.angular.z = normalize_angle(
            pose.yaw - self._last_measured_pose.yaw) / interval_sec
        odom.twist.covariance[0] = 2.5e-5
        odom.twist.covariance[7] = 2.5e-5
        odom.twist.covariance[35] = 2.5e-5
        self._last_measured_pose = pose
        self._last_measured_stamp_ns = stamp_ns


def main() -> None:
    rclpy.init()
    node = GazeboGroundTruthOdomRelay()
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
