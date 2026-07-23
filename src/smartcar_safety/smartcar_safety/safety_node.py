"""ROS 2 wrapper for the fail-closed velocity guard."""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger

from smartcar_safety.guard import SafetyGuard, validate_publish_frequency
from smartcar_safety.odometry import odometry_is_finite
from smartcar_safety.velocity import (
    ZERO_TWIST_COMPONENTS,
    sanitize_twist_components,
)


LATEST_RELIABLE_QOS = QoSProfile(depth=1)
LATEST_SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def twist_from_components(components):
    """Return a fresh Twist populated from six sanitized components."""
    result = Twist()
    result.linear.x = components[0]
    result.linear.y = components[1]
    result.linear.z = components[2]
    result.angular.x = components[3]
    result.angular.y = components[4]
    result.angular.z = components[5]
    return result


class SafetyNode(Node):
    """Gate /cmd_vel to /cmd_vel_safe and continuously publish a safe command."""

    def __init__(self):
        super().__init__("safety_node")
        self.declare_parameter("command_timeout_sec", 0.30)
        self.declare_parameter("scan_timeout_sec", 0.35)
        self.declare_parameter("odom_timeout_sec", 0.35)
        self.declare_parameter("raw_odom_timeout_sec", 0.25)
        self.declare_parameter("minimum_voltage", 0.0)
        self.declare_parameter("publish_frequency_hz", 20.0)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_odom", True)
        self.declare_parameter("require_raw_odom", True)
        self.declare_parameter("emergency_stop_on_start", False)

        self.guard = SafetyGuard(
            command_timeout_sec=self.get_parameter("command_timeout_sec").value,
            scan_timeout_sec=self.get_parameter("scan_timeout_sec").value,
            odom_timeout_sec=self.get_parameter("odom_timeout_sec").value,
            raw_odom_timeout_sec=self.get_parameter("raw_odom_timeout_sec").value,
            minimum_voltage=self.get_parameter("minimum_voltage").value,
            require_scan=self.get_parameter("require_scan").value,
            require_odom=self.get_parameter("require_odom").value,
            require_raw_odom=self.get_parameter("require_raw_odom").value,
        )
        if bool(self.get_parameter("emergency_stop_on_start").value):
            self.guard.set_emergency_stop(True)
        frequency_hz = validate_publish_frequency(
            self.get_parameter("publish_frequency_hz").value)

        self._zero_command = twist_from_components(ZERO_TWIST_COMPONENTS)
        self._last_command_components = None
        self._last_command_message = None
        self._last_status_reason = None
        self._last_blocked_status_at = None

        self._safe_publisher = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self._status_publisher = self.create_publisher(
            String, "/smartcar/safety/status", 10)
        self.create_subscription(
            Twist, "/cmd_vel", self._on_command, LATEST_RELIABLE_QOS)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, LATEST_SENSOR_QOS, raw=True)
        self.create_subscription(
            Odometry, "/odom_combined", self._on_odom, LATEST_RELIABLE_QOS)
        self.create_subscription(
            Odometry, "/odom", self._on_raw_odom, LATEST_RELIABLE_QOS)
        self.create_subscription(
            Float32, "/PowerVoltage", self._on_voltage, LATEST_RELIABLE_QOS)
        self.create_service(
            SetBool, "/smartcar/safety/emergency_stop", self._on_emergency_stop)
        self.create_service(
            Trigger,
            "/smartcar/safety/clear_localization_fault",
            self._on_clear_localization_fault,
        )
        self.create_timer(1.0 / frequency_hz, self._on_timer)

        now_sec = self._now_sec()
        self._publish_status_if_due(now_sec, self.guard.evaluate(now_sec), True)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_command(self, message):
        now_sec = self._now_sec()
        valid, components = sanitize_twist_components((
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        ))
        if valid:
            self._last_command_components = components
            self._last_command_message = twist_from_components(components)
            self.guard.mark_command(now_sec)
        else:
            self._last_command_components = None
            self._last_command_message = None
            self.guard.mark_command_invalid()
            self._publish_zero_command()
        self._publish_status_if_due(now_sec, self.guard.evaluate(now_sec), True)

    def _on_scan(self, _message):
        now_sec = self._now_sec()
        self.guard.mark_scan(now_sec)

    def _on_odom(self, message):
        now_sec = self._now_sec()
        if odometry_is_finite(message):
            self.guard.mark_odom(now_sec)
        else:
            self.guard.mark_odom_invalid()
            self._publish_zero_command()
            self._publish_status_if_due(
                now_sec, self.guard.evaluate(now_sec), True)

    def _on_raw_odom(self, message):
        now_sec = self._now_sec()
        if odometry_is_finite(message):
            self.guard.mark_raw_odom(now_sec)
        else:
            self.guard.mark_raw_odom_invalid(now_sec)
            self._publish_zero_command()
            self._publish_status_if_due(
                now_sec, self.guard.evaluate(now_sec), True)

    def _on_voltage(self, message):
        now_sec = self._now_sec()
        self.guard.mark_voltage(message.data, now_sec)
        self._publish_status_if_due(now_sec, self.guard.evaluate(now_sec), True)

    def _on_emergency_stop(self, request, response):
        self.guard.set_emergency_stop(request.data)
        now_sec = self._now_sec()
        result = self.guard.evaluate(now_sec)
        self._publish_status_if_due(now_sec, result, True)
        response.success = True
        response.message = (
            "emergency stop latched" if request.data else "emergency stop cleared")
        return response

    def _on_clear_localization_fault(self, _request, response):
        now_sec = self._now_sec()
        fresh_nonzero_command = (
            self._last_command_components is not None
            and self.guard.command_is_fresh(now_sec)
            and any(value != 0.0 for value in self._last_command_components)
        )
        if fresh_nonzero_command:
            response.success = False
            response.message = "stop navigation before clearing localization fault"
        elif self.guard.clear_localization_fault(now_sec):
            response.success = True
            response.message = (
                "localization fault cleared after caller-verified navigation "
                "stop and pose reset"
            )
        else:
            response.success = False
            response.message = "fresh raw and fused odometry required"
        self._publish_status_if_due(
            now_sec, self.guard.evaluate(now_sec), True)
        return response

    def _publish_zero_command(self):
        self._safe_publisher.publish(self._zero_command)

    def _on_timer(self):
        now_sec = self._now_sec()
        result = self.guard.evaluate(now_sec)
        if result["allowed"] and self._last_command_message is not None:
            command = self._last_command_message
        else:
            command = self._zero_command
        self._safe_publisher.publish(command)
        self._publish_status_if_due(now_sec, result)

    def _publish_status_if_due(self, now_sec, result, force=False):
        reason = result["reason"]
        changed = reason != self._last_status_reason
        blocked_repeat_due = (
            not result["allowed"]
            and (
                self._last_blocked_status_at is None
                or now_sec - self._last_blocked_status_at >= 1.0
            )
        )
        if force and changed:
            blocked_repeat_due = True
        if changed or blocked_repeat_due:
            self._status_publisher.publish(String(data=reason))
            self._last_status_reason = reason
            if not result["allowed"]:
                self._last_blocked_status_at = now_sec
            else:
                self._last_blocked_status_at = None


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
