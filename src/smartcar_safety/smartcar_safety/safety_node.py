"""ROS 2 wrapper for the fail-closed velocity guard."""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool

from smartcar_safety.guard import SafetyGuard


def copy_twist(source):
    """Return a fresh Twist message with all velocity fields copied."""
    result = Twist()
    result.linear.x = source.linear.x
    result.linear.y = source.linear.y
    result.linear.z = source.linear.z
    result.angular.x = source.angular.x
    result.angular.y = source.angular.y
    result.angular.z = source.angular.z
    return result


class SafetyNode(Node):
    """Gate /cmd_vel to /cmd_vel_safe and continuously publish a safe command."""

    def __init__(self):
        super().__init__("safety_node")
        self.declare_parameter("command_timeout_sec", 0.30)
        self.declare_parameter("scan_timeout_sec", 0.35)
        self.declare_parameter("odom_timeout_sec", 0.35)
        self.declare_parameter("minimum_voltage", 0.0)
        self.declare_parameter("publish_frequency_hz", 20.0)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_odom", True)

        self.guard = SafetyGuard(
            command_timeout_sec=self.get_parameter("command_timeout_sec").value,
            scan_timeout_sec=self.get_parameter("scan_timeout_sec").value,
            odom_timeout_sec=self.get_parameter("odom_timeout_sec").value,
            minimum_voltage=self.get_parameter("minimum_voltage").value,
            require_scan=self.get_parameter("require_scan").value,
            require_odom=self.get_parameter("require_odom").value,
        )
        frequency_hz = float(self.get_parameter("publish_frequency_hz").value)
        if frequency_hz <= 0.0:
            raise ValueError("publish_frequency_hz must be positive")

        self._last_command = None
        self._last_status_reason = None
        self._last_blocked_status_at = None

        self._safe_publisher = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self._status_publisher = self.create_publisher(
            String, "/smartcar/safety/status", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Odometry, "/odom_combined", self._on_odom, 10)
        self.create_subscription(Float32, "/PowerVoltage", self._on_voltage, 10)
        self.create_service(
            SetBool, "/smartcar/safety/emergency_stop", self._on_emergency_stop)
        self.create_timer(1.0 / frequency_hz, self._on_timer)

        self._publish_status_if_due(self._now_sec(), self.guard.evaluate(self._now_sec()), True)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_command(self, message):
        now_sec = self._now_sec()
        self._last_command = copy_twist(message)
        self.guard.mark_command(now_sec)
        self._publish_status_if_due(now_sec, self.guard.evaluate(now_sec), True)

    def _on_scan(self, _message):
        now_sec = self._now_sec()
        self.guard.mark_scan(now_sec)
        self._publish_status_if_due(now_sec, self.guard.evaluate(now_sec), True)

    def _on_odom(self, _message):
        now_sec = self._now_sec()
        self.guard.mark_odom(now_sec)
        self._publish_status_if_due(now_sec, self.guard.evaluate(now_sec), True)

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

    def _on_timer(self):
        now_sec = self._now_sec()
        result = self.guard.evaluate(now_sec)
        if result["allowed"] and self._last_command is not None:
            command = copy_twist(self._last_command)
        else:
            command = Twist()
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
