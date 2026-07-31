#!/usr/bin/env python3
"""odom_relay.py — NaN 过滤里程计中继节点

Gazebo AckermannSteering 插件在初始化时发布无效四元数 (全零或 NaN)，
导致 EKF 融合 NaN 并阻塞 Nav2。本节点过滤 NaN 后转发到 /odom_clean。
"""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry


def is_valid_quaternion(x: float, y: float, z: float, w: float) -> bool:
    """检查四元数是否有效（无 NaN、非零长度、接近单位）"""
    if any(math.isnan(v) for v in (x, y, z, w)):
        return False
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:  # 零四元数不能归一化
        return False
    if abs(norm - 1.0) > 0.1:  # 严重偏离单位
        return False
    return True


def is_valid_position(x: float, y: float, z: float) -> bool:
    return not any(math.isnan(v) for v in (x, y, z))


def is_valid_twist(vx: float, vy: float, vz: float,
                   rx: float, ry: float, rz: float) -> bool:
    return not any(math.isnan(v) for v in (vx, vy, vz, rx, ry, rz))


class OdomRelay(Node):
    def __init__(self):
        super().__init__("odom_relay")
        self.sub = self.create_subscription(
            Odometry, "/odom", self.callback, 10
        )
        self.pub = self.create_publisher(Odometry, "/odom_clean", 10)
        self.dropped = 0
        self.published = 0
        self.get_logger().info("odom_relay: /odom → /odom_clean (NaN filter)")

    def callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist

        pos_ok = is_valid_position(p.x, p.y, p.z)
        quat_ok = is_valid_quaternion(q.x, q.y, q.z, q.w)
        twist_ok = is_valid_twist(
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z
        )

        if not pos_ok or not twist_ok:
            self.dropped += 1
            if self.dropped % 50 == 1:
                self.get_logger().warn(
                    f"Dropped {self.dropped} msgs (pos_ok={pos_ok}, "
                    f"twist_ok={twist_ok})"
                )
            return

        # 修复无效四元数 → 单位四元数（无旋转）
        if not quat_ok:
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 0.0
            msg.pose.pose.orientation.w = 1.0
            self.dropped += 1
            if self.dropped % 50 == 1:
                self.get_logger().warn(
                    f"Fixed quaternion in {self.dropped} msgs"
                )
            return

        self.pub.publish(msg)
        self.published += 1
        if self.published == 1:
            self.get_logger().info(
                f"First valid odom: pos=({p.x:.3f},{p.y:.3f}), "
                f"quat=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})"
            )


def main():
    rclpy.init()
    node = OdomRelay()
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
